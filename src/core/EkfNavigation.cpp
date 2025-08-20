/**
 * @file EkfNavigation.cpp
 * @brief Implementation of Extended Kalman Filter-based INS/GPS navigation system
 *
 * Contains the core implementation of the EKF-based integrated navigation system, including:
 * - Strapdown inertial navigation mechanization
 * - EKF prediction and update steps
 * - Error state correction
 * - RTS smoothing preparation
 * - Earth model computations and coordinate transformations
 *
 * @author peanut-nav
 * @date Created: 2025-07-22
 * @last Modified: 2025-08-04
 * @version 0.3.0
 */

#include "NavigationFactory.hpp"
#include "MathUtils.hpp"
#include <iostream>
#include <iomanip>
#include <cmath>

using namespace Eigen;
using namespace std;

// ================== Initialization ==================
/**
 * @brief Initialize EKF navigation system
 */
void EkfNavigation::initialize(const NavParamsBase& base_params, 
                              NavigationState& state) {
    // Convert to EKF specific parameters
    const EkfParams& params = dynamic_cast<const EkfParams&>(base_params);
    params_ = params;
    ekf_ = params.ekf_params;  // Initialize EKF parameters
    state_ = state;            // Set initial navigation state
    
    // Calculate measurement update interval
    measurement_interval_ = params_.imu_rate / params_.gps_rate;
    
    // Initialize Earth rotation rate in navigation frame
    double lat0 = state_.Latitude[0] * M_PI / 180.0;
    wie_n_ << 0, 
             params_.earth_params.W_ie * cos(lat0),
             params_.earth_params.W_ie * sin(lat0);
    
    // Compute transport rate in tangent frame
    wet_t_ << -state_.Velocity[0](1)/(params_.earth_params.Ry + state_.Altitude[0]), 
             state_.Velocity[0](0)/(params_.earth_params.Rx + state_.Altitude[0]), 
             state_.Velocity[0](0) * tan(lat0)/(params_.earth_params.Rx + state_.Altitude[0]);
    
    // Total angular rate in tangent frame and convert to body frame
    Vector3d wit_t = wie_n_ + wet_t_;
    wit_b_ = state_.CbtM * wit_t;
    
    // Current Earth curvature radii
    current_Rx_ = params_.earth_params.Rx;
    current_Ry_ = params_.earth_params.Ry;
    
    // Initialize state vectors to zero
    int totalPoints = state_.Latitude.size();
    state_.Velocity.resize(totalPoints, Vector3d::Zero());
    state_.Pitch.resize(totalPoints);
    state_.Roll.resize(totalPoints);
    state_.Yaw.resize(totalPoints);
    
    // Initialize EKF parameters
    ekf_.disA = MatrixXd::Identity(15, 15);  // Discrete state matrix
    ekf_.disQ = MatrixXd::Zero(15, 15);      // Discrete process noise covariance
}

// ================== Strapdown Inertial Navigation Update ==================
/**
 * @brief Perform strapdown inertial navigation update
 */
void EkfNavigation::updateStrapdown(const IMUData& imu, int i) {
    // Convert gyro data from deg/h to rad/s and update attitude
    Vector3d wtb_b(imu.gx[i], imu.gy[i], imu.gz[i]);
    wtb_b = wtb_b * (M_PI/(3600.0*180.0));

    strapdownAttitudeUpdate(wtb_b, wit_b_, state_.Quaternion, params_.imu_rate, state_.CbtM);
    state_.CtbM = state_.CbtM.transpose();  // Update inverse transformation
    
    // Convert accelerometer data to navigation frame
    Vector3d f_b(imu.ax[i], imu.ay[i], imu.az[i]);
    last_f_INSt_ = state_.CtbM * f_b;  // Save for EKF prediction
    
    // Compute gravity at current position
    double g = computeGravity(state_.Latitude[i], state_.Altitude[i], params_.earth_params);
    
    // Update Earth angular rates based on current position
    double lat_i_rad = state_.Latitude[i] * M_PI / 180.0;
    wie_n_ << 0, 
             params_.earth_params.W_ie * cos(lat_i_rad),
             params_.earth_params.W_ie * sin(lat_i_rad);
    
    // Update transport rate
    wet_t_ << -state_.Velocity[i](1)/(current_Ry_ + state_.Altitude[i]),
              state_.Velocity[i](0)/(current_Rx_ + state_.Altitude[i]),
              state_.Velocity[i](0) * tan(lat_i_rad)/(current_Rx_ + state_.Altitude[i]);
    
    // Update velocity and position
    Vector3d V_new;
    double Lat_new, Lon_new, h_new, Rx_new, Ry_new;
    
    strapdownVelocityPositionUpdate(
        last_f_INSt_, 
        state_.Velocity[i], 
        state_.Latitude[i], 
        state_.Longitude[i], 
        state_.Altitude[i],
        wie_n_, 
        wet_t_, 
        g,
        params_.imu_rate,
        params_.earth_params.Re, 
        params_.earth_params.e,
        current_Rx_, 
        current_Ry_,
        V_new, 
        Lat_new, 
        Lon_new, 
        h_new, 
        Rx_new, 
        Ry_new
    );
    
    // Update navigation state
    state_.Velocity[i+1] = V_new;
    state_.Latitude[i+1] = Lat_new;
    state_.Longitude[i+1] = Lon_new;
    state_.Altitude[i+1] = h_new;
    current_Rx_ = Rx_new;
    current_Ry_ = Ry_new;
    
    // Update total angular rate in body frame
    Vector3d wit_t = wie_n_ + wet_t_;
    wit_b_ = state_.CbtM * wit_t;
    
    // Compute Euler angles from direction cosine matrix
    NavigationUtils::calculateEulerAngles(state_.CbtM, state_.Pitch[i+1], state_.Roll[i+1], state_.Yaw[i+1]);
    
    // Store parameters at different points for EKF prediction
    if (i == 0) {
        // Initial parameters
        parm_old_ = MatrixXd(6, 3);
        parm_old_ << 1.0/params_.gps_rate, state_.Latitude[i+1]*M_PI/180.0, state_.Altitude[i+1],
                     state_.Velocity[i+1](0), state_.Velocity[i+1](1), state_.Velocity[i+1](2),
                     last_f_INSt_.transpose(),
                     state_.CbtM;
        
        parm_mid_ = parm_old_;
        parm_new_ = parm_old_;
 
        // Initialize noise input matrix
        G_mid_ = MatrixXd::Zero(15, 6);
        G_mid_.block<3,3>(0,0) = state_.CtbM;  // Gyro bias mapping
        G_mid_.block<3,3>(3,3) = state_.CtbM;  // Accel bias mapping
    }
    else if (i % 50 == 0 && i % 100 != 0) {
        // Update midpoint parameters
        parm_mid_ = MatrixXd(6, 3);
        parm_mid_ << 1.0/params_.gps_rate, state_.Latitude[i+1]*M_PI/180.0, state_.Altitude[i+1],
                     state_.Velocity[i+1](0), state_.Velocity[i+1](1), state_.Velocity[i+1](2),
                     last_f_INSt_.transpose(),
                     state_.CbtM;
 
        // Update noise input matrix
        G_mid_ = MatrixXd::Zero(15, 6);
        G_mid_.block<3,3>(0,0) = state_.CtbM;
        G_mid_.block<3,3>(3,3) = state_.CtbM;
    }
    
    // Store end-point parameters at measurement steps
    if (isMeasurementStep(i)) {
        parm_new_ = MatrixXd(6, 3);
        parm_new_ << 1.0/params_.gps_rate, state_.Latitude[i+1]*M_PI/180.0, state_.Altitude[i+1],
                     state_.Velocity[i+1](0), state_.Velocity[i+1](1), state_.Velocity[i+1](2),
                     last_f_INSt_.transpose(),
                     state_.CbtM;
    }
    
    // Display progress
    if (i % 1000 == 0) {
        cout << "Processing: " << i << "/" << (state_.Latitude.size() - 1)
             << " (" << fixed << setprecision(1) 
             << (static_cast<double>(i)/(state_.Latitude.size()-1)*100) << "%)" << endl;
    }
}

/**
 * @brief Update attitude using quaternion integration
 */
void EkfNavigation::strapdownAttitudeUpdate(const Vector3d& wtb_b,
                                          const Vector3d& wit_b,
                                          Vector4d& quat,
                                          int IMUrate,
                                          Matrix3d& CbtM) {
    // Compute relative angular rate
    Vector3d delta_w = wtb_b - wit_b;
    double sita0 = delta_w.norm() * (1.0/IMUrate);
    
    // Construct quaternion update matrix
    Matrix4d sita = Matrix4d::Zero();
    sita << 0,          -delta_w(0), -delta_w(1), -delta_w(2),
            delta_w(0),  0,           delta_w(2), -delta_w(1),
            delta_w(1), -delta_w(2),  0,           delta_w(0),
            delta_w(2),  delta_w(1), -delta_w(0),  0;
    sita /= IMUrate;
    
    // Compute rotation quaternion
    Matrix4d I = Matrix4d::Identity();
    Matrix4d rotation = cos(sita0/2) * I + (sin(sita0/2)/sita0 * sita);
    
    // Update quaternion
    quat = rotation * quat;
    quat.normalize();
    
    // Convert quaternion to direction cosine matrix
    double q0 = quat(0), q1 = quat(1), q2 = quat(2), q3 = quat(3);
    CbtM << q0*q0 + q1*q1 - q2*q2 - q3*q3, 2*(q1*q2 + q0*q3),        2*(q1*q3 - q0*q2),
            2*(q1*q2 - q0*q3),        q0*q0 - q1*q1 + q2*q2 - q3*q3, 2*(q2*q3 + q0*q1),
            2*(q1*q3 + q0*q2),        2*(q2*q3 - q0*q1),        q0*q0 - q1*q1 - q2*q2 + q3*q3;
}
 
/**
 * @brief Update velocity and position using mechanization
 */
void EkfNavigation::strapdownVelocityPositionUpdate(const Vector3d& f_INSt,
                                                  const Vector3d& V_prev,
                                                  double Lat_prev,
                                                  double Lon_prev,
                                                  double h_prev,
                                                  const Vector3d& wie_n,
                                                  const Vector3d& wet_t,
                                                  double g,
                                                  int IMUrate,
                                                  double Re,
                                                  double e,
                                                  double Rx_prev,
                                                  double Ry_prev,
                                                  Vector3d& V_new,
                                                  double& Lat_new,
                                                  double& Lon_new,
                                                  double& h_new,
                                                  double& Rx,
                                                  double& Ry) {
    // Compute Coriolis acceleration
    Vector3d coriolis = 2 * wie_n + wet_t;
    Vector3d a = f_INSt - coriolis.cross(V_prev);
    a += Vector3d(0, 0, -g);  // Add gravity
    
    // Update velocity using trapezoidal integration
    V_new = V_prev + (1.0/IMUrate) * a;
    
    // Update position
    double deltaT = 1.0/IMUrate;
    double Ry_h = Ry_prev + h_prev;
    double Rx_h = Rx_prev + h_prev;
    double cosLat = cos(Lat_prev * M_PI/180.0);
    
    // Latitude update (deg)
    Lat_new = Lat_prev + deltaT * (V_prev(1) + V_new(1)) * 180.0 / (2 * Ry_h * M_PI);
    
    // Longitude update (deg)
    Lon_new = Lon_prev + deltaT * (V_prev(0) + V_new(0)) * 180.0 / (2 * Rx_h * cosLat * M_PI);
    
    // Altitude update (m)
    h_new = h_prev + deltaT * (V_prev(2) + V_new(2)) / 2.0;
    
    // Update Earth curvature radii
    double sinLat = sin(Lat_new * M_PI/180.0);
    Rx = Re / (1 - e * sinLat * sinLat);  // Meridian radius
    Ry = Re / (1 + 2*e - 3*e * sinLat * sinLat);  // Transverse radius
}
 
/**
 * @brief Compute gravity magnitude
 * 
 * @return Gravity magnitude (m/s²)
 */
double EkfNavigation::computeGravity(double Latitude, double h, const NavigationParams& params) {
    double sinLat = sin(Latitude * M_PI/180.0);
    double sin2Lat = sinLat * sinLat;
    double denominator = sqrt(1 - params.gk2 * sin2Lat);
    double factor = (1 - 2*h/params.Re) / denominator;
    return params.g0 * (1 + params.gk1 * sin2Lat) * factor;
}
 
// ================== State Prediction ==================
/**
 * @brief Predict navigation state using EKF
 */
void EkfNavigation::predictState(int i) {
    if (isMeasurementStep(i)) {
        // Compute Jacobian matrix for continuous-time model
        computeJacobianMatrix(i);
        
        // Discretize continuous-time system
        discretizeSystem(ekf_.T);
        
        // Execute EKF prediction step
        runEkfPrediction(i);
    }
}
 
/**
 * @brief Compute Jacobian matrix
 */
void EkfNavigation::computeJacobianMatrix(int i) {
    // Prepare parameters for Jacobian calculation
    MatrixXd parm(6, 3);
    parm << ekf_.T, state_.Latitude[i+1]*M_PI/180.0, state_.Altitude[i+1],
            state_.Velocity[i+1](0), state_.Velocity[i+1](1), state_.Velocity[i+1](2),
            last_f_INSt_.transpose(),
            state_.CbtM;
    
    // Compute Jacobian at midpoint
    ekf_.A = computeJacobian(ekf_.X.col(ekf_.N_kalman-1), parm_mid_);
}
 
/**
 * @brief Discretize continuous-time system
 */
void EkfNavigation::discretizeSystem(double dt) {
    // Discretize continuous-time model
    ltiDiscretize(ekf_.A, G_mid_, ekf_.Q, dt, ekf_.disA, ekf_.disQ);
}

/**
 * @brief Discretize linear time-invariant system
 */
void EkfNavigation::ltiDiscretize(const Eigen::MatrixXd& F, 
                                 const Eigen::MatrixXd& G, 
                                 const Eigen::MatrixXd& Qc, 
                                 double dt,
                                 Eigen::MatrixXd& A, 
                                 Eigen::MatrixXd& Q) {
    // Discretize state transition matrix using matrix exponential
    int n = F.rows();
    MatrixXd I = MatrixXd::Identity(n, n);
    A = (F * dt).exp();
    
    // Discretize process noise using Van Loan's method
    MatrixXd Phi = MatrixXd::Zero(2*n, 2*n);
    Phi.block(0,0,n,n) = F;
    Phi.block(0,n,n,n) = G * Qc * G.transpose();
    Phi.block(n,n,n,n) = -F.transpose();
    
    MatrixXd G0(2*n, n);
    G0 << MatrixXd::Zero(n, n), MatrixXd::Identity(n, n);
 
    MatrixXd AB = (Phi * dt).exp() * G0;
 
    Q = AB.block(0,0,n,n) * AB.block(n,0,n,n).inverse();
}
 
/**
 * @brief Execute EKF prediction step
 */
void EkfNavigation::runEkfPrediction(int i) {
    int k = ekf_.N_kalman;
    
    // Combine parameters for RK4 integration
    MatrixXd Uparm(6, 9);
    Uparm << parm_old_, parm_mid_, parm_new_;
    
    // State prediction using Runge-Kutta 4
    ekf_.X_pred = rungeKutta4(ekf_.X.col(k-1), Uparm, ekf_.T);
    
    // Covariance prediction
    ekf_.P_pred = ekf_.disA * ekf_.P * ekf_.disA.transpose() + ekf_.disQ;
}

/**
 * @brief Perform Runge-Kutta 4 integration
 * 
 * @return Predicted state
 */
Eigen::VectorXd EkfNavigation::rungeKutta4(const Eigen::VectorXd& X, 
                                          const Eigen::MatrixXd& Uparm, 
                                          double dt) {
    // Extract parameters for RK4 stages
    MatrixXd parm_old = Uparm.block<6,3>(0,0);
    MatrixXd parm_mid = Uparm.block<6,3>(0,3);
    MatrixXd parm_new = Uparm.block<6,3>(0,6);
    
    // RK4 stages
    VectorXd k1 = computeStateDerivative(X, parm_old);
    VectorXd k2 = computeStateDerivative(X + 0.5 * dt * k1, parm_mid);
    VectorXd k3 = computeStateDerivative(X + 0.5 * dt * k2, parm_mid);
    VectorXd k4 = computeStateDerivative(X + dt * k3, parm_new);
    
    return X + dt / 6.0 * (k1 + 2*k2 + 2*k3 + k4);
}

// ================== Measurement Update ==================
/**
 * @brief Update navigation state using GPS measurement
 */
void EkfNavigation::updateMeasurement(const GPSData& gps, int i) {
    int k = ekf_.N_kalman;
    int gps_index = static_cast<int>(round(static_cast<double>(i + 1) / measurement_interval_) - 1);
    if (gps_index >= gps.time.size()) gps_index = gps.time.size() - 1;
    
    // Measurement residual (innovation)
    VectorXd Z(6);
    Z << state_.Velocity[i+1](0) - gps.vx[gps_index],
         state_.Velocity[i+1](1) - gps.vy[gps_index],
         state_.Velocity[i+1](2) - gps.vz[gps_index],
         // Convert position differences from degrees to meters:
         (state_.Latitude[i+1] - gps.lat[gps_index]) * M_PI * (current_Ry_ + state_.Altitude[i+1]) / 180.0,
         (state_.Longitude[i+1] - gps.lon[gps_index]) * M_PI * (current_Rx_ + state_.Altitude[i+1]) * 
             cos(state_.Latitude[i+1] * M_PI/180.0) / 180.0,
         state_.Altitude[i+1] - gps.alt[gps_index];
    
    // Build measurement matrix
    MatrixXd H = buildMeasurementMatrix(state_.Roll[i+1], state_.Pitch[i+1], state_.Yaw[i+1],
                                       state_.Latitude[i+1], state_.Altitude[i+1], 
                                       current_Rx_, current_Ry_);
    
    // Execute EKF update
    runEkfUpdate(i, Z, H);
}
 
/**
 * @brief Execute EKF update step
 */
void EkfNavigation::runEkfUpdate(int i, const Eigen::VectorXd& Z, const Eigen::MatrixXd& H) {
    int k = ekf_.N_kalman;
    
    // Compute innovation covariance
    MatrixXd S = H * ekf_.P_pred * H.transpose() + ekf_.R;
    
    // Compute Kalman gain
    MatrixXd K = ekf_.P_pred * H.transpose() * S.inverse();
    
    // State update
    VectorXd X_new = ekf_.X_pred + K * (Z - H * ekf_.X_pred);
    
    // Covariance update (Joseph form for numerical stability)
    MatrixXd I = MatrixXd::Identity(15, 15);
    MatrixXd I_KH = I - K * H;
    MatrixXd P_new = I_KH * ekf_.P_pred * I_KH.transpose() + K * ekf_.R * K.transpose();
    
    // Save updated state and covariance
    ekf_.Xsave.col(k) = X_new;
    ekf_.P = P_new;
    ekf_.X.col(k) = X_new;
    
    // Record covariance diagonal for analysis
    for (int j = 0; j < 15; j++) {
        ekf_.P_mean_square(j, k) = sqrt(P_new(j, j));
    }
    
    // Reset error states for next cycle
    ekf_.X.block(0,k,9,1).setZero();
    
    // Increment Kalman index
    ekf_.N_kalman++;
}
 
// ================== Error Correction ==================
/**
 * @brief Correct navigation state errors
 */
void EkfNavigation::correctErrors(int i) {
    int k = ekf_.N_kalman - 1;
    VectorXd X_corr = ekf_.Xsave.col(k);  // Correction vector
    
    // Correct position and velocity
    state_.Latitude[i+1] -= X_corr(6) * (180.0/M_PI);   // Latitude correction (deg)
    state_.Longitude[i+1] -= X_corr(7) * (180.0/M_PI);  // Longitude correction (deg)
    state_.Altitude[i+1] -= X_corr(8);                  // Altitude correction (m)
    state_.Velocity[i+1] -= X_corr.segment(3, 3);       // Velocity correction (m/s)
    
    // Correct attitude using misalignment angles
    double E_err = X_corr(0) * 180 / M_PI;  // East misalignment (deg)
    double N_err = X_corr(1) * 180 / M_PI;  // North misalignment (deg)
    double U_err = X_corr(2) * 180 / M_PI;  // Up misalignment (deg)
    
    // Compute correction direction cosine matrix
    Matrix3d C_errT = NavigationUtils::bodyToNavigationDCM(E_err, N_err, U_err);
    Matrix3d C_err = C_errT.transpose();
    
    // Apply correction to transformation matrix
    state_.CtbM = C_err * state_.CtbM;
    
    // Orthogonalize using Gram-Schmidt process
    Vector3d col0 = state_.CtbM.col(0);
    Vector3d col1 = state_.CtbM.col(1) - state_.CtbM.col(1).dot(col0) * col0 / col0.squaredNorm();
    Vector3d col2 = state_.CtbM.col(2) 
                - state_.CtbM.col(2).dot(col0) * col0 / col0.squaredNorm()
                - state_.CtbM.col(2).dot(col1) * col1 / col1.squaredNorm();
    state_.CtbM.col(0) = col0.normalized();
    state_.CtbM.col(1) = col1.normalized();
    state_.CtbM.col(2) = col2.normalized();
    state_.CbtM = state_.CtbM.transpose();  // Update inverse transformation
    
    // Recompute Euler angles after correction
    NavigationUtils::calculateEulerAngles(state_.CbtM, state_.Pitch[i+1], state_.Roll[i+1], state_.Yaw[i+1]);
    
    // Update Earth parameters based on corrected position
    double lat_rad = state_.Latitude[i+1] * M_PI/180.0;
    double sinLat = sin(lat_rad);
    current_Rx_ = params_.earth_params.Re / (1 - params_.earth_params.e * sinLat * sinLat);
    current_Ry_ = params_.earth_params.Re / (1 + 2*params_.earth_params.e - 3*params_.earth_params.e * sinLat * sinLat);
    wie_n_ << 0, 
            params_.earth_params.W_ie * cos(lat_rad),
            params_.earth_params.W_ie * sin(lat_rad);
    wet_t_ << -state_.Velocity[i+1](1)/(current_Ry_ + state_.Altitude[i+1]),
             state_.Velocity[i+1](0)/(current_Rx_ + state_.Altitude[i+1]),
             state_.Velocity[i+1](0) * tan(lat_rad)/(current_Rx_ + state_.Altitude[i+1]);
    Vector3d wit_t = wie_n_ + wet_t_;
    wit_b_ = state_.CbtM * wit_t;
    
    // Update quaternion from corrected Euler angles
    state_.Quaternion = NavigationUtils::eulerToQuaternion(state_.Pitch[i+1], state_.Roll[i+1], state_.Yaw[i+1]);
 
    // Update parameters for next prediction
    parm_old_ = parm_new_;
 
    // ========== Record History for RTS Smoother ==========
    RtsSmoother::FilterHistory history_item;
    
    // Save filtered state (posterior)
    history_item.state = ekf_.Xsave.col(k);
    
    // Save predicted state (prior)
    history_item.predicted_state = ekf_.X_pred;
    
    // Save filtered covariance (posterior)
    history_item.covariance = ekf_.P;
    
    // Save predicted covariance (prior)
    history_item.predicted_covariance = ekf_.P_pred;
    
    // Save transition matrix (discretized)
    history_item.transition_matrix = ekf_.disA;
    
    // Save current navigation state snapshot
    history_item.nav_state.Latitude = {state_.Latitude[i+1]};
    history_item.nav_state.Longitude = {state_.Longitude[i+1]};
    history_item.nav_state.Altitude = {state_.Altitude[i+1]};
    history_item.nav_state.Velocity = {state_.Velocity[i+1]};
    history_item.nav_state.Pitch = {state_.Pitch[i+1]};
    history_item.nav_state.Roll = {state_.Roll[i+1]};
    history_item.nav_state.Yaw = {state_.Yaw[i+1]};
    history_item.nav_state.CbtM = state_.CbtM;
    history_item.nav_state.CtbM = state_.CtbM;
    
    // Add to RTS smoother
    rts_smoother_.addHistoryItem(history_item);
}

// ================== Main Run Loop ==================
/**
 * @brief Check if current step is measurement update step
 * 
 * @return true if measurement update should be performed
 */
bool EkfNavigation::isMeasurementStep(int i) const {
    return (i + 1) % measurement_interval_ == 0;
}

/**
 * @brief Execute full navigation processing sequence
 */
void EkfNavigation::run(const IMUData& imu, const GPSData& gps) {
    int NavEnd = imu.index.size() - 1;
    
    for (int i = 0; i < NavEnd; i++) {
        // Strapdown inertial navigation update
        updateStrapdown(imu, i);
        
        // Perform EKF steps at measurement intervals
        if (isMeasurementStep(i)) {
            // State prediction
            predictState(i);
            
            // Measurement update
            updateMeasurement(gps, i);
            
            // Error correction
            correctErrors(i);
        }
    }
}
 
// ================== EKF Specific Algorithms ==================
/**
 * @brief Compute state derivative for continuous-time model
 * 
 * @return State derivative vector
 */
Eigen::VectorXd EkfNavigation::computeStateDerivative(const Eigen::VectorXd& X, 
                                                     const Eigen::MatrixXd& parm) {
    // Extract parameters
    double TGPS = parm(0,0);
    double Lat = parm(0,1);
    double Hei = parm(0,2);
    Vector3d V(parm(1,0), parm(1,1), parm(1,2));
    Vector3d fn(parm(2,0), parm(2,1), parm(2,2));
    Matrix3d Cnb = parm.block<3,3>(3,0);
    
    // Earth parameters
    const double WIEE = 7.2921151467e-5;
    const double Re = 6378135.072;
    const double e = 1.0/298.25;
    double Rmh = Re * (1 - 2 * e + 3 * e * sin(Lat) * sin(Lat)) + Hei;
    double Rnh = Re * (1 + e * sin(Lat) * sin(Lat)) + Hei;
    
    // Extract states
    double Atheta = X(0);  // Pitch error
    double Agama = X(1);   // Roll error
    double Afai = X(2);    // Yaw error
    
    // Linear part of state derivative
    MatrixXd FN = MatrixXd::Zero(9,9);
    FN(0,4) = -1.0 / Rmh;
    FN(1,3) = 1.0 / Rnh;
    FN(2,3) = 1.0 / Rnh * tan(Lat);
    FN(1,6) = -WIEE * sin(Lat);
    FN(2,6) = WIEE * cos(Lat) + V(0) / Rnh / pow(cos(Lat),2);
    FN(0,8) = V(1) / pow(Rmh,2);
    FN(1,8) = -V(0) / pow(Rnh,2);
    FN(2,8) = -V(0) * tan(Lat) / pow(Rnh,2);
    FN(3,3) = 1.0 / Rnh * (V(1) * tan(Lat) - V(2));
    FN(3,4) = 2.0 * WIEE * sin(Lat) + V(0) / Rnh * tan(Lat);
    FN(3,5) = -2.0 * WIEE * cos(Lat) - V(0) / Rnh;
    FN(4,3) = -2.0 * WIEE * sin(Lat) - 2.0 * V(0) / Rnh * tan(Lat);
    FN(4,4) = -V(2) / Rmh;
    FN(4,5) = -V(1) / Rmh;
    FN(5,3) = 2.0 * WIEE * cos(Lat) + 2.0 * V(0) / Rnh;
    FN(5,4) = 2.0 * V(1) / Rmh;
    FN(3,6) = 2.0 * WIEE * cos(Lat) * V(1) + 2.0 * WIEE * sin(Lat) * V(2) + 
               V(0) * V(1) / Rnh / pow(cos(Lat),2);
    FN(4,6) = -V(0) * (2.0 * WIEE * cos(Lat) + V(0) / Rnh / pow(cos(Lat),2));
    FN(5,6) = -2.0 * WIEE * sin(Lat) * V(0);
    FN(3,8) = (V(0) * V(2) - V(0) * V(1) * tan(Lat)) / pow(Rnh,2);
    FN(4,8) = (V(1) * V(2) + pow(V(0),2) * tan(Lat)) / pow(Rnh,2);
    FN(5,8) = -(pow(V(0),2) + pow(V(1),2)) / pow(Rnh,2);
    FN(6,4) = 1.0 / Rmh;
    FN(7,3) = 1.0 / (Rnh * cos(Lat));
    FN(7,6) = tan(Lat) / (Rnh * cos(Lat)) * V(0);
    FN(6,8) = -V(1) / pow(Rmh,2);
    FN(7,8) = -V(0) / (pow(Rnh,2) * cos(Lat));
    FN(8,5) = 1;
    
    // Sensor bias mapping
    MatrixXd FS(9,6);
    FS << Cnb.transpose(), MatrixXd::Zero(3,3),
          MatrixXd::Zero(3,3), Cnb.transpose(),
          MatrixXd::Zero(3,3), MatrixXd::Zero(3,3);
    
    // Assemble linear state derivative
    MatrixXd F_linear = MatrixXd::Zero(15,15);
    F_linear.block<9,9>(0,0) = FN;
    F_linear.block<9,6>(0,9) = FS;
    
    // Nonlinear part of state derivative
    VectorXd fx_nonlinear = VectorXd::Zero(15);
    Matrix3d Cnp;
    Cnp << cos(Agama)*cos(Afai)-sin(Agama)*sin(Atheta)*sin(Afai), 
           cos(Agama)*sin(Afai)+sin(Agama)*sin(Atheta)*cos(Afai), 
           -sin(Agama)*cos(Atheta),
           -cos(Atheta)*sin(Afai), 
           cos(Atheta)*cos(Afai), 
           sin(Atheta),
           sin(Agama)*cos(Afai)+cos(Agama)*sin(Atheta)*sin(Afai), 
           sin(Agama)*sin(Afai)-cos(Agama)*sin(Atheta)*cos(Afai), 
           cos(Agama)*cos(Atheta);
    
    Vector3d Wien(0, WIEE * cos(Lat), WIEE * sin(Lat));
    Vector3d Wenn(-V(1) / Rmh, V(0) / Rnh, V(0) * tan(Lat) / Rnh);
    Vector3d Winn = Wien + Wenn;
    
    fx_nonlinear.segment<3>(0) = (Matrix3d::Identity() - Cnp) * Winn;
    fx_nonlinear.segment<3>(3) = (Matrix3d::Identity() - Cnp.transpose()) * fn;
    
    return F_linear * X + fx_nonlinear;
}

/*
Eigen::MatrixXd EkfNavigation::computeJacobian(const Eigen::VectorXd& X,
                                              const Eigen::MatrixXd& parm) {
    // Numerical Jacobian computation
    const double epsilon = 1e-6;
    MatrixXd jac = MatrixXd::Zero(15, 15);
    VectorXd X_pert = X;
    
    for (int i = 0; i < 15; i++) {
        // Perturb state
        double orig = X_pert(i);
        X_pert(i) = orig + epsilon;
        VectorXd f_plus = computeStateDerivative(X_pert, parm);
        
        X_pert(i) = orig - epsilon;
        VectorXd f_minus = computeStateDerivative(X_pert, parm);
        
        // Central difference
        jac.col(i) = (f_plus - f_minus) / (2 * epsilon);
        
        // Restore state
        X_pert(i) = orig;
    }
    
    return jac;
}
*/

/**
 * @brief Compute Jacobian matrix numerically
 * 
 * @return Jacobian matrix
 */
Eigen::MatrixXd EkfNavigation::computeJacobian(const Eigen::VectorXd& X,
                                              const Eigen::MatrixXd& parm) {
    // Extract parameters
    double TGPS = parm(0,0);
    double Lat = parm(0,1);
    double Hei = parm(0,2);
    double Vx = parm(1,0);
    double Vy = parm(1,1);
    double Vz = parm(1,2);
    Vector3d fn(parm(2,0), parm(2,1), parm(2,2));
    Matrix3d Cnb = parm.block<3,3>(3,0);
 
    // Earth parameters
    const double WIEE = 7.2921151467e-5;
    const double Re = 6378135.072;
    const double e = 1.0/298.25;
    
    // Compute curvature radii
    double sinLat = sin(Lat);
    double sin2Lat = sinLat * sinLat;
    double Rmh = Re * (1 - 2 * e + 3 * e * sin2Lat) + Hei;
    double Rnh = Re * (1 + e * sin2Lat) + Hei;
    
    // Extract state variables
    double Atheta = X(0);  // Pitch error
    double Agama = X(1);   // Roll error
    double Afai = X(2);    // Yaw error
    
    // Linear part of Jacobian
    MatrixXd FN = MatrixXd::Zero(9, 9);
    
    FN(0,4) = -1.0 / Rmh;
    FN(1,3) = 1.0 / Rnh;
    FN(2,3) = 1.0 / Rnh * tan(Lat);
    
    FN(1,6) = -WIEE * sinLat;
    FN(2,6) = WIEE * cos(Lat) + Vx / Rnh / pow(cos(Lat), 2);
    FN(0,8) = Vy / pow(Rmh, 2);
    FN(1,8) = -Vx / pow(Rnh, 2);
    FN(2,8) = -Vx * tan(Lat) / pow(Rnh, 2);
    
    FN(3,3) = 1.0 / Rnh * (Vy * tan(Lat) - Vz);
    FN(3,4) = 2.0 * WIEE * sinLat + Vx / Rnh * tan(Lat);
    FN(3,5) = -2.0 * WIEE * cos(Lat) - Vx / Rnh;
    FN(4,3) = -2.0 * WIEE * sinLat - 2.0 * Vx / Rnh * tan(Lat);
    FN(4,4) = -Vz / Rmh;
    FN(4,5) = -Vy / Rmh;
    FN(5,3) = 2.0 * WIEE * cos(Lat) + 2.0 * Vx / Rnh;
    FN(5,4) = 2.0 * Vy / Rmh;
    
    FN(3,6) = 2.0 * WIEE * cos(Lat) * Vy + 2.0 * WIEE * sinLat * Vz + 
               Vx * Vy / Rnh / pow(cos(Lat), 2);
    FN(4,6) = -Vx * (2.0 * WIEE * cos(Lat) + Vx / Rnh / pow(cos(Lat), 2));
    FN(5,6) = -2.0 * WIEE * sinLat * Vx;
    FN(3,8) = (Vx * Vz - Vx * Vy * tan(Lat)) / pow(Rnh, 2);
    FN(4,8) = (Vy * Vz + pow(Vx, 2) * tan(Lat)) / pow(Rnh, 2);
    FN(5,8) = -(pow(Vx, 2) + pow(Vy, 2)) / pow(Rnh, 2);
    
    FN(6,4) = 1.0 / Rmh;
    FN(7,3) = 1.0 / (Rnh * cos(Lat));
    
    FN(7,6) = tan(Lat) / (Rnh * cos(Lat)) * Vx;
    FN(6,8) = -Vy / pow(Rmh, 2);
    FN(7,8) = -Vx / (pow(Rnh, 2) * cos(Lat));
    FN(8,5) = 1;
    
    // Sensor bias mapping
    MatrixXd FS(9, 6);
    FS << Cnb.transpose(), MatrixXd::Zero(3, 3),
          MatrixXd::Zero(3, 3), Cnb.transpose(),
          MatrixXd::Zero(3, 3), MatrixXd::Zero(3, 3);
    
    // Assemble linear Jacobian
    MatrixXd F_linear = MatrixXd::Zero(15, 15);
    F_linear.block<9, 9>(0, 0) = FN;
    F_linear.block<9, 6>(0, 9) = FS;
    
    // Nonlinear part of Jacobian
    MatrixXd Ajac = MatrixXd::Zero(15, 15);
    
    Vector3d Wien(0, WIEE * cos(Lat), WIEE * sin(Lat));
    Vector3d Wenn(-Vy / Rmh, Vx / Rnh, Vx * tan(Lat) / Rnh);
    Vector3d Winn = Wien + Wenn;
    
    double cAtheta = cos(Atheta);
    double sAtheta = sin(Atheta);
    double cAgama = cos(Agama);
    double sAgama = sin(Agama);
    double cAfai = cos(Afai);
    double sAfai = sin(Afai);
    
    Matrix3d Cnp;
    Cnp << cAgama*cAfai - sAgama*sAtheta*sAfai, 
           cAgama*sAfai + sAgama*sAtheta*cAfai,
           -sAgama*cAtheta,
           -cAtheta*sAfai,
           cAtheta*cAfai,
           sAtheta,
           sAgama*cAfai + cAgama*sAtheta*sAfai,
           sAgama*sAfai - cAgama*sAtheta*cAfai,
           cAgama*cAtheta;
    

    Vector3d row1(-sAgama*cAtheta*sAfai, sAgama*cAtheta*cAfai, -sAgama*sAtheta);
    Vector3d row2(sAgama*cAfai - cAgama*sAtheta*sAfai, 
                 sAgama*sAfai + cAgama*sAtheta*cAfai,
                 -cAgama*cAtheta);
    Vector3d row3(cAgama*sAfai - sAgama*sAtheta*cAfai,
                 cAgama*cAfai + sAgama*sAtheta*sAfai,
                 0);
    
    Ajac(0,0) = row1.dot(Winn);
    Ajac(0,1) = row2.dot(Winn);
    Ajac(0,2) = row3.dot(Winn);
    
    Vector3d row4(-sAtheta*sAfai, sAtheta*cAfai, cAtheta);
    Vector3d row5(0, 0, 0);
    Vector3d row6(-cAtheta*cAfai, cAtheta*sAfai, 0);
    
    Ajac(1,0) = row4.dot(Winn);
    Ajac(1,1) = row5.dot(Winn);
    Ajac(1,2) = row6.dot(Winn);
    
    Vector3d row7(cAgama*cAtheta*sAfai, -cAgama*cAtheta*cAfai, cAgama*sAtheta);
    Vector3d row8(cAgama*cAfai + sAgama*sAtheta*sAfai,
                 cAgama*sAfai - sAgama*sAtheta*cAfai,
                 sAgama*cAtheta);
    Vector3d row9(sAgama*sAfai + cAgama*sAtheta*cAfai,
                 sAgama*cAfai - cAgama*sAtheta*sAfai,
                 0);
    
    Ajac(2,0) = row7.dot(Winn);
    Ajac(2,1) = row8.dot(Winn);
    Ajac(2,2) = row9.dot(Winn);
    
    Vector3d col1(-sAgama*cAtheta*sAfai, -sAtheta*sAfai, cAgama*cAtheta*sAfai);
    Vector3d col2(sAgama*cAfai - cAgama*sAtheta*sAfai, 0, cAgama*cAfai + sAgama*sAtheta*sAfai);
    Vector3d col3(cAgama*sAfai - sAgama*sAtheta*cAfai, -cAtheta*cAfai, sAgama*sAfai + cAgama*sAtheta*cAfai);
    
    Ajac(3,0) = col1.dot(fn);
    Ajac(3,1) = col2.dot(fn);
    Ajac(3,2) = col3.dot(fn);
    
    Vector3d col4(sAgama*cAtheta*cAfai, sAtheta*cAfai, -cAgama*cAtheta*cAfai);
    Vector3d col5(sAgama*sAfai + cAgama*sAtheta*cAfai, 0, cAgama*sAfai - sAgama*sAtheta*cAfai);
    Vector3d col6(cAgama*cAfai + sAgama*sAtheta*sAfai, cAtheta*sAfai, sAgama*cAfai - cAgama*sAtheta*sAfai);
    
    Ajac(4,0) = col4.dot(fn);
    Ajac(4,1) = col5.dot(fn);
    Ajac(4,2) = col6.dot(fn);
    
    Vector3d col7(-sAgama*sAtheta, cAtheta, cAgama*sAtheta);
    Vector3d col8(-cAgama*cAtheta, 0, sAgama*cAtheta);
    Vector3d col9(0, 0, 0);
    
    Ajac(5,0) = col7.dot(fn);
    Ajac(5,1) = col8.dot(fn);
    Ajac(5,2) = col9.dot(fn);
    
    // Combine linear and nonlinear parts
    MatrixXd result = -Ajac + F_linear;
    
    return result;
}
 
/**
 * @brief Build measurement matrix
 * 
 * @return Measurement matrix
 */
Eigen::MatrixXd EkfNavigation::buildMeasurementMatrix(double roll, double pitch, double yaw,
                                                     double lat, double h, double Rx, double Ry) {
    // Initialize measurement matrix (6x15)
    MatrixXd H = MatrixXd::Zero(6, 15);
 
    // Convert Euler angles to radians
    double roll_rad = roll * M_PI / 180.0;
    double pitch_rad = pitch * M_PI / 180.0;
    double yaw_rad = yaw * M_PI / 180.0;
 
    // Precompute trigonometric values
    double sin_roll = sin(roll_rad);
    double cos_roll = cos(roll_rad);
    double sin_pitch = sin(pitch_rad);
    double cos_pitch = cos(pitch_rad);
    double sin_yaw = sin(yaw_rad);
    double cos_yaw = cos(yaw_rad);
 
    // Compute velocity error mapping matrix
    MatrixXd H_m2(3,3);
    double a = cos_roll * cos_yaw - sin_roll * sin_pitch * sin_yaw;
    double b = cos_roll * cos_yaw + sin_roll * sin_pitch * cos_yaw;
    double c = -cos_pitch * sin_yaw;
    double d = sin_roll * cos_yaw - cos_roll * sin_pitch * sin_yaw;
 
    H_m2 << 
        1 + a * a,             a * b,              0,
        c * a,                1 + c * a,           0,
        d * a,                d * b,              1;
 
    // Compute position error mapping matrix
    MatrixXd H_m1(3,3);
    H_m1 << 
        Ry + h, 0, 0,
        0, (Rx + h) * cos(lat * M_PI / 180.0), 0,
        0, 0, 1;
 
    // Assemble measurement matrix
    H.block<3,3>(0,3) = H_m2;  // Velocity error mapping
    H.block<3,3>(3,6) = H_m1;  // Position error mapping
    
    return H;
}
