/**
 * @file KfNavigation.cpp
 * @brief Implementation of Kalman filter-based INS/GPS navigation system
 *
 * Contains the core implementation of the Kalman filter navigation algorithm,
 * including strapdown inertial navigation, Kalman prediction/update steps,
 * error correction, and RTS smoothing. Integrates IMU and GPS measurements
 * to estimate position, velocity, and attitude in the navigation frame.
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
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

using namespace Eigen;
using namespace std;

// ================== Initialization ==================
/**
 * @brief Initialize navigation system with parameters and starting state
 * 
 * Configures Earth parameters, Kalman filter settings, and initializes state vectors
 */
void KalmanFilterNavigation::initialize(const NavParamsBase& base_params, 
                                       NavigationState& state) {
    // Convert to KF specific parameters
    const KfParams& params = dynamic_cast<const KfParams&>(base_params);
    params_ = params;
    kalman_ = params.kalman_params;
    state_ = state;
    
    // Calculate measurement update interval (IMU cycles per GPS update)
    measurement_interval_ = params_.imu_rate / params_.gps_rate;
    
    // Initialize Earth parameters based on starting position
    double lat0 = state_.Latitude[0];
    wie_n_ << 0, 
             params_.earth_params.W_ie * cos(lat0 * M_PI/180.0),
             params_.earth_params.W_ie * sin(lat0 * M_PI/180.0);
    
    // Compute transport rate in navigation frame
    wet_t_ << -state_.Velocity[0](1)/params_.earth_params.Ry, 
             state_.Velocity[0](0)/params_.earth_params.Rx, 
             state_.Velocity[0](0)/params_.earth_params.Rx * tan(lat0 * M_PI/180.0);
    
    // Total angular rate in navigation frame and convert to body frame
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
}

// ================== Strapdown Inertial Navigation Update ==================
/**
 * @brief Perform strapdown inertial navigation update
 * 
 * Processes IMU data to update attitude, velocity and position in navigation frame
 */
void KalmanFilterNavigation::updateStrapdown(const IMUData& imu, int i) {
    // Convert gyro data from deg/h to rad/s and update attitude
    Vector3d wtb_b(imu.gx[i], imu.gy[i], imu.gz[i]);
    wtb_b = wtb_b * (M_PI/3600.0/180.0);  // Convert deg/h to rad/s
    updateAttitude(wtb_b);
    
    // Convert accelerometer data to navigation frame
    Vector3d f_b(imu.ax[i], imu.ay[i], imu.az[i]);
    last_f_INSt_ = state_.CtbM * f_b;  // Save for prediction step
    
    // Compute gravity at current position
    double g = computeGravity(state_.Latitude[i], state_.Altitude[i], params_.earth_params);
    
    // Update Earth angular rates based on current position
    double lat_i = state_.Latitude[i];
    wie_n_ << 0, 
             params_.earth_params.W_ie * cos(lat_i * M_PI/180.0),
             params_.earth_params.W_ie * sin(lat_i * M_PI/180.0);
    
    // Update transport rate
    wet_t_ << -state_.Velocity[i](1)/(current_Ry_ + state_.Altitude[i]),
              state_.Velocity[i](0)/(current_Rx_ + state_.Altitude[i]),
              state_.Velocity[i](0) * tan(lat_i * M_PI/180.0)/(current_Rx_ + state_.Altitude[i]);
    
    // Update velocity and position
    updateVelocityPosition(last_f_INSt_);
    
    // Update total angular rate in body frame
    Vector3d wit_t = wie_n_ + wet_t_;
    wit_b_ = state_.CbtM * wit_t;
    
    // Compute Euler angles from direction cosine matrix
    NavigationUtils::calculateEulerAngles(state_.CbtM, state_.Pitch[i+1], state_.Roll[i+1], state_.Yaw[i+1]);
    
    // Display progress every 1000 steps
    if (i % 1000 == 0) {
        cout << "Processing: " << i << "/" << (state_.Latitude.size() - 1)
             << " (" << fixed << setprecision(1) 
             << (static_cast<double>(i)/(state_.Latitude.size()-1)*100) << "%)" << endl;
    }
}

/**
 * @brief Update attitude using angular rate measurements
 */
void KalmanFilterNavigation::updateAttitude(const Vector3d& wtb_b) {
    strapdownAttitudeUpdate(wtb_b, wit_b_, state_.Quaternion, params_.imu_rate, state_.CbtM);
    state_.CtbM = state_.CbtM.transpose();
}

/**
 * @brief Update velocity and position using mechanization equations
 */
void KalmanFilterNavigation::updateVelocityPosition(const Vector3d& f_INSt) {
    int i = current_index_;
    Vector3d V_new;
    double Lat_new, Lon_new, h_new, Rx_new, Ry_new;
    
    // Perform velocity and position update
    strapdownVelocityPositionUpdate(
        f_INSt, 
        state_.Velocity[i], 
        state_.Latitude[i], 
        state_.Longitude[i], 
        state_.Altitude[i],
        wie_n_, 
        wet_t_, 
        computeGravity(state_.Latitude[i], state_.Altitude[i], params_.earth_params),
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
    
    // Update navigation state with new values
    state_.Velocity[i+1] = V_new;
    state_.Latitude[i+1] = Lat_new;
    state_.Longitude[i+1] = Lon_new;
    state_.Altitude[i+1] = h_new;
    current_Rx_ = Rx_new;
    current_Ry_ = Ry_new;
}

// ================== State Prediction ==================
/**
 * @brief Predict navigation state using Kalman filter
 */void KalmanFilterNavigation::predictState(int i) {
    // Only perform prediction at measurement steps
    if (isMeasurementStep(i)) {
        // Compute state transition matrix
        computeStateTransitionMatrix(i);
        
        // Execute Kalman prediction step
        runKalmanPrediction(i);
    }
}

/**
 * @brief Compute state transition matrix for Kalman filter
 */
void KalmanFilterNavigation::computeStateTransitionMatrix(int i) {
    kalman_.A = kalmanComputeStateMatrix(
        state_.Latitude[i+1], 
        state_.Velocity[i+1], 
        state_.Altitude[i+1], 
        current_Rx_, 
        current_Ry_, 
        state_.CtbM, 
        params_.earth_params.W_ie, 
        last_f_INSt_
    );
}

/**
 * @brief Execute Kalman prediction step
 */
void KalmanFilterNavigation::runKalmanPrediction(int i) {
    int k = kalman_index_;
    
    // Compute control matrix mapping sensor biases to state errors
    MatrixXd B = MatrixXd::Zero(15, 6);
    B.block<3,3>(0,0) = state_.CtbM;  // Gyro bias to attitude error
    B.block<3,3>(3,3) = state_.CtbM;  // Accel bias to velocity error
    
    // Perform Kalman prediction step
    VectorXd X_pred;
    MatrixXd P_pred;
    kalmanPredictStep(
        kalman_.X.col(k-1), 
        kalman_.P,
        kalman_.A, 
        B, 
        kalman_.Q, 
        kalman_.T,
        X_pred, 
        P_pred
    );
    
    // Save predicted state and covariance
    kalman_.X_pred = X_pred;
    kalman_.P_pred = P_pred;
}

// ================== Measurement Update ==================
/**
 * @brief Update navigation state using GPS measurement
 */
void KalmanFilterNavigation::updateMeasurement(const GPSData& gps, int i) {
    // Compute measurement matrix
    computeMeasurementMatrix(i);
    
    // Compute measurement residual (innovation)
    int k = kalman_index_;
    int gps_index = static_cast<int>(round(static_cast<double>(i + 1) / measurement_interval_) - 1);
    if (gps_index >= gps.time.size()) gps_index = gps.time.size() - 1;
    
    // Construct measurement vector (velocity and position differences)
    VectorXd Z(6);
    Z << state_.Velocity[i+1](0) - gps.vx[gps_index],
         state_.Velocity[i+1](1) - gps.vy[gps_index],
         state_.Velocity[i+1](2) - gps.vz[gps_index],
         (state_.Latitude[i+1] - gps.lat[gps_index]) * M_PI * (current_Ry_ + state_.Altitude[i+1]) / 180.0,
         (state_.Longitude[i+1] - gps.lon[gps_index]) * M_PI * (current_Rx_ + state_.Altitude[i+1]) * 
             cos(state_.Latitude[i+1] * M_PI/180.0) / 180.0,
         state_.Altitude[i+1] - gps.alt[gps_index];
    
    // Execute Kalman update
    runKalmanUpdate(i, Z);
}

/**
 * @brief Compute measurement matrix for Kalman filter
 */
void KalmanFilterNavigation::computeMeasurementMatrix(int i) {
    kalman_.H = kalmanComputeMeasurementMatrix(
        state_.Roll[i+1], 
        state_.Yaw[i+1],
        state_.Pitch[i+1], 
        state_.Latitude[i+1],
        state_.Altitude[i+1], 
        current_Rx_, 
        current_Ry_
    );
}

/**
 * @brief Execute Kalman update step
 */
void KalmanFilterNavigation::runKalmanUpdate(int i, const VectorXd& Z) {
    int k = kalman_index_;
    
    // Perform Kalman update step
    VectorXd X_new;
    MatrixXd P_new;
    kalmanUpdateStep(
        kalman_.X_pred, 
        kalman_.P_pred,
        Z,
        kalman_.H, 
        kalman_.R, 
        X_new, 
        P_new
    );
    
    // Save updated state and covariance
    kalman_.Xsave.col(k) = X_new;
    kalman_.P = P_new; 
    kalman_.X.col(k) = X_new;
    
    // Record covariance diagonal for analysis
    for (int j = 0; j < 15; j++) {
        kalman_.P_mean_square(j, k) = sqrt(P_new(j, j));
    }
    
    // Reset error states for next cycle
    kalman_.X.block(0,k,9,1).setZero();  // Reset first 9 states
    
    // Increment Kalman index for next update
    kalman_index_++;
}

// ================== Error Correction ==================
/**
 * @brief Correct navigation state errors using Kalman estimates
 */
void KalmanFilterNavigation::correctErrors(int i) {
    int k = kalman_index_ - 1;
    VectorXd X_corr = kalman_.Xsave.col(k);  // Correction vector
    
    // Correct position and velocity
    state_.Latitude[i+1] -= X_corr(6) * (180.0/M_PI);   // Latitude correction
    state_.Longitude[i+1] -= X_corr(7) * (180.0/M_PI);  // Longitude correction
    state_.Altitude[i+1] -= X_corr(8);                  // Altitude correction
    state_.Velocity[i+1] -= X_corr.segment(3, 3);       // Velocity correction
    
    // Correct attitude using misalignment angles
    double E_err = X_corr(0) * 180 / M_PI;  // East misalignment (degrees)
    double N_err = X_corr(1) * 180 / M_PI;  // North misalignment (degrees)
    double U_err = X_corr(2) * 180 / M_PI;  // Up misalignment (degrees)
    
    // Compute correction DCM
    Matrix3d C_errT = NavigationUtils::bodyToNavigationDCM(E_err, N_err, U_err);
    Matrix3d C_err = C_errT.transpose();
    
    // Apply correction
    state_.CtbM = C_err * state_.CtbM;

    // Orthogonalize using Gram-Schmidt process
    Vector3d col0 = state_.CtbM.col(0);
    Vector3d col1 = state_.CtbM.col(1) - state_.CtbM.col(1).dot(col0) * col0 / col0.squaredNorm();
    Vector3d col2 = state_.CtbM.col(2) 
                - state_.CtbM.col(2).dot(col0) * col0 / col0.squaredNorm()
                - state_.CtbM.col(2).dot(col1) * col1 / col1.squaredNorm();
    col0.normalize();
    col1.normalize();
    col2.normalize();
    state_.CtbM.col(0) = col0;
    state_.CtbM.col(1) = col1;
    state_.CtbM.col(2) = col2;

    state_.CbtM = state_.CtbM.transpose();
    
    // Recompute Euler angles after correction
    NavigationUtils::calculateEulerAngles(state_.CbtM, state_.Pitch[i+1], state_.Roll[i+1], state_.Yaw[i+1]);
    
    // Update Earth parameters based on corrected position
    double sinLat = sin(state_.Latitude[i+1] * M_PI/180.0);
    current_Rx_ = params_.earth_params.Re / (1 - params_.earth_params.e * sinLat * sinLat);
    current_Ry_ = params_.earth_params.Re / (1 + 2*params_.earth_params.e - 3*params_.earth_params.e * sinLat * sinLat);
    wie_n_ << 0, 
            params_.earth_params.W_ie * cos(state_.Latitude[i+1] * M_PI/180.0),
            params_.earth_params.W_ie * sin(state_.Latitude[i+1] * M_PI/180.0);
    wet_t_ << -state_.Velocity[i+1](1)/(current_Ry_ + state_.Altitude[i+1]),
             state_.Velocity[i+1](0)/(current_Rx_ + state_.Altitude[i+1]),
             state_.Velocity[i+1](0) * tan(state_.Latitude[i+1] * M_PI/180.0)/(current_Rx_ + state_.Altitude[i+1]);
    Vector3d wit_t = wie_n_ + wet_t_;
    wit_b_ = state_.CbtM * wit_t;
    
    // Update quaternion from corrected Euler angles
    state_.Quaternion = NavigationUtils::eulerToQuaternion(state_.Pitch[i+1], state_.Roll[i+1], state_.Yaw[i+1]);

    // ========== Record History for RTS Smoother ==========
    RtsSmoother::FilterHistory history_item;
    
    // Save filtered state (posterior)
    history_item.state = kalman_.Xsave.col(k);
    
    // Save predicted state (prior)
    history_item.predicted_state = kalman_.X_pred;
    
    // Save filtered covariance (posterior)
    history_item.covariance = kalman_.P;
    
    // Save predicted covariance (prior)
    history_item.predicted_covariance = kalman_.P_pred;
    
    // Save transition matrix
    history_item.transition_matrix = kalman_.A;
    
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

// ================== Time Management ==================
/**
 * @brief Advance to next time step
 */
void KalmanFilterNavigation::advance() {
    current_index_++;
}

/**
 * @brief Check if current time step requires measurement update
 * 
 * @return true if measurement update should be performed
 */
bool KalmanFilterNavigation::isMeasurementStep(int i) const {
    return (i + 1) % measurement_interval_ == 0;
}

// ================== Main Run Loop ==================
/**
 * @brief Execute full navigation processing sequence
 */
void KalmanFilterNavigation::run(const IMUData& imu, const GPSData& gps) {
    int NavEnd = imu.index.size() - 1;
    
    for (int i = 0; i < NavEnd; i++) {
        // Strapdown inertial navigation update
        updateStrapdown(imu, i);
        
        // Perform Kalman steps at measurement intervals
        if (isMeasurementStep(i)) {
            // State prediction
            predictState(i);
            
            // Measurement update
            updateMeasurement(gps, i);
            
            // Error correction
            correctErrors(i);
        }
        
        // Advance to next time step
        advance();
    }
}

// ================== Algorithm Implementations ==================
/**
 * @brief Update attitude using quaternion integration
 * 
 * Implements quaternion-based attitude update using angular rates in body frame
 */
void KalmanFilterNavigation::strapdownAttitudeUpdate(const Vector3d& wtb_b,
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
 * @brief Update velocity and position using mechanization equations
 * 
 * Implements velocity and position update in navigation frame
 */
void KalmanFilterNavigation::strapdownVelocityPositionUpdate(const Vector3d& f_INSt,
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
    a += Vector3d(0, 0, -g);
    
    // Update velocity using trapezoidal integration
    V_new = V_prev + (1.0/IMUrate) * a;
    
    // Update position
    double deltaT = 1.0/IMUrate;
    double Ry_h = Ry_prev + h_prev;
    double Rx_h = Rx_prev + h_prev;
    double cosLat = cos(Lat_prev * M_PI/180.0);
    
    Lat_new = Lat_prev + deltaT * (V_prev(1) + V_new(1)) * 180.0 / (2 * Ry_h * M_PI);
    Lon_new = Lon_prev + deltaT * (V_prev(0) + V_new(0)) * 180.0 / (2 * Rx_h * cosLat * M_PI);
    h_new = h_prev + deltaT * (V_prev(2) + V_new(2)) / 2.0;
    
    // Update Earth curvature radii
    double sinLat = sin(Lat_new * M_PI/180.0);
    Rx = Re / (1 - e * sinLat * sinLat);
    Ry = Re / (1 + 2*e - 3*e * sinLat * sinLat);
}

/**
 * @brief Compute gravity magnitude using gravity model
 * 
 * @return Gravity magnitude (m/s²)
 */
double KalmanFilterNavigation::computeGravity(double Latitude, double h, const NavigationParams& params) {
    double sinLat = sin(Latitude * M_PI/180.0);
    double sin2Lat = sinLat * sinLat;
    double denominator = sqrt(1 - params.gk2 * sin2Lat);
    double factor = (1 - 2*h/params.Re) / denominator;
    return params.g0 * (1 + params.gk1 * sin2Lat) * factor;
}

/**
 * @brief Compute Kalman state transition matrix
 * 
 * @return State transition matrix (15x15)
 */
MatrixXd KalmanFilterNavigation::kalmanComputeStateMatrix(double Latitude,
                                                        const Vector3d& V,
                                                        double h,
                                                        double Rx,
                                                        double Ry,
                                                        const Matrix3d& CtbM,
                                                        double W_ie,
                                                        const Vector3d& f_INSt) {
    MatrixXd A = MatrixXd::Zero(15, 15);
    double lat_rad = Latitude * M_PI / 180.0;
    double sin_lat = sin(lat_rad);
    double cos_lat = cos(lat_rad);
    double tan_lat = tan(lat_rad);
    
    MatrixXd A1 = MatrixXd::Zero(9, 9);
    
    // Attitude error equations (F_phi)
    A1(0, 1) = W_ie * sin_lat + V(0) / (Rx + h) * tan_lat;
    A1(0, 2) = -W_ie * cos_lat - V(0) / (Rx + h);
    A1(0, 4) = -1.0 / (Ry + h);
    A1(0, 8) = V(1) / pow(Ry + h, 2);
    
    A1(1, 0) = -W_ie * sin_lat - V(0) / (Rx + h) * tan_lat;
    A1(1, 2) = -V(1) / (Ry + h);
    A1(1, 3) = 1.0 / (Rx + h);
    A1(1, 6) = -W_ie * sin_lat;
    A1(1, 8) = -V(0) / pow(Rx + h, 2);
    
    A1(2, 0) = W_ie * cos_lat + V(0) / (Rx + h);
    A1(2, 1) = V(1) / (Ry + h);
    A1(2, 3) = tan_lat / (Rx + h);
    A1(2, 6) = W_ie * cos_lat + V(0) / (Rx + h) / pow(cos_lat, 2);
    A1(2, 8) = -V(0) * tan_lat / pow(Rx + h, 2);
    
    // Velocity error equations (F_v)
    A1(3, 1) = -f_INSt(2);
    A1(3, 2) = f_INSt(1);
    A1(3, 3) = (V(1) * tan_lat - V(2)) / (Rx + h);
    A1(3, 4) = 2 * W_ie * sin_lat + V(0) * tan_lat / (Rx + h);
    A1(3, 5) = -2 * W_ie * cos_lat - V(0) / (Rx + h);
    A1(3, 6) = 2 * W_ie * cos_lat * V(1) + 2 * W_ie * sin_lat * V(2) + 
               V(0) * V(1) / (Rx + h) / pow(cos_lat, 2);
    A1(3, 8) = (V(0) * V(2) - V(0) * V(1) * tan_lat) / pow(Rx + h, 2);
    
    A1(4, 0) = f_INSt(2);
    A1(4, 2) = -f_INSt(0);
    A1(4, 3) = -2 * W_ie * sin_lat - 2 * V(0) * tan_lat / (Rx + h);
    A1(4, 4) = -V(2) / (Ry + h);
    A1(4, 5) = -V(1) / (Ry + h);
    A1(4, 6) = -V(0) * (2 * W_ie * cos_lat + V(0) / (Rx + h) / pow(cos_lat, 2));
    A1(4, 8) = (V(1) * V(2) + pow(V(0), 2) * tan_lat) / pow(Rx + h, 2);
    
    A1(5, 0) = -f_INSt(1);
    A1(5, 1) = f_INSt(0);
    A1(5, 3) = 2 * W_ie * cos_lat + 2 * V(0) / (Rx + h);
    A1(5, 4) = 2 * V(1) / (Ry + h);
    A1(5, 6) = -2 * W_ie * sin_lat * V(0);
    A1(5, 8) = -(pow(V(0), 2) + pow(V(1), 2)) / pow(Rx + h, 2);
    
    // Position error equations (F_p)
    A1(6, 4) = 1.0 / (Ry + h);
    A1(6, 8) = -V(1) / pow(Ry + h, 2);
    
    A1(7, 3) = 1.0 / ((Rx + h) * cos_lat);
    A1(7, 6) = V(0) * tan_lat / ((Rx + h) * cos_lat);
    A1(7, 8) = -V(0) / (pow(Rx + h, 2) * cos_lat);
    
    A1(8, 5) = 1.0;
    
    // Sensor bias mapping (A2)
    MatrixXd A2 = MatrixXd::Zero(9, 6);
    A2.block<3,3>(0,0) = CtbM;  // Gyro bias to attitude error
    A2.block<3,3>(3,3) = CtbM;  // Accel bias to velocity error
    
    // Assemble state transition matrix
    A.block<9,9>(0,0) = A1;
    A.block<9,6>(0,9) = A2;
    
    return A;
}

/**
 * @brief Compute Kalman measurement matrix
 * 
 * @return Measurement matrix (6x15)
 */
MatrixXd KalmanFilterNavigation::kalmanComputeMeasurementMatrix(double roll,
                                                              double yaw,
                                                              double pitch,
                                                              double Latitude,
                                                              double h,
                                                              double Rx,
                                                              double Ry) {
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

    // Compute H_m2 (velocity error to measurement)
    MatrixXd H_m2(3,3);
    double a = cos_roll * cos_yaw - sin_roll * sin_pitch * sin_yaw;
    double b = cos_roll * cos_yaw + sin_roll * sin_pitch * cos_yaw;
    double c = -cos_pitch * sin_yaw;
    double d = sin_roll * cos_yaw - cos_roll * sin_pitch * sin_yaw;

    H_m2 << 
        1 + a * a,             a * b,              0,
        c * a,                1 + c * a,           0,
        d * a,                d * b,              1;

    // Compute H_m1 (position error to measurement)
    MatrixXd H_m1(3,3);
    H_m1 << 
        Ry + h, 0, 0,
        0, (Rx + h) * cos(Latitude * M_PI / 180.0), 0,
        0, 0, 1;

    // Map velocity errors to measurements (first 3 measurements)
    H.block<3,3>(0,3) = H_m2;
    // Map position errors to measurements (last 3 measurements)
    H.block<3,3>(3,6) = H_m1;
    
    return H;
}

/**
 * @brief Execute Kalman prediction step
 * 
 * Implements discrete-time state prediction using Taylor series approximation
 */
void KalmanFilterNavigation::kalmanPredictStep(const VectorXd& X_prev,
                                             const MatrixXd& P_prev,
                                             const MatrixXd& A,
                                             const MatrixXd& B,
                                             const Eigen::MatrixXd& Q,
                                             double T,
                                             VectorXd& X_pred,
                                             MatrixXd& P_pred) {
    // Compute state transition using Taylor series expansion
    MatrixXd I = MatrixXd::Identity(A.rows(), A.cols());
    MatrixXd one_step_tran = I + A * T + (A * A * T * T) / 2.0 + 
                           (A * A * A * T * T * T) / 6.0 + 
                           (A * A * A * A * T * T * T * T) / 24.0;
    
    // Compute process noise covariance using Taylor series
    MatrixXd qq1 = B * Q * B.transpose() * T;
    MatrixXd sys_noise_drive = qq1;
    for (int kk = 2; kk <= 10; kk++) {
        MatrixXd qq2 = A * qq1;
        qq1 = (qq2 + qq2.transpose()) * T / kk;
        sys_noise_drive += qq1;
    }
    
    // Predict state and covariance
    P_pred = one_step_tran * P_prev * one_step_tran.transpose() + sys_noise_drive;
    X_pred = one_step_tran * X_prev;
}

/**
 * @brief Execute Kalman update step
 * 
 * Implements measurement update using Joseph form for covariance stability
 */
void KalmanFilterNavigation::kalmanUpdateStep(const VectorXd& X_pred,
                                            const MatrixXd& P_pred,
                                            const VectorXd& Z,
                                            const MatrixXd& H,
                                            const MatrixXd& R,
                                            VectorXd& X_new,
                                            MatrixXd& P_new) {
    // Compute innovation covariance
    MatrixXd S = H * P_pred * H.transpose() + R;

    // Compute Kalman gain
    MatrixXd K_filter_gain = P_pred * H.transpose() * S.inverse();
    
    // Update state estimate
    X_new = X_pred + K_filter_gain * (Z - H * X_pred);

    // Update covariance (Joseph form for numerical stability)
    MatrixXd I = MatrixXd::Identity(X_pred.size(), X_pred.size());
    MatrixXd I_KH = I - K_filter_gain * H;
    P_new = I_KH * P_pred * I_KH.transpose() + K_filter_gain * R * K_filter_gain.transpose();
}
