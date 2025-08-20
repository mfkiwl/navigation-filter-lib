/**
 * @file UkfNavigation.cpp
 * @brief Implementation of UKF-based navigation system
 *
 * @author peanut-nav
 * @date Created: 2025-08-10
 * @last Modified: 2025-08-20
 * @version 0.4.0
 */

#include "NavigationFactory.hpp"
#include "MathUtils.hpp"
#include <iostream>
#include <iomanip>
#include <cmath>

using namespace Eigen;
using namespace std;

static inline double deg2rad(double d){ return d * M_PI / 180.0; }
static inline double rad2deg(double r){ return r * 180.0 / M_PI; }

// ================== Initialization ==================
/**
 * @brief Initialize UKF navigation system.
 */
void UkfNavigation::initialize(const NavParamsBase& base_params, 
                              NavigationState& state) {
    // Downcast to UKF-specific parameter struct; throws if mismatched type.
    const UkfParams& params = dynamic_cast<const UkfParams&>(base_params);
    params_ = params;

    // Initialize UKF configuration container (dimensions, Q/R, etc.).
    ukf_ = params.ukf_params;

    // Copy initial navigation state (quat, velocity, LLA, DCMs, etc.).
    state_ = state;
    
    // Number of IMU cycles between consecutive GPS updates (integer cadence).
    measurement_interval_ = params_.imu_rate / params_.gps_rate;
    
    // Earth rotation expressed in navigation frame at initial latitude.
    // State lat is stored in degrees, so convert to radians first.
    double lat0 = state_.Latitude[0] * M_PI / 180.0;
    wie_n_ << 0, 
             params_.earth_params.W_ie * cos(lat0),
             params_.earth_params.W_ie * sin(lat0);
    
    // Compute transport rate in navigation frame
    wet_t_ << -state_.Velocity[0](1)/(params_.earth_params.Ry + state_.Altitude[0]), 
             state_.Velocity[0](0)/(params_.earth_params.Rx + state_.Altitude[0]), 
             state_.Velocity[0](0) * tan(lat0)/(params_.earth_params.Rx + state_.Altitude[0]);
    
    // Total inertial rate in t==n, then map to body for attitude integration.
    Vector3d wit_t = wie_n_ + wet_t_;
    wit_b_ = state_.CbtM * wit_t;  // body←t mapping
    
    // Initialize curvature radii (will be updated as latitude changes).
    current_Rx_ = params_.earth_params.Rx;
    current_Ry_ = params_.earth_params.Ry;

    // ---- UKF weights for the unscented transform ----
    // lambda = α²(n+κ) - n; here provided/derived as 3 - n (per settings).
    ukf_.lambda = 3.0 - ukf_.Nk;
    ukf_.w = VectorXd::Zero(2*ukf_.Nk + 1);
    ukf_.w(0) = ukf_.lambda / (ukf_.lambda + ukf_.Nk);
    for (int i=1;i<ukf_.w.size();++i)
        ukf_.w(i) = 1.0 / (2.0*(ukf_.lambda + ukf_.Nk));

    // ---- Pre-size state trajectories and UKF matrices ----
    int totalPoints = state_.Latitude.size();
    state_.Velocity.resize(totalPoints, Vector3d::Zero());
    state_.Pitch.resize(totalPoints);
    state_.Roll.resize(totalPoints);
    state_.Yaw.resize(totalPoints);
    
    ukf_.Q   = MatrixXd::Zero(15, 15);      // Discrete Qd
    ukf_.X_prop.resize(ukf_.Nk, 2*ukf_.Nk + 1);
    ukf_.X_pred.setZero(ukf_.Nk);
    ukf_.P_pred.setZero(ukf_.Nk, ukf_.Nk);
}

// ================== Strapdown Inertial Navigation Update ==================
/**
 * @brief Perform strapdown inertial navigation update.
 */
void UkfNavigation::updateStrapdown(const IMUData& imu, int i) {
    // Gyro input is in deg/hour; convert to rad/s for integration.
    Vector3d wtb_b(imu.gx[i], imu.gy[i], imu.gz[i]);
    wtb_b = wtb_b * (M_PI/(3600.0*180.0));

    // Attitude mechanization: integrate quaternion using relative angular rate.
    strapdownAttitudeUpdate(wtb_b, wit_b_, state_.Quaternion, params_.imu_rate, state_.CbtM);
    state_.CtbM = state_.CbtM.transpose();  // Inverse mapping (n←b is transpose since DCM is orthonormal)
    
    // Resolve specific force to navigation (t==n) frame using current attitude.
    Vector3d f_b(imu.ax[i], imu.ay[i], imu.az[i]);
    last_f_INSt_ = state_.CtbM * f_b;  // store for UKF prediction
    
    // Local gravity magnitude at current position.
    double g = computeGravity(state_.Latitude[i], state_.Altitude[i], params_.earth_params);
    
    // Update Earth rotation in n-frame at current latitude (state lat in degrees).
    double lat_i_rad = deg2rad(state_.Latitude[i]);
    wie_n_ << 0, 
             params_.earth_params.W_ie * cos(lat_i_rad),
             params_.earth_params.W_ie * sin(lat_i_rad);
    
    // Update transport rate
    wet_t_ << -state_.Velocity[i](1)/(current_Ry_ + state_.Altitude[i]),
              state_.Velocity[i](0)/(current_Rx_ + state_.Altitude[i]),
              state_.Velocity[i](0) * tan(lat_i_rad)/(current_Rx_ + state_.Altitude[i]);
    
    // Integrate velocity and LLA over one IMU interval.
    Vector3d V_new;
    double Lat_new, Lon_new, h_new, Rx_new, Ry_new;
    
    strapdownVelocityPositionUpdate(
        last_f_INSt_, state_.Velocity[i],
        state_.Latitude[i], state_.Longitude[i], state_.Altitude[i],
        wie_n_, wet_t_, g, params_.imu_rate,
        params_.earth_params.Re, params_.earth_params.e,
        current_Rx_, current_Ry_,
        V_new, Lat_new, Lon_new, h_new, Rx_new, Ry_new
    );

    // Commit propagated nav state.
    state_.Velocity[i+1]  = V_new;
    state_.Latitude[i+1]  = Lat_new;
    state_.Longitude[i+1] = Lon_new;
    state_.Altitude[i+1]  = h_new;
    current_Rx_ = Rx_new;
    current_Ry_ = Ry_new;
    
    // Refresh total inertial rate in body frame for the next attitude step.
    Vector3d wit_t = wie_n_ + wet_t_;
    wit_b_ = state_.CbtM * wit_t;
    
    // Compute Euler angles from the updated DCM for logging/diagnostics.
    NavigationUtils::calculateEulerAngles(state_.CbtM, state_.Pitch[i+1], state_.Roll[i+1], state_.Yaw[i+1]);
    
    // Store parameters at different points for UKF prediction
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

    // Lightweight progress indicator for long runs.
    if (i % 1000 == 0) {
        cout << "Processing: " << i << "/" << (state_.Latitude.size() - 1)
             << " (" << fixed << setprecision(1) 
             << (static_cast<double>(i)/(state_.Latitude.size()-1)*100) << "%)" << endl;
    }
}

/**
 * @brief Attitude mechanization via quaternion integration.
 */
void UkfNavigation::strapdownAttitudeUpdate(const Vector3d& wtb_b,
                                          const Vector3d& wit_b,
                                          Vector4d& quat,
                                          int IMUrate,
                                          Matrix3d& CbtM) {
    // Relative angular rate: body-observed minus inertial (Earth + transport mapped to body).
    Vector3d delta_w = wtb_b - wit_b;

    // Rotation angle over one IMU interval.
    double sita0 = delta_w.norm() * (1.0/IMUrate);
    
    // Skew-symmetric 4×4 quaternion-rate operator for small rotation.
    Matrix4d sita = Matrix4d::Zero();
    sita << 0,          -delta_w(0), -delta_w(1), -delta_w(2),
            delta_w(0),  0,           delta_w(2), -delta_w(1),
            delta_w(1), -delta_w(2),  0,           delta_w(0),
            delta_w(2),  delta_w(1), -delta_w(0),  0;
    sita /= IMUrate;  // scale by Δt
    
    // Closed-form small-angle quaternion update:
    // q_k+1 = [cos(θ/2) I + sin(θ/2)/θ * Ω(ΔwΔt)] * q_k
    Matrix4d I = Matrix4d::Identity();
    Matrix4d rotation = cos(sita0/2) * I + (sin(sita0/2)/sita0 * sita);
    
    // Apply update and re-normalize for numerical stability.
    quat = rotation * quat;
    quat.normalize();
    
    // Convert quaternion (scalar-first) to body→navigation (t==n) DCM.
    double q0 = quat(0), q1 = quat(1), q2 = quat(2), q3 = quat(3);
    CbtM << q0*q0 + q1*q1 - q2*q2 - q3*q3, 2*(q1*q2 + q0*q3),        2*(q1*q3 - q0*q2),
            2*(q1*q2 - q0*q3),        q0*q0 - q1*q1 + q2*q2 - q3*q3, 2*(q2*q3 + q0*q1),
            2*(q1*q3 + q0*q2),        2*(q2*q3 - q0*q1),        q0*q0 - q1*q1 - q2*q2 + q3*q3;
}
 
/**
 * @brief Velocity and position mechanization (t==n).
 */
void UkfNavigation::strapdownVelocityPositionUpdate(const Vector3d& f_INSt,
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
    // Effective Coriolis term (2ω_ie + ω_en)×V in n-frame.
    Vector3d coriolis = 2 * wie_n + wet_t;

    // Specific force minus Coriolis; gravity is added as [0,0,-g].
    Vector3d a = f_INSt - coriolis.cross(V_prev);
    a += Vector3d(0, 0, -g);
    
    // First-order velocity integration over one IMU interval (explicit Euler).
    V_new = V_prev + (1.0/IMUrate) * a;
    
    // Position kinematics on the reference ellipsoid.
    // NOTE: Lat/Lon are stored in DEGREES in state_ (convert only for trig).
    double deltaT = 1.0/IMUrate;
    double Ry_h = Ry_prev + h_prev;  // prime-vertical radius + height
    double Rx_h = Rx_prev + h_prev;  // meridian radius + height
    double cosLat = cos(Lat_prev * M_PI/180.0);
    
    // Latitude update (deg): Δφ ≈ V_N / (Ry + h)
    Lat_new = Lat_prev + deltaT * (V_prev(1) + V_new(1)) * 180.0 / (2 * Ry_h * M_PI);
    
    // Longitude update (deg): Δλ ≈ V_E / ((Rx + h) cosφ)
    Lon_new = Lon_prev + deltaT * (V_prev(0) + V_new(0)) * 180.0 / (2 * Rx_h * cosLat * M_PI);
    
    // Height update (m): trapezoidal on Up component.
    h_new = h_prev + deltaT * (V_prev(2) + V_new(2)) / 2.0;
    
    // Update curvature radii with the new latitude (simple parametric model).
    double sinLat = sin(Lat_new * M_PI/180.0);
    Rx = Re / (1 - e * sinLat * sinLat);                      // meridian radius
    Ry = Re / (1 + 2*e - 3*e * sinLat * sinLat);              // transverse radius
}
 
/**
 * @brief Local gravity model (latitude/height dependent).
 */
double UkfNavigation::computeGravity(double Latitude, double h, const NavigationParams& params) {
    // Convert degrees → radians for model terms.
    double sinLat = sin(Latitude * M_PI/180.0);
    double sin2Lat = sinLat * sinLat;

    // Somigliana-like: g(φ,h) ≈ g0 * (1 + k1 sin²φ) / sqrt(1 - k2 sin²φ) * (1 - 2h/Re)
    double denominator = sqrt(1 - params.gk2 * sin2Lat);
    double factor = (1 - 2*h/params.Re) / denominator;
    return params.g0 * (1 + params.gk1 * sin2Lat) * factor;
}
 
// ================== State Prediction ==================
/**
 * @brief UKF time update (propagation).
 */
void UkfNavigation::predictState(int i) {
    int k = ukf_.N_kalman;

    // Combine parameters for RK4 integration
    MatrixXd Uparm(6, 9);
    Uparm << parm_old_, parm_mid_, parm_new_;

    ukf_.Q = ukf_.T * (G_mid_ * ukf_.Q0 * G_mid_.transpose());

    // Unscented transform: generate sigma points
    const int L = 2 * ukf_.Nk + 1;
    MatrixXd Xsig(ukf_.Nk, L);
    generateSigmaPoints(ukf_.X.col(ukf_.N_kalman-1), ukf_.P, ukf_.lambda, Xsig);

    ukf_.X_prop.resize(ukf_.Nk, L);
    for (int c = 0; c < L; ++c) ukf_.X_prop.col(c) = rungeKutta4(Xsig.col(c), Uparm, ukf_.T);

    // Compute predicted mean and covariance
    ukf_.X_pred.setZero(ukf_.Nk);
    for (int c = 0; c < L; ++c) ukf_.X_pred += ukf_.w(c) * ukf_.X_prop.col(c);

    ukf_.P_pred = ukf_.Q;  // start with process noise
    for (int c = 0; c < L; ++c) {
        VectorXd d = ukf_.X_prop.col(c) - ukf_.X_pred;
        ukf_.P_pred += ukf_.w(c) * (d * d.transpose());
    }

    VectorXd x_post_prev = ukf_.X.col(ukf_.N_kalman-1);
    ukf_.P_cross.setZero(ukf_.Nk, ukf_.Nk);
    for (int c = 0; c < L; ++c) {
        VectorXd d0 = Xsig.col(c) - x_post_prev;   // χ_{k-1} - x_{k-1|k-1}
        VectorXd d1 = ukf_.X_prop.col(c) - ukf_.X_pred; // χ_k - x_{k|k-1}
        ukf_.P_cross += ukf_.w(c) * (d0 * d1.transpose());
    }
}

/**
 * @brief Perform Runge-Kutta 4 integration
 * 
 * @return Predicted state
 */
Eigen::VectorXd UkfNavigation::rungeKutta4(const Eigen::VectorXd& X, 
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

/**
 * @brief Compute state derivative for continuous-time model
 * 
 * @return State derivative vector
 */
Eigen::VectorXd UkfNavigation::computeStateDerivative(const Eigen::VectorXd& X, 
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

/**
 * @brief Generate sigma points for UKF.
 */
void UkfNavigation::generateSigmaPoints(const VectorXd& x, const MatrixXd& P,
                                        double lambda, MatrixXd& Xsig) const
{
    // Sigma point spread scaling.
    const int L = 2*ukf_.Nk + 1;
    const double scale = std::sqrt(ukf_.Nk + lambda);

    // Cholesky factor of covariance (lower-triangular).
    Eigen::LLT<MatrixXd> llt(P);
    MatrixXd U = llt.matrixL();

    // 2n+1 sigma points: x, x±√((n+λ)P) columns.
    Xsig.resize(ukf_.Nk, L);
    Xsig.col(0) = x;
    for (int j=0;j<ukf_.Nk;++j){
        VectorXd col = U.col(j) * scale;
        Xsig.col(1+j)         = x + col;
        Xsig.col(1+ukf_.Nk+j) = x - col;
    }
}

// ================== Measurement Update ==================
/**
 * @brief UKF measurement update with GPS/GNSS.
 */
void UkfNavigation::updateMeasurement(const GPSData& gps, int i) {
    // Determine the corresponding GPS index for the (i+1)-th IMU step.
    int gps_index = static_cast<int>(round(static_cast<double>(i + 1) / measurement_interval_) - 1);
    if (gps_index >= gps.time.size()) gps_index = gps.time.size() - 1;

    // 1) Form innovation Z = z_meas - z_from_nav_state.
    // Velocity components are already in m/s; position residuals converted from
    // degrees to meters using current curvature radii and local cos(lat).
    VectorXd Z(6);
    Z << state_.Velocity[i+1](0) - gps.vx[gps_index],
         state_.Velocity[i+1](1) - gps.vy[gps_index],
         state_.Velocity[i+1](2) - gps.vz[gps_index],
         // Convert position differences from degrees to meters:
         (state_.Latitude[i+1] - gps.lat[gps_index]) * M_PI * (current_Ry_ + state_.Altitude[i+1]) / 180.0,
         (state_.Longitude[i+1] - gps.lon[gps_index]) * M_PI * (current_Rx_ + state_.Altitude[i+1]) * 
             cos(state_.Latitude[i+1] * M_PI/180.0) / 180.0,
         state_.Altitude[i+1]   - gps.alt[gps_index];
    
    // Measurement sensitivity matrix H (6×15) based on current attitude and LLA.
    MatrixXd H = buildMeasurementMatrix(state_.Roll[i+1], state_.Pitch[i+1], state_.Yaw[i+1],
                                state_.Latitude[i+1], state_.Altitude[i+1],
                                current_Rx_, current_Ry_);

    // 2) Project sigma points into the measurement space: γ_i = H χ_i.
    const int L = 2*ukf_.Nk + 1;
    MatrixXd Yprop(ukf_.Mk, L);
    for (int c=0;c<L;++c) Yprop.col(c) = H * ukf_.X_prop.col(c);

    // 3) Predicted measurement mean/covariance and cross-covariance.
    VectorXd y_pred = VectorXd::Zero(ukf_.Mk);
    for (int c=0;c<L;++c) y_pred += ukf_.w(c) * Yprop.col(c);

    MatrixXd Pyy = params_.ukf_params.R;  // add measurement noise
    MatrixXd Pxy = MatrixXd::Zero(ukf_.Nk, ukf_.Mk);
    for (int c=0;c<L;++c){
        VectorXd dy = Yprop.col(c) - y_pred;
        VectorXd dx = ukf_.X_prop.col(c) - ukf_.X_pred;
        Pyy += ukf_.w(c) * (dy * dy.transpose());
        Pxy += ukf_.w(c) * (dx * dy.transpose());
    }

    // 4) Kalman gain, innovation, posterior mean/covariance.
    MatrixXd K = Pxy * Pyy.inverse();
    VectorXd innovation = Z - y_pred;

    VectorXd x_new = ukf_.X_pred + K * innovation;
    MatrixXd P_new = ukf_.P_pred - K * Pyy * K.transpose();

    // Save posterior state/covariance for this measurement epoch.
    const int k = ukf_.N_kalman;
    ukf_.X.col(k)      = x_new;
    ukf_.Xsave.col(k)  = x_new;
    ukf_.P             = P_new;
    for (int j=0;j<15;++j) ukf_.P_mean_square(j,k) = std::sqrt(std::max(0.0, P_new(j,j)));

    // Reset the first 9 error-state components (att, vel, pos) for next cycle if required by design.
    ukf_.X.block(0,k,9,1).setZero();

    // Prepare for next measurement epoch.
    ukf_.N_kalman++;
}
 
// ================== Error Correction ==================
/**
 * @brief Error-state feedback into the mechanized navigation solution.
 */
void UkfNavigation::correctErrors(int i) {
    const int k = ukf_.N_kalman - 1;
    const VectorXd& Xcorr = ukf_.Xsave.col(k);
    
    // Apply position corrections (state holds lat/lon in degrees).
    state_.Latitude[i+1]  -= rad2deg(Xcorr(6));
    state_.Longitude[i+1] -= rad2deg(Xcorr(7));
    state_.Altitude[i+1]  -= Xcorr(8);

    // Apply velocity corrections (m/s).
    state_.Velocity[i+1]  -= Xcorr.segment<3>(3);

    // Attitude correction: small-angle misalignment (rad) → DCM, left-multiply.
    double E_err = rad2deg(Xcorr(0));
    double N_err = rad2deg(Xcorr(1));
    double U_err = rad2deg(Xcorr(2));
    Matrix3d C_errT = NavigationUtils::bodyToNavigationDCM(E_err, N_err, U_err);
    Matrix3d C_err  = C_errT.transpose();
    state_.CtbM = C_err * state_.CtbM;  // update n←b
    
    // Re-orthonormalize DCM to mitigate numerical drift (Gram–Schmidt).
    Vector3d col0 = state_.CtbM.col(0);
    Vector3d col1 = state_.CtbM.col(1) - state_.CtbM.col(1).dot(col0) * col0 / col0.squaredNorm();
    Vector3d col2 = state_.CtbM.col(2) 
                - state_.CtbM.col(2).dot(col0) * col0 / col0.squaredNorm()
                - state_.CtbM.col(2).dot(col1) * col1 / col1.squaredNorm();
    state_.CtbM.col(0) = col0.normalized();
    state_.CtbM.col(1) = col1.normalized();
    state_.CtbM.col(2) = col2.normalized();
    state_.CbtM = state_.CtbM.transpose();  // body→n
    
    // Update Euler angles for diagnostics after correction.
    NavigationUtils::calculateEulerAngles(state_.CbtM, state_.Pitch[i+1], state_.Roll[i+1], state_.Yaw[i+1]);
    
    // Refresh Earth model terms and rates using corrected position.
    double lat_rad = deg2rad(state_.Latitude[i+1]);
    double sinLat  = std::sin(lat_rad);
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
    
    // Synchronize quaternion with corrected Euler angles for downstream use.
    state_.Quaternion = NavigationUtils::eulerToQuaternion(state_.Pitch[i+1], state_.Roll[i+1], state_.Yaw[i+1]);
 
    // Update parameters for next prediction
    parm_old_ = parm_new_;
    
    // ========== Log history for RTS smoother (backward pass) ==========
    RtsSmoother::FilterHistory h;
    
    // Save posterior filter quantities.
    h.state                 = ukf_.Xsave.col(k);
    h.predicted_state       = ukf_.X_pred;
    h.covariance            = ukf_.P;
    h.predicted_covariance  = ukf_.P_pred;
    h.cross_covariance      = ukf_.P_cross;

    // Snapshot of corrected navigation state (scalarized to single entry vectors).
    h.nav_state.Latitude  = {state_.Latitude[i+1]};
    h.nav_state.Longitude = {state_.Longitude[i+1]};
    h.nav_state.Altitude  = {state_.Altitude[i+1]};
    h.nav_state.Velocity  = {state_.Velocity[i+1]};
    h.nav_state.Pitch     = {state_.Pitch[i+1]};
    h.nav_state.Roll      = {state_.Roll[i+1]};
    h.nav_state.Yaw       = {state_.Yaw[i+1]};
    h.nav_state.CbtM      = state_.CbtM;
    h.nav_state.CtbM      = state_.CtbM;

    rts_smoother_.addHistoryItem(h);
}

// ================== Main Run Loop ==================
/**
 * @brief Check if current step is a GNSS measurement step.
 * @return true if an update should occur this IMU step.
 */
bool UkfNavigation::isMeasurementStep(int i) const {
    return (i + 1) % measurement_interval_ == 0;
}

/**
 * @brief Execute the full per-step navigation sequence.
 */
void UkfNavigation::run(const IMUData& imu, const GPSData& gps) {
    const int NavEnd = static_cast<int>(imu.index.size()) - 1;
    
    for (int i = 0; i < NavEnd; i++) {
        // Strapdown mechanization from IMU at step i → state at i+1.
        updateStrapdown(imu, i);
        
        // Perform UKF at defined cadence (prediction + measurement + correction).
        if (isMeasurementStep(i)) {
            predictState(i);
            updateMeasurement(gps, i);
            correctErrors(i);
        }
    }
}
 
// ================== UKF Specific Algorithms ==================

/**
 * @brief Build measurement matrix H for a 6D GPS measurement (v, p).
 */
Eigen::MatrixXd UkfNavigation::buildMeasurementMatrix(double roll, double pitch, double yaw,
                                                     double lat, double h, double Rx, double Ry) {
    // Initialize measurement matrix (6x15): [vel(3); pos(3)] wrt [att(3), vel(3), pos(3), gyro b(3), accel b(3)].
    MatrixXd H = MatrixXd::Zero(6, 15);
 
    // Convert Euler angles to radians (state stores degrees).
    double roll_rad  = roll  * M_PI / 180.0;
    double pitch_rad = pitch * M_PI / 180.0;
    double yaw_rad   = yaw   * M_PI / 180.0;
 
    // Precompute trigonometric values used by an illustrative velocity sensitivity model.
    double sin_roll = sin(roll_rad);
    double cos_roll = cos(roll_rad);
    double sin_pitch = sin(pitch_rad);
    double cos_pitch = cos(pitch_rad);
    double sin_yaw = sin(yaw_rad);
    double cos_yaw = cos(yaw_rad);
 
    // Example velocity error mapping (placeholder structure seeded by attitude).
    MatrixXd H_m2(3,3);
    double a = cos_roll * cos_yaw - sin_roll * sin_pitch * sin_yaw;
    double b = cos_roll * cos_yaw + sin_roll * sin_pitch * cos_yaw;
    double c = -cos_pitch * sin_yaw;
    double d = sin_roll * cos_yaw - cos_roll * sin_pitch * sin_yaw;
 
    H_m2 << 
        1 + a * a,   a * b,         0,
        c * a,       1 + c * a,     0,
        d * a,       d * b,         1;
 
    // Position error mapping: convert [δlat, δlon, δh] to local meters.
    // Lat/Lon are in degrees; H maps errors in states (rad/m) into measurement units (m).
    MatrixXd H_m1(3,3);
    H_m1 << 
        Ry + h, 0, 0,
        0, (Rx + h) * cos(lat * M_PI / 180.0), 0,
        0, 0, 1;
 
    // Assemble: velocity residual sensitivity to velocity errors; position to position errors.
    H.block<3,3>(0,3) = H_m2;  // d(z_vel)/d(δv)
    H.block<3,3>(3,6) = H_m1;  // d(z_pos)/d(δp)
    
    return H;
}
