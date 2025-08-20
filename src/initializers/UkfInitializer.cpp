/**
 * @file UkfInitializer.cpp
 * @brief Implementation of UKF-based navigation system initialization
 *
 * Handles parameter configuration, state initialization, and Kalman filter setup
 * for an Unscented Kalman Filter navigation solution.
 *
 * @author peanut-nav
 * @date Created: 2025-08-10
 * @last Modified: 2025-08-20
 * @version 0.4.0
 */

#include "initializers/UkfInitializer.hpp"
#include "MathUtils.hpp"
#include <fstream>
#include <cmath>
#include <stdexcept>

UkfInitializer::UkfInitializer(int IMUrate, int GPSrate, int simTime)
    : IMUrate_(IMUrate), GPSrate_(GPSrate), simTime_(simTime) {}

/**
 * @brief Initializes navigation system parameters
 * @param base_params Base parameters container to populate
 * @param dataDir Directory containing initialization data files
 * 
 * Loads initial navigation data, configures Earth model parameters, 
 * calculates Earth curvature radii, and sets initial state values.
 */
void UkfInitializer::initialize_params(NavParamsBase& base_params, 
                                     const std::string& dataDir) {
    UkfParams& params = dynamic_cast<UkfParams&>(base_params);
    loadInitialNavData(dataDir + "/navdata.dat");
    
    // Configure Earth model parameters (WGS-84 ellipsoid)
    params.earth_params.Re = 6378135.072;             // Semi-major axis [m]
    params.earth_params.e = 1.0/298.257223563;        // Flattening
    params.earth_params.W_ie = 7.292115147e-5;        // Earth rotation rate [rad/s]
    params.earth_params.g0 = 9.7803267714;            // Equatorial gravity [m/s²]
    params.earth_params.gk1 = 0.00193185138639;       // Gravity formula constant
    params.earth_params.gk2 = 0.00669437999013;       // First eccentricity squared
    
    // Calculate meridian (Rx) and transverse (Ry) radii of curvature
    double lat_deg = lat_rad_ * 180.0 / M_PI;
    double sinLat = std::sin(lat_rad_);
    double e_sq = params.earth_params.e * params.earth_params.e;
    params.earth_params.Rx = params.earth_params.Re / std::sqrt(1 - e_sq * sinLat * sinLat);
    params.earth_params.Ry = params.earth_params.Re * (1 - e_sq) / 
                             std::pow(1 - e_sq * sinLat * sinLat, 1.5);
    
    // Configure sensor sampling rates
    params.imu_rate = IMUrate_;
    params.gps_rate = GPSrate_;
    
    // Initialize navigation parameters with initial state (with intentional errors)
    params.init_Latitude = {lat_deg};
    params.init_Longitude = {lon_rad_ * 180.0 / M_PI};
    params.init_Altitude = {h0_};
    params.init_Velocity = {Eigen::Vector3d(v_east_, v_north_, v_up_)};
    params.init_Pitch = {st_rad_ * 180.0 / M_PI + 0.05};    // Pitch initialization error: +0.05°
    params.init_Roll = {r_rad_ * 180.0 / M_PI + 0.05};      // Roll initialization error: +0.05°
    params.init_Yaw = {360.0 - fai_rad_ * 180.0 / M_PI + 0.1}; // Yaw error +0.1° (true heading)
}

/**
 * @brief Initializes navigation state vectors
 * @param state Navigation state container to initialize
 * @param totalPoints Number of trajectory points to allocate
 * 
 * Sets initial position, velocity, and attitude values in ENU frame,
 * and computes initial Direction Cosine Matrix (DCM) and quaternion.
 */
void UkfInitializer::initialize_state(NavigationState& state, int totalPoints) {
    // Allocate state vectors
    state.Latitude.resize(totalPoints);
    state.Longitude.resize(totalPoints);
    state.Altitude.resize(totalPoints);
    state.Velocity.resize(totalPoints, Eigen::Vector3d::Zero());
    state.Pitch.resize(totalPoints);
    state.Roll.resize(totalPoints);
    state.Yaw.resize(totalPoints);

    // Set initial state values (ENU frame)
    state.Latitude[0]  = lat_rad_ * 180.0 / M_PI;      // Geodetic latitude [°]
    state.Longitude[0] = lon_rad_ * 180.0 / M_PI;      // Longitude [°]
    state.Altitude[0]  = h0_;                          // Ellipsoidal height [m]
    state.Velocity[0]  = Eigen::Vector3d(v_east_, v_north_, v_up_);  // ENU velocity [m/s]
    state.Pitch[0]     = st_rad_ * 180.0 / M_PI + 0.05;  // Pitch angle [°]
    state.Roll[0]      = r_rad_ * 180.0 / M_PI + 0.05;   // Roll angle [°]
    state.Yaw[0]       = 360.0 - fai_rad_ * 180.0 / M_PI + 0.1;  // True heading [°]

    // Initialize transformation matrices and quaternion
    state.CbtM = NavigationUtils::bodyToNavigationDCM(state.Pitch[0], state.Roll[0], state.Yaw[0]);
    state.CtbM = state.CbtM.transpose();
    state.Quaternion = NavigationUtils::eulerToQuaternion(state.Pitch[0], state.Roll[0], state.Yaw[0]);
}

/**
 * @brief Configures Unscented Kalman Filter parameters
 * @param base_params Parameter container to populate
 * @param totalPoints Total trajectory points for buffer sizing
 * 
 * Sets UKF dimensions, sampling parameters, noise matrices, 
 * and initializes state/covariance matrices.
 */
void UkfInitializer::initialize_kalman(NavParamsBase& base_params, int totalPoints) {
    UnscentedKalmanFilterParams& ukf_params = static_cast<UkfParams&>(base_params).ukf_params;

    // Configure IMU/GPS measurement ratio
    ukf_params.N = IMUrate_ / GPSrate_;

    // Calculate Kalman filter buffer size and measurement period
    ukf_params.M = static_cast<int>(totalPoints / ukf_params.N) + 10;  // Kalman states buffer
    ukf_params.T = static_cast<double>(ukf_params.N) / IMUrate_;        // Measurement interval [s]

    // Initialize noise covariance matrices
    ukf_params.R  = setupMeasurementNoise();   // Measurement noise
    ukf_params.Q0 = setupProcessNoise();       // Continuous process noise
    
    // Initialize state error covariance matrix
    ukf_params.P  = initializeCovarianceMatrix(lat_rad_ * 180.0 / M_PI);

    // Allocate state and measurement matrices
    ukf_params.X = Eigen::MatrixXd::Zero(15, ukf_params.M);  // State vector buffer
    ukf_params.Z = Eigen::MatrixXd::Zero(6,  ukf_params.M);   // Measurement vector buffer
    ukf_params.Xsave = Eigen::MatrixXd::Zero(15, ukf_params.M);  // State history
    ukf_params.P_mean_square = Eigen::MatrixXd::Zero(15, ukf_params.M);  // Covariance history
    ukf_params.N_kalman = 2;  // UKF update counter

    // Initialize prediction vectors
    ukf_params.X_pred = Eigen::VectorXd::Zero(15);     // Predicted state
    ukf_params.P_pred = Eigen::MatrixXd::Zero(15,15);  // Predicted covariance

    // Initialize state transition matrices
    ukf_params.Q = Eigen::MatrixXd::Zero(15, 15);          // Discrete process noise
}

/**
 * @brief Loads initial navigation state from file
 * @param filePath Path to navigation data file
 * @throws std::runtime_error on file access or format errors
 * 
 * File format: 
 * [unused] lat[rad] lon[rad] h[m] v_east[m/s] v_north[m/s] v_up[m/s] yaw[rad] pitch[rad] roll[rad]
 */
void UkfInitializer::loadInitialNavData(const std::string& filePath) {
    std::ifstream file(filePath);
    if (!file.is_open()) throw std::runtime_error("Cannot open navdata.dat");
    
    double unused;
    if (!(file >> unused)) throw std::runtime_error("Invalid navdata format");
    
    if (!(file >> lat_rad_ >> lon_rad_ >> h0_ >> v_east_ >> v_north_ >> v_up_
               >> fai_rad_ >> st_rad_ >> r_rad_))
        throw std::runtime_error("Invalid navdata values");
    
    file.close();
}

/**
 * @brief Configures measurement noise covariance matrix
 * @return 6x6 diagonal measurement noise matrix R
 * 
 * Diagonal elements correspond to:
 * [Velocity East, Velocity North, Velocity Up, 
 *  Horizontal Position, Horizontal Position, Altitude] variances
 */
Eigen::MatrixXd UkfInitializer::setupMeasurementNoise() const {
    const double Pos_Horizon_Precision = 0.3;      // Horizontal position noise SD [m]
    const double Pos_altitude_Precision = 0.3;     // Vertical position noise SD [m]
    const double Velocity_East_Precision = 0.01;   // East velocity noise SD [m/s]
    const double Velocity_North_Precision = 0.01;  // North velocity noise SD [m/s]
    const double Velocity_Up_Precision = 0.01;     // Up velocity noise SD [m/s]

    Eigen::VectorXd r(6);
    r << std::pow(Velocity_East_Precision,2),
         std::pow(Velocity_North_Precision,2),
         std::pow(Velocity_Up_Precision,2),
         std::pow(Pos_Horizon_Precision,2),
         std::pow(Pos_Horizon_Precision,2),
         std::pow(Pos_altitude_Precision,2);
    return r.asDiagonal();  // Diagonal covariance matrix
}

/**
 * @brief Configures process noise covariance matrix
 * @return 6x6 diagonal process noise matrix Q0
 * 
 * Diagonal elements correspond to:
 * [Gyro X, Gyro Y, Gyro Z, Accel X, Accel Y, Accel Z] variances
 * Gyro noise in [rad/s], Accel noise in [m/s²]
 */
Eigen::MatrixXd UkfInitializer::setupProcessNoise() const {
    const double X_gyro=0.02, Y_gyro=0.02, Z_gyro=0.02;          // Gyro bias stability [°/hr]
    const double X_acc=50.0, Y_acc=50.0, Z_acc=50.0;              // Accel bias [μg]
    const double deg2rad = M_PI/180.0;
    const double gyro_rad = deg2rad / 3600.0;                     // [°/hr] to [rad/s]
    const double g_noise = 9.8;                                   // Gravity for accel conversion

    Eigen::VectorXd q(6);
    q << std::pow(X_gyro*gyro_rad,2),         // Converted to [rad²/s²]
         std::pow(Y_gyro*gyro_rad,2),
         std::pow(Z_gyro*gyro_rad,2),
         std::pow(X_acc*1e-6*g_noise,2),      // Converted to [m²/s⁴]
         std::pow(Y_acc*1e-6*g_noise,2),
         std::pow(Z_acc*1e-6*g_noise,2);
    return q.asDiagonal();  // Diagonal covariance matrix
}

/**
 * @brief Initializes state error covariance matrix
 * @param lat Initial latitude [degrees]
 * @return 15x15 diagonal covariance matrix P
 * 
 * Diagonal elements correspond to variances of:
 * [Attitude_E, Attitude_N, Heading, 
 *  Vel_E, Vel_N, Vel_U, 
 *  Lat, Lon, Alt, 
 *  Gyro_bias_X, Gyro_bias_Y, Gyro_bias_Z,
 *  Accel_bias_X, Accel_bias_Y, Accel_bias_Z]
 */
Eigen::MatrixXd UkfInitializer::initializeCovarianceMatrix(double lat) const {
    // Initial uncertainty specifications
    const double P0_Attitude = 1.0/60.0;      // Attitude error SD [°]
    const double P0_Heading = 0.1;            // Heading error SD [°]
    const double P0_VE=0.01, P0_VN=0.01, P0_VU=0.01;  // Velocity error SD [m/s]
    const double P0_Pos_H=0.1;                // Horizontal position error SD [m]
    const double P0_Pos_alt=0.15;             // Vertical position error SD [m]
    const double P0_Gx=0.01, P0_Gy=0.01, P0_Gz=0.01; // Gyro bias SD [°/hr]
    const double P0_Ax=50.0, P0_Ay=50.0, P0_Az=50.0; // Accel bias SD [μg]

    // Conversion constants
    const double Re = 6378135.072;             // Earth semi-major axis [m]
    const double deg2rad = M_PI/180.0;
    const double gyro_rad = deg2rad/3600.0;    // [°/hr] to [rad/s]
    const double lat_rad = lat*deg2rad;        // Latitude in radians

    // Initialize covariance vector (diagonal elements)
    Eigen::VectorXd p(15);
    p << std::pow(P0_Attitude*deg2rad,2),  // Attitude error variance [rad²]
         std::pow(P0_Attitude*deg2rad,2),
         std::pow(P0_Heading *deg2rad,2),
         std::pow(P0_VE,2), std::pow(P0_VN,2), std::pow(P0_VU,2),  // Velocity variances [m²/s²]
         std::pow(P0_Pos_H/Re,2),                                  // Latitude variance [rad²]
         std::pow(P0_Pos_H/(Re*std::cos(lat_rad)),2),              // Longitude variance [rad²]
         std::pow(P0_Pos_alt,2),                                   // Altitude variance [m²]
         std::pow(P0_Gx*gyro_rad,2),                               // Gyro bias variances [rad²/s²]
         std::pow(P0_Gy*gyro_rad,2),
         std::pow(P0_Gz*gyro_rad,2),
         std::pow(P0_Ax*1e-6*9.8,2),                               // Accel bias variances [m²/s⁴]
         std::pow(P0_Ay*1e-6*9.8,2),
         std::pow(P0_Az*1e-6*9.8,2);
    return p.asDiagonal();  // Return diagonal covariance matrix
}
