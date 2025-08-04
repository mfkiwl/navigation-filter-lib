/**
 * @file KfInitializer.cpp
 * @brief Implementation of KF system initializer
 *
 * @author peanut-nav
 * @date Created: 2025-07-22
 * @last Modified: 2025-08-04
 * @version 0.3.0
 */
 
#include "initializers/KfInitializer.hpp"
#include "MathUtils.hpp"
#include <fstream>
#include <cmath>
#include <stdexcept>
 
// Constructor with system timing parameters
KfInitializer::KfInitializer(int IMUrate, int GPSrate, int simTime)
    : IMUrate_(IMUrate), GPSrate_(GPSrate), simTime_(simTime) {}
 
// Initialize navigation system parameters
void KfInitializer::initialize_params(NavParamsBase& base_params, 
                                     const std::string& dataDir) {
    // Dynamic cast to KF-specific parameters
    KfParams& params = dynamic_cast<KfParams&>(base_params);
    
    // Load initial navigation conditions from file
    loadInitialNavData(dataDir + "/navdata.dat");
    
    // Configure Earth model parameters (WGS84 ellipsoid)
    params.earth_params.Re = 6378135.072;
    params.earth_params.e = 1.0/298.257223563;
    params.earth_params.W_ie = 7.292115147e-5;
    params.earth_params.g0 = 9.7803267714;
    params.earth_params.gk1 = 0.00193185138639;
    params.earth_params.gk2 = 0.00669437999013;
    
    // Calculate meridian and transverse curvature radii
    double lat_deg = lat_rad_ * 180.0 / M_PI;
    double sinLat = std::sin(lat_deg * M_PI / 180.0);
    double e_sq = params.earth_params.e * params.earth_params.e;
    params.earth_params.Rx = params.earth_params.Re / sqrt(1 - e_sq * sinLat * sinLat);
    params.earth_params.Ry = params.earth_params.Re * (1 - e_sq) / 
                             pow(1 - e_sq * sinLat * sinLat, 1.5);
    
    // Set sensor sampling rates
    params.imu_rate = IMUrate_;
    params.gps_rate = GPSrate_;
    
    // Configure initial navigation state with intentional errors
    params.init_Latitude = {lat_rad_ * 180.0 / M_PI};
    params.init_Longitude = {lon_rad_ * 180.0 / M_PI};
    params.init_Altitude = {h0_};
    params.init_Velocity = {Eigen::Vector3d(v_east_, v_north_, v_up_)};
    params.init_Pitch = {st_rad_ * 180.0 / M_PI + 0.05};   // Intentional pitch error
    params.init_Roll = {r_rad_ * 180.0 / M_PI + 0.05};     // Intentional roll error
    params.init_Yaw = {360.0 - fai_rad_ * 180.0 / M_PI + 0.1}; // Heading conversion + error
}
 
// Initialize navigation state vectors
void KfInitializer::initialize_state(NavigationState& state, 
                                    int totalPoints) {
    // Allocate memory for navigation state arrays
    state.Latitude.resize(totalPoints);
    state.Longitude.resize(totalPoints);
    state.Altitude.resize(totalPoints);
    state.Velocity.resize(totalPoints, Eigen::Vector3d::Zero());
    state.Pitch.resize(totalPoints);
    state.Roll.resize(totalPoints);
    state.Yaw.resize(totalPoints);
    
    // Set initial position and velocity
    state.Latitude[0] = lat_rad_ * 180.0 / M_PI;
    state.Longitude[0] = lon_rad_ * 180.0 / M_PI;
    state.Altitude[0] = h0_;
    state.Velocity[0] = Eigen::Vector3d(v_east_, v_north_, v_up_);
    
    // Set initial attitude with intentional errors
    state.Pitch[0] = st_rad_ * 180.0 / M_PI + 0.05;
    state.Roll[0] = r_rad_ * 180.0 / M_PI + 0.05;
    state.Yaw[0] = (360.0 - fai_rad_ * 180.0 / M_PI) + 0.1;
    
    // Initialize direction cosine matrices and quaternion
    state.CbtM = NavigationUtils::bodyToNavigationDCM(state.Pitch[0], state.Roll[0], state.Yaw[0]);
    state.CtbM = state.CbtM.transpose();
    state.Quaternion = NavigationUtils::eulerToQuaternion(state.Pitch[0], state.Roll[0], state.Yaw[0]);
}
 
// Initialize Kalman filter parameters
void KfInitializer::initialize_kalman(KalmanFilterParams& kalman, 
                                     int totalPoints) {
    // Calculate measurement update interval
    kalman.N = IMUrate_ / GPSrate_;
    
    // Determine Kalman filter points and propagation period
    kalman.M = static_cast<int>(totalPoints / kalman.N) + 10; // Buffer allocation
    kalman.T = static_cast<double>(kalman.N) / IMUrate_; // Propagation interval
    
    // Configure noise and covariance matrices
    kalman.R = setupMeasurementNoise();
    kalman.Q = setupProcessNoise();
    kalman.P = initializeCovarianceMatrix(lat_rad_ * 180.0 / M_PI);
    
    // Initialize Kalman filter state matrices
    kalman.X = Eigen::MatrixXd::Zero(15, kalman.M);
    kalman.Z = Eigen::MatrixXd::Zero(6, kalman.M);
    kalman.Xsave = Eigen::MatrixXd::Zero(15, kalman.M);
    kalman.P_mean_square = Eigen::MatrixXd::Zero(15, kalman.M);
    kalman.N_kalman = 2; // Starting index for storage
 
    // Initialize prediction states
    kalman.X_pred = Eigen::VectorXd::Zero(15);
    kalman.P_pred = Eigen::MatrixXd::Zero(15, 15);
}
 
// Load initial navigation conditions from file
void KfInitializer::loadInitialNavData(const std::string& filePath) {
    // Open navigation data file
    std::ifstream file(filePath);
    if (!file.is_open()) {
        throw std::runtime_error("Unable to open navdata.dat file");
    }
    
    // Skip header value
    double unused;
    if (!(file >> unused)) {
        throw std::runtime_error("Invalid format in navdata.dat");
    }
    
    // Read initial navigation parameters (radians for angles)
    if (!(file >> lat_rad_ >> lon_rad_ >> h0_ >> v_east_ >> v_north_ >> v_up_ 
             >> fai_rad_ >> st_rad_ >> r_rad_)) {
        throw std::runtime_error("Invalid data in navdata.dat");
    }
    
    file.close();
}
 
// Configure measurement noise covariance matrix
Eigen::MatrixXd KfInitializer::setupMeasurementNoise() const {
    // Position measurement precision (meters)
    const double Pos_Horizon_Precision = 3.0;
    const double Pos_altitude_Precision = 3.0;
    
    // Velocity measurement precision (m/s)
    const double Velocity_East_Precision = 0.01;
    const double Velocity_North_Precision = 0.01;
    const double Velocity_Up_Precision = 0.01;
    
    // Create diagonal covariance matrix
    Eigen::VectorXd r(6);
    r << std::pow(Velocity_East_Precision, 2),
         std::pow(Velocity_North_Precision, 2),
         std::pow(Velocity_Up_Precision, 2),
         std::pow(Pos_Horizon_Precision, 2),
         std::pow(Pos_Horizon_Precision, 2),
         std::pow(Pos_altitude_Precision, 2);
    
    return r.asDiagonal();
}
 
// Configure process noise covariance matrix
Eigen::MatrixXd KfInitializer::setupProcessNoise() const {
    // Gyroscope noise characteristics (deg/hr)
    const double X_gyro = 0.02;
    const double Y_gyro = 0.02;
    const double Z_gyro = 0.02;
    
    // Accelerometer noise characteristics (μg)
    const double X_acc = 50.0;
    const double Y_acc = 50.0;
    const double Z_acc = 50.0;
    
    // Unit conversions
    const double g_noise = 9.8;
    const double deg2rad = M_PI / 180.0;
    const double gyro_rad = deg2rad / 3600.0; // deg/hr to rad/s
    
    // Create diagonal covariance matrix
    Eigen::VectorXd q(6);
    q << std::pow(X_gyro * gyro_rad, 2),
         std::pow(Y_gyro * gyro_rad, 2),
         std::pow(Z_gyro * gyro_rad, 2),
         std::pow(X_acc * 1e-6 * g_noise, 2),
         std::pow(Y_acc * 1e-6 * g_noise, 2),
         std::pow(Z_acc * 1e-6 * g_noise, 2);
    
    return q.asDiagonal();
}
 
// Initialize error covariance matrix
Eigen::MatrixXd KfInitializer::initializeCovarianceMatrix(double lat) const {
    // Initial uncertainty specifications
    const double P0_Pos_Horizon = 0.1;       // Horizontal position (m)
    const double P0_Pos_altitude = 0.15;     // Vertical position (m)
    const double P0_Velocity_East = 0.01;    // East velocity (m/s)
    const double P0_Velocity_North = 0.01;    // North velocity (m/s)
    const double P0_Velocity_Up = 0.01;       // Up velocity (m/s)
    const double P0_Attitude = 1.0/60.0;     // Attitude angle (deg)
    const double P0_Heading = 0.5;           // Heading angle (deg)
    const double P0_Acc_X = 50.0;            // X-accelerometer (μg)
    const double P0_Acc_Y = 50.0;            // Y-accelerometer (μg)
    const double P0_Acc_Z = 50.0;            // Z-accelerometer (μg)
    const double P0_Gyro_X = 0.02;           // X-gyro (deg/hr)
    const double P0_Gyro_Y = 0.02;           // Y-gyro (deg/hr)
    const double P0_Gyro_Z = 0.02;           // Z-gyro (deg/hr)
    
    // Unit conversions and constants
    const double Re = 6378135.072;
    const double deg2rad = M_PI / 180.0;
    const double gyro_rad = deg2rad / 3600.0;
    const double lat_rad = lat * deg2rad;
    
    // Create diagonal covariance matrix for 15 states
    Eigen::VectorXd p(15);
    p << std::pow(P0_Attitude * deg2rad, 2),       // Pitch error (rad²)
         std::pow(P0_Attitude * deg2rad, 2),       // Roll error (rad²)
         std::pow(P0_Heading * deg2rad, 2),        // Heading error (rad²)
         std::pow(P0_Velocity_East, 2),            // East velocity (m²/s²)
         std::pow(P0_Velocity_North, 2),           // North velocity (m²/s²)
         std::pow(P0_Velocity_Up, 2),              // Up velocity (m²/s²)
         std::pow(P0_Pos_Horizon / Re, 2),         // Latitude (rad²)
         std::pow(P0_Pos_Horizon / (Re * std::cos(lat_rad)), 2), // Longitude (rad²)
         std::pow(P0_Pos_altitude, 2),             // Altitude (m²)
         std::pow(P0_Gyro_X * gyro_rad, 2),        // X-gyro bias (rad²/s²)
         std::pow(P0_Gyro_Y * gyro_rad, 2),        // Y-gyro bias (rad²/s²)
         std::pow(P0_Gyro_Z * gyro_rad, 2),        // Z-gyro bias (rad²/s²)
         std::pow(P0_Acc_X * 1e-6 * 9.8, 2),       // X-accel bias (m²/s⁴)
         std::pow(P0_Acc_Y * 1e-6 * 9.8, 2),       // Y-accel bias (m²/s⁴)
         std::pow(P0_Acc_Z * 1e-6 * 9.8, 2);       // Z-accel bias (m²/s⁴)
    
    return p.asDiagonal();
}
