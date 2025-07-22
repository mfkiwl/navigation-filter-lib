/**
 * @file SystemInitializer.cpp
 * @brief Implementation of system initialization utilities
 *
 * Implements the system initialization functions
 * declared in SystemInitializer.hpp.
 *
 * @author peanut-nav
 * @date Created: 2025-07-22
 * @last Modified: 2025-07-22
 * @version 0.1
 */

#include "SystemInitializer.hpp"
#include <fstream>
#include <cmath>
#include <stdexcept>

/**
 * @brief Initialize the entire navigation system
 * 
 * Coordinates the initialization of navigation parameters, state variables,
 * and Kalman filter settings based on input data.
 */
void SystemInitializer::initializeSystem(int IMUrate, 
                                        int GPSrate, 
                                        int simTime,
                                        const std::string& dataDir,
                                        NavigationParams& params,
                                        NavigationState& state,
                                        KalmanFilterParams& kalman) {
    // Load initial navigation parameters from file
    double lat_rad, lon_rad, h0, v_east, v_north, v_up, fai_rad, st_rad, r_rad;
    loadInitialNavData(dataDir + "/navdata.dat", 
                      lat_rad, lon_rad, h0, v_east, v_north, v_up, 
                      fai_rad, st_rad, r_rad);
    
    // Calculate total data points based on simulation time and IMU rate
    int totalPoints = simTime * IMUrate + 1;
    
    // Initialize navigation state vectors
    state.Latitude.resize(totalPoints);
    state.Longitude.resize(totalPoints);
    state.Altitude.resize(totalPoints);
    state.Velocity.resize(totalPoints, Eigen::Vector3d::Zero());
    state.Pitch.resize(totalPoints);
    state.Roll.resize(totalPoints);
    state.Yaw.resize(totalPoints);
    
    // Set initial position and velocity
    state.Latitude[0] = lat_rad * 180.0 / M_PI;  // Convert to degrees
    state.Longitude[0] = lon_rad * 180.0 / M_PI;
    state.Altitude[0] = h0;
    state.Velocity[0] = Eigen::Vector3d(v_east, v_north, v_up);
    
    // Set initial attitude with calibration offsets
    state.Pitch[0] = st_rad * 180.0 / M_PI + 0.05;         // Pitch angle (degrees)
    state.Roll[0] = r_rad * 180.0 / M_PI + 0.05;           // Roll angle (degrees)
    state.Yaw[0] = (360.0 - fai_rad * 180.0 / M_PI) + 0.1; // Yaw angle (degrees)
    
    // Initialize orientation matrices
    state.CbtM = NavigationUtils::bodyToNavigationDCM(state.Pitch[0], state.Roll[0], state.Yaw[0]);
    state.CtbM = state.CbtM.transpose();
    
    // Initialize attitude quaternion
    state.Quaternion = NavigationUtils::eulerToQuaternion(state.Pitch[0], state.Roll[0], state.Yaw[0]);
    
    // Calculate Earth curvature radii at initial position
    double sinLat = std::sin(state.Latitude[0] * M_PI / 180.0);
    params.Rx = params.Re / (1 - params.e * sinLat * sinLat);
    params.Ry = params.Re / (1 + 2 * params.e - 3 * params.e * sinLat * sinLat);
    
    // Initialize Kalman filter parameters
    kalman.N = IMUrate / GPSrate;  // IMU/GPS update ratio
    kalman.M = static_cast<int>(totalPoints / kalman.N) + 10;  // Number of Kalman points
    kalman.T = static_cast<double>(kalman.N) / IMUrate;        // Kalman period
    
    // Set up noise and covariance matrices
    kalman.R = setupMeasurementNoise();    // Measurement noise
    kalman.Q = setupProcessNoise();        // Process noise
    kalman.P = initializeCovarianceMatrix(state.Latitude[0]);  // Initial covariance
    
    // Initialize state vectors and storage
    kalman.X = Eigen::MatrixXd::Zero(15, kalman.M);          // State vector
    kalman.Z = Eigen::MatrixXd::Zero(6, kalman.M);           // Measurement vector
    kalman.Xsave = Eigen::MatrixXd::Zero(15, kalman.M);      // Saved state estimates
    kalman.P_mean_square = Eigen::MatrixXd::Zero(15, kalman.M);  // Mean square covariance
    kalman.N_kalman = 2;  // Kalman update counter
}

/**
 * @brief Load initial navigation data from file
 * 
 * Reads key initial navigation parameters from a data file.
 */
void SystemInitializer::loadInitialNavData(const std::string& filePath,
                                          double& lat_rad,
                                          double& lon_rad,
                                          double& h0,
                                          double& v_east,
                                          double& v_north,
                                          double& v_up,
                                          double& fai_rad,
                                          double& st_rad,
                                          double& r_rad) {
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
    
    // Read initial navigation parameters
    if (!(file >> lat_rad >> lon_rad >> h0 >> v_east >> v_north >> v_up 
             >> fai_rad >> st_rad >> r_rad)) {
        throw std::runtime_error("Invalid data in navdata.dat");
    }
    
    file.close();
}

/**
 * @brief Set up measurement noise covariance matrix
 * 
 * Defines the measurement noise characteristics for GPS sensors.
 */
Eigen::MatrixXd SystemInitializer::setupMeasurementNoise() {
    // Position measurement precision
    const double Pos_Horizon_Precision = 3.0;     // Horizontal position precision (m)
    const double Pos_altitude_Precision = 3.0;    // Vertical position precision (m)
    
    // Velocity measurement precision
    const double Velocity_East_Precision = 0.01;    // East velocity precision (m/s)
    const double Velocity_North_Precision = 0.01;   // North velocity precision (m/s)
    const double Velocity_Up_Precision = 0.01;      // Up velocity precision (m/s)
    
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

/**
 * @brief Set up process noise covariance matrix
 * 
 * Defines the process noise characteristics for IMU sensors.
 */
Eigen::MatrixXd SystemInitializer::setupProcessNoise() {
    // Gyroscope noise characteristics
    const double X_gyro = 0.02;  // X-axis gyro noise (°/h)
    const double Y_gyro = 0.02;  // Y-axis gyro noise (°/h)
    const double Z_gyro = 0.02;  // Z-axis gyro noise (°/h)
    
    // Accelerometer noise characteristics
    const double X_acc = 50.0;   // X-axis accelerometer noise (μg)
    const double Y_acc = 50.0;   // Y-axis accelerometer noise (μg)
    const double Z_acc = 50.0;   // Z-axis accelerometer noise (μg)
    
    const double g_noise = 9.8;  // Gravity reference (m/s²)
    
    // Unit conversions
    const double deg2rad = M_PI / 180.0;
    const double gyro_rad = deg2rad / 3600.0;  // Convert °/h to rad/s
    
    // Create diagonal covariance matrix
    Eigen::VectorXd q(6);
    q << std::pow(X_gyro * gyro_rad, 2),
         std::pow(Y_gyro * gyro_rad, 2),
         std::pow(Z_gyro * gyro_rad, 2),
         std::pow(X_acc * 1e-6 * g_noise, 2),  // Convert μg to m/s²
         std::pow(Y_acc * 1e-6 * g_noise, 2),
         std::pow(Z_acc * 1e-6 * g_noise, 2);
    
    return q.asDiagonal();
}

/**
 * @brief Initialize state error covariance matrix
 * 
 * Defines the initial uncertainty in the state estimation.
 */
Eigen::MatrixXd SystemInitializer::initializeCovarianceMatrix(double lat) {
    // Initial uncertainty parameters
    const double P0_Pos_Horizon = 0.1;     // Horizontal position uncertainty (m)
    const double P0_Pos_altitude = 0.15;   // Vertical position uncertainty (m)
    const double P0_Velocity_East = 0.01;  // East velocity uncertainty (m/s)
    const double P0_Velocity_North = 0.01; // North velocity uncertainty (m/s)
    const double P0_Velocity_Up = 0.01;    // Up velocity uncertainty (m/s)
    const double P0_Attitude = 1.0/60.0;   // Attitude uncertainty (degrees)
    const double P0_Heading = 0.5;         // Heading uncertainty (degrees)
    const double P0_Acc_X = 50.0;          // X accelerometer bias uncertainty (μg)
    const double P0_Acc_Y = 50.0;          // Y accelerometer bias uncertainty (μg)
    const double P0_Acc_Z = 50.0;          // Z accelerometer bias uncertainty (μg)
    const double P0_Gyro_X = 0.02;         // X gyro bias uncertainty (°/h)
    const double P0_Gyro_Y = 0.02;         // Y gyro bias uncertainty (°/h)
    const double P0_Gyro_Z = 0.02;         // Z gyro bias uncertainty (°/h)
    
    const double Re = 6378135.072;  // Earth equatorial radius (m)
    
    // Unit conversions
    const double deg2rad = M_PI / 180.0;
    const double gyro_rad = deg2rad / 3600.0;  // Convert °/h to rad/s
    const double lat_rad = lat * deg2rad;      // Convert degrees to radians
    
    // Create diagonal covariance matrix
    Eigen::VectorXd p(15);
    p << std::pow(P0_Attitude * deg2rad, 2),       // Pitch uncertainty (rad²)
         std::pow(P0_Attitude * deg2rad, 2),       // Roll uncertainty (rad²)
         std::pow(P0_Heading * deg2rad, 2),        // Yaw uncertainty (rad²)
         std::pow(P0_Velocity_East, 2),            // East velocity uncertainty (m²/s²)
         std::pow(P0_Velocity_North, 2),           // North velocity uncertainty (m²/s²)
         std::pow(P0_Velocity_Up, 2),              // Up velocity uncertainty (m²/s²)
         std::pow(P0_Pos_Horizon / Re, 2),         // Latitude uncertainty (rad²)
         std::pow(P0_Pos_Horizon / (Re * std::cos(lat_rad)), 2),  // Longitude uncertainty (rad²)
         std::pow(P0_Pos_altitude, 2),             // Altitude uncertainty (m²)
         std::pow(P0_Gyro_X * gyro_rad, 2),        // X gyro bias uncertainty (rad²/s²)
         std::pow(P0_Gyro_Y * gyro_rad, 2),        // Y gyro bias uncertainty (rad²/s²)
         std::pow(P0_Gyro_Z * gyro_rad, 2),        // Z gyro bias uncertainty (rad²/s²)
         std::pow(P0_Acc_X * 1e-6 * 9.8, 2),       // X accel bias uncertainty (m²/s⁴)
         std::pow(P0_Acc_Y * 1e-6 * 9.8, 2),       // Y accel bias uncertainty (m²/s⁴)
         std::pow(P0_Acc_Z * 1e-6 * 9.8, 2);       // Z accel bias uncertainty (m²/s⁴)
    
    return p.asDiagonal();
}
