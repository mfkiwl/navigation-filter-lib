/**
 * @file NavigationParams.hpp
 * @brief Parameters and data structures for navigation systems
 *
 * Defines core parameters and data structures used in inertial navigation
 * and Kalman filtering implementations.
 *
 * @author peanut-nav
 * @date Created: 2025-07-22
 * @last Modified: 2025-07-22
 * @version 0.1
 */

#pragma once
#include <Eigen/Dense>
#include <vector>

/**
 * @brief Earth model parameters
 * 
 * Contains physical constants and parameters defining the Earth's model
 * used in navigation calculations.
 */
struct NavigationParams {
    // Earth parameters
    double Re = 6378135.072;          ///< Equatorial radius (meters)
    double e = 1.0/298.257223563;     ///< Earth's flattening coefficient
    double W_ie = 7.292115147e-5;     ///< Earth rotation rate (rad/s)
    
    // Gravity model parameters
    double g0 = 9.7803267714;         ///< Gravity at equator (m/s²)
    double gk1 = 0.00193185138639;    ///< Gravity formula constant
    double gk2 = 0.00669437999013;    ///< Gravity formula constant
    
    // Earth curvature radii
    double Rx = 0.0;                  ///< Meridian radius of curvature
    double Ry = 0.0;                  ///< Prime vertical radius of curvature
};

/**
 * @brief Navigation state container
 * 
 * Stores the complete state of a navigation system at any given time,
 * including position, velocity, attitude and orientation representations.
 */
struct NavigationState {
    // Position states
    std::vector<double> Latitude;      ///< Geodetic latitude (degrees)
    std::vector<double> Longitude;     ///< Geodetic longitude (degrees)
    std::vector<double> Altitude;      ///< Altitude above ellipsoid (meters)
    
    // Velocity states
    std::vector<Eigen::Vector3d> Velocity;  ///< Velocity vector in navigation frame (m/s)
    
    // Attitude states
    std::vector<double> Pitch;         ///< Pitch angle (degrees)
    std::vector<double> Roll;          ///< Roll angle (degrees)
    std::vector<double> Yaw;           ///< Yaw angle (degrees)
    
    // Orientation matrices
    Eigen::Matrix3d CbtM;              ///< Body to navigation frame DCM
    Eigen::Matrix3d CtbM;              ///< Navigation to body frame DCM
    
    // Attitude quaternion
    Eigen::Vector4d Quaternion;        ///< Attitude quaternion [w, x, y, z]
};

/**
 * @brief Kalman filter parameters and state
 * 
 * Contains all configuration parameters and state variables
 * for a Kalman filter implementation.
 */
struct KalmanFilterParams {
    int N;  ///< IMU/GPS update rate ratio
    int M;  ///< Number of Kalman filter points
    double T;  ///< Kalman filter period (seconds)
    
    // Noise matrices
    Eigen::MatrixXd R;  ///< Measurement noise covariance matrix
    Eigen::MatrixXd Q;  ///< Process noise covariance matrix
    
    // Covariance matrix
    Eigen::MatrixXd P;  ///< State estimation error covariance matrix
    
    // State and measurement vectors
    Eigen::MatrixXd X;  ///< State vector
    Eigen::MatrixXd Z;  ///< Measurement vector
    
    // Result storage
    Eigen::MatrixXd Xsave;             ///< Saved state estimates
    Eigen::MatrixXd P_mean_square;     ///< Mean square covariance
    
    int N_kalman = 2;  ///< Update counter
};

/**
 * @brief IMU measurement data container
 * 
 * Stores raw IMU measurements including gyroscope and accelerometer data.
 */
struct IMUData {
    std::vector<int> index;        ///< Measurement indices
    std::vector<double> gx;        ///< Gyroscope X-axis measurements (deg/s)
    std::vector<double> gy;        ///< Gyroscope Y-axis measurements (deg/s)
    std::vector<double> gz;        ///< Gyroscope Z-axis measurements (deg/s)
    std::vector<double> ax;        ///< Accelerometer X-axis measurements (m/s²)
    std::vector<double> ay;        ///< Accelerometer Y-axis measurements (m/s²)
    std::vector<double> az;        ///< Accelerometer Z-axis measurements (m/s²)
};

/**
 * @brief GPS measurement data container
 * 
 * Stores processed GPS measurements including position and velocity.
 */
struct GPSData {
    std::vector<double> time;      ///< Measurement timestamps (seconds)
    std::vector<double> vx;        ///< Velocity X-component (m/s)
    std::vector<double> vy;        ///< Velocity Y-component (m/s)
    std::vector<double> vz;        ///< Velocity Z-component (m/s)
    std::vector<double> lat;       ///< Latitude (degrees)
    std::vector<double> lon;       ///< Longitude (degrees)
    std::vector<double> alt;       ///< Altitude (meters)
};

/**
 * @brief Reference trajectory data container
 * 
 * Stores ground truth or reference trajectory information
 * for navigation system evaluation.
 */
struct TrajectoryData {
    std::vector<double> time;      ///< Time points (seconds)
    std::vector<double> yaw;       ///< Reference yaw angle (degrees)
    std::vector<double> pitch;     ///< Reference pitch angle (degrees)
    std::vector<double> roll;      ///< Reference roll angle (degrees)
    std::vector<double> vx;        ///< Reference velocity X (m/s)
    std::vector<double> vy;        ///< Reference velocity Y (m/s)
    std::vector<double> vz;        ///< Reference velocity Z (m/s)
    std::vector<double> lat;       ///< Reference latitude (degrees)
    std::vector<double> lon;       ///< Reference longitude (degrees)
    std::vector<double> alt;       ///< Reference altitude (meters)
};
