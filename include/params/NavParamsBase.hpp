/**
 * @file NavParamsBase.hpp
 * @brief Base parameters and data structures for navigation systems
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
 */
struct NavigationParams {
    double Re = 6378135.072;          ///< Equatorial radius (meters)
    double e = 1.0/298.257223563;     ///< Earth's flattening coefficient (dimensionless)
    double W_ie = 7.292115147e-5;     ///< Earth rotation rate (rad/s)
    double g0 = 9.7803267714;         ///< Gravity at equator (m/s²)
    double gk1 = 0.00193185138639;    ///< Gravity formula constant (dimensionless)
    double gk2 = 0.00669437999013;    ///< Gravity formula constant (dimensionless)
    double Rx = 0.0;                  ///< Meridian radius of curvature (meters)
    double Ry = 0.0;                  ///< Prime vertical radius of curvature (meters)
};

/**
 * @brief Navigation state container
 */
struct NavigationState {
    std::vector<double> Latitude;      ///< Geodetic latitude (degrees)
    std::vector<double> Longitude;     ///< Geodetic longitude (degrees)
    std::vector<double> Altitude;      ///< Altitude above ellipsoid (meters)
    std::vector<Eigen::Vector3d> Velocity;  ///< Velocity vector in navigation frame (m/s)
    std::vector<double> Pitch;         ///< Pitch angle (degrees)
    std::vector<double> Roll;          ///< Roll angle (degrees)
    std::vector<double> Yaw;           ///< Yaw angle (degrees)
    Eigen::Matrix3d CbtM;              ///< Body to navigation frame DCM (dimensionless)
    Eigen::Matrix3d CtbM;              ///< Navigation to body frame DCM (dimensionless)
    Eigen::Vector4d Quaternion;        ///< Attitude quaternion [w, x, y, z] (dimensionless)
};

/**
 * @brief IMU measurement data container
 */
struct IMUData {
    std::vector<int> index;        ///< Measurement indices (dimensionless)
    std::vector<double> gx;        ///< Gyroscope X-axis measurements (deg/h)
    std::vector<double> gy;        ///< Gyroscope Y-axis measurements (deg/h)
    std::vector<double> gz;        ///< Gyroscope Z-axis measurements (deg/h)
    std::vector<double> ax;        ///< Accelerometer X-axis measurements (m/s²)
    std::vector<double> ay;        ///< Accelerometer Y-axis measurements (m/s²)
    std::vector<double> az;        ///< Accelerometer Z-axis measurements (m/s²)
};

/**
 * @brief GPS measurement data container
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

/**
 * @brief Base navigation parameters
 * 
 * Contains parameters common to all navigation filters.
 */
struct NavParamsBase {
    // 采样频率
    double imu_rate = 200.0;      ///< IMU sample rate (Hz)
    double gps_rate = 20.0;        ///< GPS sample rate (Hz)
    
    // 地球模型参数
    NavigationParams earth_params; ///< Earth model parameters
    
    // 初始状态
    std::vector<double> init_Latitude;      ///< Initial geodetic latitude (degrees)
    std::vector<double> init_Longitude;     ///< Initial geodetic longitude (degrees)
    std::vector<double> init_Altitude;      ///< Initial altitude above ellipsoid (meters)
    std::vector<Eigen::Vector3d> init_Velocity; ///< Initial velocity vector in navigation frame (m/s)
    std::vector<double> init_Pitch;         ///< Initial pitch angle (degrees)
    std::vector<double> init_Roll;          ///< Initial roll angle (degrees)
    std::vector<double> init_Yaw;           ///< Initial yaw angle (degrees)
    
    virtual ~NavParamsBase() = default;
};
