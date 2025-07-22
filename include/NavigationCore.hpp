/**
 * @file NavigationCore.hpp
 * @brief Core navigation algorithms implementation
 *
 * Contains implementations of the core navigation algorithms including
 * attitude update, velocity and position update, Euler angle calculation,
 * gravity calculation, and the main navigation loop with Kalman filtering.
 *
 * @author peanut-nav
 * @date Created: 2025-07-22
 * @last Modified: 2025-07-22
 * @version 0.1
 */

#pragma once
#include "NavigationParams.hpp"
#include "MathUtils.hpp"
#include <vector>
#include <functional>

class NavigationCore {
public:
    /**
     * @brief Update the attitude using gyroscope measurements
     * 
     * @param wtb_b Angular velocity of body relative to inertial frame, in body frame (rad/s)
     * @param wit_b Earth rotation rate in body frame (rad/s)
     * @param quat Current attitude quaternion (updated in-place)
     * @param IMUrate IMU sampling rate (Hz)
     * @param CbtM Direction cosine matrix from body to navigation frame (updated in-place)
     */
    static void updateAttitude(const Eigen::Vector3d& wtb_b,
                              const Eigen::Vector3d& wit_b,
                              Eigen::Vector4d& quat,
                              int IMUrate,
                              Eigen::Matrix3d& CbtM);
    
    /**
     * @brief Update velocity and position using accelerometer measurements
     * 
     * @param f_INSt Specific force in navigation frame (m/s²)
     * @param V_prev Previous velocity in navigation frame (m/s)
     * @param Lat_prev Previous latitude (degrees)
     * @param Lon_prev Previous longitude (degrees)
     * @param h_prev Previous altitude (m)
     * @param wie_n Earth rotation rate in navigation frame (rad/s)
     * @param wet_t Transport rate in navigation frame (rad/s)
     * @param g Gravity magnitude at current location (m/s²)
     * @param IMUrate IMU sampling rate (Hz)
     * @param Re Earth equatorial radius (m)
     * @param e Earth eccentricity
     * @param Rx_prev Previous meridian radius of curvature (m)
     * @param Ry_prev Previous prime vertical radius of curvature (m)
     * @param V_new Updated velocity (output)
     * @param Lat_new Updated latitude (degrees) (output)
     * @param Lon_new Updated longitude (degrees) (output)
     * @param h_new Updated altitude (m) (output)
     * @param Rx Updated meridian radius of curvature (m) (output)
     * @param Ry Updated prime vertical radius of curvature (m) (output)
     */
    static void updateVelocityPosition(const Eigen::Vector3d& f_INSt,
                                      const Eigen::Vector3d& V_prev,
                                      double Lat_prev,
                                      double Lon_prev,
                                      double h_prev,
                                      const Eigen::Vector3d& wie_n,
                                      const Eigen::Vector3d& wet_t,
                                      double g,
                                      int IMUrate,
                                      double Re,
                                      double e,
                                      double Rx_prev,
                                      double Ry_prev,
                                      Eigen::Vector3d& V_new,
                                      double& Lat_new,
                                      double& Lon_new,
                                      double& h_new,
                                      double& Rx,
                                      double& Ry);
    
    /**
     * @brief Calculate Euler angles from a direction cosine matrix
     * 
     * @param CbtM Direction cosine matrix from body to navigation frame
     * @param pitch Pitch angle (degrees) (output)
     * @param roll Roll angle (degrees) (output)
     * @param yaw Yaw angle (degrees) (output)
     */
    static void calculateEulerAngles(const Eigen::Matrix3d& CbtM,
                                     double& pitch,
                                     double& roll,
                                     double& yaw);
    
    /**
     * @brief Calculate gravity at a given latitude and altitude
     * 
     * @param Latitude Geodetic latitude (degrees)
     * @param h Altitude above ellipsoid (m)
     * @param params Navigation parameters
     * @return double Gravity magnitude (m/s²)
     */
    static double calculateGravity(double Latitude, double h, const NavigationParams& params);
    
    /**
     * @brief Main navigation processing loop
     * 
     * @param imu IMU data
     * @param gps GPS data
     * @param params Navigation parameters
     * @param state Navigation state (updated in-place)
     * @param kalman Kalman filter parameters (updated in-place)
     * @param IMUrate IMU sampling rate (Hz)
     * @param GPSrate GPS sampling rate (Hz)
     */
    static void runNavigation(const IMUData& imu,
                             const GPSData& gps,
                             const NavigationParams& params,
                             NavigationState& state,
                             KalmanFilterParams& kalman,
                             int IMUrate,
                             int GPSrate);
    
    // Kalman filter related functions
    
    /**
     * @brief Execute a Kalman filter update step
     * 
     * @param i Current time index
     * @param state Navigation state
     * @param kalman Kalman filter parameters (updated in-place)
     * @param gps GPS data
     * @param params Navigation parameters
     * @param IMUrate IMU sampling rate (Hz)
     * @param GPSrate GPS sampling rate (Hz)
     * @param Rx Current meridian radius of curvature (m)
     * @param Ry Current prime vertical radius of curvature (m)
     * @param f_INSt Specific force in navigation frame (m/s²)
     */
    static void runKalmanFilter(int i, 
                               NavigationState& state, 
                               KalmanFilterParams& kalman, 
                               const GPSData& gps,
                               const NavigationParams& params,
                               int IMUrate,
                               int GPSrate,
                               double Rx,
                               double Ry,
                               const Eigen::Vector3d& f_INSt);
    
    /**
     * @brief Compute the state transition matrix for Kalman filter
     * 
     * @param Latitude Current latitude (degrees)
     * @param V Current velocity in navigation frame (m/s)
     * @param h Current altitude (m)
     * @param Rx Meridian radius of curvature (m)
     * @param Ry Prime vertical radius of curvature (m)
     * @param CtbM Direction cosine matrix from navigation to body frame
     * @param W_ie Earth rotation rate (rad/s)
     * @param f_INSt Specific force in navigation frame (m/s²)
     * @return Eigen::MatrixXd State transition matrix (15x15)
     */
    static Eigen::MatrixXd computeStateMatrix(double Latitude,
                                             const Eigen::Vector3d& V,
                                             double h,
                                             double Rx,
                                             double Ry,
                                             const Eigen::Matrix3d& CtbM,
                                             double W_ie,
                                             const Eigen::Vector3d& f_INSt);
    
    /**
     * @brief Compute the measurement matrix for Kalman filter
     * 
     * @param roll Roll angle (degrees)
     * @param yaw Yaw angle (degrees)
     * @param pitch Pitch angle (degrees)
     * @param Latitude Latitude (degrees)
     * @param h Altitude (m)
     * @param Rx Meridian radius of curvature (m)
     * @param Ry Prime vertical radius of curvature (m)
     * @return Eigen::MatrixXd Measurement matrix (6x15)
     */
    static Eigen::MatrixXd computeMeasurementMatrix(double roll,
                                                   double yaw,
                                                   double pitch,
                                                   double Latitude,
                                                   double h,
                                                   double Rx,
                                                   double Ry);
    
    /**
     * @brief Perform a single Kalman filter step
     * 
     * @param X_prev Previous state vector
     * @param P_prev Previous covariance matrix
     * @param Z Measurement vector
     * @param H Measurement matrix
     * @param R Measurement noise covariance
     * @param Q Process noise covariance
     * @param T Time step (s)
     * @param A State transition matrix
     * @param B Control matrix
     * @param X_new Updated state vector (output)
     * @param P_new Updated covariance matrix (output)
     */
    static void kalmanFilterStep(const Eigen::VectorXd& X_prev,
                                const Eigen::MatrixXd& P_prev,
                                const Eigen::VectorXd& Z,
                                const Eigen::MatrixXd& H,
                                const Eigen::MatrixXd& R,
                                const Eigen::MatrixXd& Q,
                                double T,
                                const Eigen::MatrixXd& A,
                                const Eigen::MatrixXd& B,
                                Eigen::VectorXd& X_new,
                                Eigen::MatrixXd& P_new);
};
