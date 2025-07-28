/**
 * @file KfParams.hpp
 * @brief Parameters for Kalman filter based navigation
 * 
 * @author peanut-nav
 * @date Created: 2025-07-22
 * @last Modified: 2025-07-22
 * @version 0.1
 */

#pragma once
#include "NavParamsBase.hpp"
#include <Eigen/Dense>

/**
 * @brief Kalman filter parameters and state
 */
struct KalmanFilterParams {
    int N;  ///< IMU/GPS update rate ratio (dimensionless)
    int M;  ///< Number of Kalman filter points (dimensionless)
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
    
    int N_kalman = 2;  ///< Update counter (dimensionless)
};

/**
 * @brief Kalman filter specific parameters
 */
struct KfParams : public NavParamsBase {
    KalmanFilterParams kalman_params; ///< KF-specific parameters
};
