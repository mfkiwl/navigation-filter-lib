/**
 * @file EkfParams.hpp
 * @brief Parameters for Extended Kalman Filter based navigation
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
 * @brief EKF-specific parameters and state
 */
struct ExtendedKalmanFilterParams {
    // Kalman filter parameters
    int N = 0;           ///< IMU/GPS update rate ratio (dimensionless)
    int M = 0;           ///< Number of Kalman filter points (dimensionless)
    double T = 0.0;      ///< Kalman filter period (seconds)
    int N_kalman = 2;    ///< Update counter (dimensionless)
    
    // Noise matrices
    Eigen::MatrixXd R;  ///< Measurement noise covariance matrix
    Eigen::MatrixXd Q;  ///< Process noise covariance matrix (continuous time)
    
    // Covariance matrix
    Eigen::MatrixXd P;  ///< State estimation error covariance matrix
    
    // State and measurement vectors
    Eigen::MatrixXd X;  ///< State vector
    Eigen::MatrixXd Z;  ///< Measurement vector
    
    // Prediction states
    Eigen::VectorXd X_pred;  ///< Predicted state vector
    Eigen::MatrixXd P_pred;  ///< Predicted covariance matrix
    
    // Measurement matrix
    Eigen::MatrixXd H;       ///< Measurement matrix (Hz in MATLAB)
    
    // Discrete system matrices (from MATLAB)
    Eigen::MatrixXd A;    ///< state transition matrix
    Eigen::MatrixXd disA;    ///< Discrete state transition matrix
    Eigen::MatrixXd disQ;    ///< Discrete process noise covariance
    
    // Result storage
    Eigen::MatrixXd Xsave;             ///< Saved state estimates
    Eigen::MatrixXd P_mean_square;     ///< Mean square covariance
};

/**
 * @brief Extended Kalman filter specific parameters
 */
struct EkfParams : public NavParamsBase {
    ExtendedKalmanFilterParams ekf_params; ///< EKF-specific parameters
};
