/**
 * @file UkfParams.hpp
 * @brief Parameters for Unscented Kalman Filter (UKF) based navigation system
 * 
 * Defines data structures for UKF parameters, state vectors, and covariance matrices.
 * 
 * @author peanut-nav
 * @date Created: 2025-08-10
 * @last Modified: 2025-08-10
 * @version 0.3.3
 */

#pragma once
#include "NavParamsBase.hpp"
#include <Eigen/Dense>

/**
 * @brief Parameters and state variables for Unscented Kalman Filter
 */
struct UnscentedKalmanFilterParams {
    // Kalman filter dimensions
    int Nk = 15;            ///< State vector dimension (attitude(3) + velocity(3) + position(3) + biases(6))
    int Lk = 6;             ///< Process noise dimension (gyro(3) + accelerometer(3))
    int Mk = 6;             ///< Measurement dimension (velocity(3) + position(3))
    int N  = 0;             ///< IMU/GPS update ratio (IMU_steps / GPS_steps)
    int M  = 0;             ///< Number of Kalman states (buffer size)
    double T = 0.0;         ///< Measurement update period [s]
    int N_kalman = 2;       ///< Kalman update counter

    // UKF sigma point weights
    double lambda = 0.0;    ///< Scaling parameter (3 - Nk)
    Eigen::VectorXd w;      ///< Weight vector (length: 2Nk+1)

    // Covariance matrices
    Eigen::MatrixXd P;      ///< State error covariance (15x15)
    Eigen::MatrixXd R;      ///< Measurement noise covariance (6x6)
    Eigen::MatrixXd Q0;     ///< Continuous process noise covariance (6x6)
    Eigen::MatrixXd Q;      ///< Discrete process noise covariance (15x15)

    // State and measurement
    Eigen::MatrixXd X;      ///< State vector matrix (15xM)
    Eigen::MatrixXd Z;      ///< Measurement vector matrix (6xM)

    // Prediction states
    Eigen::VectorXd X_pred; ///< Predicted state vector (15x1)
    Eigen::MatrixXd P_pred; ///< Predicted covariance matrix (15x15)
    Eigen::MatrixXd X_prop; ///< Propagated sigma points (Nk x (2Nk+1))

    // UKF intermediate matrices
    Eigen::Matrix<double,15,15> f0;  ///< Previous state function for endpoint averaging
    Eigen::MatrixXd Hz;              ///< Measurement matrix (6x15)
    Eigen::MatrixXd Fai;             ///< State transition matrix (Φ, 15x15)
    Eigen::MatrixXd Gf;              ///< Noise propagation matrix (for RTS smoothing)

    // Result storage
    Eigen::MatrixXd Xsave;           ///< Saved state estimates (15xM)
    Eigen::MatrixXd P_mean_square;   ///< Mean square covariance (15xM)
};

/**
 * @brief UKF-specific navigation parameters
 * 
 * Extends base navigation parameters with UKF-specific configuration
 */
struct UkfParams : public NavParamsBase {
    UnscentedKalmanFilterParams ukf_params; ///< UKF-specific parameters and state
};
