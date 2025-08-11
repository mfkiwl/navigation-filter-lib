/**
 * @file UkfParams.hpp
 * @brief Parameters for Unscented Kalman Filter based navigation
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
 * @brief UKF-specific parameters and state
 */
struct UnscentedKalmanFilterParams {
    // Kalman filter parameters
    int Nk = 15;            // 状态维度
    int Lk = 6;             // 噪声维度（陀螺+加计）
    int Mk = 6;             // 量测维度（Vx,Vy,Vz,lat,lon,h）
    int N  = 0;             // IMU/GPS 步比
    int M  = 0;             // 卡尔曼点个数
    double T = 0.0;         // 量测周期
    int N_kalman = 2;    ///< Update counter

    // UKF 权重
    double lambda = 0.0;          // = 3 - Nk
    Eigen::VectorXd w;            // 2Nk+1
    
    // 协方差/噪声
    Eigen::MatrixXd P;            // 15x15
    Eigen::MatrixXd R;            // 6x6
    Eigen::MatrixXd Q0;           // 6x6 连续噪声（陀螺/加计）
    Eigen::MatrixXd Q;            // 15x15 离散噪声（按系列展开求）
    
    // State and measurement vectors
    Eigen::MatrixXd X;  ///< State vector
    Eigen::MatrixXd Z;  ///< Measurement vector

    // Prediction states
    Eigen::VectorXd X_pred;  ///< Predicted state vector
    Eigen::MatrixXd P_pred;  ///< Predicted covariance matrix
    Eigen::MatrixXd X_prop;   // Nk x (2Nk+1)  传播后的 σ 点

    // 端点平均所需的上一次 f0
    Eigen::Matrix<double,15,15> f0;

    Eigen::MatrixXd Hz;           // 6x15

    Eigen::MatrixXd Fai;          // 15x15 (Φ)
    Eigen::MatrixXd Gf;           // 15x15 (记录到RTS)
    
    // Result storage
    Eigen::MatrixXd Xsave;             ///< Saved state estimates
    Eigen::MatrixXd P_mean_square;     ///< Mean square covariance
};

/**
 * @brief Unscented Kalman Filter specific parameters
 */
struct UkfParams : public NavParamsBase {
    UnscentedKalmanFilterParams ukf_params; ///< UKF-specific parameters
};
