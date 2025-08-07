/**
 * @file RtsSmoother.hpp
 * @brief Implements Rauch-Tung-Striebel (RTS) smoothing for navigation systems
 *
 * This class provides backward-pass smoothing for Kalman filter-based navigation systems,
 * improving state estimates by incorporating future measurements. It stores filter history
 * during forward processing and performs smoothing offline.
 *
 * @author peanut-nav
 * @date Created: 2025-08-04
 * @last Modified: 2025-08-04
 * @version 0.3.0
 */

#pragma once
#include <vector>
#include <Eigen/Dense>
#include "../params/NavParamsBase.hpp"
#include "MathUtils.hpp"

class RtsSmoother {
public:
    /// History data structure for storing Kalman filter states
    struct FilterHistory {
        Eigen::VectorXd state;               ///< Posterior filter state
        Eigen::VectorXd predicted_state;     ///< Predicted state
        Eigen::MatrixXd covariance;          ///< Posterior covariance
        Eigen::MatrixXd predicted_covariance;///< Predicted covariance
        Eigen::MatrixXd transition_matrix;   ///< State transition matrix
        NavigationState nav_state;           ///< Navigation state snapshot
    };

    /// Result structure for RTS smoothing
    struct SmoothResult {
        std::vector<Eigen::VectorXd> smoothed_states;      ///< Smoothed state vectors
        std::vector<Eigen::MatrixXd> smoothed_covariances; ///< Smoothed covariance matrices
        std::vector<NavigationState> smoothed_nav_states;  ///< Smoothed navigation states
        std::vector<Eigen::Vector3d> velocity_smooth;      ///< Smoothed velocities [vE, vN, vU]
        std::vector<Eigen::Vector3d> position_smooth;      ///< Smoothed positions [lat, lon, alt]
        std::vector<Eigen::Vector3d> attitude_smooth;      ///< Smoothed attitudes [yaw, pitch, roll]
    };

    /**
     * @brief Add filter history item for smoothing
     * 
     * Stores Kalman filter state at each measurement update for later smoothing
     * 
     * @param history_item Contains filter state, covariance and navigation state
     */
    void addHistoryItem(const FilterHistory& history_item);
    
    /**
     * @brief Perform RTS smoothing on stored history
     * 
     * Executes backward pass smoothing algorithm to improve state estimates
     * 
     * @param process_noise Process noise matrix (Q)
     * @return SmoothResult containing smoothed states and covariances
     */
    SmoothResult smooth(const Eigen::MatrixXd& process_noise);
    
    /**
     * @brief Generate navigation solution from smoothing results
     * 
     * Formats smoothed states into a navigation solution matrix
     * 
     * @param result Smoothing results from smooth() method
     * @param totalPoints Total points in navigation solution
     * @return Matrix of smoothed navigation parameters
     */
    std::vector<std::vector<double>> generateSmoothedNavigation(const SmoothResult& result, int totalPoints);
    
    /**
     * @brief Post-process navigation using smoothing results
     * 
     * Re-runs navigation with smoothed corrections applied at measurement points
     * 
     * @param imu IMU data sequence
     * @param track Reference trajectory data
     * @param initial_state Initial navigation state
     * @param earth_params Earth model parameters
     * @param smooth_result Results from smoothing algorithm
     * @param gps_rate GPS measurement rate (Hz)
     * @param imu_rate IMU sampling rate (Hz)
     * @return Refined navigation state with smoothing corrections
     */
    NavigationState postProcessNavigation(
        const IMUData& imu,
        const TrajectoryData& track,
        const NavigationState& initial_state,
        const NavigationParams& earth_params,
        const SmoothResult& smooth_result,
        int gps_rate,
        int imu_rate);

private:
    std::vector<FilterHistory> history_;  ///< Storage for filter history data

    /**
     * @brief Compute misalignment DCM
     * 
     * Calculates direction cosine matrix from misalignment angles
     * 
     * @param E_err East misalignment angle (rad)
     * @param N_err North misalignment angle (rad)
     * @param U_err Up misalignment angle (rad)
     * @return Direction cosine matrix for misalignment correction
     */
    Eigen::Matrix3d computeCtn(double E_err, double N_err, double U_err) const;
    
    /**
     * @brief Compute body-to-navigation DCM
     * 
     * Calculates direction cosine matrix from Euler angles
     * 
     * @param yaw Yaw angle (degrees)
     * @param pitch Pitch angle (degrees)
     * @param roll Roll angle (degrees)
     * @return Direction cosine matrix (Cnb)
     */
    Eigen::Matrix3d computeCnb(double yaw, double pitch, double roll) const;
};
