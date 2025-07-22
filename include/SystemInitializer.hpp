/**
 * @file SystemInitializer.hpp
 * @brief System initialization utilities for navigation systems
 *
 * Provides functionality to initialize navigation parameters, state variables,
 * and Kalman filter settings based on simulation configuration and input data.
 *
 * @author peanut-nav
 * @date Created: 2025-07-22
 * @last Modified: 2025-07-22
 * @version 0.1
 */

#pragma once
#include "NavigationParams.hpp"
#include "MathUtils.hpp"

/**
 * @brief System initialization class
 * 
 * Handles the initialization of navigation parameters, state variables,
 * and Kalman filter settings based on input data and simulation configuration.
 */
class SystemInitializer {
public:
    /**
     * @brief Initialize the entire navigation system
     * 
     * @param IMUrate IMU sampling rate (Hz)
     * @param GPSrate GPS sampling rate (Hz)
     * @param simTime Total simulation time (seconds)
     * @param dataDir Directory containing initialization data files
     * @param[out] params Navigation parameters to be initialized
     * @param[out] state Navigation state to be initialized
     * @param[out] kalman Kalman filter parameters to be initialized
     */
    static void initializeSystem(int IMUrate, 
                                int GPSrate, 
                                int simTime,
                                const std::string& dataDir,
                                NavigationParams& params,
                                NavigationState& state,
                                KalmanFilterParams& kalman);
    
private:
    /**
     * @brief Load initial navigation data from file
     * 
     * @param filePath Path to navigation data file
     * @param[out] lat_rad Initial latitude (radians)
     * @param[out] lon_rad Initial longitude (radians)
     * @param[out] h0 Initial altitude (meters)
     * @param[out] v_east Initial east velocity (m/s)
     * @param[out] v_north Initial north velocity (m/s)
     * @param[out] v_up Initial up velocity (m/s)
     * @param[out] fai_rad Initial yaw angle (radians)
     * @param[out] st_rad Initial pitch angle (radians)
     * @param[out] r_rad Initial roll angle (radians)
     */
    static void loadInitialNavData(const std::string& filePath,
                                  double& lat_rad,
                                  double& lon_rad,
                                  double& h0,
                                  double& v_east,
                                  double& v_north,
                                  double& v_up,
                                  double& fai_rad,
                                  double& st_rad,
                                  double& r_rad);
    
    /**
     * @brief Set up measurement noise covariance matrix
     * @return 6x6 measurement noise covariance matrix
     */
    static Eigen::MatrixXd setupMeasurementNoise();
    
    /**
     * @brief Set up process noise covariance matrix
     * @return 6x6 process noise covariance matrix
     */
    static Eigen::MatrixXd setupProcessNoise();
    
    /**
     * @brief Initialize state error covariance matrix
     * @param lat Initial latitude (degrees)
     * @return 15x15 initial covariance matrix
     */
    static Eigen::MatrixXd initializeCovarianceMatrix(double lat);
};
