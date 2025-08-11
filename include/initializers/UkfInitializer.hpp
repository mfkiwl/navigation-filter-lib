/**
 * @file UkfInitializer.hpp
 * @brief UKF navigation system initializer
 *
 * Handles initialization of UKF parameters, navigation state, 
 * and Kalman filter components.
 *
 * @author peanut-nav
 * @date Created: 2025-08-10
 * @last Modified: 2025-08-10
 * @version 0.3.3
 */

#pragma once
#include "ISystemInitializer.hpp"
#include "../params/UkfParams.hpp"

/**
 * @brief Initializes UKF-based navigation system components
 * 
 * Responsible for:
 * - Loading initial navigation state from files
 * - Configuring Earth model parameters
 * - Setting initial navigation state vectors
 * - Initializing Kalman filter parameters
 */
class UkfInitializer : public ISystemInitializer {
public:
    /**
     * @brief Constructor
     * @param IMUrate IMU sampling frequency [Hz]
     * @param GPSrate GPS sampling frequency [Hz]
     * @param simTime Total simulation time [s]
     */
    UkfInitializer(int IMUrate, int GPSrate, int simTime);
    
    /**
     * @brief Initializes navigation parameters
     * @param base_params Parameter container to populate
     * @param dataDir Directory containing initialization data
     */
    void initialize_params(NavParamsBase& base_params, 
                          const std::string& dataDir) override;
    
    /**
     * @brief Initializes navigation state vectors
     * @param state Navigation state container to initialize
     * @param totalPoints Number of trajectory points
     */
    void initialize_state(NavigationState& state, 
                         int totalPoints) override;

    /**
     * @brief Initializes Kalman filter components
     * @param base_params Parameter container to populate
     * @param totalPoints Total trajectory points for buffer sizing
     */
    void initialize_kalman(NavParamsBase& base_params, 
                          int totalPoints) override;

private:
    int IMUrate_;       ///< IMU sampling rate [Hz]
    int GPSrate_;       ///< GPS sampling rate [Hz]
    int simTime_;       ///< Total simulation time [s]
    
    // Initial navigation state (loaded from file)
    double lat_rad_ = 0.0;      ///< Initial latitude [rad]
    double lon_rad_ = 0.0;      ///< Initial longitude [rad]
    double h0_ = 0.0;           ///< Initial altitude [m]
    double v_east_ = 0.0;       ///< East velocity [m/s]
    double v_north_ = 0.0;      ///< North velocity [m/s]
    double v_up_ = 0.0;         ///< Up velocity [m/s]
    double fai_rad_ = 0.0;      ///< Initial yaw [rad]
    double st_rad_ = 0.0;       ///< Initial pitch [rad]
    double r_rad_ = 0.0;        ///< Initial roll [rad]
    
    /**
     * @brief Loads initial navigation state from file
     * @param filePath Path to navigation data file
     */
    void loadInitialNavData(const std::string& filePath);
    
    /**
     * @brief Configures measurement noise covariance matrix
     * @return 6x6 diagonal measurement noise matrix
     */
    Eigen::MatrixXd setupMeasurementNoise() const;
    
    /**
     * @brief Configures process noise covariance matrix
     * @return 6x6 diagonal process noise matrix
     */
    Eigen::MatrixXd setupProcessNoise() const;
    
    /**
     * @brief Initializes state error covariance matrix
     * @param lat Initial latitude [degrees]
     * @return 15x15 diagonal covariance matrix
     */
    Eigen::MatrixXd initializeCovarianceMatrix(double lat) const;
};
