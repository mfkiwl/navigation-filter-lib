/**
 * @file EkfInitializer.hpp
 * @brief EKF system initializer
 *
 * @author peanut-nav
 * @date Created: 2025-07-22
 * @last Modified: 2025-08-07
 * @version 0.3.2
 */

#pragma once
#include "ISystemInitializer.hpp"
#include "../params/EkfParams.hpp"
#include "MathUtils.hpp"

class EkfInitializer : public ISystemInitializer {
public:
    EkfInitializer(int IMUrate, int GPSrate, int simTime);
    
    void initialize_params(NavParamsBase& base_params, 
                          const std::string& dataDir) override;
    
    void initialize_state(NavigationState& state, 
                         int totalPoints) override;

    void initialize_kalman(NavParamsBase& base_params, 
                          int totalPoints) override;

private:
    // Configuration
    int IMUrate_;    ///< IMU sampling rate (Hz)
    int GPSrate_;    ///< GPS sampling rate (Hz)
    int simTime_;    ///< Total simulation time (seconds)
    
    // Initial navigation data
    double lat_rad_ = 0.0;    ///< Initial latitude (radians)
    double lon_rad_ = 0.0;    ///< Initial longitude (radians)
    double h0_ = 0.0;         ///< Initial altitude (meters)
    double v_east_ = 0.0;     ///< Initial east velocity (m/s)
    double v_north_ = 0.0;    ///< Initial north velocity (m/s)
    double v_up_ = 0.0;       ///< Initial up velocity (m/s)
    double fai_rad_ = 0.0;    ///< Initial yaw angle (radians)
    double st_rad_ = 0.0;     ///< Initial pitch angle (radians)
    double r_rad_ = 0.0;      ///< Initial roll angle (radians)
    
    /**
     * @brief Load initial navigation data from file
     * 
     * @param filePath Path to navigation data file
     */
    void loadInitialNavData(const std::string& filePath);
    
    /**
     * @brief Set up measurement noise covariance matrix
     * @return 6x6 measurement noise covariance matrix
     */
    Eigen::MatrixXd setupMeasurementNoise() const;
    
    /**
     * @brief Set up process noise covariance matrix
     * @return 6x6 process noise covariance matrix
     */
    Eigen::MatrixXd setupProcessNoise() const;
    
    /**
     * @brief Initialize state error covariance matrix
     * @param lat Initial latitude (degrees)
     * @param Re Earth radius (meters)
     * @return 15x15 initial covariance matrix
     */
    Eigen::MatrixXd initializeCovarianceMatrix(double lat) const;
};
