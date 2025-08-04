/**
 * @file NavigationBase.hpp
 * @brief Base class for navigation systems
 *
 * Defines the core interface for navigation systems with clear separation of
 * strapdown inertial navigation, state prediction, measurement update, and error correction.
 *
 * @author peanut-nav
 * @date Created: 2025-07-22
 * @last Modified: 2025-07-22
 * @version 0.3.0
 */

#pragma once
#include "NavigationParams.hpp"
#include <Eigen/Dense>

class NavigationBase {
public:
    virtual ~NavigationBase() = default;
    
    /**
     * @brief Initialize the navigation system
     * 
     * @param params Navigation parameters
     * @param state Initial navigation state
     */
    virtual void initialize(const NavParamsBase& params, 
                           NavigationState& state) = 0;
    
    /**
     * @brief Perform strapdown inertial navigation update
     * 
     * @param imu IMU data for the current time step
     * @param i Current time index
     */
    virtual void updateStrapdown(const IMUData& imu, int i) = 0;
    
    /**
     * @brief Perform state prediction step
     * 
     * @param i Current time index
     */
    virtual void predictState(int i) = 0;
    
    /**
     * @brief Perform measurement update step
     * 
     * @param gps GPS data for the current time step
     * @param i Current time index
     */
    virtual void updateMeasurement(const GPSData& gps, int i) = 0;
    
    /**
     * @brief Perform error correction step
     * 
     * @param i Current time index
     */
    virtual void correctErrors(int i) = 0;
    
    /**
     * @brief Get the current navigation state
     */
    virtual NavigationState& getState() = 0;
    
    /**
     * @brief Advance to the next time step
     */
    virtual void advance() = 0;
    
    /**
     * @brief Check if at measurement update step
     * 
     * @param i Current time index
     */
    virtual bool isMeasurementStep(int i) const = 0;
    
    /**
     * @brief Run the navigation algorithm
     * 
     * @param imu IMU data
     * @param gps GPS data
     */
    virtual void run(const IMUData& imu, const GPSData& gps) = 0;
};
