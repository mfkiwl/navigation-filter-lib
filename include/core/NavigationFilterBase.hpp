/**
 * @file NavigationFilterBase.hpp
 * @brief Base class for navigation filters
 *
 * Defines the core structure for navigation filters with four key components:
 * 1. Strapdown inertial navigation update
 * 2. State prediction
 * 3. Measurement update
 * 4. Error correction
 *
 * @author peanut-nav
 * @date Created: 2025-07-22
 * @last Modified: 2025-07-22
 * @version 0.1
 */

#pragma once
#include "NavigationParams.hpp"
#include <memory>

/**
 * @brief Base class for navigation filters
 * 
 * Provides the fundamental structure for navigation filters with
 * clear separation of the four core components.
 */
class NavigationFilterBase {
public:
    virtual ~NavigationFilterBase() = default;
    
    /**
     * @brief Initialize the navigation filter
     * 
     * @param params Navigation parameters
     * @param state Initial navigation state
     */
    virtual void initialize(const NavParamsBase& params, 
                           NavigationState& state) = 0;
    
    // ================== Core Components ==================
    
    /**
     * @brief Strapdown inertial navigation update
     * 
     * Updates attitude, velocity, and position using IMU measurements.
     * 
     * @param imu IMU data for the current time step
     */
    virtual void updateStrapdown(const IMUData& imu) = 0;
    
    /**
     * @brief State prediction step
     * 
     * Predicts the next state based on the current state and system model.
     */
    virtual void predictState() = 0;
    
    /**
     * @brief Measurement update step
     * 
     * Updates the state estimate using external measurements (e.g., GPS).
     * 
     * @param measurement External measurement data
     */
    virtual void updateMeasurement(const GPSData& measurement) = 0;
    
    /**
     * @brief Error correction step
     * 
     * Corrects the navigation state based on estimated errors.
     */
    virtual void correctErrors() = 0;
    
    // ================== State Management ==================
    
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
     */
    virtual bool isMeasurementStep() const = 0;
    
    /**
     * @brief Get the filter type
     */
    virtual std::string getType() const = 0;
    
    /**
     * @brief Get current time index
     */
    virtual int getCurrentIndex() const = 0;
};

/**
 * @brief Create a navigation filter instance
 * 
 * @param type Filter type (KF, EKF, UKF)
 * @return std::unique_ptr<NavigationFilterBase> Filter instance
 */
std::unique_ptr<NavigationFilterBase> create_navigation_filter(FilterType type);
