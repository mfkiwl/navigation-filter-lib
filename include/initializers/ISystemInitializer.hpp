/**
 * @file ISystemInitializer.hpp
 * @brief Interface for navigation system initialization
 *
 * Defines the interface for initializing navigation systems.
 *
 * @author peanut-nav
 * @date Created: 2025-07-22
 * @last Modified: 2025-08-07
 * @version 0.3.2
 */

#pragma once
#include "../params/NavParamsBase.hpp"

/**
 * @brief System initializer interface
 * 
 * Provides a common interface for initializing different types of navigation systems.
 */
class ISystemInitializer {
public:
    virtual ~ISystemInitializer() = default;
    
    /**
     * @brief Initialize navigation parameters
     * 
     * @param base_params Base navigation parameters to be initialized
     * @param dataDir Directory containing initialization data files
     */
    virtual void initialize_params(NavParamsBase& base_params, 
                                  const std::string& dataDir) = 0;
    
    /**
     * @brief Initialize navigation state
     * 
     * @param state Navigation state to be initialized
     * @param totalPoints Total number of data points
     */
    virtual void initialize_state(NavigationState& state, 
                                 int totalPoints) = 0;
    
    /**
     * @brief Initialize Kalman filter parameters
     * 
     * @param kalman Kalman filter parameters to be initialized
     * @param totalPoints Total number of data points
     */
    virtual void initialize_kalman(NavParamsBase& base_params, 
                                  int totalPoints) = 0;
};
