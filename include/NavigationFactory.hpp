/**
 * @file NavigationFactory.hpp
 * @brief Factory for creating navigation system components
 *
 * Provides unified creation of parameters, initializers and navigation processors
 *
 * @author peanut-nav
 * @date Created: 2025-07-22
 * @last Modified: 2025-08-20
 * @version 0.4.0
 */

#pragma once

#include "params/KfParams.hpp"
#include "params/EkfParams.hpp"
#include "params/UkfParams.hpp"

#include "initializers/KfInitializer.hpp"
#include "initializers/EkfInitializer.hpp"
#include "initializers/UkfInitializer.hpp"

#include "core/KfNavigation.hpp"
#include "core/EkfNavigation.hpp"
#include "core/UkfNavigation.hpp"

#include "FilterType.hpp"
#include <memory>
#include <stdexcept>

/**
 * @brief Navigation system factory
 */
class NavigationFactory {
public:
    /**
     * @brief Create navigation parameters based on filter type
     * 
     * @param type Filter type (KF, EKF, UKF)
     * @param IMUrate IMU sampling rate (Hz)
     * @param GPSrate GPS sampling rate (Hz)
     * @return std::unique_ptr<NavParamsBase> Pointer to created parameters
     */
    static std::unique_ptr<NavParamsBase> create_params(
        FilterType type,
        double IMUrate,
        double GPSrate
    ) {
        switch (type) {
            case FilterType::KF: {
                auto params = std::make_unique<KfParams>();
                params->imu_rate = IMUrate;
                params->gps_rate = GPSrate;
                return params;
            }
            case FilterType::EKF: {
                auto params = std::make_unique<EkfParams>();
                params->imu_rate = IMUrate;
                params->gps_rate = GPSrate;
                return params;
            }
            case FilterType::UKF: {
                auto params = std::make_unique<UkfParams>();
                params->imu_rate = IMUrate;
                params->gps_rate = GPSrate;
                return params;
            }
            default:
                throw std::runtime_error("Unsupported filter type");
        }
    }

    /**
     * @brief Create system initializer based on filter type
     * 
     * @param type Filter type
     * @param IMUrate IMU sampling rate (Hz)
     * @param GPSrate GPS sampling rate (Hz)
     * @param simTime Simulation time (seconds)
     * @return std::unique_ptr<ISystemInitializer> Pointer to created initializer
     */
    static std::unique_ptr<ISystemInitializer> create_initializer(
        FilterType type,
        int IMUrate,
        int GPSrate,
        double simTime
    ) {
        switch (type) {
            case FilterType::KF:
                return std::make_unique<KfInitializer>(IMUrate, GPSrate, simTime);
            case FilterType::EKF:
                return std::make_unique<EkfInitializer>(IMUrate, GPSrate, simTime);
            case FilterType::UKF:
                return std::make_unique<UkfInitializer>(IMUrate, GPSrate, simTime);
            default:
                throw std::runtime_error("Unsupported filter type");
        }
    }

    /**
     * @brief Create navigation processor based on filter type
     * 
     * @param type Filter type
     * @return std::unique_ptr<NavigationBase> Pointer to created navigation processor
     */
    static std::unique_ptr<NavigationBase> create_navigation(FilterType type) {
        switch (type) {
            case FilterType::KF:
                return std::make_unique<KalmanFilterNavigation>();
            case FilterType::EKF:
                return std::make_unique<EkfNavigation>();
            case FilterType::UKF:
                return std::make_unique<UkfNavigation>();
            default:
                throw std::runtime_error("Unsupported filter type");
        }
    }
};
