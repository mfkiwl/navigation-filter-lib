/**
 * @file NavigationParams.hpp
 * @brief Aggregation header for navigation parameters
 *
 * @author peanut-nav
 * @date Created: 2025-07-22
 * @last Modified: 2025-07-22
 * @version 0.1
 */

#pragma once
#include <memory>
#include "params/NavParamsBase.hpp"
#include "params/KfParams.hpp"

/**
 * @brief Filter type enumeration
 */
enum class FilterType {
    KF,   ///< Kalman Filter
    EKF,  ///< Extended Kalman Filter
    UKF   ///< Unscented Kalman Filter (reserved)
};

/**
 * @brief Create navigation parameters based on filter type
 * 
 * @param type Filter type (KF, EKF, UKF)
 * @param IMUrate IMU sampling rate (Hz)
 * @param GPSrate GPS sampling rate (Hz)
 * @return std::unique_ptr<NavParamsBase> Pointer to created parameters
 */
inline std::unique_ptr<NavParamsBase> create_nav_params(
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
        // 预留其他滤波器类型的创建逻辑
        case FilterType::EKF:
        case FilterType::UKF:
        default:
            throw std::runtime_error("Unsupported filter type");
    }
}
