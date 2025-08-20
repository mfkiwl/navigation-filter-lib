// FilterType.hpp
#pragma once

/**
 * @brief Filter type enumeration
 */
enum class FilterType {
    KF,   ///< Kalman Filter
    EKF,  ///< Extended Kalman Filter
    UKF   ///< Unscented Kalman Filter
};
