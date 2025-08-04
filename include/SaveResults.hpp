/**
 * @file SaveResults.hpp
 * @brief Navigation results saving utilities
 *
 * Provides functionality to save navigation results to output files.
 *
 * @author peanut-nav
 * @date Created: 2025-07-22
 * @last Modified: 2025-07-22
 * @version 0.1
 */

#pragma once
#include "NavigationParams.hpp"
#include "DataLoader.hpp"
#include <fstream>
#include <iomanip>
#include <string>  // 添加 string 头文件

/**
 * @brief Navigation results saving class
 * 
 * Handles the saving of navigation results to output files.
 */
class SaveResults {
public:
    /**
     * @brief Save navigation results to a file
     * 
     * @param state Navigation state containing the results
     * @param imu IMU data used for indexing
     * @param prefix Optional prefix for filename (default: "KF")
     */
    static void saveNavigationResults(const NavigationState& state, 
                                     const IMUData& imu,
                                     const std::string& prefix = "KF");
};
