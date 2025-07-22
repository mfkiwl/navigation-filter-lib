/**
 * @file test_math_utils.cpp
 * @brief Legacy unit tests for MathUtils functionality
 * 
 * Contains tests for mathematical utility functions including:
 * - Degree to radian conversion
 * - Direction Cosine Matrix (DCM) calculation
 * - Euler angle to quaternion conversion
 * 
 * NOTE: These tests were developed against previous versions of the MathUtils
 * implementation and are no longer compatible with the current codebase.
 * Preserved for historical reference only.
 * 
 * @author peanut-nav
 * @date Created: 2025-07-22
 * @last Modified: 2025-07-22 (marked deprecated)
 * @version 0.1 (deprecated)
 */

#include <gtest/gtest.h>
#include <cmath>
#include <iostream>
#include "MathUtils.hpp"

using namespace NavigationUtils;

constexpr double EPSILON = 1e-15;  ///< Tolerance for floating-point comparisons

// Test degree to radian conversion
TEST(MathUtilsTest, Deg2Rad) {
    EXPECT_NEAR(deg2rad(0.0), 0.0, EPSILON);
    EXPECT_NEAR(deg2rad(180.0), M_PI, EPSILON);
    EXPECT_NEAR(deg2rad(90.0), M_PI/2, EPSILON);
    EXPECT_NEAR(deg2rad(360.0), 2*M_PI, EPSILON);
}

// Test Direction Cosine Matrix (DCM) calculation for combined angles
TEST(MathUtilsTest, Cbt) {
    // Test combined angles (30° roll, 45° pitch, 60° yaw)
    auto dcm_combo = bodyToNavigationDCM(30.0, 45.0, 60.0);
    
    // Expected output (legacy reference)
    Eigen::Matrix3d expected_combo;
    expected_combo << 0.047367172745376,   0.789149130992431,  -0.612372435695795,
                     -0.750000000000000,   0.433012701892219,   0.500000000000000,
                      0.659739608441171,   0.435595740399158,   0.612372435695795;
    
    // Debug output (legacy implementation)
    std::cout << "Computed DCM:\n" << dcm_combo << std::endl;
    std::cout << "Expected DCM:\n" << expected_combo << std::endl;
    
    // Element-wise comparison (legacy validation)
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            EXPECT_NEAR(dcm_combo(i, j), expected_combo(i, j), 1e-12)
                << "Element (" << i << "," << j << ") mismatch";
        }
    }
}

// Test Euler angle to quaternion conversion
TEST(MathUtilsTest, Zitai4) {
    // Test combined angles (30° roll, 45° pitch, 60° yaw)
    auto quat_combo = eulerToQuaternion(30.0, 45.0, 60.0);
    
    // Expected output (legacy reference)
    Eigen::Vector4d expected_combo(
        0.723317411364712,
        0.022260026714734,
        0.439679739540910,
        0.531975695182167
    );
    
    // Debug output (legacy implementation)
    std::cout << "Computed Quat: " << quat_combo.transpose() << std::endl;
    std::cout << "Expected Quat: " << expected_combo.transpose() << std::endl;
    
    // Element-wise comparison (legacy validation)
    for (int i = 0; i < 4; i++) {
        EXPECT_NEAR(quat_combo[i], expected_combo[i], 1e-12)
            << "Index " << i << " mismatch";
    }
    
    // Verify quaternion normalization (legacy check)
    double norm = quat_combo.norm();
    EXPECT_NEAR(norm, 1.0, EPSILON);
}

// Test runner main function (legacy)
int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
