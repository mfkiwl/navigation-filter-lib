/**
 * @file test_navigation_core.cpp
 * @brief Legacy unit tests for NavigationCore functionality
 * 
 * Contains tests for core navigation algorithms including:
 * - Attitude update
 * - Velocity and position update
 * - Euler angle calculation
 * - Full navigation run (disabled)
 * 
 * NOTE: These tests were developed against previous versions of the NavigationCore
 * implementation and are no longer compatible with the current codebase.
 * Preserved for historical reference only.
 * 
 * @author peanut-nav
 * @date Created: 2025-07-22
 * @last Modified: 2025-07-22 (marked deprecated)
 * @version 0.1 (deprecated)
 */

#include <gtest/gtest.h>
#include "NavigationCore.hpp"
#include "DataLoader.hpp"
#include "SystemInitializer.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <iostream>

// Test fixture for NavigationCore tests (legacy implementation)
class NavigationCoreTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Load test data (legacy format)
        const int IMUrate = 200;
        const int simTime = 10; // 10-second simulation
        DataLoader::loadData("data", IMUrate, simTime, imu, gps, track);
        
        // Initialize system (legacy parameters)
        SystemInitializer::initializeSystem(IMUrate, 20, simTime, "data", params, state, kalman);
    }
    
    // Legacy data structures
    IMUData imu;
    GPSData gps;
    TrajectoryData track;
    NavigationParams params;
    NavigationState state;
    KalmanFilterParams kalman;
};

// Test attitude update algorithm (legacy implementation)
TEST_F(NavigationCoreTest, AttitudeUpdate) {
    // Initial conditions
    Eigen::Vector3d wtb_b(0.01, 0.02, 0.03);
    Eigen::Vector3d wit_b(0.001, 0.002, 0.003);
    Eigen::Vector4d quat = state.Quaternion;
    Eigen::Matrix3d CbtM = state.CbtM;
    
    // Execute attitude update (legacy method)
    NavigationCore::updateAttitude(wtb_b, wit_b, quat, 200, CbtM);
    
    // Verify quaternion normalization (legacy check)
    EXPECT_NEAR(quat.norm(), 1.0, 1e-12);
    
    // Verify DCM orthogonality (legacy check)
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    EXPECT_TRUE((CbtM * CbtM.transpose()).isApprox(I, 1e-6));
}

// Test velocity and position update algorithm (legacy implementation)
TEST_F(NavigationCoreTest, VelocityPositionUpdate) {
    // Initial conditions
    Eigen::Vector3d f_INSt(0.1, 0.2, 9.8);
    Eigen::Vector3d V_prev(10.0, 5.0, 0.0);
    double Lat_prev = 30.0;
    double Lon_prev = 120.0;
    double h_prev = 50.0;
    Eigen::Vector3d wie_n(0, 7.292115e-5*std::cos(30*M_PI/180), 
                         7.292115e-5*std::sin(30*M_PI/180));
    Eigen::Vector3d wet_t(-5.0/6378135, 10.0/6378135, 
                          10.0*std::tan(30*M_PI/180)/6378135);
    double g = 9.7803;
    double Re = 6378135.0;
    double e = 1/298.257;
    
    // Execute update (legacy method)
    Eigen::Vector3d V_new;
    double Lat_new, Lon_new, h_new, Rx, Ry;
    NavigationCore::updateVelocityPosition(f_INSt, V_prev, Lat_prev, Lon_prev, h_prev,
                                         wie_n, wet_t, g, 200, Re, e, 
                                         Re/(1-e*std::sin(30*M_PI/180)*std::sin(30*M_PI/180)), 
                                         Re/(1+2*e-3*e*std::sin(30*M_PI/180)*std::sin(30*M_PI/180)),
                                         V_new, Lat_new, Lon_new, h_new, Rx, Ry);
    
    // Validate velocity update (legacy expectations)
    EXPECT_GT(V_new.norm(), 0.0);
    
    // Validate position update (legacy expectations)
    EXPECT_NEAR(Lat_new, 30.0, 0.01);
    EXPECT_NEAR(Lon_new, 120.0, 0.01);
    EXPECT_NEAR(h_new, 50.0, 0.01);
}

// Test Euler angle calculation from DCM (legacy implementation)
TEST_F(NavigationCoreTest, EulerAnglesCalculation) {
    // Create DCM with specific angles (30° pitch, 45° roll, 45° yaw)
    Eigen::Matrix3d CbtM;
    
    // Precomputed values (legacy reference)
    CbtM << 0.8535533905932737, -0.14644660940672627, 0.49999999999999994,
            0.49999999999999994, 0.49999999999999994, -0.7071067811865475,
            -0.14644660940672627, 0.8535533905932737, 0.49999999999999994;
    
    // Calculate Euler angles (legacy method)
    double pitch, roll, yaw;
    NavigationCore::calculateEulerAngles(CbtM, pitch, roll, yaw);
    
    // Output results (legacy debug)
    std::cout << "Calculated Euler angles: "
              << "Pitch = " << pitch << "°, "
              << "Roll = " << roll << "°, "
              << "Yaw = " << yaw << "°" << std::endl;
    
    // Validate against expected values (legacy)
    EXPECT_NEAR(pitch, 30.0, 0.001);
    EXPECT_NEAR(roll, 45.0, 0.001);
    EXPECT_NEAR(yaw, 45.0, 0.001);
}

// Full navigation run test (disabled - legacy implementation)
TEST_F(NavigationCoreTest, DISABLED_FullNavigationRun) {
    // Save initial state
    NavigationState initialState = state;
    
    // Execute navigation algorithm (legacy method)
    NavigationCore::runNavigation(imu, gps, params, state, kalman, 200, 20);
    
    // Validate state changes (legacy expectations)
    EXPECT_NE(state.Latitude[0], state.Latitude.back());
    EXPECT_NE(state.Longitude[0], state.Longitude.back());
    
    // Validate attitude changes (legacy expectations)
    EXPECT_NE(state.Pitch[0], state.Pitch.back());
    EXPECT_NE(state.Roll[0], state.Roll.back());
    EXPECT_NE(state.Yaw[0], state.Yaw.back());
    
    // Validate Kalman filter execution (legacy expectations)
    EXPECT_GT(kalman.N_kalman, 2);
}

// Test runner main function (legacy)
int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
