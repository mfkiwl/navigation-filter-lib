/**
 * @file test_data_loader.cpp
 * @brief Legacy unit tests for DataLoader functionality
 * 
 * Contains tests for data loading and system initialization:
 * - Loading real IMU/GPS/trajectory data
 * - System initialization with real data
 * - C++/Matlab consistency validation
 * 
 * NOTE: These tests were developed against previous versions of the DataLoader
 * and SystemInitializer implementations and are no longer compatible with the
 * current codebase. Preserved for historical reference only.
 * 
 * @author peanut-nav
 * @date Created: 2025-07-22
 * @last Modified: 2025-07-22 (marked deprecated)
 * @version 0.1 (deprecated)
 */

#include <gtest/gtest.h>
#include "DataLoader.hpp"
#include "SystemInitializer.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <cmath>
#include <iomanip>  // For precision control

namespace fs = std::filesystem;

// Test fixture for real data tests (legacy implementation)
class RealDataTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Set path to real data directory (legacy location)
        realDataDir = fs::absolute(fs::path("data"));
        
        // Load Matlab reference data (legacy format)
        loadMatlabReferenceData();
    }
    
    // Load Matlab reference data (legacy method)
    void loadMatlabReferenceData() {
        std::ifstream refFile("matlab_reference.txt");
        if (refFile.is_open()) {
            refFile >> matlab_lat;
            refFile >> matlab_lon;
            refFile >> matlab_alt;
            refFile >> matlab_pitch;
            refFile >> matlab_roll;
            refFile >> matlab_yaw;
            refFile.close();
            std::cout << "Successfully loaded Matlab reference data" << std::endl;
        } else {
            ADD_FAILURE() << "Failed to open matlab_reference.txt file";
        }
    }
    
    // Legacy data paths and reference values
    fs::path realDataDir;
    double matlab_lat = 0.0;
    double matlab_lon = 0.0;
    double matlab_alt = 0.0;
    double matlab_pitch = 0.0;
    double matlab_roll = 0.0;
    double matlab_yaw = 0.0;
};

// Test loading real IMU data (legacy implementation)
TEST_F(RealDataTest, LoadRealIMUData) {
    IMUData imu;
    GPSData gps;
    TrajectoryData track;
    
    const int IMUrate = 200;
    const int simTime = 10; // 10-second simulation
    
    // Load data using legacy method
    DataLoader::loadData(realDataDir.string(), IMUrate, simTime, imu, gps, track);
    
    // Validate data loading (legacy checks)
    EXPECT_GT(imu.index.size(), 0) << "No IMU data loaded";
    EXPECT_GT(gps.time.size(), 0) << "No GPS data loaded";
    EXPECT_GT(track.time.size(), 0) << "No trajectory data loaded";
    
    // Print basic information (legacy debug)
    std::cout << "\nData loading results:" << std::endl;
    std::cout << "IMU points: " << imu.index.size() << " (expected: " << (simTime * IMUrate + 1) << ")" << std::endl;
    std::cout << "GPS points: " << gps.time.size() << std::endl;
    std::cout << "Trajectory points: " << track.time.size() << std::endl;
    
    // Output first IMU data point (legacy debug)
    if (imu.index.size() > 0) {
        std::cout << "First IMU data: " 
                  << "gx=" << imu.gx[0] << ", "
                  << "gy=" << imu.gy[0] << ", "
                  << "gz=" << imu.gz[0] << ", "
                  << "ax=" << imu.ax[0] << ", "
                  << "ay=" << imu.ay[0] << ", "
                  << "az=" << imu.az[0] << std::endl;
    }
}

// Test system initialization with real data (legacy implementation)
TEST_F(RealDataTest, SystemInitializationWithRealData) {
    NavigationParams params;
    NavigationState state;
    KalmanFilterParams kalman;
    
    const int IMUrate = 200;
    const int GPSrate = 20;
    const int simTime = 10; // 10-second simulation
    
    // Initialize system using legacy method
    SystemInitializer::initializeSystem(IMUrate, GPSrate, simTime, 
                                      realDataDir.string(), params, state, kalman);
    
    // Print C++ initialization results (legacy debug)
    std::cout << "\nC++ initialization results:" << std::endl;
    std::cout << "Latitude: " << std::setprecision(15) << state.Latitude[0] << "°" << std::endl;
    std::cout << "Longitude: " << std::setprecision(15) << state.Longitude[0] << "°" << std::endl;
    std::cout << "Altitude: " << std::setprecision(15) << state.Altitude[0] << " m" << std::endl;
    std::cout << "Pitch: " << std::setprecision(15) << state.Pitch[0] << "°" << std::endl;
    std::cout << "Roll: " << std::setprecision(15) << state.Roll[0] << "°" << std::endl;
    std::cout << "Yaw: " << std::setprecision(15) << state.Yaw[0] << "°" << std::endl;
    
    // Print Matlab reference values (legacy debug)
    std::cout << "\nMatlab reference values:" << std::endl;
    std::cout << "Latitude: " << std::setprecision(15) << matlab_lat << "°" << std::endl;
    std::cout << "Longitude: " << std::setprecision(15) << matlab_lon << "°" << std::endl;
    std::cout << "Altitude: " << std::setprecision(15) << matlab_alt << " m" << std::endl;
    std::cout << "Pitch: " << std::setprecision(15) << matlab_pitch << "°" << std::endl;
    std::cout << "Roll: " << std::setprecision(15) << matlab_roll << "°" << std::endl;
    std::cout << "Yaw: " << std::setprecision(15) << matlab_yaw << "°" << std::endl;
    
    // Compare C++ and Matlab results (legacy validation)
    const double tolerance = 1e-8; // High-precision comparison
    
    // Position comparison
    EXPECT_NEAR(state.Latitude[0], matlab_lat, tolerance) 
        << "Latitude mismatch";
    EXPECT_NEAR(state.Longitude[0], matlab_lon, tolerance)
        << "Longitude mismatch";
    EXPECT_NEAR(state.Altitude[0], matlab_alt, tolerance)
        << "Altitude mismatch";
    
    // Attitude comparison
    EXPECT_NEAR(state.Pitch[0], matlab_pitch, tolerance)
        << "Pitch angle mismatch";
    EXPECT_NEAR(state.Roll[0], matlab_roll, tolerance)
        << "Roll angle mismatch";
    
    // Handle yaw angle wrap-around (0-360°)
    double cpp_yaw = state.Yaw[0];
    double mat_yaw = matlab_yaw;
    
    // Adjust for 360° wrap-around
    if (std::abs(cpp_yaw - mat_yaw) > 180.0) {
        if (cpp_yaw > mat_yaw) {
            mat_yaw += 360.0;
        } else {
            cpp_yaw += 360.0;
        }
    }
    
    EXPECT_NEAR(cpp_yaw, mat_yaw, tolerance)
        << "Yaw angle mismatch (adjusted: C++=" << cpp_yaw << ", Matlab=" << mat_yaw << ")";
}

// Test runner main function with instructions (legacy)
int main(int argc, char **argv) {
    std::cout << "=== C++/Matlab Consistency Verification Test ===" << std::endl;
    std::cout << "Prerequisites:" << std::endl;
    std::cout << "1. Matlab script has generated reference data" << std::endl;
    std::cout << "2. matlab_reference.txt exists in current directory" << std::endl;
    std::cout << "3. Real data files are in 'data' directory" << std::endl;
    
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
