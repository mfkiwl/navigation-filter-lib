/**
 * @file main.cpp
 * @brief Main application for integrated navigation system
 *
 * Implements the main workflow of the integrated navigation system,
 * including data loading, system initialization, navigation processing,
 * results saving, and performance evaluation.
 *
 * @author peanut-nav
 * @date Created: 2025-07-22
 * @last Modified: 2025-07-22
 * @version 0.1
 */

#include "DataLoader.hpp"
#include "SystemInitializer.hpp"
#include "NavigationCore.hpp"
#include "SaveResults.hpp"
#include <iostream>
#include <random>

int main() {
    // Initialize random number generator with fixed seed for reproducibility
    std::mt19937 rng(10);
    
    // System configuration parameters
    const int IMUrate = 200;     ///< IMU sampling rate (Hz)
    const int GPSrate = 20;      ///< GPS sampling rate (Hz)
    const double simTime = 600;  ///< Total simulation time (seconds)
    
    // Print system configuration
    std::cout << "=== Integrated Navigation System Startup ===" << std::endl;
    std::cout << "System Configuration: " << std::endl;
    std::cout << "  IMU Rate: " << IMUrate << " Hz" << std::endl;
    std::cout << "  GPS Rate: " << GPSrate << " Hz" << std::endl;
    std::cout << "  Simulation Time: " << simTime << " seconds" << std::endl;
    
    // Load navigation data
    std::cout << "\nLoading navigation data..." << std::endl;
    IMUData imu;
    GPSData gps;
    TrajectoryData track;
    DataLoader::loadData("../data", IMUrate, simTime, imu, gps, track);
    std::cout << "Data loading completed: " << std::endl;
    std::cout << "  IMU points: " << imu.index.size() << std::endl;
    std::cout << "  GPS points: " << gps.time.size() << std::endl;
    std::cout << "  Trajectory points: " << track.time.size() << std::endl;
    
    // Initialize navigation system
    std::cout << "\nInitializing navigation system..." << std::endl;
    NavigationParams params;     ///< Navigation physical parameters
    NavigationState state;       ///< Navigation state variables
    KalmanFilterParams kalman;   ///< Kalman filter parameters
    SystemInitializer::initializeSystem(IMUrate, GPSrate, simTime, "../data", params, state, kalman);
    std::cout << "System initialization completed" << std::endl;
    std::cout << "Initial State: " << std::endl;
    std::cout << "  Latitude: " << state.Latitude[0] << "°" << std::endl;
    std::cout << "  Longitude: " << state.Longitude[0] << "°" << std::endl;
    std::cout << "  Altitude: " << state.Altitude[0] << " m" << std::endl;
    std::cout << "  Pitch: " << state.Pitch[0] << "°" << std::endl;
    std::cout << "  Roll: " << state.Roll[0] << "°" << std::endl;
    std::cout << "  Yaw: " << state.Yaw[0] << "°" << std::endl;
    
    // Run navigation algorithm
    std::cout << "\nRunning navigation algorithm..." << std::endl;
    NavigationCore::runNavigation(imu, gps, params, state, kalman, IMUrate, GPSrate);
    std::cout << "\nNavigation algorithm completed" << std::endl;
    
    // Save navigation results
    std::cout << "\nSaving navigation results..." << std::endl;
    SaveResults::saveNavigationResults(state, imu);
    
    // Performance evaluation
    std::cout << "\n===== Performance Evaluation =====" << std::endl;
    int z_up = 20000;     ///< Start index for performance evaluation
    int z_down = std::min(120000, static_cast<int>(track.time.size()));  ///< End index
    
    if (z_down > z_up) {
        double latErrSq = 0.0, lonErrSq = 0.0, altErrSq = 0.0;
        double yawErrSq = 0.0, pitchErrSq = 0.0, rollErrSq = 0.0;
        const double Re = 6378135.072;  ///< Earth equatorial radius
        
        // Calculate RMS errors for each navigation parameter
        for (int i = z_up; i < z_down; i++) {
            // Position errors in meters
            latErrSq += pow((state.Latitude[i] - track.lat[i]) * M_PI / 180.0 * Re, 2);
            lonErrSq += pow((state.Longitude[i] - track.lon[i]) * M_PI / 180.0 * Re, 2);
            altErrSq += pow(state.Altitude[i] - track.alt[i], 2);
            
            // Attitude errors in degrees
            yawErrSq += pow(state.Yaw[i] - track.yaw[i], 2);
            pitchErrSq += pow(state.Pitch[i] - track.pitch[i], 2);
            rollErrSq += pow(state.Roll[i] - track.roll[i], 2);
        }
        
        // Calculate and display RMS errors
        int n = z_down - z_up;
        std::cout << std::fixed << std::setprecision(6);
        std::cout << "Latitude RMS error: " << sqrt(latErrSq / n) << " m" << std::endl;
        std::cout << "Longitude RMS error: " << sqrt(lonErrSq / n) << " m" << std::endl;
        std::cout << "Altitude RMS error: " << sqrt(altErrSq / n) << " m" << std::endl;
        std::cout << "Yaw RMS error: " << sqrt(yawErrSq / n) << " °" << std::endl;
        std::cout << "Pitch RMS error: " << sqrt(pitchErrSq / n) << " °" << std::endl;
        std::cout << "Roll RMS error: " << sqrt(rollErrSq / n) << " °" << std::endl;
    }
    
    std::cout << "\n=== Integrated Navigation System Completed ===" << std::endl;
    return 0;
}
