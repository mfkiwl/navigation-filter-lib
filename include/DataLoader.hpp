/**
 * @file DataLoader.hpp
 * @brief Data loading utilities for navigation systems
 *
 * Provides functionality to load and process navigation sensor data
 * from various file formats.
 *
 * @author peanut-nav
 * @date Created: 2025-07-22
 * @last Modified: 2025-07-22
 * @version 0.1
 */

#pragma once
#include "NavigationParams.hpp"
#include <string>

/**
 * @brief Data loading and preprocessing class
 * 
 * Handles loading of IMU, GPS, and trajectory data from files,
 * and applies necessary preprocessing and noise injection.
 */
class DataLoader {
public:
    /**
     * @brief Load all navigation data from specified directory
     * 
     * @param dataDir Directory containing data files
     * @param IMUrate IMU sampling rate (Hz)
     * @param simTime Total simulation time (seconds)
     * @param[out] imu Container for loaded IMU data
     * @param[out] gps Container for loaded GPS data
     * @param[out] track Container for loaded trajectory data
     */
    static void loadData(const std::string& dataDir, 
                         int IMUrate, 
                         int simTime, 
                         IMUData& imu, 
                         GPSData& gps, 
                         TrajectoryData& track);
    
private:
    /**
     * @brief Load IMU data from file
     * 
     * @param filePath Path to IMU data file
     * @param IMUrate IMU sampling rate (Hz)
     * @param totalPoints Total number of data points to load
     * @param[out] imu Container for loaded IMU data
     */
    static void loadIMUData(const std::string& filePath, 
                           int IMUrate, 
                           int totalPoints, 
                           IMUData& imu);
    
    /**
     * @brief Load GPS data from file
     * 
     * @param filePath Path to GPS data file
     * @param[out] gps Container for loaded GPS data
     */
    static void loadGPSData(const std::string& filePath, GPSData& gps);
    
    /**
     * @brief Load trajectory data from file
     * 
     * @param filePath Path to trajectory data file
     * @param totalPoints Total number of data points to load
     * @param[out] track Container for loaded trajectory data
     */
    static void loadTrackData(const std::string& filePath, 
                             int totalPoints, 
                             TrajectoryData& track);
    
    /**
     * @brief Add realistic noise to IMU measurements
     * 
     * @param[in,out] imu IMU data to be modified with added noise
     */
    static void addIMUNoise(IMUData& imu);
};
