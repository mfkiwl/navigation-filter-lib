/**
 * @file DataLoader.cpp
 * @brief Implementation of data loading utilities
 *
 * Implements the data loading and preprocessing functions
 * declared in DataLoader.hpp.
 *
 * @author peanut-nav
 * @date Created: 2025-07-22
 * @last Modified: 2025-07-22
 * @version 0.1
 */

#include "DataLoader.hpp"
#include "MathUtils.hpp"
#include <fstream>
#include <iostream>
#include <random>
#include <cmath>
#include <algorithm>
#include <stdexcept>

/**
 * @brief Load all navigation data from specified directory
 * 
 * Coordinates the loading of all navigation data types and applies preprocessing.
 */
void DataLoader::loadData(const std::string& dataDir, 
                         int IMUrate, 
                         int simTime, 
                         IMUData& imu, 
                         GPSData& gps, 
                         TrajectoryData& track) {
    // Calculate total data points based on simulation time and IMU rate
    int totalPoints = simTime * IMUrate + 1;
    
    // Load IMU data from file
    loadIMUData(dataDir + "/navdata.dat", IMUrate, totalPoints, imu);
    
    // Add realistic noise to IMU measurements
    addIMUNoise(imu);
    
    // Load GPS data from file
    loadGPSData(dataDir + "/gpsdata.dat", gps);
    
    // Load reference trajectory data
    loadTrackData(dataDir + "/track.dat", totalPoints, track);
}

/**
 * @brief Load IMU data from file
 * 
 * Reads IMU measurements from a text file and applies unit conversions.
 */
void DataLoader::loadIMUData(const std::string& filePath, 
                            int IMUrate, 
                            int totalPoints, 
                            IMUData& imu) {
    // Open data file
    std::ifstream file(filePath);
    if (!file.is_open()) {
        throw std::runtime_error("Unable to open navdata.dat file");
    }
    
    // Skip initial header values
    double unused;
    for (int i = 0; i < 10; i++) {
        if (!(file >> unused)) {
            throw std::runtime_error("Invalid format in navdata.dat");
        }
    }
    
    // Preallocate memory for IMU data
    imu.index.resize(totalPoints);
    imu.gx.resize(totalPoints);
    imu.gy.resize(totalPoints);
    imu.gz.resize(totalPoints);
    imu.ax.resize(totalPoints);
    imu.ay.resize(totalPoints);
    imu.az.resize(totalPoints);
    
    // Unit conversion factors
    double gyro_scale = IMUrate / M_PI * 180 * 3600;
    double accel_scale = IMUrate;
    
    // Read data points
    for (int i = 0; i < totalPoints; i++) {
        imu.index[i] = i + 1;  // 1-based indexing
        
        // Read gyroscope measurements
        if (!(file >> imu.gx[i] >> imu.gy[i] >> imu.gz[i])) {
            throw std::runtime_error("Invalid gyro data in navdata.dat");
        }
        
        // Apply unit conversion to gyro data
        imu.gx[i] *= gyro_scale;
        imu.gy[i] *= gyro_scale;
        imu.gz[i] *= gyro_scale;
        
        // Read accelerometer measurements
        if (!(file >> imu.ax[i] >> imu.ay[i] >> imu.az[i])) {
            throw std::runtime_error("Invalid accel data in navdata.dat");
        }
        
        // Apply unit conversion to accel data
        imu.ax[i] *= accel_scale;
        imu.ay[i] *= accel_scale;
        imu.az[i] *= accel_scale;
    }
    
    file.close();
}

/**
 * @brief Add realistic noise to IMU measurements
 * 
 * Simulates sensor noise characteristics using Gaussian distributions.
 */
void DataLoader::addIMUNoise(IMUData& imu) {
    // Initialize random number generator with fixed seed for reproducibility
    std::mt19937 gen(10);
    std::normal_distribution<> dist(0.0, 1.0);
    
    // Add noise to each measurement point
    for (size_t i = 0; i < imu.index.size(); i++) {
        // Add bias and noise to gyroscope measurements
        imu.gx[i] += 0.01 + 0.01 * dist(gen);
        imu.gy[i] += 0.01 + 0.01 * dist(gen);
        imu.gz[i] += 0.01 + 0.01 * dist(gen);
        
        // Add bias and noise to accelerometer measurements
        imu.ax[i] += 0.00005 + 0.00005 * dist(gen);
        imu.ay[i] += 0.00005 + 0.00005 * dist(gen);
        imu.az[i] += 0.00005 + 0.00005 * dist(gen);
    }
}

/**
 * @brief Load GPS data from file
 * 
 * Reads GPS measurements from a text file and applies unit conversions.
 */
void DataLoader::loadGPSData(const std::string& filePath, GPSData& gps) {
    // Open GPS data file
    std::ifstream file(filePath);
    if (!file.is_open()) {
        throw std::runtime_error("Unable to open gpsdata.dat file");
    }
    
    // Temporary variables for reading
    double time, vx, vy, vz, lat, lon, alt;
    
    // Read all data points
    while (file >> time >> vx >> vy >> vz >> lat >> lon >> alt) {
        // Store timestamp and velocity
        gps.time.push_back(time);
        gps.vx.push_back(vx);
        gps.vy.push_back(vy);
        gps.vz.push_back(vz);
        
        // Convert angles from radians to degrees
        double rad2deg = 180.0 / M_PI;
        gps.lat.push_back(lat * rad2deg);
        gps.lon.push_back(lon * rad2deg);
        gps.alt.push_back(alt);
    }
    
    file.close();
}

/**
 * @brief Load trajectory data from file
 * 
 * Reads reference trajectory data from a text file, applies necessary
 * transformations, and ensures the correct data length.
 */
void DataLoader::loadTrackData(const std::string& filePath, 
                              int totalPoints, 
                              TrajectoryData& track) {
    // Open trajectory data file
    std::ifstream file(filePath);
    if (!file.is_open()) {
        throw std::runtime_error("Unable to open track.dat file");
    }
    
    // Temporary storage for raw data
    std::vector<std::vector<double>> tempData;
    double time, yaw, pitch, roll, vx, vy, vz, lat, lon, alt;
    
    // Read all data points
    while (file >> time >> yaw >> pitch >> roll >> vx >> vy >> vz >> lat >> lon >> alt) {
        tempData.push_back({time, yaw, pitch, roll, vx, vy, vz, lat, lon, alt});
    }
    file.close();
    
    // Normalize yaw angles to [0, 2π)
    for (auto& row : tempData) {
        if (row[1] < 0) {  // yaw < 0
            row[1] += 2 * M_PI;
        }
    }
    
    // Ensure correct number of data points
    if (tempData.size() < static_cast<size_t>(totalPoints)) {
        // Pad with last available data point if insufficient
        auto lastRow = tempData.back();
        while (tempData.size() < static_cast<size_t>(totalPoints)) {
            tempData.push_back(lastRow);
        }
        std::cerr << "Warning: Trajectory data insufficient, padded with last point to " 
                  << totalPoints << " points" << std::endl;
    } 
    else if (tempData.size() > static_cast<size_t>(totalPoints)) {
        // Truncate if too many points
        tempData.resize(totalPoints);
        std::cerr << "Warning: Trajectory data truncated to " 
                  << totalPoints << " points" << std::endl;
    }
    
    // Preallocate memory for trajectory data
    track.time.resize(totalPoints);
    track.yaw.resize(totalPoints);
    track.pitch.resize(totalPoints);
    track.roll.resize(totalPoints);
    track.vx.resize(totalPoints);
    track.vy.resize(totalPoints);
    track.vz.resize(totalPoints);
    track.lat.resize(totalPoints);
    track.lon.resize(totalPoints);
    track.alt.resize(totalPoints);
    
    // Convert to degrees and store in output structure
    double rad2deg = 180.0 / M_PI;
    for (int i = 0; i < totalPoints; i++) {
        track.time[i] = tempData[i][0];
        track.yaw[i] = 360.0 - tempData[i][1] * rad2deg;  // Convert and adjust heading
        track.pitch[i] = tempData[i][2] * rad2deg;
        track.roll[i] = tempData[i][3] * rad2deg;
        track.vx[i] = tempData[i][4];
        track.vy[i] = tempData[i][5];
        track.vz[i] = tempData[i][6];
        track.lat[i] = tempData[i][7] * rad2deg;
        track.lon[i] = tempData[i][8] * rad2deg;
        track.alt[i] = tempData[i][9];
    }
}
