/**
 * @file DataLoader.hpp
 * @brief Data loading utilities for navigation systems
 *
 * Keeps legacy interface intact and adds support for AwesomeGINS dataset,
 * without redefining data structs that live in params/NavParamsBase.hpp.
 *
 * Author: peanut-nav
 * Created: 2025-07-22
 * Last Modified: 2025-09-07
 * Version: 0.4.1
 */

#pragma once
#include "params/NavParamsBase.hpp"
#include <string>

// 数据集类型：LegacyDat 为原 *.dat；AwesomeGINS 为新数据集（*.txt/*.pos/*.nav）
enum class DatasetFormat {
    LegacyDat,
    AwesomeGINS
};

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
    
    // ===== 新增：按数据集类型读取（解耦新旧） =====
    static void loadData(const std::string& dataDir,
                  int IMUrate,
                  int simTime,
                  IMUData& imu,
                  GPSData& gps,
                  TrajectoryData& track,
                  DatasetFormat format);

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

    // ===== 新数据集（AwesomeGINS）的读取 =====
    // IMU文件：ADIS16465.txt，列：SOW dθx(rad) dθy(rad) dθz(rad) dvx(m/s) dvy(m/s) dvz(m/s)
    static void loadAG_IMU(const std::string& imuFile,
                    int IMUrate,
                    int totalPoints,
                    IMUData& imu);

    // GNSS文件：GNSS_RTK.pos，列：SOW Lat(deg) Lon(deg) H(m) σLat σLon σH
    static void loadAG_GNSS(const std::string& posFile, GPSData& gps);

    // 真值文件：truth.nav，列：Week SOW Lat(deg) Lon(deg) H vN vE vD Roll(deg) Pitch(deg) Yaw(deg)
    static void loadAG_Track(const std::string& navFile,
                      int totalPoints,
                      TrajectoryData& track);

    template <typename T>
    static void padOrTruncate(std::vector<T>& v, std::size_t totalPoints, const T& padVal) {
        if (v.size() < totalPoints) v.resize(totalPoints, padVal);
        if (v.size() > totalPoints) v.resize(totalPoints);
    }
};
