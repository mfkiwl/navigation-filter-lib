/**
 * @file main_ekf.cpp
 * @brief Main application for EKF-based integrated navigation system
 *
 * Implements the main workflow of the EKF-based integrated navigation system,
 * including data loading, system initialization, navigation processing,
 * results saving, and performance evaluation.
 *
 * @author peanut-nav
 * @date Created: 2025-07-22
 * @last Modified: 2025-07-22
 * @version 0.1
 */

#include "DataLoader.hpp"
#include "initializers/EkfInitializer.hpp"
#include "core/EkfNavigation.hpp"
#include "SaveResults.hpp"
#include "NavigationParams.hpp"
#include <iostream>
#include <random>
#include <iomanip>

int main() {
    // Initialize random number generator with fixed seed for reproducibility
    std::mt19937 rng(1);
    
    // System configuration parameters
    const int IMUrate = 200;     ///< IMU sampling rate (Hz)
    const int GPSrate = 1;      ///< GPS sampling rate (Hz)
    const double simTime = 1400;  ///< Total simulation time (seconds)
    const std::string dataDir = "../data";  ///< 数据目录
    
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
    DataLoader::loadData(dataDir, IMUrate, simTime, imu, gps, track);
    std::cout << "Data loading completed: " << std::endl;
    std::cout << "  IMU points: " << imu.index.size() << std::endl;
    std::cout << "  GPS points: " << gps.time.size() << std::endl;
    std::cout << "  Trajectory points: " << track.time.size() << std::endl;
    
    // Initialize navigation system
    std::cout << "\nInitializing navigation system..." << std::endl;

    // 创建EKF初始化器
    EkfInitializer initializer(IMUrate, GPSrate, simTime);
    
    // 创建参数对象 - 使用 EkfParams 类型
    EkfParams params;             ///< 导航参数（包含地球参数和EKF参数）
    NavigationState state;       ///< 导航状态
    
    // 初始化参数、状态和EKF参数
    int totalPoints = simTime * IMUrate + 1;
    initializer.initialize_params(params, dataDir);
    initializer.initialize_state(state, totalPoints);
    initializer.initialize_ekf(params.ekf_params, totalPoints);

    std::cout << "System initialization completed" << std::endl;
    std::cout << "Initial State: " << std::endl;
    std::cout << "  Latitude: " << state.Latitude[0] << "°" << std::endl;
    std::cout << "  Longitude: " << state.Longitude[0] << "°" << std::endl;
    std::cout << "  Altitude: " << state.Altitude[0] << " m" << std::endl;
    std::cout << "  Pitch: " << state.Pitch[0] << "°" << std::endl;
    std::cout << "  Roll: " << state.Roll[0] << "°" << std::endl;
    std::cout << "  Yaw: " << state.Yaw[0] << "°" << std::endl;
    
    // 打印地球参数
    std::cout << "\nEarth Parameters:" << std::endl;
    std::cout << "  Re: " << params.earth_params.Re << " m" << std::endl;
    std::cout << "  e: " << params.earth_params.e << std::endl;
    std::cout << "  W_ie: " << params.earth_params.W_ie << " rad/s" << std::endl;
    std::cout << "  g0: " << params.earth_params.g0 << " m/s²" << std::endl;
    
    // 打印KF参数
    std::cout << "\nKalman Filter Parameters:" << std::endl;
    std::cout << "  N: " << params.ekf_params.N << std::endl;
    std::cout << "  M: " << params.ekf_params.M << std::endl;
    std::cout << "  T: " << params.ekf_params.T << " s" << std::endl;
    std::cout << "  P matrix size: " << params.ekf_params.P.rows() 
              << "x" << params.ekf_params.P.cols() << std::endl;
    
    // ================== EKF导航解算 ==================
    // Run navigation algorithm
    std::cout << "\nRunning navigation algorithm..." << std::endl;
    
    // 创建并初始化扩展卡尔曼滤波导航对象
    EkfNavigation nav;
    nav.initialize(params, state);
    
    // 执行导航解算
    nav.run(imu, gps);
    
    // 更新状态引用
    state = nav.getState();

    std::cout << "\nNavigation algorithm completed" << std::endl;
    // ================== 导航解算部分更新结束 ==================
    
    // Save navigation results
    std::cout << "\nSaving navigation results..." << std::endl;
    // Save navigation results
    SaveResults::saveNavigationResults(state, imu);
    
    // Performance evaluation
    std::cout << "\n===== Performance Evaluation =====" << std::endl;
    int z_up = 220000;     ///< Start index for performance evaluation
    int z_down = std::min(280000, static_cast<int>(track.time.size()));  ///< End index
    
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
        std::cout << "=== EKF Performance Evaluation ===" << std::endl;
        std::cout << "Latitude RMS error: " << sqrt(latErrSq / n) << " m" << std::endl;
        std::cout << "Longitude RMS error: " << sqrt(lonErrSq / n) << " m" << std::endl;
        std::cout << "Altitude RMS error: " << sqrt(altErrSq / n) << " m" << std::endl;
        std::cout << "Yaw RMS error: " << sqrt(yawErrSq / n) << " °" << std::endl;
        std::cout << "Pitch RMS error: " << sqrt(pitchErrSq / n) << " °" << std::endl;
        std::cout << "Roll RMS error: " << sqrt(rollErrSq / n) << " °" << std::endl;
    }
    
    std::cout << "\n=== EKF Integrated Navigation System Completed ===" << std::endl;
    return 0;
}
