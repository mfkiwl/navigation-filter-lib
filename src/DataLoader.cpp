/**
 * @file DataLoader.cpp
 * @brief Implementation of data loading utilities
 *
 * Keeps legacy data-loading behavior intact, and adds support for
 * the AwesomeGINS dataset with clean separation.
 *
 * Author: peanut-nav
 * Created: 2025-07-22
 * Last Modified: 2025-09-07
 * Version: 0.4.1
 */

#include "DataLoader.hpp"
#include "MathUtils.hpp"
#include <fstream>
#include <iostream>
#include <random>
#include <cmath>
#include <algorithm>
#include <stdexcept>
#include <limits>
#include <cstdint>
#include <filesystem>

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

// =======================================================
// 新增重载：按数据集类型读取（解耦新旧）
// =======================================================
void DataLoader::loadData(const std::string& dataDir,
                          int IMUrate,
                          int simTime,
                          IMUData& imu,
                          GPSData& gps,
                          TrajectoryData& track,
                          DatasetFormat format) {
    if (format == DatasetFormat::LegacyDat) {
        loadData(dataDir, IMUrate, simTime, imu, gps, track);
        return;
    }

    // 新数据集（AwesomeGINS）：固定文件名，不做补齐/截断
    const int totalPoints = simTime * IMUrate + 1; // 仅保持签名一致；实际不使用做截断

    // 1) IMU：Δθ(rad)/Δv(m/s) -> deg/h & m/s^2；不加噪
    loadAG_IMU(dataDir + "/ADIS16465.txt", IMUrate, totalPoints, imu);

    // 2) GNSS：只有位置（deg, m），速度置 0；time 使用 SOW
    loadAG_GNSS(dataDir + "/GNSS_RTK.pos", gps);

    // 3) 真值：N,E,D -> E,N,U；姿态 Roll,Pitch,Yaw(deg)，内部存 360 - Yaw
    loadAG_Track(dataDir + "/truth.nav", totalPoints, track);
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
// =======================================================
// 原有：读取旧 GNSS（gpsdata.dat）
// 列：time vx vy vz lat(rad) lon(rad) alt
// 内部存储：lat/lon 转成 degrees
// =======================================================
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
// =======================================================
// 原有：读取旧真值（track.dat）
// 列：time yaw(rad) pitch(rad) roll(rad) vx vy vz lat(rad) lon(rad) alt
// 内部：角度转度；yaw -> 360 - yawDeg；长度对齐 totalPoints
// =======================================================
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

// =======================================================
// ============  新数据集（AwesomeGINS）实现  ============
// =======================================================

// ---- 读取 IMU（ADIS16465.txt）并统一到 FRD ----
// 原始列：SOW dθx(rad) dθy(rad) dθz(rad) dvx(m/s) dvy(m/s) dvz(m/s)
// 目标：FRD（Forward-Right-Down）上的 gx/gy/gz(deg/h), ax/ay/az(m/s²)
void DataLoader::loadAG_IMU(const std::string& imuFile,
                            int IMUrate,
                            int totalPoints,
                            IMUData& imu) {
    std::ifstream fin(imuFile);
    if (!fin.is_open()) {
        throw std::runtime_error("Unable to open IMU file: " + imuFile);
    }

    std::vector<double> dthx, dthy, dthz, dvx, dvy, dvz;
    double sow, a, b, c, x, y, z;
    while (fin >> sow >> a >> b >> c >> x >> y >> z) {
        dthx.push_back(a); dthy.push_back(b); dthz.push_back(c);
        dvx.push_back(x);  dvy.push_back(y);  dvz.push_back(z);
    }
    fin.close();
    if (dthx.empty()) throw std::runtime_error("IMU file is empty: " + imuFile);

    // --------- 1) 体轴 → FRD 的候选映射 ---------
    // RFU -> FRD:  [F,R,D] = [y, x, -z]
    auto map_RFU_to_FRD = [](double ix, double iy, double iz, double& ox, double& oy, double& oz){
        ox =  iy;  oy =  ix;  oz = -iz;
    };
    // FLU -> FRD:  [F,R,D] = [x, -y, -z]
    auto map_FLU_to_FRD = [](double ix, double iy, double iz, double& ox, double& oy, double& oz){
        ox =  ix;  oy = -iy;  oz = -iz;
    };

    // --------- 2) 自动判别（看重力方向）---------
    // 用前 2 秒（或可用长度）求 dv 的平均（再乘 IMUrate ≈ 加速度）
    const int N = static_cast<int>(dvx.size());
    int n_probe = std::min(N, std::max(1, IMUrate * 2));  // 约 2 秒
    auto score_mapping = [&](auto mapper)->std::pair<double, std::array<double,3>>{
        double axm=0, aym=0, azm=0;
        for (int i = 0; i < n_probe; ++i) {
            double fx, fy, fz;
            mapper(dvx[i], dvy[i], dvz[i], fx, fy, fz);  // 映射到 FRD 的 Δv
            axm += fx; aym += fy; azm += fz;
        }
        axm = axm * IMUrate / n_probe;  // 近似加速度均值
        aym = aym * IMUrate / n_probe;
        azm = azm * IMUrate / n_probe;
        // 目标是 FRD 下重力 ≈ (0,0,+g)。打分越小越好。
        const double g = 9.80665;
        double cost = std::abs(axm)/g + std::abs(aym)/g + std::abs(std::abs(azm) - g)/g;
        // 希望 azm 为正（Down），惩罚负号
        if (azm < 0) cost += 1.0;
        return {cost, {axm, aym, azm}};
    };

    auto s_rfu = score_mapping(map_RFU_to_FRD);
    auto s_flu = score_mapping(map_FLU_to_FRD);
    // bool use_rfu = (s_rfu.first < s_flu.first);
    bool use_rfu = true;   // ← 先强制 RFU→FRD 试跑

    // --------- 3) 按最佳映射把全量 Δθ/Δv 转到 FRD ---------
    std::vector<double> dth_f(N), dtr_f(N), dtd_f(N);
    std::vector<double> dv_f (N), dvr_f(N), dvd_f(N);
    for (int i = 0; i < N; ++i) {
        double fx, fy, fz;
        if (use_rfu) {
            map_RFU_to_FRD(dthx[i], dthy[i], dthz[i], dth_f[i], dtr_f[i], dtd_f[i]);
            map_RFU_to_FRD(dvx[i],  dvy[i],  dvz[i],  dv_f[i],  dvr_f[i], dvd_f[i]);
        } else {
            map_FLU_to_FRD(dthx[i], dthy[i], dthz[i], dth_f[i], dtr_f[i], dtd_f[i]);
            map_FLU_to_FRD(dvx[i],  dvy[i],  dvz[i],  dv_f[i],  dvr_f[i], dvd_f[i]);
        }
    }

    // 自检日志：映射选择与静止重力
    std::cerr << "[DataLoader] New-IMU frame detected as "
              << (use_rfu ? "RFU->FRD" : "FLU->FRD")
              << ", mean acc (FRD) over ~2s = ["
              << s_rfu.second[0] << ", " << s_rfu.second[1] << ", " << s_rfu.second[2]
              << "] (RFU test), ["
              << s_flu.second[0] << ", " << s_flu.second[1] << ", " << s_flu.second[2]
              << "] (FLU test)\n";

    // --------- 4) 单位转换：Δθ/Δv → 角速度deg/h / 加速度m/s² ---------
    const double gyro_scale  = static_cast<double>(IMUrate) * 180.0 / M_PI * 3600.0; // Δθ->deg/h
    const double accel_scale = static_cast<double>(IMUrate);                          // Δv->m/s²

    for (int i = 0; i < N; ++i) {
        dth_f[i] *= gyro_scale; dtr_f[i] *= gyro_scale; dtd_f[i] *= gyro_scale;
        dv_f[i]  *= accel_scale; dvr_f[i] *= accel_scale; dvd_f[i] *= accel_scale;
    }

    // --------- 5) 长度对齐到 totalPoints（截断/末样本补齐） ---------
    DataLoader::padOrTruncate(dth_f, totalPoints, dth_f.back());
    DataLoader::padOrTruncate(dtr_f, totalPoints, dtr_f.back());
    DataLoader::padOrTruncate(dtd_f, totalPoints, dtd_f.back());
    DataLoader::padOrTruncate(dv_f,  totalPoints, dv_f.back());
    DataLoader::padOrTruncate(dvr_f, totalPoints, dvr_f.back());
    DataLoader::padOrTruncate(dvd_f, totalPoints, dvd_f.back());

    imu.index.resize(totalPoints);
    imu.gx.resize(totalPoints); imu.gy.resize(totalPoints); imu.gz.resize(totalPoints);
    imu.ax.resize(totalPoints); imu.ay.resize(totalPoints); imu.az.resize(totalPoints);
    for (int i = 0; i < totalPoints; ++i) {
        imu.index[i] = i + 1;
        imu.gx[i] = dth_f[i];  imu.gy[i] = dtr_f[i];  imu.gz[i] = dtd_f[i];
        imu.ax[i] = dv_f[i];   imu.ay[i] = dvr_f[i];  imu.az[i] = dvd_f[i];
    }

    if (N != totalPoints) {
        std::cerr << "[DataLoader] IMU length " << N
                  << " -> aligned to totalPoints " << totalPoints << "\n";
    }
}


// ---- 2) 读取 GNSS（GNSS_RTK.pos）——位置-only（不强制对齐） ----
// 列：SOW Lat(deg) Lon(deg) H(m) σLat(m) σLon(m) σH(m)
void DataLoader::loadAG_GNSS(const std::string& posFile, GPSData& gps) {
    std::ifstream fin(posFile);
    if (!fin.is_open()) {
        throw std::runtime_error("Unable to open POS file: " + posFile);
    }

    double sow, latDeg, lonDeg, h, sLat, sLon, sH;
    while (fin >> sow >> latDeg >> lonDeg >> h >> sLat >> sLon >> sH) {
        gps.time.push_back(sow);
        gps.vx.push_back(0.0);
        gps.vy.push_back(0.0);
        gps.vz.push_back(0.0);
        gps.lat.push_back(latDeg);
        gps.lon.push_back(lonDeg);
        gps.alt.push_back(h);
    }
    fin.close();
}

// ---- 3) 读取真值（truth.nav）并对齐到 totalPoints ----
// 列：Week SOW Lat(deg) Lon(deg) H vN vE vD Roll(deg) Pitch(deg) Yaw(deg)
// 内部：vx=vE, vy=vN, vz=-vD；yaw=360 - Yaw；pitch=Pitch；roll=Roll；lat/lon 度
void DataLoader::loadAG_Track(const std::string& navFile,
                              int totalPoints,
                              TrajectoryData& track) {
    std::ifstream fin(navFile);
    if (!fin.is_open()) {
        throw std::runtime_error("Unable to open NAV file: " + navFile);
    }

    std::vector<double> time, yaw, pitch, roll, vx, vy, vz, lat, lon, alt;

    int week;
    double sow, latDeg, lonDeg, h, vN, vE, vD, rDeg, pDeg, yDeg;

    auto norm360 = [](double a){ double x = std::fmod(a, 360.0); return (x < 0) ? x + 360.0 : x; };

    while (fin >> week >> sow >> latDeg >> lonDeg >> h >> vN >> vE >> vD >> rDeg >> pDeg >> yDeg) {
        time.push_back(sow);
        vx.push_back(vE);
        vy.push_back(vN);
        vz.push_back(-vD);
        yaw.push_back(360.0 - norm360(yDeg));
        pitch.push_back(pDeg);
        roll.push_back(rDeg);
        lat.push_back(latDeg);
        lon.push_back(lonDeg);
        alt.push_back(h);
    }
    fin.close();

    if (time.empty()) throw std::runtime_error("NAV file is empty: " + navFile);

    // 对齐到 totalPoints（截断或用最后一个样本补齐）
    const auto t_last = time.back();
    padOrTruncate(time,  totalPoints, t_last);
    padOrTruncate(yaw,   totalPoints, yaw.back());
    padOrTruncate(pitch, totalPoints, pitch.back());
    padOrTruncate(roll,  totalPoints, roll.back());
    padOrTruncate(vx,    totalPoints, vx.back());
    padOrTruncate(vy,    totalPoints, vy.back());
    padOrTruncate(vz,    totalPoints, vz.back());
    padOrTruncate(lat,   totalPoints, lat.back());
    padOrTruncate(lon,   totalPoints, lon.back());
    padOrTruncate(alt,   totalPoints, alt.back());

    track.time = std::move(time);
    track.yaw  = std::move(yaw);
    track.pitch= std::move(pitch);
    track.roll = std::move(roll);
    track.vx   = std::move(vx);
    track.vy   = std::move(vy);
    track.vz   = std::move(vz);
    track.lat  = std::move(lat);
    track.lon  = std::move(lon);
    track.alt  = std::move(alt);
}