/**
 * @file KfInitializer.cpp
 * @brief Implementation of KF system initializer
 *
 * @author peanut-nav
 * @date Created: 2025-07-22
 * @last Modified: 2025-07-22
 * @version 0.1
 */
 
#include "initializers/KfInitializer.hpp"
#include "MathUtils.hpp"
#include <fstream>
#include <cmath>
#include <stdexcept>
 
KfInitializer::KfInitializer(int IMUrate, int GPSrate, int simTime)
    : IMUrate_(IMUrate), GPSrate_(GPSrate), simTime_(simTime) {}
 
void KfInitializer::initialize_params(NavParamsBase& base_params, 
                                     const std::string& dataDir) {
    // 安全转换为 KF 参数
    KfParams& params = dynamic_cast<KfParams&>(base_params);
    
    // 加载初始导航数据
    loadInitialNavData(dataDir + "/navdata.dat");
    
    // 设置地球参数
    params.earth_params.Re = 6378135.072;
    params.earth_params.e = 1.0/298.257223563;
    params.earth_params.W_ie = 7.292115147e-5;
    params.earth_params.g0 = 9.7803267714;
    params.earth_params.gk1 = 0.00193185138639;
    params.earth_params.gk2 = 0.00669437999013;
    
    // 计算地球曲率半径
    double lat_deg = lat_rad_ * 180.0 / M_PI;
    double sinLat = std::sin(lat_deg * M_PI / 180.0);
    double e_sq = params.earth_params.e * params.earth_params.e;
    params.earth_params.Rx = params.earth_params.Re / sqrt(1 - e_sq * sinLat * sinLat);
    params.earth_params.Ry = params.earth_params.Re * (1 - e_sq) / 
                             pow(1 - e_sq * sinLat * sinLat, 1.5);
    
    // 设置采样率
    params.imu_rate = IMUrate_;
    params.gps_rate = GPSrate_;
    
    // 设置初始状态
    params.init_Latitude = {lat_rad_ * 180.0 / M_PI};
    params.init_Longitude = {lon_rad_ * 180.0 / M_PI};
    params.init_Altitude = {h0_};
    params.init_Velocity = {Eigen::Vector3d(v_east_, v_north_, v_up_)};
    params.init_Pitch = {st_rad_ * 180.0 / M_PI};
    params.init_Roll = {r_rad_ * 180.0 / M_PI};
    params.init_Yaw = {fai_rad_ * 180.0 / M_PI};
}

void KfInitializer::initialize_state(NavigationState& state, 
                                    int totalPoints) {
    // 初始化导航状态向量
    state.Latitude.resize(totalPoints);
    state.Longitude.resize(totalPoints);
    state.Altitude.resize(totalPoints);
    state.Velocity.resize(totalPoints, Eigen::Vector3d::Zero());
    state.Pitch.resize(totalPoints);
    state.Roll.resize(totalPoints);
    state.Yaw.resize(totalPoints);
    
    // 设置初始位置和速度
    state.Latitude[0] = lat_rad_ * 180.0 / M_PI;
    state.Longitude[0] = lon_rad_ * 180.0 / M_PI;
    state.Altitude[0] = h0_;
    state.Velocity[0] = Eigen::Vector3d(v_east_, v_north_, v_up_);
    
    // 设置初始姿态
    state.Pitch[0] = st_rad_ * 180.0 / M_PI + 0.05;
    state.Roll[0] = r_rad_ * 180.0 / M_PI + 0.05;
    state.Yaw[0] = (360.0 - fai_rad_ * 180.0 / M_PI) + 0.1;
    
    // 初始化方向余弦矩阵和四元数
    state.CbtM = NavigationUtils::bodyToNavigationDCM(state.Pitch[0], state.Roll[0], state.Yaw[0]);
    state.CtbM = state.CbtM.transpose();
    state.Quaternion = NavigationUtils::eulerToQuaternion(state.Pitch[0], state.Roll[0], state.Yaw[0]);
}

void KfInitializer::initialize_kalman(KalmanFilterParams& kalman, 
                                     int totalPoints) {
    // 计算 IMU/GPS 更新比率
    kalman.N = IMUrate_ / GPSrate_;
    
    // 计算 Kalman 点数和周期
    kalman.M = static_cast<int>(totalPoints / kalman.N) + 10;
    kalman.T = static_cast<double>(kalman.N) / IMUrate_;
    
    // 设置噪声和协方差矩阵
    kalman.R = setupMeasurementNoise();
    kalman.Q = setupProcessNoise();
    kalman.P = initializeCovarianceMatrix(lat_rad_ * 180.0 / M_PI);
    
    // 初始化状态向量和存储
    kalman.X = Eigen::MatrixXd::Zero(15, kalman.M);
    kalman.Z = Eigen::MatrixXd::Zero(6, kalman.M);
    kalman.Xsave = Eigen::MatrixXd::Zero(15, kalman.M);
    kalman.P_mean_square = Eigen::MatrixXd::Zero(15, kalman.M);
    kalman.N_kalman = 2;
}

void KfInitializer::loadInitialNavData(const std::string& filePath) {
    // Open navigation data file
    std::ifstream file(filePath);
    if (!file.is_open()) {
        throw std::runtime_error("Unable to open navdata.dat file");
    }
    
    // Skip header value
    double unused;
    if (!(file >> unused)) {
        throw std::runtime_error("Invalid format in navdata.dat");
    }
    
    // Read initial navigation parameters
    if (!(file >> lat_rad_ >> lon_rad_ >> h0_ >> v_east_ >> v_north_ >> v_up_ 
             >> fai_rad_ >> st_rad_ >> r_rad_)) {
        throw std::runtime_error("Invalid data in navdata.dat");
    }
    
    file.close();
}

Eigen::MatrixXd KfInitializer::setupMeasurementNoise() const {
    // Position measurement precision
    const double Pos_Horizon_Precision = 3.0;
    const double Pos_altitude_Precision = 3.0;
    
    // Velocity measurement precision
    const double Velocity_East_Precision = 0.01;
    const double Velocity_North_Precision = 0.01;
    const double Velocity_Up_Precision = 0.01;
    
    // Create diagonal covariance matrix
    Eigen::VectorXd r(6);
    r << std::pow(Velocity_East_Precision, 2),
         std::pow(Velocity_North_Precision, 2),
         std::pow(Velocity_Up_Precision, 2),
         std::pow(Pos_Horizon_Precision, 2),
         std::pow(Pos_Horizon_Precision, 2),
         std::pow(Pos_altitude_Precision, 2);
    
    return r.asDiagonal();
}

Eigen::MatrixXd KfInitializer::setupProcessNoise() const {
    // Gyroscope noise characteristics
    const double X_gyro = 0.02;
    const double Y_gyro = 0.02;
    const double Z_gyro = 0.02;
    
    // Accelerometer noise characteristics
    const double X_acc = 50.0;
    const double Y_acc = 50.0;
    const double Z_acc = 50.0;
    
    const double g_noise = 9.8;
    const double deg2rad = M_PI / 180.0;
    const double gyro_rad = deg2rad / 3600.0;
    
    // Create diagonal covariance matrix
    Eigen::VectorXd q(6);
    q << std::pow(X_gyro * gyro_rad, 2),
         std::pow(Y_gyro * gyro_rad, 2),
         std::pow(Z_gyro * gyro_rad, 2),
         std::pow(X_acc * 1e-6 * g_noise, 2),
         std::pow(Y_acc * 1e-6 * g_noise, 2),
         std::pow(Z_acc * 1e-6 * g_noise, 2);
    
    return q.asDiagonal();
}

Eigen::MatrixXd KfInitializer::initializeCovarianceMatrix(double lat) const {
    // Initial uncertainty parameters
    const double P0_Pos_Horizon = 0.1;
    const double P0_Pos_altitude = 0.15;
    const double P0_Velocity_East = 0.01;
    const double P0_Velocity_North = 0.01;
    const double P0_Velocity_Up = 0.01;
    const double P0_Attitude = 1.0/60.0;
    const double P0_Heading = 0.5;
    const double P0_Acc_X = 50.0;
    const double P0_Acc_Y = 50.0;
    const double P0_Acc_Z = 50.0;
    const double P0_Gyro_X = 0.02;
    const double P0_Gyro_Y = 0.02;
    const double P0_Gyro_Z = 0.02;
    
    const double Re = 6378135.072;
    const double deg2rad = M_PI / 180.0;
    const double gyro_rad = deg2rad / 3600.0;
    const double lat_rad = lat * deg2rad;
    
    // Create diagonal covariance matrix
    Eigen::VectorXd p(15);
    p << std::pow(P0_Attitude * deg2rad, 2),
         std::pow(P0_Attitude * deg2rad, 2),
         std::pow(P0_Heading * deg2rad, 2),
         std::pow(P0_Velocity_East, 2),
         std::pow(P0_Velocity_North, 2),
         std::pow(P0_Velocity_Up, 2),
         std::pow(P0_Pos_Horizon / Re, 2),
         std::pow(P0_Pos_Horizon / (Re * std::cos(lat_rad)), 2),
         std::pow(P0_Pos_altitude, 2),
         std::pow(P0_Gyro_X * gyro_rad, 2),
         std::pow(P0_Gyro_Y * gyro_rad, 2),
         std::pow(P0_Gyro_Z * gyro_rad, 2),
         std::pow(P0_Acc_X * 1e-6 * 9.8, 2),
         std::pow(P0_Acc_Y * 1e-6 * 9.8, 2),
         std::pow(P0_Acc_Z * 1e-6 * 9.8, 2);
    
    return p.asDiagonal();
}
