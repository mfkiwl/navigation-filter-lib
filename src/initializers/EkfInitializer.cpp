/**
 * @file EkfInitializer.cpp
 * @brief Implementation of EKF system initializer
 *
 * @author peanut-nav
 * @date Created: 2025-07-22
 * @last Modified: 2025-08-07
 * @version 0.3.2
 */

#include "initializers/EkfInitializer.hpp"
#include "MathUtils.hpp"
#include <fstream>
#include <cmath>
#include <stdexcept>

EkfInitializer::EkfInitializer(int IMUrate, int GPSrate, int simTime)
    : IMUrate_(IMUrate), GPSrate_(GPSrate), simTime_(simTime) {}

void EkfInitializer::initialize_params(NavParamsBase& base_params, 
                                     const std::string& dataDir) {
    EkfParams& params = dynamic_cast<EkfParams&>(base_params);
    loadInitialNavData(dataDir + "/navdata.dat");
    
    // Set Earth parameters (same as MATLAB)
    params.earth_params.Re = 6378135.072;
    params.earth_params.e = 1.0/298.257223563;
    params.earth_params.W_ie = 7.292115147e-5;
    params.earth_params.g0 = 9.7803267714;
    params.earth_params.gk1 = 0.00193185138639;
    params.earth_params.gk2 = 0.00669437999013;
    
    // Calculate Earth curvature radii at initial position
    double lat_deg = lat_rad_ * 180.0 / M_PI;
    double sinLat = std::sin(lat_rad_);
    double e_sq = params.earth_params.e * params.earth_params.e;
    params.earth_params.Rx = params.earth_params.Re / std::sqrt(1 - e_sq * sinLat * sinLat);
    params.earth_params.Ry = params.earth_params.Re * (1 - e_sq) / 
                             std::pow(1 - e_sq * sinLat * sinLat, 1.5);
    
    // Set sampling rates
    params.imu_rate = IMUrate_;
    params.gps_rate = GPSrate_;
    
    // Set initial state in base parameters
    params.init_Latitude = {lat_deg};
    params.init_Longitude = {lon_rad_ * 180.0 / M_PI};
    params.init_Altitude = {h0_};
    params.init_Velocity = {Eigen::Vector3d(v_east_, v_north_, v_up_)};
    params.init_Pitch = {st_rad_ * 180.0 / M_PI + 0.05};   // Add error as in MATLAB
    params.init_Roll = {r_rad_ * 180.0 / M_PI + 0.05};     // Add error
    params.init_Yaw = {360.0 - fai_rad_ * 180.0 / M_PI + 0.1}; // Add error and convert to true heading
}

// 新接口：useTruthInit = true 时，改用 truth.nav 第一行做初始化
void EkfInitializer::initialize_params(NavParamsBase& base_params, const std::string& dataDir, bool useTruthInit) {
    if (!useTruthInit) {
        initialize_params(base_params, dataDir); // 复用旧实现
        return;
    }

    EkfParams& params = dynamic_cast<EkfParams&>(base_params);

    // 新流程：从 truth.nav 第一行读取
    loadInitialFromTruthNav(dataDir + "/truth.nav");

    // 地球参数（同旧）
    params.earth_params.Re  = 6378135.072;
    params.earth_params.e   = 1.0 / 298.257223563;
    params.earth_params.W_ie = 7.292115147e-5;
    params.earth_params.g0   = 9.7803267714;
    params.earth_params.gk1  = 0.00193185138639;
    params.earth_params.gk2  = 0.00669437999013;

    // 曲率半径（按初始纬度）
    const double sinLat = std::sin(lat_rad_);
    const double e_sq = params.earth_params.e * params.earth_params.e;
    params.earth_params.Rx = params.earth_params.Re / std::sqrt(1 - e_sq * sinLat * sinLat);
    params.earth_params.Ry = params.earth_params.Re * (1 - e_sq) / std::pow(1 - e_sq * sinLat * sinLat, 1.5);

    // 采样率
    params.imu_rate = IMUrate_;
    params.gps_rate = GPSrate_;

    // 初始状态（保持项目内部的角度/轴系约定）
    const double rad2deg = 180.0 / M_PI;
    params.init_Latitude  = { lat_rad_ * rad2deg };
    params.init_Longitude = { lon_rad_ * rad2deg };
    params.init_Altitude  = { h0_ };
    params.init_Velocity  = { Eigen::Vector3d(v_east_, v_north_, v_up_) };
    params.init_Pitch     = { st_rad_ * rad2deg + 0.05 };
    params.init_Roll      = { r_rad_ * rad2deg + 0.05 };
    params.init_Yaw       = { 360.0 - (fai_rad_ * rad2deg) + 0.1 };
}


void EkfInitializer::initialize_state(NavigationState& state, 
                                    int totalPoints) {
    // Resize state vectors
    state.Latitude.resize(totalPoints);
    state.Longitude.resize(totalPoints);
    state.Altitude.resize(totalPoints);
    state.Velocity.resize(totalPoints, Eigen::Vector3d::Zero());
    state.Pitch.resize(totalPoints);
    state.Roll.resize(totalPoints);
    state.Yaw.resize(totalPoints);
    
    // Set initial values
    state.Latitude[0] = lat_rad_ * 180.0 / M_PI;
    state.Longitude[0] = lon_rad_ * 180.0 / M_PI;
    state.Altitude[0] = h0_;
    state.Velocity[0] = Eigen::Vector3d(v_east_, v_north_, v_up_);
    state.Pitch[0] = st_rad_ * 180.0 / M_PI + 0.05;
    state.Roll[0] = r_rad_ * 180.0 / M_PI + 0.05;
    state.Yaw[0] = 360.0 - fai_rad_ * 180.0 / M_PI + 0.1;
    
    // Initialize DCM and quaternion
    state.CbtM = NavigationUtils::bodyToNavigationDCM(state.Pitch[0], state.Roll[0], state.Yaw[0]);
    state.CtbM = state.CbtM.transpose();
    state.Quaternion = NavigationUtils::eulerToQuaternion(state.Pitch[0], state.Roll[0], state.Yaw[0]);
}

void EkfInitializer::initialize_kalman(NavParamsBase& base_params, 
                                   int totalPoints) {
    ExtendedKalmanFilterParams& ekf_params = static_cast<EkfParams&>(base_params).ekf_params;
    
    // Calculate IMU/GPS update ratio
    ekf_params.N = IMUrate_ / GPSrate_;
    
    // Calculate Kalman points and period
    ekf_params.M = static_cast<int>(totalPoints / ekf_params.N) + 10;
    ekf_params.T = static_cast<double>(ekf_params.N) / IMUrate_;
    
    // Initialize noise matrices
    ekf_params.R = setupMeasurementNoise();
    ekf_params.Q = setupProcessNoise();
    // Initialize covariance matrix
    ekf_params.P = initializeCovarianceMatrix(lat_rad_ * 180.0 / M_PI);
    
    // 缓存尺寸（Z 按 R 行数自适应）
    const int m = static_cast<int>(ekf_params.R.rows());
    // Initialize state vector and measurement vector
    ekf_params.X = Eigen::MatrixXd::Zero(15, ekf_params.M);
    ekf_params.Z = Eigen::MatrixXd::Zero(6, ekf_params.M);
    // Initialize result storage
    ekf_params.Xsave = Eigen::MatrixXd::Zero(15, ekf_params.M);
    ekf_params.P_mean_square = Eigen::MatrixXd::Zero(15, ekf_params.M);
    ekf_params.N_kalman = 2;
    
    // Initialize prediction states
    ekf_params.X_pred = Eigen::VectorXd::Zero(15);
    ekf_params.P_pred = Eigen::MatrixXd::Zero(15, 15);
    
    // Initialize discrete matrices (will be updated during prediction)
    ekf_params.disA = Eigen::MatrixXd::Identity(15, 15);
    ekf_params.disQ = Eigen::MatrixXd::Zero(15, 15);
}

// ===== 新：EKF 初始化（position-only: 3D）=====
void EkfInitializer::initialize_kalman(NavParamsBase& base_params, int totalPoints, bool positionOnly) {
    if (!positionOnly) { initialize_kalman(base_params, totalPoints); return; }

    ExtendedKalmanFilterParams& ekf = static_cast<EkfParams&>(base_params).ekf_params;

    // 采样配比
    ekf.N = IMUrate_ / GPSrate_;
    ekf.M = static_cast<int>(totalPoints / ekf.N) + 10;
    ekf.T = static_cast<double>(ekf.N) / IMUrate_;

    // 噪声矩阵（3×3，仅位置 E/N/U）
    ekf.R = setupMeasurementNoisePosOnly();
    ekf.Q = setupProcessNoise();
    ekf.P = initializeCovarianceMatrix(lat_rad_ * 180.0 / M_PI);

    // 缓存尺寸（按 R 自适应）
    const int m = static_cast<int>(ekf.R.rows()); // =3
    ekf.X = Eigen::MatrixXd::Zero(15, ekf.M);
    ekf.Z = Eigen::MatrixXd::Zero(m,  ekf.M);     // 3×M
    ekf.Xsave = Eigen::MatrixXd::Zero(15, ekf.M);
    ekf.P_mean_square = Eigen::MatrixXd::Zero(15, ekf.M);
    ekf.N_kalman = 2;

    // 预测缓存
    ekf.X_pred = Eigen::VectorXd::Zero(15);
    ekf.P_pred = Eigen::MatrixXd::Zero(15, 15);
    ekf.disA   = Eigen::MatrixXd::Identity(15, 15);
    ekf.disQ   = Eigen::MatrixXd::Zero(15, 15);
}

void EkfInitializer::loadInitialNavData(const std::string& filePath) {
    std::ifstream file(filePath);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open navdata.dat");
    }
    
    double unused;
    if (!(file >> unused)) {
        throw std::runtime_error("Invalid navdata format");
    }
    
    if (!(file >> lat_rad_ >> lon_rad_ >> h0_ >> v_east_ >> v_north_ >> v_up_ 
             >> fai_rad_ >> st_rad_ >> r_rad_)) {
        throw std::runtime_error("Invalid navdata values");
    }
    
    file.close();
}

// ====== 新：用 truth.nav 第一行做初始化 ======
// truth.nav 列：Week SOW Lat(deg) Lon(deg) H(m) vN vE vD Roll(deg) Pitch(deg) Yaw(deg)
void EkfInitializer::loadInitialFromTruthNav(const std::string& truthNavPath) {
    std::ifstream fin(truthNavPath);
    if (!fin.is_open()) {
        throw std::runtime_error("Cannot open truth.nav: " + truthNavPath);
    }

    int week = 0;
    double sow = 0.0;
    double latDeg = 0.0, lonDeg = 0.0, h = 0.0;
    double vN = 0.0, vE = 0.0, vD = 0.0;
    double rDeg = 0.0, pDeg = 0.0, yDeg = 0.0;

    // 只读第一行
    if (!(fin >> week >> sow >> latDeg >> lonDeg >> h >> vN >> vE >> vD >> rDeg >> pDeg >> yDeg)) {
        throw std::runtime_error("Invalid first line in truth.nav");
    }
    fin.close();

    // 单位与轴系转换
    const double deg2rad = M_PI / 180.0;

    lat_rad_ = latDeg * deg2rad;
    lon_rad_ = lonDeg * deg2rad;
    h0_      = h;

    v_east_  = vE;
    v_north_ = vN;
    v_up_    = -vD;   // N,E,D -> E,N,U

    r_rad_   = rDeg * deg2rad;  // roll
    st_rad_  = pDeg * deg2rad;  // pitch
    fai_rad_ = yDeg * deg2rad;  // yaw（注意：state/Yaw=360 - yawDeg，外层再处理）
}

Eigen::MatrixXd EkfInitializer::setupMeasurementNoise() const {
    // Measurement precision parameters
    const double Pos_Horizon_Precision = 0.3;
    const double Pos_altitude_Precision = 0.3;

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

// ===== 量测噪声（3D：位置-only）=====
Eigen::MatrixXd EkfInitializer::setupMeasurementNoisePosOnly() const {
    const double Pos_E_Precision  = 0.5;  // 先给保守值（可改成 0.3）
    const double Pos_N_Precision  = 0.5;
    const double Pos_U_Precision  = 1.0;  // 垂直更差

    Eigen::Vector3d r;
    r << std::pow(Pos_E_Precision, 2),
         std::pow(Pos_N_Precision, 2),
         std::pow(Pos_U_Precision, 2);
    return r.asDiagonal();
}

Eigen::MatrixXd EkfInitializer::setupProcessNoise() const {
    // Gyroscope noise characteristics
    const double X_gyro = 0.02;
    const double Y_gyro = 0.02;
    const double Z_gyro = 0.02;
    
    // Accelerometer noise characteristics
    const double X_acc = 50.0;
    const double Y_acc = 50.0;
    const double Z_acc = 50.0;

    // Conversion constants
    const double deg2rad = M_PI / 180.0;
    const double gyro_rad = deg2rad / 3600.0; // deg/h to rad/s
    const double g_noise = 9.8;
    
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

Eigen::MatrixXd EkfInitializer::initializeCovarianceMatrix(double lat) const {
    // Initial error parameters
    const double P0_Attitude = 1.0/60.0;
    const double P0_Heading = 0.1;
    const double P0_Velocity_East = 0.01;
    const double P0_Velocity_North = 0.01;
    const double P0_Velocity_Up = 0.01;
    const double P0_Pos_Horizon = 0.1;
    const double P0_Pos_altitude = 0.15;
    const double P0_Gyro_X = 0.01;
    const double P0_Gyro_Y = 0.01;
    const double P0_Gyro_Z = 0.01;
    const double P0_Acc_X = 50.0;
    const double P0_Acc_Y = 50.0;
    const double P0_Acc_Z = 50.0;
    
    // Conversion constants
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