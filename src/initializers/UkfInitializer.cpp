/**
 * @file UkfInitializer.cpp
 * @brief Implementation of UKF system initializer
 *
 * @author peanut-nav
 * @date Created: 2025-08-10
 * @last Modified: 2025-08-10
 * @version 0.3.3
 */

#include "initializers/UkfInitializer.hpp"
#include "MathUtils.hpp"
#include <fstream>
#include <cmath>
#include <stdexcept>

UkfInitializer::UkfInitializer(int IMUrate, int GPSrate, int simTime)
    : IMUrate_(IMUrate), GPSrate_(GPSrate), simTime_(simTime) {}

void UkfInitializer::initialize_params(NavParamsBase& base_params, 
                                     const std::string& dataDir) {
    UkfParams& params = dynamic_cast<UkfParams&>(base_params);
    loadInitialNavData(dataDir + "/navdata.dat");
    
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

void UkfInitializer::initialize_state(NavigationState& state, int totalPoints) {
    // Resize state vectors
    state.Latitude.resize(totalPoints);
    state.Longitude.resize(totalPoints);
    state.Altitude.resize(totalPoints);
    state.Velocity.resize(totalPoints, Eigen::Vector3d::Zero());
    state.Pitch.resize(totalPoints);
    state.Roll.resize(totalPoints);
    state.Yaw.resize(totalPoints);

    // Set initial values
    state.Latitude[0]  = lat_rad_ * 180.0 / M_PI;
    state.Longitude[0] = lon_rad_ * 180.0 / M_PI;
    state.Altitude[0]  = h0_;
    state.Velocity[0]  = Eigen::Vector3d(v_east_, v_north_, v_up_);
    state.Pitch[0]     = st_rad_ * 180.0 / M_PI + 0.05;
    state.Roll[0]      = r_rad_ * 180.0 / M_PI + 0.05;
    state.Yaw[0]       = 360.0 - fai_rad_ * 180.0 / M_PI + 0.1;

    // Initialize DCM and quaternion
    state.CbtM = NavigationUtils::bodyToNavigationDCM(state.Pitch[0], state.Roll[0], state.Yaw[0]);
    state.CtbM = state.CbtM.transpose();
    state.Quaternion = NavigationUtils::eulerToQuaternion(state.Pitch[0], state.Roll[0], state.Yaw[0]);
}

void UkfInitializer::initialize_kalman(NavParamsBase& base_params, int totalPoints) {
    UnscentedKalmanFilterParams& ukf_params = static_cast<UkfParams&>(base_params).ukf_params;

    // Calculate IMU/GPS update ratio
    ukf_params.N = IMUrate_ / GPSrate_;

    // Calculate Kalman points and period
    ukf_params.M = static_cast<int>(totalPoints / ukf_params.N) + 10;
    ukf_params.T = static_cast<double>(ukf_params.N) / IMUrate_;

    // Initialize noise matrices
    ukf_params.R  = setupMeasurementNoise();
    ukf_params.Q0 = setupProcessNoise();
    
    ukf_params.P  = initializeCovarianceMatrix(lat_rad_ * 180.0 / M_PI);

    ukf_params.X = Eigen::MatrixXd::Zero(15, ukf_params.M);
    ukf_params.Z = Eigen::MatrixXd::Zero(6,  ukf_params.M);
    ukf_params.Xsave = Eigen::MatrixXd::Zero(15, ukf_params.M);
    ukf_params.P_mean_square = Eigen::MatrixXd::Zero(15, ukf_params.M);
    ukf_params.N_kalman = 2;

    ukf_params.X_pred = Eigen::VectorXd::Zero(15);
    ukf_params.P_pred = Eigen::MatrixXd::Zero(15,15);

    ukf_params.Fai    = Eigen::MatrixXd::Identity(15,15);
    ukf_params.Q = Eigen::MatrixXd::Zero(15, 15);
}

void UkfInitializer::loadInitialNavData(const std::string& filePath) {
    std::ifstream file(filePath);
    if (!file.is_open()) throw std::runtime_error("Cannot open navdata.dat");
    
    double unused;
    if (!(file >> unused)) throw std::runtime_error("Invalid navdata format");
    
    if (!(file >> lat_rad_ >> lon_rad_ >> h0_ >> v_east_ >> v_north_ >> v_up_
               >> fai_rad_ >> st_rad_ >> r_rad_))
        throw std::runtime_error("Invalid navdata values");
    
    file.close();
}

Eigen::MatrixXd UkfInitializer::setupMeasurementNoise() const {
    const double Pos_Horizon_Precision = 0.3, Pos_altitude_Precision=0.3;
    const double Velocity_East_Precision = 0.01, Velocity_North_Precision=0.01, Velocity_Up_Precision=0.01;
    Eigen::VectorXd r(6);
    r << std::pow(Velocity_East_Precision,2),
         std::pow(Velocity_North_Precision,2),
         std::pow(Velocity_Up_Precision,2),
         std::pow(Pos_Horizon_Precision,2),
         std::pow(Pos_Horizon_Precision,2),
         std::pow(Pos_altitude_Precision,2);
    return r.asDiagonal();
}

Eigen::MatrixXd UkfInitializer::setupProcessNoise() const {
    const double X_gyro=0.02,Y_gyro=0.02,Z_gyro=0.02;
    const double X_acc=50.0,Y_acc=50.0,Z_acc=50.0;
    const double deg2rad = M_PI/180.0;
    const double gyro_rad = deg2rad / 3600.0;
    const double g_noise = 9.8;
    Eigen::VectorXd q(6);
    q << std::pow(X_gyro*gyro_rad,2),
         std::pow(Y_gyro*gyro_rad,2),
         std::pow(Z_gyro*gyro_rad,2),
         std::pow(X_acc*1e-6*g_noise,2),
         std::pow(Y_acc*1e-6*g_noise,2),
         std::pow(Z_acc*1e-6*g_noise,2);
    return q.asDiagonal();
}

Eigen::MatrixXd UkfInitializer::initializeCovarianceMatrix(double lat) const {
    const double P0_Attitude = 1.0/60.0, P0_Heading=0.1;
    const double P0_VE=0.01,P0_VN=0.01,P0_VU=0.01;
    const double P0_Pos_H=0.1, P0_Pos_alt=0.15;
    const double P0_Gx=0.01,P0_Gy=0.01,P0_Gz=0.01;
    const double P0_Ax=50.0,P0_Ay=50.0,P0_Az=50.0;
    const double Re = 6378135.072, deg2rad=M_PI/180.0, gyro_rad = deg2rad/3600.0;
    const double lat_rad = lat*deg2rad;

    Eigen::VectorXd p(15);
    p << std::pow(P0_Attitude*deg2rad,2),
         std::pow(P0_Attitude*deg2rad,2),
         std::pow(P0_Heading *deg2rad,2),
         std::pow(P0_VE,2), std::pow(P0_VN,2), std::pow(P0_VU,2),
         std::pow(P0_Pos_H/Re,2),
         std::pow(P0_Pos_H/(Re*std::cos(lat_rad)),2),
         std::pow(P0_Pos_alt,2),
         std::pow(P0_Gx*gyro_rad,2), std::pow(P0_Gy*gyro_rad,2), std::pow(P0_Gz*gyro_rad,2),
         std::pow(P0_Ax*1e-6*9.8,2), std::pow(P0_Ay*1e-6*9.8,2), std::pow(P0_Az*1e-6*9.8,2);
    return p.asDiagonal();
}