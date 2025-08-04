#include "core/RtsSmoother.hpp"
#include "core/NavigationBase.hpp"
#include <iostream>
#include <cmath>
#include <unsupported/Eigen/MatrixFunctions>

using namespace Eigen;
using namespace std;

void RtsSmoother::addHistoryItem(const FilterHistory& history_item) {
    history_.push_back(history_item);
}

RtsSmoother::SmoothResult RtsSmoother::smooth(const Eigen::MatrixXd& process_noise) {
    int n = history_.size();
    if (n == 0) {
        return SmoothResult();
    }
    
    SmoothResult result;
    
    // 初始化最后一个点
    result.smoothed_states.resize(n);
    result.smoothed_covariances.resize(n);
    result.smoothed_nav_states.resize(n);
    result.velocity_smooth.resize(n);
    result.position_smooth.resize(n);
    result.attitude_smooth.resize(n);
    
    // 最后一个点直接使用滤波结果
    int last_idx = n-1;
    result.smoothed_states[last_idx] = history_[last_idx].state;
    result.smoothed_covariances[last_idx] = history_[last_idx].covariance;
    result.smoothed_nav_states[last_idx] = history_[last_idx].nav_state;
    
    // 后向传递
    for (int i = n-2; i >= 0; --i) {
        // 计算平滑增益
        MatrixXd Ks = history_[i].covariance * 
                     history_[i].transition_matrix.transpose() * 
                     history_[i+1].predicted_covariance.inverse();
        
        // 状态平滑更新
        VectorXd state_diff = result.smoothed_states[i+1] - history_[i+1].predicted_state;
        result.smoothed_states[i] = history_[i].state + Ks * state_diff;
        
        // 协方差平滑更新
        MatrixXd cov_diff = history_[i+1].predicted_covariance - result.smoothed_covariances[i+1];
        result.smoothed_covariances[i] = history_[i].covariance - 
                                        Ks * cov_diff * Ks.transpose();
        
        // 保存平滑后的导航状态
        result.smoothed_nav_states[i] = history_[i].nav_state;
        
        // 应用校正到导航状态
        // 速度校正
        Vector3d velocity_correction = result.smoothed_states[i].segment(3, 3);
        result.smoothed_nav_states[i].Velocity[0] -= velocity_correction;
        result.velocity_smooth[i] = result.smoothed_nav_states[i].Velocity[0];
        
        // 位置校正
        result.smoothed_nav_states[i].Latitude[0] -= result.smoothed_states[i](6) * (180.0/M_PI);
        result.smoothed_nav_states[i].Longitude[0] -= result.smoothed_states[i](7) * (180.0/M_PI);
        result.smoothed_nav_states[i].Altitude[0] -= result.smoothed_states[i](8);
        result.position_smooth[i] = {
            result.smoothed_nav_states[i].Latitude[0],
            result.smoothed_nav_states[i].Longitude[0],
            result.smoothed_nav_states[i].Altitude[0]
        };
        
        // 姿态校正
        double E_err = result.smoothed_states[i](0);  // 东失准角 (弧度)
        double N_err = result.smoothed_states[i](1);  // 北失准角 (弧度)
        double U_err = result.smoothed_states[i](2);  // 天失准角 (弧度)
        
        // 计算失准角对应的方向余弦矩阵
        Matrix3d Ctn = computeCtn(E_err, N_err, U_err);
        
        // 获取原始姿态角
        double yaw = history_[i].nav_state.Yaw[0];
        double pitch = history_[i].nav_state.Pitch[0];
        double roll = history_[i].nav_state.Roll[0];
        
        // 计算原始方向余弦矩阵
        Matrix3d Cnb = computeCnb(yaw, pitch, roll);
        
        // 应用平滑修正 (左乘Ctn)
        Cnb = Ctn * Cnb;
        
        // 更新姿态矩阵
        result.smoothed_nav_states[i].CbtM = Cnb;
        result.smoothed_nav_states[i].CtbM = Cnb.transpose();
        
        // 重新计算欧拉角
        double new_pitch, new_roll, new_yaw;
        NavigationUtils::calculateEulerAngles(Cnb, new_pitch, new_roll, new_yaw);
        
        result.smoothed_nav_states[i].Pitch[0] = new_pitch;
        result.smoothed_nav_states[i].Roll[0] = new_roll;
        result.smoothed_nav_states[i].Yaw[0] = new_yaw;
        result.attitude_smooth[i] = {new_yaw, new_pitch, new_roll};
    }
    
    return result;
}

Eigen::Matrix3d RtsSmoother::computeCtn(double E_err, double N_err, double U_err) const {
    // 直接实现MATLAB中的Ctn计算
    double cosE = cos(E_err);
    double sinE = sin(E_err);
    double cosN = cos(N_err);
    double sinN = sin(N_err);
    double cosU = cos(U_err);
    double sinU = sin(U_err);
    
    Matrix3d Ctn;
    Ctn << cosN*cosU - sinN*sinE*sinU, 
           cosN*sinU + sinN*sinE*cosU,
           -sinN*cosE,
           -cosE*sinU,
           cosE*cosU,
           sinE,
           sinN*cosU + cosN*sinE*sinU,
           sinN*sinU - cosN*sinE*cosU,
           cosN*cosE;
    
    return Ctn;
}

Eigen::Matrix3d RtsSmoother::computeCnb(double yaw, double pitch, double roll) const {
    // 将角度转换为弧度
    double yaw_rad = yaw * M_PI/180.0;
    double pitch_rad = pitch * M_PI/180.0;
    double roll_rad = roll * M_PI/180.0;
    
    // 计算三角函数值
    double cy = cos(yaw_rad);
    double sy = sin(yaw_rad);
    double cp = cos(pitch_rad);
    double sp = sin(pitch_rad);
    double cr = cos(roll_rad);
    double sr = sin(roll_rad);
    
    // 直接实现MATLAB中的Cnb计算
    Matrix3d Cnb;
    Cnb << cr*cy - sr*sp*sy,
           cr*sy + sr*sp*cy,
           -sr*cp,
           -cp*sy,
           cp*cy,
           sp,
           sr*cy + cr*sp*sy,
           sr*sy - cr*sp*cy,
           cr*cp;
    
    return Cnb;
}

std::vector<std::vector<double>> RtsSmoother::generateSmoothedNavigation(
    const SmoothResult& result, int totalPoints) {
    
    std::vector<std::vector<double>> smooth_filter;
    int n = result.smoothed_states.size();
    
    // 确保有足够的数据点
    if (n == 0) return smooth_filter;
    
    // 创建并初始化矩阵 (类似MATLAB的zeros)
    smooth_filter.resize(totalPoints, std::vector<double>(10, 0.0));
    
    // 填充平滑结果 (严格遵循MATLAB逻辑)
    for (int i = 0; i < n; i++) {
        smooth_filter[i][0] = i + 1;  // 时间索引
        smooth_filter[i][1] = result.position_smooth[i][0];  // 平滑纬度
        smooth_filter[i][2] = result.position_smooth[i][1];  // 平滑经度
        smooth_filter[i][3] = result.position_smooth[i][2];  // 平滑高度
        smooth_filter[i][4] = result.velocity_smooth[i][0];  // 平滑东向速度
        smooth_filter[i][5] = result.velocity_smooth[i][1];  // 平滑北向速度
        smooth_filter[i][6] = result.velocity_smooth[i][2];  // 平滑天向速度
        smooth_filter[i][7] = result.attitude_smooth[i][0];  // 平滑航向角
        smooth_filter[i][8] = result.attitude_smooth[i][1];  // 平滑俯仰角
        smooth_filter[i][9] = result.attitude_smooth[i][2];  // 平滑横滚角
    }
    
    return smooth_filter;
}

NavigationState RtsSmoother::postProcessNavigation(
    const IMUData& imu,
    const TrajectoryData& track,
    const NavigationState& initial_state,
    const NavigationParams& earth_params,
    const SmoothResult& smooth_result,
    int gps_rate,
    int imu_rate) {
    
    // 初始化导航状态
    NavigationState nav_state = initial_state;
    int totalPoints = nav_state.Latitude.size();
    
    // 重置状态向量 (类似MATLAB中的置零)
    for (int i = 0; i < totalPoints; i++) {
        nav_state.Latitude[i] = 0;
        nav_state.Longitude[i] = 0;
        nav_state.Altitude[i] = 0;
        nav_state.Velocity[i] = Eigen::Vector3d::Zero();
        nav_state.Pitch[i] = 0;
        nav_state.Roll[i] = 0;
        nav_state.Yaw[i] = 0;
    }
    
    // 设置初始值 (严格遵循MATLAB逻辑)
    nav_state.Pitch[0] = initial_state.Pitch[0];
    nav_state.Roll[0] = initial_state.Roll[0];
    nav_state.Yaw[0] = initial_state.Yaw[0];
    nav_state.Latitude[0] = initial_state.Latitude[0];
    nav_state.Longitude[0] = initial_state.Longitude[0];
    nav_state.Altitude[0] = initial_state.Altitude[0];
    nav_state.Velocity[0] = initial_state.Velocity[0];
    nav_state.CbtM = initial_state.CbtM;
    nav_state.CtbM = initial_state.CtbM;
    
    // 地球参数
    double Re = earth_params.Re;
    double e = earth_params.e;
    double W_ie = earth_params.W_ie;
    
    // 计算初始曲率半径
    double lat_rad = nav_state.Latitude[0] * M_PI/180.0;
    double sinLat = sin(lat_rad);
    double Rx = Re / (1 - e * sinLat * sinLat);
    double Ry = Re / (1 + 2*e - 3*e * sinLat * sinLat);
    
    // 地球自转相关向量初始化
    Eigen::Vector3d wie_n(0, W_ie * cos(lat_rad), W_ie * sin(lat_rad));
    Eigen::Vector3d wet_t(
        -nav_state.Velocity[0][1]/(Ry + nav_state.Altitude[0]),
        nav_state.Velocity[0][0]/(Rx + nav_state.Altitude[0]),
        nav_state.Velocity[0][0] * tan(lat_rad)/(Rx + nav_state.Altitude[0])
    );
    Eigen::Vector3d wit_t = wie_n + wet_t;
    Eigen::Vector3d wit_b = nav_state.CbtM * wit_t;
    
    // 初始化四元数
    Eigen::Vector4d zt4 = NavigationUtils::eulerToQuaternion(
        nav_state.Pitch[0], nav_state.Roll[0], nav_state.Yaw[0]);
    
    // 循环变量初始化
    int i = 0;
    int ii = 0; // 平滑结果索引
    int NavEnd = imu.index.size();
    int measurement_interval = imu_rate / gps_rate;
    bool Kalman_flag = false;
    
    // *************** 后处理导航解算主循环 ***************
    for (int i = 0; i < NavEnd - 1; ++i) {
        // 角速度测量处理
        Eigen::Vector3d wtb_b(
            imu.gx[i] * M_PI/(180.0*3600.0),  // deg/h -> rad/s
            imu.gy[i] * M_PI/(180.0*3600.0),
            imu.gz[i] * M_PI/(180.0*3600.0)
        );
        wtb_b -= wit_b;
        
        // 四元数更新 (严格遵循MATLAB实现)
        double sita0 = wtb_b.norm() / imu_rate;
        Eigen::Matrix4d sita = Eigen::Matrix4d::Zero();
        sita << 0, -wtb_b[0], -wtb_b[1], -wtb_b[2],
                wtb_b[0], 0, wtb_b[2], -wtb_b[1],
                wtb_b[1], -wtb_b[2], 0, wtb_b[0],
                wtb_b[2], wtb_b[1], -wtb_b[0], 0;
        sita /= imu_rate;
        
        Eigen::Matrix4d rotation = cos(sita0/2) * Eigen::Matrix4d::Identity() + 
                                  (sin(sita0/2)/sita0) * sita;
        zt4 = rotation * zt4;
        zt4.normalize();
        
        // 更新方向余弦矩阵
        double q0 = zt4[0], q1 = zt4[1], q2 = zt4[2], q3 = zt4[3];
        nav_state.CbtM << 
            q0*q0 + q1*q1 - q2*q2 - q3*q3, 2*(q1*q2 + q0*q3), 2*(q1*q3 - q0*q2),
            2*(q1*q2 - q0*q3), q0*q0 - q1*q1 + q2*q2 - q3*q3, 2*(q2*q3 + q0*q1),
            2*(q1*q3 + q0*q2), 2*(q2*q3 - q0*q1), q0*q0 - q1*q1 - q2*q2 + q3*q3;
        nav_state.CtbM = nav_state.CbtM.transpose();
        
        // 比力转换到导航系
        Eigen::Vector3d f_b(imu.ax[i], imu.ay[i], imu.az[i]);
        Eigen::Vector3d f_INSt = nav_state.CtbM * f_b;
        
        // 重力计算
        double g = earth_params.g0 * (1 + earth_params.gk1 * pow(sin(lat_rad), 2)) * 
                  (1 - 2*nav_state.Altitude[i]/earth_params.Re) / 
                  sqrt(1 - earth_params.gk2 * pow(sin(lat_rad), 2));
        
        // 加速度计算
        Eigen::Vector3d coriolis = 2 * wie_n + wet_t;
        Eigen::Vector3d a = f_INSt - coriolis.cross(nav_state.Velocity[i]) + Eigen::Vector3d(0, 0, -g);
        
        // 速度更新
        nav_state.Velocity[i+1] = nav_state.Velocity[i] + a / imu_rate;
        
        // 位置更新
        double deltaT = 1.0/imu_rate;
        double Ry_h = Ry + nav_state.Altitude[i];
        double Rx_h = Rx + nav_state.Altitude[i];
        double cosLat = cos(lat_rad);
        
        nav_state.Latitude[i+1] = nav_state.Latitude[i] + 
            deltaT * (nav_state.Velocity[i][1] + nav_state.Velocity[i+1][1]) * 180.0 / (2 * Ry_h * M_PI);
        nav_state.Longitude[i+1] = nav_state.Longitude[i] + 
            deltaT * (nav_state.Velocity[i][0] + nav_state.Velocity[i+1][0]) * 180.0 / (2 * Rx_h * cosLat * M_PI);
        nav_state.Altitude[i+1] = nav_state.Altitude[i] + 
            deltaT * (nav_state.Velocity[i][2] + nav_state.Velocity[i+1][2]) / 2.0;
        
        // 更新地球曲率半径
        lat_rad = nav_state.Latitude[i+1] * M_PI/180.0;
        sinLat = sin(lat_rad);
        Rx = Re / (1 - e * sinLat * sinLat);
        Ry = Re / (1 + 2*e - 3*e * sinLat * sinLat);
        
        // 更新导航系角速度
        wie_n = Eigen::Vector3d(0, W_ie * cos(lat_rad), W_ie * sin(lat_rad));
        wet_t = Eigen::Vector3d(
            -nav_state.Velocity[i+1][1]/(Ry + nav_state.Altitude[i+1]),
            nav_state.Velocity[i+1][0]/(Rx + nav_state.Altitude[i+1]),
            nav_state.Velocity[i+1][0] * tan(lat_rad)/(Rx + nav_state.Altitude[i+1])
        );
        wit_t = wie_n + wet_t;
        wit_b = nav_state.CbtM * wit_t;
        
        // 在滤波点处应用平滑修正
        if (i > 2) {
            if ((i + 1) % measurement_interval == 0) {
                if (ii < smooth_result.position_smooth.size()) {
                    // 应用平滑后的位置、速度和高度
                    nav_state.Latitude[i+1] = smooth_result.position_smooth[ii][0];
                    nav_state.Longitude[i+1] = smooth_result.position_smooth[ii][1];
                    nav_state.Altitude[i+1] = smooth_result.position_smooth[ii][2];
                    nav_state.Velocity[i+1] = smooth_result.velocity_smooth[ii];
                    
                    // 应用平滑后的姿态
                    nav_state.Yaw[i+1] = smooth_result.attitude_smooth[ii][0];
                    nav_state.Pitch[i+1] = smooth_result.attitude_smooth[ii][1];
                    nav_state.Roll[i+1] = smooth_result.attitude_smooth[ii][2];
                    
                    Kalman_flag = true;
                    
                    // 更新四元数以匹配新的姿态
                    zt4 = NavigationUtils::eulerToQuaternion(
                        nav_state.Pitch[i+1], nav_state.Roll[i+1], nav_state.Yaw[i+1]);
                }
                ii++;
            }
        }
        
        // 如果未应用平滑修正，计算姿态角
        if (!Kalman_flag) {
            // 俯仰角计算
            nav_state.Pitch[i+1] = asin(nav_state.CbtM(1, 2)) * 180.0 / M_PI;
            
            // 横滚角计算 (处理奇异情况)
            if (abs(nav_state.CbtM(2, 2)) < 1e-16) {
                if (nav_state.CbtM(0, 2) > 0) {
                    nav_state.Roll[i+1] = -90.0;
                } else {
                    nav_state.Roll[i+1] = 90.0;
                }
            } else {
                nav_state.Roll[i+1] = atan(-nav_state.CbtM(0, 2)/nav_state.CbtM(2, 2)) * 180.0 / M_PI;
                if (nav_state.CbtM(2, 2) < 0) {
                    if (nav_state.CbtM(0, 2) > 0) {
                        nav_state.Roll[i+1] -= 180.0;
                    } else {
                        nav_state.Roll[i+1] += 180.0;
                    }
                }
            }
            
            // 航向角计算 (处理奇异情况)
            if (abs(nav_state.CbtM(1, 1)) > 1e-16) {
                nav_state.Yaw[i+1] = atan(-nav_state.CbtM(1, 0)/nav_state.CbtM(1, 1)) * 180.0 / M_PI;
                if (nav_state.CbtM(1, 1) > 0) {
                    if (nav_state.Yaw[i+1] < 0) {
                        nav_state.Yaw[i+1] += 360.0;
                    }
                } else {
                    nav_state.Yaw[i+1] += 180.0;
                }
            } else {
                if (nav_state.CbtM(1, 0) < 0) {
                    nav_state.Yaw[i+1] = 90.0;
                } else {
                    nav_state.Yaw[i+1] = 270.0;
                }
            }
        }
        
        Kalman_flag = false;
    }
    
    return nav_state;
}
