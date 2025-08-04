#pragma once
#include <vector>
#include <Eigen/Dense>
#include "NavigationParams.hpp"
#include "MathUtils.hpp"

class RtsSmoother {
public:
    // 历史数据结构
    struct FilterHistory {
        Eigen::VectorXd state;               // 后验滤波状态 (x_filter_save)
        Eigen::VectorXd predicted_state;     // 预测状态 (x_expect_save)
        Eigen::MatrixXd covariance;          // 后验滤波协方差 (p_filter_save)
        Eigen::MatrixXd predicted_covariance;// 预测协方差 (p_expect_save)
        Eigen::MatrixXd transition_matrix;   // 状态转移矩阵
        NavigationState nav_state;           // 导航状态快照
    };

    // 平滑结果
    struct SmoothResult {
        std::vector<Eigen::VectorXd> smoothed_states;      // 平滑后的状态
        std::vector<Eigen::MatrixXd> smoothed_covariances; // 平滑后的协方差
        std::vector<NavigationState> smoothed_nav_states;  // 平滑后的导航状态
        std::vector<Eigen::Vector3d> velocity_smooth;      // 平滑后的速度
        std::vector<Eigen::Vector3d> position_smooth;      // 平滑后的位置 [纬度, 经度, 高度]
        std::vector<Eigen::Vector3d> attitude_smooth;      // 平滑后的姿态 [航向, 俯仰, 横滚]
    };

    /**
     * @brief 添加历史数据点
     * 
     * @param history_item 历史数据结构
     */
    void addHistoryItem(const FilterHistory& history_item);
    
    /**
     * @brief 执行RTS平滑
     * 
     * @param process_noise 过程噪声矩阵
     * @return SmoothResult 平滑结果
     */
    SmoothResult smooth(const Eigen::MatrixXd& process_noise);
    
    /**
     * @brief 生成平滑后的导航结果
     * 
     * @param result 平滑结果
     * @param totalPoints 总点数
     * @return std::vector<std::vector<double>> 平滑后的导航数据矩阵
     */
    std::vector<std::vector<double>> generateSmoothedNavigation(const SmoothResult& result, int totalPoints);
    
    /**
     * @brief 执行后处理导航解算
     * 
     * @param imu IMU数据
     * @param track 参考轨迹
     * @param initial_state 初始导航状态
     * @param earth_params 地球参数
     * @param smooth_result 平滑结果
     * @param gps_rate GPS更新率
     * @param imu_rate IMU更新率
     * @return NavigationState 后处理导航状态
     */
    NavigationState postProcessNavigation(
        const IMUData& imu,
        const TrajectoryData& track,
        const NavigationState& initial_state,
        const NavigationParams& earth_params,
        const SmoothResult& smooth_result,
        int gps_rate,
        int imu_rate);

private:
    std::vector<FilterHistory> history_;  // 历史数据存储

    /**
     * @brief 计算失准角对应的方向余弦矩阵
     * 
     * @param E_err 东失准角 (弧度)
     * @param N_err 北失准角 (弧度)
     * @param U_err 天失准角 (弧度)
     * @return Eigen::Matrix3d 方向余弦矩阵
     */
    Eigen::Matrix3d computeCtn(double E_err, double N_err, double U_err) const;
    
    /**
     * @brief 计算原始方向余弦矩阵
     * 
     * @param yaw 航向角 (度)
     * @param pitch 俯仰角 (度)
     * @param roll 横滚角 (度)
     * @return Eigen::Matrix3d 方向余弦矩阵
     */
    Eigen::Matrix3d computeCnb(double yaw, double pitch, double roll) const;
};

