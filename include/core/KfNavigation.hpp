/**
 * @file KalmanFilterNavigation.hpp
 * @brief Kalman filter implementation for navigation
 *
 * @author peanut-nav
 * @date Created: 2025-07-22
 * @last Modified: 2025-07-22
 * @version 0.1
 */

#pragma once
#include "core/NavigationFilterBase.hpp"
#include "NavigationParams.hpp"

class KalmanFilterNavigation : public NavigationFilterBase {
public:
    KalmanFilterNavigation() = default;
    
    // NavigationFilterBase interface implementation
    void initialize(const NavParamsBase& base_params, 
                   NavigationState& state) override;
    
    void updateStrapdown(const IMUData& imu) override;
    
    void predictState() override;
    
    void updateMeasurement(const GPSData& measurement) override;
    
    void correctErrors() override;
    
    NavigationState& getState() override { return state_; }
    
    void advance() override;
    
    bool isMeasurementStep() const override;
    
    std::string getType() const override { return "Kalman Filter"; }
    
    int getCurrentIndex() const override { return current_index_; }

private:
    // Configuration
    KfParams params_;
    int measurement_interval_;
    
    // State
    NavigationState state_;
    int current_index_ = 0;
    int kalman_index_ = 2; // Kalman update counter
    
    // Earth parameters
    double current_Rx_ = 0.0;
    double current_Ry_ = 0.0;
    Eigen::Vector3d wie_n_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d wet_t_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d wit_b_ = Eigen::Vector3d::Zero();
    
    // Kalman filter specific
    KalmanFilterParams kalman_;
    Eigen::Vector3d last_f_INSt_; // Last specific force in nav frame
    
    // Internal strapdown update components
    void updateAttitude(const Eigen::Vector3d& wtb_b);
    void updateVelocityPosition(const Eigen::Vector3d& f_INSt);
    
    // Kalman filter components
    void computeStateTransitionMatrix();
    void computeMeasurementMatrix();
    void runKalmanUpdate();
    
    // Algorithm implementations
    void strapdownAttitudeUpdate(const Eigen::Vector3d& wtb_b,
                                const Eigen::Vector3d& wit_b,
                                Eigen::Vector4d& quat,
                                int IMUrate,
                                Eigen::Matrix3d& CbtM);
    
    void strapdownVelocityPositionUpdate(const Eigen::Vector3d& f_INSt,
                                        const Eigen::Vector3d& V_prev,
                                        double Lat_prev,
                                        double Lon_prev,
                                        double h_prev,
                                        const Eigen::Vector3d& wie_n,
                                        const Eigen::Vector3d& wet_t,
                                        double g,
                                        int IMUrate,
                                        double Re,
                                        double e,
                                        double Rx_prev,
                                        double Ry_prev,
                                        Eigen::Vector3d& V_new,
                                        double& Lat_new,
                                        double& Lon_new,
                                        double& h_new,
                                        double& Rx,
                                        double& Ry);
    
    void computeEulerAngles(const Eigen::Matrix3d& CbtM,
                           double& pitch,
                           double& roll,
                           double& yaw);
    
    double computeGravity(double Latitude, double h, const NavigationParams& params);
    
    Eigen::MatrixXd kalmanComputeStateMatrix(double Latitude,
                                            const Eigen::Vector3d& V,
                                            double h,
                                            double Rx,
                                            double Ry,
                                            const Eigen::Matrix3d& CtbM,
                                            double W_ie,
                                            const Eigen::Vector3d& f_INSt);
    
    Eigen::MatrixXd kalmanComputeMeasurementMatrix(double roll,
                                                  double yaw,
                                                  double pitch,
                                                  double Latitude,
                                                  double h,
                                                  double Rx,
                                                  double Ry);
    
    void kalmanFilterStep(const Eigen::VectorXd& X_prev,
                          const Eigen::MatrixXd& P_prev,
                          const Eigen::VectorXd& Z,
                          const Eigen::MatrixXd& H,
                          const Eigen::MatrixXd& R,
                          const Eigen::MatrixXd& Q,
                          double T,
                          const Eigen::MatrixXd& A,
                          const Eigen::MatrixXd& B,
                          Eigen::VectorXd& X_new,
                          Eigen::MatrixXd& P_new);
};
