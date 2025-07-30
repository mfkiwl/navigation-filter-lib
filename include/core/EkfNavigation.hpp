/**
 * @file EkfNavigation.hpp
 * @brief Extended Kalman Filter navigation implementation
 *
 * @author peanut-nav
 * @date Created: 2025-07-22
 * @last Modified: 2025-07-22
 * @version 0.1
 */

#pragma once
#include "core/NavigationBase.hpp"
#include "NavigationParams.hpp"
#include "MathUtils.hpp"
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

class EkfNavigation : public NavigationBase {
public:
    EkfNavigation() = default;
    
    // NavigationBase interface implementation
    void initialize(const NavParamsBase& base_params, 
                   NavigationState& state) override;
    
    void updateStrapdown(const IMUData& imu, int i) override;
    
    void predictState(int i) override;
    
    void updateMeasurement(const GPSData& gps, int i) override;
    
    void correctErrors(int i) override;
    
    NavigationState& getState() override { return state_; }
    
    void advance() override;
    
    bool isMeasurementStep(int i) const override;
    
    void run(const IMUData& imu, const GPSData& gps) override;

private:
    // Configuration
    EkfParams params_;
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
    
    // EKF specific
    ExtendedKalmanFilterParams ekf_;
    Eigen::Vector3d last_f_INSt_; // Last specific force in nav frame
    
    // Internal strapdown update components
    void updateAttitude(const Eigen::Vector3d& wtb_b);
    void updateVelocityPosition(const Eigen::Vector3d& f_INSt);
    
    // EKF components
    void computeJacobianMatrix(int i);
    void discretizeSystem(double dt);
    void runEkfPrediction(int i);
    void runEkfUpdate(int i, const Eigen::VectorXd& Z);
    
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
    
    double computeGravity(double Latitude, double h, const NavigationParams& params);
    
    // EKF specific functions
    Eigen::VectorXd computeStateDerivative(const Eigen::VectorXd& X, 
                                          const Eigen::MatrixXd& parm);
    
    Eigen::VectorXd rungeKutta4(const Eigen::VectorXd& X, 
                               const Eigen::MatrixXd& Uparm, 
                               double dt);
    
    Eigen::MatrixXd computeJacobian(const Eigen::VectorXd& X,
                                   const Eigen::MatrixXd& parm);
    
    void ltiDiscretize(const Eigen::MatrixXd& F, 
                      const Eigen::MatrixXd& G, 
                      const Eigen::MatrixXd& Qc, 
                      double dt,
                      Eigen::MatrixXd& A, 
                      Eigen::MatrixXd& Q);
    
    // Helper functions
    Eigen::MatrixXd buildMeasurementMatrix(double roll, double pitch, double yaw,
                                          double lat, double h, double Rx, double Ry);
    
    // Parameters for EKF prediction
    Eigen::MatrixXd parm_old_;
    Eigen::MatrixXd parm_mid_;
    Eigen::MatrixXd parm_new_;
};
