/**
 * @file UkfNavigation.hpp
 * @brief Unscented Kalman Filter (UKF) based navigation implementation
 *
 * @author peanut-nav
 * @date Created: 2025-08-10
 * @last Modified: 2025-08-10
 * @version 0.3.3
 */

#pragma once
#include "core/NavigationBase.hpp"
#include "../params/UkfParams.hpp"
#include "core/RtsSmoother.hpp"
#include "MathUtils.hpp"
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

class UkfNavigation : public NavigationBase {
public:
    UkfNavigation() = default;
    
    // NavigationBase interface implementation
    
    /**
     * @brief Initialize the UKF navigation system
     * 
     * @param base_params Navigation parameters
     * @param state Initial navigation state
     */
    void initialize(const NavParamsBase& base_params, 
                   NavigationState& state) override;
    
    /**
     * @brief Perform strapdown inertial navigation update
     * 
     * @param imu Current IMU measurement
     * @param i Time index
     */
    void updateStrapdown(const IMUData& imu, int i) override;
    
    /**
     * @brief Predict navigation state using UKF
     * 
     * @param i Time index
     */
    void predictState(int i) override;
    
    /**
     * @brief Update navigation state using GPS measurement
     * 
     * @param gps Current GPS measurement
     * @param i Time index
     */
    void updateMeasurement(const GPSData& gps, int i) override;
    
    /**
     * @brief Correct navigation state errors
     * 
     * @param i Time index
     */
    void correctErrors(int i) override;
    
    /**
     * @brief Get current navigation state
     * 
     * @return Reference to navigation state
     */
    NavigationState& getState() override { return state_; }
    
    /**
     * @brief Check if current step is measurement update step
     * 
     * @param i Time index
     * @return true if measurement update should be performed
     */
    bool isMeasurementStep(int i) const override;
    
    /**
     * @brief Execute full navigation processing sequence
     * 
     * @param imu IMU data
     * @param gps GPS data
     */
    void run(const IMUData& imu, const GPSData& gps) override;

    /**
     * @brief Get reference to RTS smoother
     * 
     * @return Reference to RTS smoother
     */
    RtsSmoother& getRtsSmoother() override { return rts_smoother_; }

private:
    // Configuration
    UkfParams params_;                 ///< UKF specific parameters
    int measurement_interval_;         ///< GPS update interval in IMU cycles
    
    // State
    NavigationState state_;            ///< Current navigation state
    int current_index_ = 0;            ///< Current time index
    int kalman_index_ = 2;             ///< Kalman update counter
    
    // Earth reference parameters
    double current_Rx_ = 0.0;          ///< Current meridian radius of curvature
    double current_Ry_ = 0.0;          ///< Current transverse radius of curvature
    Eigen::Vector3d wie_n_ = Eigen::Vector3d::Zero();  ///< Earth rotation rate in nav frame
    Eigen::Vector3d wet_t_ = Eigen::Vector3d::Zero();  ///< Transport rate in tangent frame
    Eigen::Vector3d wit_b_ = Eigen::Vector3d::Zero();  ///< Total angular rate in body frame
    
    // UKF state
    UnscentedKalmanFilterParams ukf_;   ///< UKF covariance matrices
    Eigen::Vector3d last_f_INSt_;      ///< Previous specific force in nav frame

    // UKF processing components
    Eigen::Matrix<double,15,15> errorDynamics(
    const Eigen::Matrix3d& Cnb,
    const Eigen::Vector3d& vn0,   // [Ve, Vn, Vu]
    const Eigen::Vector3d& aibn,  // f in n-frame
    double wie, double lat, double rxn, double ryn);
    
    // Core algorithm implementations
    /**
     * @brief Update attitude using quaternion integration
     */
    void strapdownAttitudeUpdate(const Eigen::Vector3d& wtb_b,
                                const Eigen::Vector3d& wit_b,
                                Eigen::Vector4d& quat,
                                int IMUrate,
                                Eigen::Matrix3d& CbtM);
    
    /**
     * @brief Update velocity and position using mechanization
     */
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
    
    /**
     * @brief Compute gravity magnitude
     * 
     * @return Gravity magnitude
     */
    double computeGravity(double Latitude, double h, const NavigationParams& params);
    
    // UKF specific functions
    // 连续到离散：Gz、级数法离散化 Φ, Q
    void buildGz(const Eigen::Matrix3d& CtbM, Eigen::MatrixXd& Gz) const;
    void discretizeBySeries(const Eigen::MatrixXd& f, double Tao,
                            const Eigen::MatrixXd& Gz, const Eigen::MatrixXd& Q0,
                            Eigen::MatrixXd& Fai, Eigen::MatrixXd& Q) const;

    // 生成 σ 点并线性传播：Xsig -> Xprop
    void generateSigmaPoints(const Eigen::VectorXd& x, const Eigen::MatrixXd& P,
                             double lambda, Eigen::MatrixXd& Xsig) const;
    
    /**
     * @brief Build measurement matrix
     * 
     * @return Measurement matrix
     */
    Eigen::MatrixXd buildMeasurementMatrix(double roll, double pitch, double yaw,
                                          double lat, double h, double Rx, double Ry);

    RtsSmoother rts_smoother_;  ///< RTS smoother for backward pass
};
