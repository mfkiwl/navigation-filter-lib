/**
 * @file EkfNavigation.hpp
 * @brief Extended Kalman Filter (EKF) based navigation implementation
 *
 * This class implements an INS/GPS integrated navigation system using Extended Kalman Filtering.
 * It features strapdown inertial navigation, EKF prediction/update steps, and RTS smoothing.
 * The EKF handles nonlinear state transitions through Jacobian computation and Runge-Kutta integration.
 *
 * @author peanut-nav
 * @date Created: 2025-07-22
 * @last Modified: 2025-08-07
 * @version 0.3.2
 */

#pragma once
#include "core/NavigationBase.hpp"
#include "../params/EkfParams.hpp"
#include "core/RtsSmoother.hpp"
#include "MathUtils.hpp"
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

class EkfNavigation : public NavigationBase {
public:
    EkfNavigation() = default;
    
    // NavigationBase interface implementation
    
    /**
     * @brief Initialize the EKF navigation system
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
     * @brief Predict navigation state using EKF
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
    EkfParams params_;                 ///< EKF specific parameters
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
    
    // EKF state
    ExtendedKalmanFilterParams ekf_;   ///< EKF covariance matrices
    Eigen::Vector3d last_f_INSt_;      ///< Previous specific force in nav frame
    
    // EKF processing components
    /**
     * @brief Compute Jacobian matrix for EKF
     * 
     * @param i Time index
     */
    void computeJacobianMatrix(int i);
    
    /**
     * @brief Discretize continuous-time system
     * 
     * @param dt Time step
     */
    void discretizeSystem(double dt);
    
    /**
     * @brief Execute EKF prediction step
     * 
     * @param i Time index
     */
    void runEkfPrediction(int i);
    
    /**
     * @brief Execute EKF update step
     * 
     * @param i Time index
     * @param Z Measurement vector
     * @param H Measurement matrix
     */
    void runEkfUpdate(int i, const Eigen::VectorXd& Z, const Eigen::MatrixXd& H);
    
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
    
    // EKF specific functions
    /**
     * @brief Compute state derivative for continuous-time model
     * 
     * @return State derivative vector
     */
    Eigen::VectorXd computeStateDerivative(const Eigen::VectorXd& X, 
                                          const Eigen::MatrixXd& parm);
    
    /**
     * @brief Perform Runge-Kutta 4 integration
     * 
     * @return Predicted state
     */
    Eigen::VectorXd rungeKutta4(const Eigen::VectorXd& X, 
                               const Eigen::MatrixXd& Uparm, 
                               double dt);
    
    /**
     * @brief Compute Jacobian matrix numerically
     * 
     * @return Jacobian matrix
     */
    Eigen::MatrixXd computeJacobian(const Eigen::VectorXd& X,
                                   const Eigen::MatrixXd& parm);
    
    /**
     * @brief Discretize linear time-invariant system
     * 
     * @param F Continuous state matrix
     * @param G Continuous noise input matrix
     * @param Qc Continuous process noise covariance
     * @param dt Time step
     * @param A Discrete state matrix (output)
     * @param Q Discrete process noise covariance (output)
     */
    void ltiDiscretize(const Eigen::MatrixXd& F, 
                      const Eigen::MatrixXd& G, 
                      const Eigen::MatrixXd& Qc, 
                      double dt,
                      Eigen::MatrixXd& A, 
                      Eigen::MatrixXd& Q);
    
    /**
     * @brief Build measurement matrix
     * 
     * @return Measurement matrix
     */
    Eigen::MatrixXd buildMeasurementMatrix(double roll, double pitch, double yaw,
                                          double lat, double h, double Rx, double Ry);
    
    // Parameters for EKF prediction
    Eigen::MatrixXd parm_old_;  ///< Parameters at start of prediction interval
    Eigen::MatrixXd parm_mid_;  ///< Parameters at midpoint of prediction interval
    Eigen::MatrixXd parm_new_;  ///< Parameters at end of prediction interval

    Eigen::MatrixXd G_mid_;     ///< Noise input matrix at midpoint

    RtsSmoother rts_smoother_;  ///< RTS smoother for backward pass
};
