/**
 * @file KfNavigation.hpp
 * @brief Implementation of Kalman Filter-based navigation system
 *
 * This class implements an INS/GPS integrated navigation system using Kalman filtering.
 * It combines inertial measurements (IMU) with GPS updates to provide robust navigation solutions.
 * Features include strapdown inertial navigation, Kalman prediction/update steps, and RTS smoothing.
 *
 * @author peanut-nav
 * @date Created: 2025-07-22
 * @last Modified: 2025-08-04
 * @version 0.3.0
 */

#pragma once
#include "core/NavigationBase.hpp"   // Base navigation interface
#include "core/RtsSmoother.hpp"      // Rauch-Tung-Striebel smoother
#include "NavigationParams.hpp"      // Navigation parameters
#include <Eigen/Dense>               // Matrix and vector operations
#include <vector>                    // Standard vector container

class KalmanFilterNavigation : public NavigationBase {
public:
    KalmanFilterNavigation() = default;
    
    // NavigationBase interface implementation
    /**
     * @brief Initialize navigation system with parameters and initial state
     * 
     * @param base_params Base navigation parameters (sensor specs, initial conditions)
     * @param state Initial navigation state (position, velocity, attitude)
     */
    void initialize(const NavParamsBase& base_params, 
                   NavigationState& state) override;
    
    /**
     * @brief Perform strapdown inertial navigation update using IMU data
     * 
     * @param imu Current IMU measurement (accelerometer + gyroscope)
     * @param i Time index for data processing
     */
    void updateStrapdown(const IMUData& imu, int i) override;
    
    /**
     * @brief Predict navigation state using Kalman filter prediction step
     * 
     * @param i Time index for state prediction
     */
    void predictState(int i) override;
    
    /**
     * @brief Update navigation state using GPS measurement
     * 
     * @param gps Current GPS measurement (position, velocity)
     * @param i Time index for measurement update
     */
    void updateMeasurement(const GPSData& gps, int i) override;
    
    /**
     * @brief Correct navigation state errors using Kalman filter estimates
     * 
     * @param i Time index for error correction
     */
    void correctErrors(int i) override;
    
    /**
     * @brief Get current navigation state
     * 
     * @return Reference to current navigation state
     */
    NavigationState& getState() override { return state_; }
    
    /**
     * @brief Advance to next time step in navigation processing
     */
    void advance() override;
    
    /**
     * @brief Check if current time step requires measurement update
     * 
     * @param i Current time index
     * @return true if measurement update should be performed
     */
    bool isMeasurementStep(int i) const override;
    
    /**
     * @brief Execute full navigation processing sequence
     * 
     * @param imu IMU data sequence
     * @param gps GPS data sequence
     */
    void run(const IMUData& imu, const GPSData& gps) override;

    /**
     * @brief Get reference to RTS smoother instance
     * 
     * @return Reference to Rauch-Tung-Striebel smoother
     */
    RtsSmoother& getRtsSmoother() { return rts_smoother_; }

private:
    // Configuration parameters
    KfParams params_;                 // Kalman filter specific parameters
    int measurement_interval_;        // GPS update interval (in IMU cycles)
    
    // Navigation state
    NavigationState state_;           // Current navigation state (position, velocity, attitude)
    int current_index_ = 0;           // Current time index in processing sequence
    int kalman_index_ = 2;            // Kalman filter update cycle counter
    
    // Earth reference parameters
    double current_Rx_ = 0.0;         // Current meridian radius of curvature
    double current_Ry_ = 0.0;         // Current transverse radius of curvature
    Eigen::Vector3d wie_n_ = Eigen::Vector3d::Zero();  // Earth rotation rate in navigation frame
    Eigen::Vector3d wet_t_ = Eigen::Vector3d::Zero();  // Transport rate in navigation frame
    Eigen::Vector3d wit_b_ = Eigen::Vector3d::Zero();  // Total angular rate in body frame
    
    // Kalman filter state
    KalmanFilterParams kalman_;       // Kalman filter covariance matrices
    Eigen::Vector3d last_f_INSt_;     // Previous specific force in navigation frame (for state transition)
    
    // Strapdown inertial navigation components
    /**
     * @brief Update attitude using angular rate measurements
     * 
     * @param wtb_b Angular rate of body w.r.t inertial frame (body frame)
     */
    void updateAttitude(const Eigen::Vector3d& wtb_b);
    
    /**
     * @brief Update velocity and position using specific force
     * 
     * @param f_INSt Specific force in navigation frame
     */
    void updateVelocityPosition(const Eigen::Vector3d& f_INSt);
    
    // Kalman filter processing components
    /**
     * @brief Compute state transition matrix for Kalman filter
     * 
     * @param i Current time index
     */
    void computeStateTransitionMatrix(int i);
    
    /**
     * @brief Compute measurement matrix for Kalman filter
     * 
     * @param i Current time index
     */
    void computeMeasurementMatrix(int i);
    
    /**
     * @brief Execute Kalman filter prediction step
     * 
     * @param i Current time index
     */
    void runKalmanPrediction(int i);
    
    /**
     * @brief Execute Kalman filter update step
     * 
     * @param i Current time index
     * @param Z Measurement vector
     */
    void runKalmanUpdate(int i, const Eigen::VectorXd& Z);
    
    // Core algorithm implementations
    /**
     * @brief Update attitude using quaternion integration
     * 
     * @param wtb_b Angular rate (body frame)
     * @param wit_b Total angular rate (body frame)
     * @param quat Attitude quaternion (input/output)
     * @param IMUrate IMU sampling rate (Hz)
     * @param CbtM Direction cosine matrix (output)
     */
    void strapdownAttitudeUpdate(const Eigen::Vector3d& wtb_b,
                                const Eigen::Vector3d& wit_b,
                                Eigen::Vector4d& quat,
                                int IMUrate,
                                Eigen::Matrix3d& CbtM);
    
    /**
     * @brief Update velocity and position using mechanization equations
     * 
     * @param f_INSt Specific force in navigation frame
     * @param V_prev Previous velocity
     * @param Lat_prev Previous latitude
     * @param Lon_prev Previous longitude
     * @param h_prev Previous altitude
     * @param wie_n Earth rotation rate (nav frame)
     * @param wet_t Transport rate (nav frame)
     * @param g Gravity magnitude
     * @param IMUrate IMU sampling rate (Hz)
     * @param Re Earth radius
     * @param e Eccentricity
     * @param Rx_prev Previous meridian radius
     * @param Ry_prev Previous transverse radius
     * @param V_new Updated velocity (output)
     * @param Lat_new Updated latitude (output)
     * @param Lon_new Updated longitude (output)
     * @param h_new Updated altitude (output)
     * @param Rx Updated meridian radius (output)
     * @param Ry Updated transverse radius (output)
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
     * @brief Compute gravity magnitude using gravity model
     * 
     * @param Latitude Current latitude
     * @param h Current altitude
     * @param params Navigation parameters
     * @return Gravity magnitude
     */
    double computeGravity(double Latitude, double h, const NavigationParams& params);
    
    /**
     * @brief Compute Kalman state transition matrix
     * 
     * @param Latitude Current latitude
     * @param V Current velocity
     * @param h Current altitude
     * @param Rx Meridian radius of curvature
     * @param Ry Transverse radius of curvature
     * @param CtbM Direction cosine matrix (navigation to body)
     * @param W_ie Earth rotation rate
     * @param f_INSt Specific force in navigation frame
     * @return State transition matrix
     */
    Eigen::MatrixXd kalmanComputeStateMatrix(double Latitude,
                                            const Eigen::Vector3d& V,
                                            double h,
                                            double Rx,
                                            double Ry,
                                            const Eigen::Matrix3d& CtbM,
                                            double W_ie,
                                            const Eigen::Vector3d& f_INSt);
    
    /**
     * @brief Compute Kalman measurement matrix
     * 
     * @param roll Current roll angle
     * @param yaw Current yaw angle
     * @param pitch Current pitch angle
     * @param Latitude Current latitude
     * @param h Current altitude
     * @param Rx Meridian radius of curvature
     * @param Ry Transverse radius of curvature
     * @return Measurement matrix
     */
    Eigen::MatrixXd kalmanComputeMeasurementMatrix(double roll,
                                                  double yaw,
                                                  double pitch,
                                                  double Latitude,
                                                  double h,
                                                  double Rx,
                                                  double Ry);
    
    /**
     * @brief Execute Kalman prediction step
     * 
     * @param X_prev Previous state vector
     * @param P_prev Previous error covariance
     * @param A State transition matrix
     * @param B Control matrix
     * @param Q Process noise covariance
     * @param T Time step
     * @param X_pred Predicted state (output)
     * @param P_pred Predicted covariance (output)
     */
    void kalmanPredictStep(const Eigen::VectorXd& X_prev,
                           const Eigen::MatrixXd& P_prev,
                           const Eigen::MatrixXd& A,
                           const Eigen::MatrixXd& B,
                           const Eigen::MatrixXd& Q,
                           double T,
                           Eigen::VectorXd& X_pred,
                           Eigen::MatrixXd& P_pred);
    
    /**
     * @brief Execute Kalman update step
     * 
     * @param X_pred Predicted state
     * @param P_pred Predicted covariance
     * @param Z Measurement vector
     * @param H Measurement matrix
     * @param R Measurement noise covariance
     * @param X_new Updated state (output)
     * @param P_new Updated covariance (output)
     */
    void kalmanUpdateStep(const Eigen::VectorXd& X_pred,
                          const Eigen::MatrixXd& P_pred,
                          const Eigen::VectorXd& Z,
                          const Eigen::MatrixXd& H,
                          const Eigen::MatrixXd& R,
                          Eigen::VectorXd& X_new,
                          Eigen::MatrixXd& P_new);

    RtsSmoother rts_smoother_;  // Rauch-Tung-Striebel smoother for backward pass
};
