/**
 * @file UkfNavigation.hpp
 * @brief Unscented Kalman Filter (UKF) based navigation implementation.
 *
 * This header declares a UKF-driven strapdown INS/GNSS integrated navigation
 * algorithm. It provides quaternion-based attitude mechanization, velocity and
 * position updates in a local navigation frame, UKF time/measurement updates,
 * and an optional Rauch–Tung–Striebel (RTS) smoother for backward smoothing.
 *
 * Pipeline
 * --------
 * 1) `initialize()` — set up parameters, state, and covariances.
 * 2) `updateStrapdown()` — mechanize attitude/velocity/position from IMU.
 * 3) `predictState()` — UKF time update (propagate state and covariance).
 * 4) `isMeasurementStep()` — check if a GNSS update is due.
 * 5) `updateMeasurement()` — UKF measurement update using GPS/GNSS.
 * 6) `correctErrors()` — feed back estimated errors into the navigation state.
 * 7) `run()` — orchestrate the full step (mechanize → predict → update → correct).
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

/**
 * @class UkfNavigation
 * @brief Strapdown INS/GNSS integration using an Unscented Kalman Filter.
 *
 * Implements the `NavigationBase` interface to provide a complete INS/GNSS
 * integration cycle with UKF estimation and optional RTS smoothing. The class
 * maintains Earth-related rates and curvature radii and exposes the smoother
 * for post-processing.
 */
class UkfNavigation : public NavigationBase {
public:
    UkfNavigation() = default;
    
    // NavigationBase interface implementation
    
    /**
     * @brief Initialize the UKF navigation system.
     *
     * Sets UKF parameters, the measurement cadence, and the internal navigation
     * state including quaternion attitude, velocity, and position.
     *
     * @param base_params Navigation parameters (e.g., rates, Earth model).
     * @param state Initial navigation state; copied into the internal state.
     */
    void initialize(const NavParamsBase& base_params, 
                   NavigationState& state) override;
    
    /**
     * @brief Perform strapdown inertial navigation update.
     *
     * Executes the strapdown mechanization for one IMU sample:
     *  - Attitude update via quaternion integration using body angular rate.
     *  - Velocity update using specific force (resolved to n/t frame) with
     *    Coriolis, gravity, and transport-rate effects.
     *  - Position update (lat, lon, h) on the reference ellipsoid using the
     *    updated velocity and curvature radii.
     *
     * @param imu Current IMU measurement (gyro in rad/s, accel in m/s²).
     * @param i   Global time-step index (non-decreasing).
     */
    void updateStrapdown(const IMUData& imu, int i) override;
    
    /**
     * @brief Predict navigation state using the UKF (time update).
     *
     * Forms/propagates the continuous-time error dynamics, discretizes to obtain
     * the discrete transition matrix Φ and process noise Q, generates sigma
     * points, and performs the UKF time update on the error state.
     *
     * @param i Time-step index (used for book-keeping or triggers).
     */
    void predictState(int i) override;
    
    /**
     * @brief Update navigation state using a GPS measurement (measurement update).
     *
     * Builds the measurement model (matrix/function) given the current state,
     * computes the innovation, and applies the UKF measurement update to refine
     * the state estimate and covariance.
     *
     * @param gps Current GPS measurement (typically position and/or velocity).
     * @param i   Time-step index (aligns with measurement cadence).
     */
    void updateMeasurement(const GPSData& gps, int i) override;
    
    /**
     * @brief Correct the mechanized navigation solution using estimated errors.
     *
     * Applies the estimated small-angle, velocity, and position errors to the
     * mechanized solution; resets the error state as required and re-normalizes
     * the quaternion to unit length.
     *
     * @param i Time-step index.
     */
    void correctErrors(int i) override;
    
    /**
     * @brief Get the current navigation state.
     *
     * @return Reference to the internal navigation state (`state_`).
     */
    NavigationState& getState() override { return state_; }
    
    /**
     * @brief Check if the current step is a measurement-update step.
     *
     * Returns true if the current IMU index `i` matches the configured GPS/GNSS
     * update cadence (e.g., every N IMU cycles).
     *
     * @param i IMU time-step index.
     * @return true if a measurement update should be performed.
     */
    bool isMeasurementStep(int i) const override;
    
    /**
     * @brief Execute the full per-step navigation processing sequence.
     *
     * Performs strapdown mechanization → UKF time update → conditional GPS update
     * → error-state feedback correction for a single time step.
     *
     * @param imu IMU data for this step.
     * @param gps GPS data for this step (used if `isMeasurementStep(i)` is true).
     */
    void run(const IMUData& imu, const GPSData& gps) override;

    /**
     * @brief Access the RTS smoother.
     *
     * Exposes the RTS smoother to enable optional backward smoothing after the
     * forward UKF pass (useful for batch post-processing).
     *
     * @return Reference to the RTS smoother.
     */
    RtsSmoother& getRtsSmoother() override { return rts_smoother_; }

private:
    // Configuration
    UkfParams params_;                 ///< UKF parameters.
    int measurement_interval_;         ///< GPS update interval in IMU cycles.
    
    // State
    NavigationState state_;            ///< Current navigation state.
    int current_index_ = 0;            ///< Global time index for the current step.
    int kalman_index_ = 2;             ///< Internal counter for filter updates.
    
    // Earth reference parameters (updated per step as needed)
    double current_Rx_ = 0.0;          ///< Meridian radius of curvature Rx [m].
    double current_Ry_ = 0.0;          ///< Transverse (prime-vertical) radius of curvature Ry [m].
    Eigen::Vector3d wie_n_ = Eigen::Vector3d::Zero();  ///< Earth rotation rate expressed in the nav frame ω_ie^n [rad/s].
    Eigen::Vector3d wet_t_ = Eigen::Vector3d::Zero();  ///< Transport rate expressed in the navigation frame ω_en^t [rad/s].
    Eigen::Vector3d wit_b_ = Eigen::Vector3d::Zero();  ///< Total inertial angular rate in the body frame ω_it^b [rad/s].
    
    // UKF state
    UnscentedKalmanFilterParams ukf_;   ///< UKF covariance/weights and related configuration.
    Eigen::Vector3d last_f_INSt_;      ///< Previous specific force resolved in the nav frame [m/s²].

    // UKF processing components

    /**
     * @brief Build the continuous-time linearized error-dynamics matrix.
     *
     * Constructs the 15×15 continuous error-state dynamics matrix `f` for the INS
     * error model given current attitude, velocity, and kinematics. The canonical
     * error-state ordering is often:
     *   [δθ (3), δv (3), δp (3), gyro bias (3), accel bias (3)]^T
     * but should match the configuration used in `UnscentedKalmanFilterParams`.
     *
     * @param Cnb Body-to-nav DCM (b→n).
     * @param vn0 Velocity vector [Ve, Vn, Vu]^T (east, north, up).
     * @param aibn Specific force resolved in the nav frame [m/s²].
     * @param wie Earth rotation rate magnitude [rad/s].
     * @param lat Geodetic latitude [rad].
     * @param rxn Meridian radius Rx [m].
     * @param ryn Transverse radius Ry [m].
     * @return 15×15 continuous-time error-dynamics matrix `f`.
     */
    Eigen::Matrix<double,15,15> errorDynamics(
    const Eigen::Matrix3d& Cnb,
    const Eigen::Vector3d& vn0,   // [Ve, Vn, Vu]
    const Eigen::Vector3d& aibn,  // f in n-frame
    double wie, double lat, double rxn, double ryn);
    
    // Core algorithm implementations

    /**
     * @brief Update attitude using quaternion integration.
     *
     * Integrates the quaternion using body angular rates over Δt = 1/IMUrate.
     * May include coning compensation via helper utilities. The quaternion is
     * re-normalized to maintain unit length. Outputs `CbtM` (b→t) for use in
     * force/velocity updates.
     *
     * @param wtb_b Body angular rate ω_tb^b [rad/s].
     * @param wit_b Total inertial angular rate ω_it^b [rad/s] (may embed Earth/transport components).
     * @param quat  In/out quaternion [q0, q1, q2, q3]^T (scalar first).
     * @param IMUrate IMU update rate [Hz].
     * @param CbtM Output DCM from body to navigation frame (b→t).
     */
    void strapdownAttitudeUpdate(const Eigen::Vector3d& wtb_b,
                                const Eigen::Vector3d& wit_b,
                                Eigen::Vector4d& quat,
                                int IMUrate,
                                Eigen::Matrix3d& CbtM);
    
    /**
     * @brief Update velocity and position using strapdown mechanization.
     *
     * Resolves the specific force into the nav frame, subtracts gravity,
     * and applies Coriolis and transport-rate effects to update velocity. Then
     * integrates kinematics on the ellipsoid (using `Rx`/`Ry`) to update latitude,
     * longitude, and height.
     *
     * @param f_INSt  Specific force in the nav frame [m/s²].
     * @param V_prev  Previous velocity [m/s].
     * @param Lat_prev Previous latitude [rad].
     * @param Lon_prev Previous longitude [rad].
     * @param h_prev   Previous height [m].
     * @param wie_n   Earth rotation rate in nav frame ω_ie^n [rad/s].
     * @param wet_t   Transport rate in navigation frame ω_en^t [rad/s].
     * @param g       Local gravity magnitude [m/s²].
     * @param IMUrate IMU update rate [Hz].
     * @param Re      Reference Earth radius or semi-major axis [m] (per model).
     * @param e       Ellipsoid eccentricity (first eccentricity or variant per model).
     * @param Rx_prev Previous meridian radius [m].
     * @param Ry_prev Previous transverse radius [m].
     * @param V_new   Output velocity [m/s].
     * @param Lat_new Output latitude [rad].
     * @param Lon_new Output longitude [rad].
     * @param h_new   Output height [m].
     * @param Rx      Updated meridian radius [m].
     * @param Ry      Updated transverse radius [m].
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
     * @brief Compute the local gravity magnitude.
     *
     * Computes gravity using a standard gravity model (e.g., Somigliana / WGS-84),
     * adjusted for latitude and height above the ellipsoid.
     *
     * @param Latitude Geodetic latitude [rad].
     * @param h        Height above the ellipsoid [m].
     * @param params   Navigation model parameters (Earth constants, nominal gravity).
     * @return Gravity magnitude [m/s²].
     */
    double computeGravity(double Latitude, double h, const NavigationParams& params);
    
    // UKF-specific functions
    // Continuous to discrete: Gz, and series-based discretization for Φ and Q

    /**
     * @brief Build the process noise mapping matrix Gz (continuous-time).
     *
     * Constructs the continuous-time process noise mapping matrix such that
     * the continuous process noise covariance satisfies:
     *   Q_c = Gz * Q0 * Gzᵀ
     * where `Q0` is the driving white-noise covariance.
     *
     * @param CtbM Body-to-navigation DCM (b→t), used to map sensor noise into the error state.
     * @param Gz   Output noise mapping matrix.
     */
    void buildGz(const Eigen::Matrix3d& CtbM, Eigen::MatrixXd& Gz) const;

    /**
     * @brief Discretize error dynamics via matrix series expansion.
     *
     * Uses a truncated series (matrix exponential approximation) to obtain the
     * discrete transition matrix Φ and process noise covariance Q from the
     * continuous dynamics `f` and noise mapping (`Gz`, `Q0`) over the sample
     * interval `Tao`.
     *
     * @param f   Continuous-time error-dynamics matrix.
     * @param Tao Sample interval Δt [s].
     * @param Gz  Continuous-time noise mapping matrix.
     * @param Q0  Driving white-noise covariance (continuous-time).
     * @param Fai Output discrete transition matrix Φ.
     * @param Q   Output discrete process noise covariance Q.
     */
    void discretizeBySeries(const Eigen::MatrixXd& f, double Tao,
                            const Eigen::MatrixXd& Gz, const Eigen::MatrixXd& Q0,
                            Eigen::MatrixXd& Fai, Eigen::MatrixXd& Q) const;

    /**
     * @brief Generate sigma points for the UKF.
     *
     * Generates sigma points `Xsig` from mean `x` and covariance `P` using the
     * scaling parameter `lambda` (a function of α, κ, and state dimension `n`).
     * For an `n`-dimensional state, `Xsig` has `2n+1` columns.
     *
     * @param x      State mean (n×1).
     * @param P      State covariance (n×n, symmetric positive definite).
     * @param lambda UKF scaling parameter.
     * @param Xsig   Output sigma points (n×(2n+1)).
     */
    void generateSigmaPoints(const Eigen::VectorXd& x, const Eigen::MatrixXd& P,
                             double lambda, Eigen::MatrixXd& Xsig) const;
    
    /**
     * @brief Build the measurement matrix / Jacobian for GPS updates.
     *
     * Creates the measurement sensitivity matrix given current attitude
     * (roll, pitch, yaw) and geodetic parameters (lat, h, Rx, Ry). The returned
     * matrix must be dimensionally consistent with the configured measurement
     * noise covariance `R` and the chosen measurement set (e.g., position-only,
     * velocity-only, or combined).
     *
     * @param roll  Roll angle [rad].
     * @param pitch Pitch angle [rad].
     * @param yaw   Yaw/heading [rad].
     * @param lat   Latitude [rad].
     * @param h     Height [m].
     * @param Rx    Meridian radius [m].
     * @param Ry    Transverse radius [m].
     * @return Measurement matrix `H`.
     */
    Eigen::MatrixXd buildMeasurementMatrix(double roll, double pitch, double yaw,
                                          double lat, double h, double Rx, double Ry);

    RtsSmoother rts_smoother_;  ///< RTS smoother instance for backward pass post-processing.
};
