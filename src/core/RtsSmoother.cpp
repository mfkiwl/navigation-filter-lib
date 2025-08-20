/**
 * @file RtsSmoother.cpp
 * @brief Implementation of Rauch-Tung-Striebel (RTS) smoothing for navigation systems
 *
 * Contains the core implementation of the RTS smoothing algorithm for refining navigation solutions.
 * Performs backward-pass smoothing using stored Kalman filter history to improve position, velocity,
 * and attitude estimates by incorporating future measurements. Also includes functions for applying
 * smoothed corrections to navigation states and generating refined navigation solutions.
 *
 * @author peanut-nav
 * @date Created: 2025-08-04
 * @last Modified: 2025-08-20
 * @version 0.4.0
 */

#include "core/RtsSmoother.hpp"
#include "core/NavigationBase.hpp"
#include <iostream>
#include <cmath>
#include <unsupported/Eigen/MatrixFunctions>

using namespace Eigen;
using namespace std;

/**
 * @brief Store filter history for smoothing
 * 
 * Saves Kalman filter state, covariance, and navigation state at each measurement update
 */
void RtsSmoother::addHistoryItem(const FilterHistory& history_item) {
    history_.push_back(history_item);
}

/**
 * @brief Perform Rauch-Tung-Striebel smoothing
 * 
 * Implements backward-pass smoothing algorithm to refine state estimates using
 * future measurements. Starts from last time point and moves backward.
 * 
 * @param process_noise Process noise matrix (Q)
 * @return Structure containing smoothed states and covariances
 */
RtsSmoother::SmoothResult RtsSmoother::smooth(const Eigen::MatrixXd& process_noise, FilterType filterType) {
    int n = history_.size();
    SmoothResult result;
    
    // Initialize result structures
    if (n == 0) return result;
    result.smoothed_states.resize(n);
    result.smoothed_covariances.resize(n);
    result.smoothed_nav_states.resize(n);
    result.velocity_smooth.resize(n);
    result.position_smooth.resize(n);
    result.attitude_smooth.resize(n);
    
    // Initialize last point with filter results (no future data for smoothing)
    int last_idx = n-1;
    result.smoothed_states[last_idx] = history_[last_idx].state;
    result.smoothed_covariances[last_idx] = history_[last_idx].covariance;
    result.smoothed_nav_states[last_idx] = history_[last_idx].nav_state;
    
    // Backward pass: process from second last point to first
    for (int i = n-2; i >= 0; --i) {
        // Compute smoothing gain
        MatrixXd Ks;
        switch (filterType) {
            case FilterType::KF:
                Ks = history_[i].covariance * 
                     history_[i].transition_matrix.transpose() * 
                     history_[i+1].predicted_covariance.inverse();
                break;
            case FilterType::EKF:
                Ks = history_[i].covariance * 
                     history_[i].transition_matrix.transpose() * 
                     history_[i+1].predicted_covariance.inverse();
                break;
            case FilterType::UKF:
                Ks = history_[i+1].cross_covariance * history_[i+1].predicted_covariance.inverse();
                break;
            default:
                break;
        }
        
        // State smoothing: blend current filter with future smoothed state
        VectorXd state_diff = result.smoothed_states[i+1] - history_[i+1].predicted_state;
        result.smoothed_states[i] = history_[i].state + Ks * state_diff;
        
        // Covariance smoothing: reduce uncertainty using future information
        MatrixXd cov_diff = history_[i+1].predicted_covariance - result.smoothed_covariances[i+1];
        result.smoothed_covariances[i] = history_[i].covariance - 
                                        Ks * cov_diff * Ks.transpose();
        
        // Save navigation state before correction
        result.smoothed_nav_states[i] = history_[i].nav_state;
        
        // Apply smoothed corrections to navigation state
        // Velocity correction
        Vector3d velocity_correction = result.smoothed_states[i].segment(3, 3);
        result.smoothed_nav_states[i].Velocity[0] -= velocity_correction;
        result.velocity_smooth[i] = result.smoothed_nav_states[i].Velocity[0];
        
        // Position correction (convert radians to degrees)
        result.smoothed_nav_states[i].Latitude[0] -= result.smoothed_states[i](6) * (180.0/M_PI);
        result.smoothed_nav_states[i].Longitude[0] -= result.smoothed_states[i](7) * (180.0/M_PI);
        result.smoothed_nav_states[i].Altitude[0] -= result.smoothed_states[i](8);
        result.position_smooth[i] = {
            result.smoothed_nav_states[i].Latitude[0],
            result.smoothed_nav_states[i].Longitude[0],
            result.smoothed_nav_states[i].Altitude[0]
        };
        
        // Attitude correction using misalignment angles
        double E_err = result.smoothed_states[i](0);  // East misalignment (rad)
        double N_err = result.smoothed_states[i](1);  // North misalignment (rad)
        double U_err = result.smoothed_states[i](2);  // Up misalignment (rad)
        
        // Compute misalignment DCM
        Matrix3d Ctn = computeCtn(E_err, N_err, U_err);
        
        // Get original attitude
        double yaw = history_[i].nav_state.Yaw[0];
        double pitch = history_[i].nav_state.Pitch[0];
        double roll = history_[i].nav_state.Roll[0];
        
        // Compute original DCM
        Matrix3d Cnb = computeCnb(yaw, pitch, roll);
        
        // Apply correction: Cnb_smooth = Ctn * Cnb
        Cnb = Ctn * Cnb;
        
        // Update DCMs in navigation state
        result.smoothed_nav_states[i].CbtM = Cnb;
        result.smoothed_nav_states[i].CtbM = Cnb.transpose();
        
        // Recompute Euler angles from corrected DCM
        double new_pitch, new_roll, new_yaw;
        NavigationUtils::calculateEulerAngles(Cnb, new_pitch, new_roll, new_yaw);
        
        result.smoothed_nav_states[i].Pitch[0] = new_pitch;
        result.smoothed_nav_states[i].Roll[0] = new_roll;
        result.smoothed_nav_states[i].Yaw[0] = new_yaw;
        result.attitude_smooth[i] = {new_yaw, new_pitch, new_roll};
    }
    
    return result;
}

/**
 * @brief Compute misalignment DCM from error angles
 * 
 * Constructs direction cosine matrix for transforming from true
 * navigation frame to computed navigation frame
 * 
 * @return Ctn direction cosine matrix
 */
Eigen::Matrix3d RtsSmoother::computeCtn(double E_err, double N_err, double U_err) const {
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

/**
 * @brief Compute body-to-navigation DCM from Euler angles
 * 
 * Constructs direction cosine matrix for transforming from
 * body frame to navigation frame
 * 
 * @return Cnb direction cosine matrix
 */
Eigen::Matrix3d RtsSmoother::computeCnb(double yaw, double pitch, double roll) const {
    // Convert degrees to radians
    double yaw_rad = yaw * M_PI/180.0;
    double pitch_rad = pitch * M_PI/180.0;
    double roll_rad = roll * M_PI/180.0;
    
    // Precompute trigonometric values
    double cy = cos(yaw_rad);
    double sy = sin(yaw_rad);
    double cp = cos(pitch_rad);
    double sp = sin(pitch_rad);
    double cr = cos(roll_rad);
    double sr = sin(roll_rad);
    
    // Construct DCM
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

/**
 * @brief Generate smoothed navigation solution matrix
 * 
 * Formats smoothing results into a matrix compatible with MATLAB processing
 * 
 * @return Matrix with columns: [index, lat, lon, alt, vE, vN, vU, yaw, pitch, roll]
 */
std::vector<std::vector<double>> RtsSmoother::generateSmoothedNavigation(
    const SmoothResult& result, int totalPoints) {
    
    std::vector<std::vector<double>> smooth_filter;
    int n = result.smoothed_states.size();
    
    if (n == 0) return smooth_filter;
    
    // Initialize output matrix
    smooth_filter.resize(totalPoints, std::vector<double>(10, 0.0));
    
    // Fill matrix with smoothed navigation parameters
    for (int i = 0; i < n; i++) {
        smooth_filter[i][0] = i + 1;  // Time index
        smooth_filter[i][1] = result.position_smooth[i][0];  // Latitude
        smooth_filter[i][2] = result.position_smooth[i][1];  // Longitude
        smooth_filter[i][3] = result.position_smooth[i][2];  // Altitude
        smooth_filter[i][4] = result.velocity_smooth[i][0];  // East velocity
        smooth_filter[i][5] = result.velocity_smooth[i][1];  // North velocity
        smooth_filter[i][6] = result.velocity_smooth[i][2];  // Up velocity
        smooth_filter[i][7] = result.attitude_smooth[i][0];  // Yaw
        smooth_filter[i][8] = result.attitude_smooth[i][1];  // Pitch
        smooth_filter[i][9] = result.attitude_smooth[i][2];  // Roll
    }
    
    return smooth_filter;
}

/**
 * @brief Re-run navigation with smoothing corrections
 * 
 * Executes full navigation processing while applying smoothed corrections
 * at measurement points. This produces the final refined navigation solution.
 * 
 * @return Refined navigation state with smoothing corrections applied
 */
NavigationState RtsSmoother::postProcessNavigation(
    const IMUData& imu,
    const TrajectoryData& track,
    const NavigationState& initial_state,
    const NavigationParams& earth_params,
    const SmoothResult& smooth_result,
    int gps_rate,
    int imu_rate) {
    
    // Initialize navigation state
    NavigationState nav_state = initial_state;
    int totalPoints = nav_state.Latitude.size();
    
    // Reset state vectors
    for (int i = 0; i < totalPoints; i++) {
        nav_state.Latitude[i] = 0;
        nav_state.Longitude[i] = 0;
        nav_state.Altitude[i] = 0;
        nav_state.Velocity[i] = Eigen::Vector3d::Zero();
        nav_state.Pitch[i] = 0;
        nav_state.Roll[i] = 0;
        nav_state.Yaw[i] = 0;
    }
    
    // Set initial values
    nav_state.Pitch[0] = initial_state.Pitch[0];
    nav_state.Roll[0] = initial_state.Roll[0];
    nav_state.Yaw[0] = initial_state.Yaw[0];
    nav_state.Latitude[0] = initial_state.Latitude[0];
    nav_state.Longitude[0] = initial_state.Longitude[0];
    nav_state.Altitude[0] = initial_state.Altitude[0];
    nav_state.Velocity[0] = initial_state.Velocity[0];
    nav_state.CbtM = initial_state.CbtM;
    nav_state.CtbM = initial_state.CtbM;
    
    // Earth parameters
    double Re = earth_params.Re;
    double e = earth_params.e;
    double W_ie = earth_params.W_ie;
    
    // Compute initial curvature radii
    double lat_rad = nav_state.Latitude[0] * M_PI/180.0;
    double sinLat = sin(lat_rad);
    double Rx = Re / (1 - e * sinLat * sinLat);
    double Ry = Re / (1 + 2*e - 3*e * sinLat * sinLat);
    
    // Initialize Earth rotation parameters
    Eigen::Vector3d wie_n(0, W_ie * cos(lat_rad), W_ie * sin(lat_rad));
    Eigen::Vector3d wet_t(
        -nav_state.Velocity[0][1]/(Ry + nav_state.Altitude[0]),
        nav_state.Velocity[0][0]/(Rx + nav_state.Altitude[0]),
        nav_state.Velocity[0][0] * tan(lat_rad)/(Rx + nav_state.Altitude[0])
    );
    Eigen::Vector3d wit_t = wie_n + wet_t;
    Eigen::Vector3d wit_b = nav_state.CbtM * wit_t;
    
    // Initialize quaternion
    Eigen::Vector4d zt4 = NavigationUtils::eulerToQuaternion(
        nav_state.Pitch[0], nav_state.Roll[0], nav_state.Yaw[0]);
    
    // Initialize loop variables
    int i = 0;
    int ii = 0; // Smoothing result index
    int NavEnd = imu.index.size();
    int measurement_interval = imu_rate / gps_rate;
    bool Kalman_flag = false;
    
    // Main navigation processing loop
    for (int i = 0; i < NavEnd - 1; ++i) {
        // Convert gyro data to rad/s and remove Earth rotation
        Eigen::Vector3d wtb_b(
            imu.gx[i] * M_PI/(180.0*3600.0),
            imu.gy[i] * M_PI/(180.0*3600.0),
            imu.gz[i] * M_PI/(180.0*3600.0)
        );
        wtb_b -= wit_b;
        
        // Update attitude using quaternion integration
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
        
        // Update direction cosine matrix
        double q0 = zt4[0], q1 = zt4[1], q2 = zt4[2], q3 = zt4[3];
        nav_state.CbtM << 
            q0*q0 + q1*q1 - q2*q2 - q3*q3, 2*(q1*q2 + q0*q3), 2*(q1*q3 - q0*q2),
            2*(q1*q2 - q0*q3), q0*q0 - q1*q1 + q2*q2 - q3*q3, 2*(q2*q3 + q0*q1),
            2*(q1*q3 + q0*q2), 2*(q2*q3 - q0*q1), q0*q0 - q1*q1 - q2*q2 + q3*q3;
        nav_state.CtbM = nav_state.CbtM.transpose();
        
        // Convert specific force to navigation frame
        Eigen::Vector3d f_b(imu.ax[i], imu.ay[i], imu.az[i]);
        Eigen::Vector3d f_INSt = nav_state.CtbM * f_b;
        
        // Compute gravity at current position
        double g = earth_params.g0 * (1 + earth_params.gk1 * pow(sin(lat_rad), 2)) * 
                  (1 - 2*nav_state.Altitude[i]/earth_params.Re) / 
                  sqrt(1 - earth_params.gk2 * pow(sin(lat_rad), 2));
        
        // Compute acceleration (corrected for Coriolis and gravity)
        Eigen::Vector3d coriolis = 2 * wie_n + wet_t;
        Eigen::Vector3d a = f_INSt - coriolis.cross(nav_state.Velocity[i]) + Eigen::Vector3d(0, 0, -g);
        
        // Update velocity
        nav_state.Velocity[i+1] = nav_state.Velocity[i] + a / imu_rate;
        
        // Update position
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
        
        // Update Earth parameters
        lat_rad = nav_state.Latitude[i+1] * M_PI/180.0;
        sinLat = sin(lat_rad);
        Rx = Re / (1 - e * sinLat * sinLat);
        Ry = Re / (1 + 2*e - 3*e * sinLat * sinLat);
        
        wie_n = Eigen::Vector3d(0, W_ie * cos(lat_rad), W_ie * sin(lat_rad));
        wet_t = Eigen::Vector3d(
            -nav_state.Velocity[i+1][1]/(Ry + nav_state.Altitude[i+1]),
            nav_state.Velocity[i+1][0]/(Rx + nav_state.Altitude[i+1]),
            nav_state.Velocity[i+1][0] * tan(lat_rad)/(Rx + nav_state.Altitude[i+1])
        );
        wit_t = wie_n + wet_t;
        wit_b = nav_state.CbtM * wit_t;
        
        // Apply smoothing corrections at measurement points
        if (i > 2) {
            if ((i + 1) % measurement_interval == 0) {
                if (ii < smooth_result.position_smooth.size() && smooth_result.position_smooth[ii][0] != 0) {
                    // Apply smoothed position, velocity and altitude
                    nav_state.Latitude[i+1] = smooth_result.position_smooth[ii][0];
                    nav_state.Longitude[i+1] = smooth_result.position_smooth[ii][1];
                    nav_state.Altitude[i+1] = smooth_result.position_smooth[ii][2];
                    nav_state.Velocity[i+1] = smooth_result.velocity_smooth[ii];
                    
                    // Apply smoothed attitude
                    nav_state.Yaw[i+1] = smooth_result.attitude_smooth[ii][0];
                    nav_state.Pitch[i+1] = smooth_result.attitude_smooth[ii][1];
                    nav_state.Roll[i+1] = smooth_result.attitude_smooth[ii][2];
                    
                    Kalman_flag = true;
                    
                    // Update quaternion to match new attitude
                    zt4 = NavigationUtils::eulerToQuaternion(
                        nav_state.Pitch[i+1], nav_state.Roll[i+1], nav_state.Yaw[i+1]);
                }
                ii++;
            }
        }
        
        // Compute Euler angles if no smoothing applied
        if (!Kalman_flag) {
            // Pitch calculation
            nav_state.Pitch[i+1] = asin(nav_state.CbtM(1, 2)) * 180.0 / M_PI;
            
            // Roll calculation (handle singularity)
            if (abs(nav_state.CbtM(2, 2)) < 1e-16) {
                nav_state.Roll[i+1] = (nav_state.CbtM(0, 2) > 0) ? -90.0 : 90.0;
            } else {
                nav_state.Roll[i+1] = atan(-nav_state.CbtM(0, 2)/nav_state.CbtM(2, 2)) * 180.0 / M_PI;
                if (nav_state.CbtM(2, 2) < 0) {
                    nav_state.Roll[i+1] += (nav_state.CbtM(0, 2) > 0) ? -180.0 : 180.0;
                }
            }
            
            // Yaw calculation (handle singularity)
            if (abs(nav_state.CbtM(1, 1)) > 1e-16) {
                nav_state.Yaw[i+1] = atan(-nav_state.CbtM(1, 0)/nav_state.CbtM(1, 1)) * 180.0 / M_PI;
                if (nav_state.CbtM(1, 1) > 0) {
                    if (nav_state.Yaw[i+1] < 0) nav_state.Yaw[i+1] += 360.0;
                } else {
                    nav_state.Yaw[i+1] += 180.0;
                }
            } else {
                nav_state.Yaw[i+1] = (nav_state.CbtM(1, 0) < 0) ? 90.0 : 270.0;
            }
        }
        
        Kalman_flag = false;
    }
    
    return nav_state;
}
