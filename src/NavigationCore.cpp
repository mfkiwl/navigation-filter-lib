/**
 * @file NavigationCore.cpp
 * @brief Implementation of core navigation algorithms
 *
 * @author peanut-nav
 * @date Created: 2025-07-22
 * @last Modified: 2025-07-22
 * @version 0.1
 */

#include "NavigationCore.hpp"
#include "MathUtils.hpp"
#include <cmath>
#include <iostream>
#include <iomanip>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

using namespace Eigen;

void NavigationCore::updateAttitude(const Vector3d& wtb_b,
                                   const Vector3d& wit_b,
                                   Vector4d& quat,
                                   int IMUrate,
                                   Matrix3d& CbtM) {
    // Compute angular increment relative to Earth-fixed frame
    Vector3d delta_w = wtb_b - wit_b;
    
    // Compute rotation angle magnitude
    double sita0 = delta_w.norm() * (1.0/IMUrate);
    
    // Construct the skew-symmetric matrix for quaternion update
    Matrix4d sita = Matrix4d::Zero();
    sita << 0,          -delta_w(0), -delta_w(1), -delta_w(2),
            delta_w(0),  0,           delta_w(2), -delta_w(1),
            delta_w(1), -delta_w(2),  0,           delta_w(0),
            delta_w(2),  delta_w(1), -delta_w(0),  0;
    sita /= IMUrate;
    
    // Compute rotation matrix using Taylor series expansion
    Matrix4d I = Matrix4d::Identity();
    Matrix4d rotation;
    rotation = std::cos(sita0/2) * I + (std::sin(sita0/2)/sita0 * sita);
    
    // Update quaternion
    quat = rotation * quat;
    quat.normalize();  // Normalize to maintain unit quaternion
    
    // Update direction cosine matrix from the updated quaternion
    double q0 = quat(0), q1 = quat(1), q2 = quat(2), q3 = quat(3);
    CbtM << q0*q0 + q1*q1 - q2*q2 - q3*q3, 2*(q1*q2 + q0*q3),        2*(q1*q3 - q0*q2),
            2*(q1*q2 - q0*q3),        q0*q0 - q1*q1 + q2*q2 - q3*q3, 2*(q2*q3 + q0*q1),
            2*(q1*q3 + q0*q2),        2*(q2*q3 - q0*q1),        q0*q0 - q1*q1 - q2*q2 + q3*q3;
}

void NavigationCore::updateVelocityPosition(const Vector3d& f_INSt,
                                           const Vector3d& V_prev,
                                           double Lat_prev,
                                           double Lon_prev,
                                           double h_prev,
                                           const Vector3d& wie_n,
                                           const Vector3d& wet_t,
                                           double g,
                                           int IMUrate,
                                           double Re,
                                           double e,
                                           double Rx_prev,
                                           double Ry_prev,
                                           Vector3d& V_new,
                                           double& Lat_new,
                                           double& Lon_new,
                                           double& h_new,
                                           double& Rx,
                                           double& Ry) {
    // Compute Coriolis acceleration
    Vector3d coriolis = 2 * wie_n + wet_t;
    Vector3d a = f_INSt - coriolis.cross(V_prev);
    a += Eigen::Vector3d(0, 0, -g);  // Add gravity
    
    // Velocity update (forward Euler method)
    V_new = V_prev + (1.0/IMUrate) * a;
    
    // Position update (trapezoidal method)
    double deltaT = 1.0/IMUrate;
    double Ry_h = Ry_prev + h_prev;
    double Rx_h = Rx_prev + h_prev;
    double cosLat = std::cos(Lat_prev * M_PI/180.0);
    
    // Latitude update
    Lat_new = Lat_prev + deltaT * (V_prev(1) + V_new(1)) * 180.0 / (2 * Ry_h * M_PI);
    
    // Longitude update
    Lon_new = Lon_prev + deltaT * (V_prev(0) + V_new(0)) * 180.0 / (2 * Rx_h * cosLat * M_PI);
    
    // Altitude update
    h_new = h_prev + deltaT * (V_prev(2) + V_new(2)) / 2.0;
    
    // Update Earth curvature radii
    double sinLat = std::sin(Lat_new * M_PI/180.0);
    Rx = Re / (1 - e * sinLat * sinLat);
    Ry = Re / (1 + 2*e - 3*e * sinLat * sinLat);
}

void NavigationCore::calculateEulerAngles(const Matrix3d& CbtM,
                                         double& pitch,
                                         double& roll,
                                         double& yaw) {
    // Compute pitch angle (arcsin of element at row 1, column 2)
    pitch = asin(CbtM(1,2)) * 180.0 / M_PI;
    
    // Compute roll angle (using elements at row 0, column 2 and row 2, column 2)
    const double eps = 2e-16;  // Small value to avoid division by zero
    if (abs(CbtM(2,2)) < eps) {
        if (CbtM(0,2) > 0) {
            roll = -90.0;
        } else {
            roll = 90.0;
        }
    } else {
        roll = atan(-CbtM(0,2)/CbtM(2,2)) * 180.0 / M_PI;
        // Adjust roll angle based on quadrants
        if (CbtM(2,2) < 0) {
            if (CbtM(0,2) > 0) {
                roll = roll - 180.0;
            } else {
                roll = roll + 180.0;
            }
        }
    }
    
    // Compute yaw angle (using elements at row 1, column 0 and row 1, column 1)
    if (abs(CbtM(1,1)) > eps) {
        yaw = atan(-CbtM(1,0)/CbtM(1,1)) * 180.0 / M_PI;
        // Adjust yaw angle to [0, 360) degrees
        if (CbtM(1,1) > 0) {
            if (yaw < 0) {
                yaw += 360.0;
            }
        } else {
            yaw += 180.0;
        }
    } else {
        if (CbtM(1,0) < 0) {
            yaw = 90.0;
        } else {
            yaw = 270.0;
        }
    }
}


double NavigationCore::calculateGravity(double Latitude, double h, const NavigationParams& params) {
    // Compute gravity using the Somigliana model with altitude correction
    double sinLat = std::sin(Latitude * M_PI/180.0);
    double sin2Lat = sinLat * sinLat;
    double denominator = std::sqrt(1 - params.gk2 * sin2Lat);
    double factor = (1 - 2*h/params.Re) / denominator;
    return params.g0 * (1 + params.gk1 * sin2Lat) * factor;
}

void NavigationCore::runNavigation(const IMUData& imu,
                                  const GPSData& gps,
                                  const NavigationParams& params,
                                  NavigationState& state,
                                  KalmanFilterParams& kalman,
                                  int IMUrate,
                                  int GPSrate) {
    int NavEnd = imu.index.size() - 1;
    bool Kalman_flag = false;
    
    // Initialize Earth angular rates
    double lat0 = state.Latitude[0];
    Vector3d wie_n(0, params.W_ie * std::cos(lat0 * M_PI/180.0), 
                   params.W_ie * std::sin(lat0 * M_PI/180.0));
    Vector3d wet_t(-state.Velocity[0](1)/params.Ry, 
                   state.Velocity[0](0)/params.Rx, 
                   state.Velocity[0](0)/params.Rx * std::tan(lat0 * M_PI/180.0));
    Vector3d wit_t = wie_n + wet_t;
    Vector3d wit_b = state.CbtM * wit_t;
    
    // Current Earth curvature radii
    double current_Rx = params.Rx;
    double current_Ry = params.Ry;
    
    for (int i = 0; i < NavEnd; i++) {
        // Attitude update
        Vector3d wtb_b(imu.gx[i], imu.gy[i], imu.gz[i]);
        wtb_b = wtb_b * (M_PI/3600.0/180.0);  // Convert deg/h to rad/s
        
        updateAttitude(wtb_b, wit_b, state.Quaternion, IMUrate, state.CbtM);
        state.CtbM = state.CbtM.transpose();
        
        // Convert specific force to navigation frame
        Vector3d f_b(imu.ax[i], imu.ay[i], imu.az[i]);
        Vector3d f_INSt = state.CtbM * f_b;
        
        // Compute gravity at current position
        double g = calculateGravity(state.Latitude[i], state.Altitude[i], params);
        
        // Update Earth angular rates
        double lat_i = state.Latitude[i];
        wie_n << 0, 
                params.W_ie * std::cos(lat_i * M_PI/180.0),
                params.W_ie * std::sin(lat_i * M_PI/180.0);
        wet_t << -state.Velocity[i](1)/(current_Ry + state.Altitude[i]),
                 state.Velocity[i](0)/(current_Rx + state.Altitude[i]),
                 state.Velocity[i](0) * std::tan(lat_i * M_PI/180.0)/(current_Rx + state.Altitude[i]);
        
        // Update velocity and position
        Vector3d V_new;
        double Lat_new, Lon_new, h_new, Rx_new, Ry_new;
        updateVelocityPosition(f_INSt, state.Velocity[i], state.Latitude[i], state.Longitude[i], 
                              state.Altitude[i], wie_n, wet_t, g, IMUrate, params.Re, params.e, 
                              current_Rx, current_Ry, V_new, Lat_new, Lon_new, h_new, Rx_new, Ry_new);
        
        // Update state
        state.Velocity[i+1] = V_new;
        state.Latitude[i+1] = Lat_new;
        state.Longitude[i+1] = Lon_new;
        state.Altitude[i+1] = h_new;
        current_Rx = Rx_new;
        current_Ry = Ry_new;
        
        // Update angular rates for next iteration
        wit_t = wie_n + wet_t;
        wit_b = state.CbtM * wit_t;
        
        // Compute Euler angles from updated DCM
        calculateEulerAngles(state.CbtM, state.Pitch[i+1], state.Roll[i+1], state.Yaw[i+1]);
        
        // Perform Kalman filter update at GPS rate
        if ((i+1) % (IMUrate/GPSrate) == 0) {
            runKalmanFilter(i, state, kalman, gps, params, IMUrate, GPSrate, current_Rx, current_Ry, f_INSt);
            Kalman_flag = true;
            
            // Get current Kalman filter index
            int k = kalman.N_kalman - 1;
            VectorXd X_corr = kalman.Xsave.col(k);  // Correction vector
            
            // Correct position and velocity
            state.Latitude[i+1] -= X_corr(6) * (180.0/M_PI);   // Latitude correction
            state.Longitude[i+1] -= X_corr(7) * (180.0/M_PI);  // Longitude correction
            state.Altitude[i+1] -= X_corr(8);                  // Altitude correction
            state.Velocity[i+1] -= X_corr.segment(3, 3);       // Velocity correction
            
            // Correct attitude using misalignment angles
            double E_err = X_corr(0) * 180 / M_PI;  // East misalignment (degrees)
            double N_err = X_corr(1) * 180 / M_PI;  // North misalignment (degrees)
            double U_err = X_corr(2) * 180 / M_PI;  // Up misalignment (degrees)
            
            // Compute correction DCM from misalignment angles
            Matrix3d C_errT = NavigationUtils::bodyToNavigationDCM(E_err, N_err, U_err);
            Matrix3d C_err = C_errT.transpose();
            
            // Apply correction to navigation to body DCM
            state.CtbM = C_err * state.CtbM;
            // Orthogonalize using Gram-Schmidt process
            Vector3d col0 = state.CtbM.col(0);
            Vector3d col1 = state.CtbM.col(1) - state.CtbM.col(1).dot(col0) * col0 / col0.squaredNorm();
            Vector3d col2 = state.CtbM.col(2) 
                        - state.CtbM.col(2).dot(col0) * col0 / col0.squaredNorm()
                        - state.CtbM.col(2).dot(col1) * col1 / col1.squaredNorm();
            col0.normalize();
            col1.normalize();
            col2.normalize();
            state.CtbM.col(0) = col0;
            state.CtbM.col(1) = col1;
            state.CtbM.col(2) = col2;

            state.CbtM = state.CtbM.transpose();
            
            // Recompute Euler angles after correction
            calculateEulerAngles(state.CbtM, state.Pitch[i+1], state.Roll[i+1], state.Yaw[i+1]);
            
            // Update Earth curvature radii and angular rates
            double sinLat = std::sin(state.Latitude[i+1] * M_PI/180.0);
            current_Rx = params.Re / (1 - params.e * sinLat * sinLat);
            current_Ry = params.Re / (1 + 2*params.e - 3*params.e * sinLat * sinLat);
            wie_n << 0, 
                    params.W_ie * std::cos(state.Latitude[i+1] * M_PI/180.0),
                    params.W_ie * std::sin(state.Latitude[i+1] * M_PI/180.0);
            wet_t << -state.Velocity[i+1](1)/(current_Ry + state.Altitude[i+1]),
                     state.Velocity[i+1](0)/(current_Rx + state.Altitude[i+1]),
                     state.Velocity[i+1](0) * std::tan(state.Latitude[i+1] * M_PI/180.0)/(current_Rx + state.Altitude[i+1]);
            wit_t = wie_n + wet_t;
            wit_b = state.CbtM * wit_t;
        }
        
        // Update quaternion if Kalman correction was applied
        if (Kalman_flag) {
            state.Quaternion = NavigationUtils::eulerToQuaternion(state.Pitch[i+1], state.Roll[i+1], state.Yaw[i+1]);
            Kalman_flag = false;
        }
        
        // Display progress
        if (i % 1000 == 0) {
            std::cout << "Processing: " << i << "/" << NavEnd 
                      << " (" << std::fixed << std::setprecision(1) 
                      << (static_cast<double>(i)/NavEnd*100) << "%)" << std::endl;
        }
    }
}

// Kalman filter implementation

void NavigationCore::runKalmanFilter(int i, 
                                    NavigationState& state, 
                                    KalmanFilterParams& kalman, 
                                    const GPSData& gps,
                                    const NavigationParams& params,
                                    int IMUrate,
                                    int GPSrate,
                                    double Rx,
                                    double Ry,
                                    const Vector3d& f_INSt) {
    int k = kalman.N_kalman;
    
    // Compute state transition matrix
    MatrixXd A = computeStateMatrix(state.Latitude[i+1], state.Velocity[i+1], 
                                   state.Altitude[i+1], Rx, Ry, state.CtbM, 
                                   params.W_ie, f_INSt);
    
    // Compute measurement matrix
    MatrixXd H = computeMeasurementMatrix(state.Roll[i+1], state.Yaw[i+1],
                                         state.Pitch[i+1], state.Latitude[i+1],
                                         state.Altitude[i+1], Rx, Ry);
    
    // Compute control matrix (noise input)
    MatrixXd B = MatrixXd::Zero(15, 6);
    B.block<3,3>(0,0) = state.CtbM;  // Gyro bias to attitude error
    B.block<3,3>(3,3) = state.CtbM;  // Accel bias to velocity error

    // Determine GPS measurement index
    double ratio = static_cast<double>(i + 1) / (static_cast<double>(IMUrate) / GPSrate);
    int gps_index = static_cast<int>(std::round(ratio) - 1);

    // Clamp index to valid range
    if (gps_index >= gps.time.size()) gps_index = gps.time.size() - 1;
    
    // Compute measurement residual (innovation)
    kalman.Z.col(k) << 
        state.Velocity[i+1](0) - gps.vx[gps_index],
        state.Velocity[i+1](1) - gps.vy[gps_index],
        state.Velocity[i+1](2) - gps.vz[gps_index],
        (state.Latitude[i+1] - gps.lat[gps_index]) * M_PI * (Ry + state.Altitude[i+1]) / 180.0,
        (state.Longitude[i+1] - gps.lon[gps_index]) * M_PI * (Rx + state.Altitude[i+1]) * 
            std::cos(state.Latitude[i+1] * M_PI/180.0) / 180.0,
        state.Altitude[i+1] - gps.alt[gps_index];
    
    // Perform Kalman filter step
    VectorXd X_new;
    MatrixXd P_new;
    kalmanFilterStep(kalman.X.col(k-1), 
                    kalman.P,
                    kalman.Z.col(k),
                    H, kalman.R, kalman.Q, kalman.T, A, B, 
                    X_new, P_new);
    
    // Save state and covariance
    kalman.Xsave.col(k) = X_new;
    kalman.P = P_new; 
    
    // Record covariance diagonal for analysis
    for (int j = 0; j < 15; j++) {
        kalman.P_mean_square(j, k) = std::sqrt(P_new(j, j));
    }
    
    // Reset error states
    kalman.X.col(k) = X_new;
    kalman.X.block(0,k,9,1).setZero();  // Reset first 9 states (attitude, velocity, position errors)
    
    // Increment Kalman filter index
    kalman.N_kalman = k + 1;
}

MatrixXd NavigationCore::computeStateMatrix(double Latitude,
                                          const Vector3d& V,
                                          double h,
                                          double Rx,
                                          double Ry,
                                          const Matrix3d& CtbM,
                                          double W_ie,
                                          const Vector3d& f_INSt) {
    // Initialize 15x15 state transition matrix
    MatrixXd A = MatrixXd::Zero(15, 15);
    
    // Convert latitude to radians
    double lat_rad = Latitude * M_PI / 180.0;
    double sin_lat = sin(lat_rad);
    double cos_lat = cos(lat_rad);
    double tan_lat = tan(lat_rad);
    
    // Submatrix A1 (9x9) for attitude, velocity, position errors
    MatrixXd A1 = MatrixXd::Zero(9, 9);
    
    // Attitude error equations
    A1(0, 1) = W_ie * sin_lat + V(0) / (Rx + h) * tan_lat;
    A1(0, 2) = -W_ie * cos_lat - V(0) / (Rx + h);
    A1(0, 4) = -1.0 / (Ry + h);
    A1(0, 8) = V(1) / pow(Ry + h, 2);
    
    A1(1, 0) = -W_ie * sin_lat - V(0) / (Rx + h) * tan_lat;
    A1(1, 2) = -V(1) / (Ry + h);
    A1(1, 3) = 1.0 / (Rx + h);
    A1(1, 6) = -W_ie * sin_lat;
    A1(1, 8) = -V(0) / pow(Rx + h, 2);
    
    A1(2, 0) = W_ie * cos_lat + V(0) / (Rx + h);
    A1(2, 1) = V(1) / (Ry + h);
    A1(2, 3) = tan_lat / (Rx + h);
    A1(2, 6) = W_ie * cos_lat + V(0) / (Rx + h) / pow(cos_lat, 2);
    A1(2, 8) = -V(0) * tan_lat / pow(Rx + h, 2);
    
    // Velocity error equations
    A1(3, 1) = -f_INSt(2);
    A1(3, 2) = f_INSt(1);
    A1(3, 3) = (V(1) * tan_lat - V(2)) / (Rx + h);
    A1(3, 4) = 2 * W_ie * sin_lat + V(0) * tan_lat / (Rx + h);
    A1(3, 5) = -2 * W_ie * cos_lat - V(0) / (Rx + h);
    A1(3, 6) = 2 * W_ie * cos_lat * V(1) + 2 * W_ie * sin_lat * V(2) + 
               V(0) * V(1) / (Rx + h) / pow(cos_lat, 2);
    A1(3, 8) = (V(0) * V(2) - V(0) * V(1) * tan_lat) / pow(Rx + h, 2);
    
    A1(4, 0) = f_INSt(2);
    A1(4, 2) = -f_INSt(0);
    A1(4, 3) = -2 * W_ie * sin_lat - 2 * V(0) * tan_lat / (Rx + h);
    A1(4, 4) = -V(2) / (Ry + h);
    A1(4, 5) = -V(1) / (Ry + h);
    A1(4, 6) = -V(0) * (2 * W_ie * cos_lat + V(0) / (Rx + h) / pow(cos_lat, 2));
    A1(4, 8) = (V(1) * V(2) + pow(V(0), 2) * tan_lat) / pow(Rx + h, 2);
    
    A1(5, 0) = -f_INSt(1);
    A1(5, 1) = f_INSt(0);
    A1(5, 3) = 2 * W_ie * cos_lat + 2 * V(0) / (Rx + h);
    A1(5, 4) = 2 * V(1) / (Ry + h);
    A1(5, 6) = -2 * W_ie * sin_lat * V(0);
    A1(5, 8) = -(pow(V(0), 2) + pow(V(1), 2)) / pow(Rx + h, 2);
    
    // Position error equations
    A1(6, 4) = 1.0 / (Ry + h);
    A1(6, 8) = -V(1) / pow(Ry + h, 2);
    
    A1(7, 3) = 1.0 / ((Rx + h) * cos_lat);
    A1(7, 6) = V(0) * tan_lat / ((Rx + h) * cos_lat);
    A1(7, 8) = -V(0) / (pow(Rx + h, 2) * cos_lat);
    
    A1(8, 5) = 1.0;
    
    // Submatrix A2 (9x6) for sensor bias errors
    MatrixXd A2 = MatrixXd::Zero(9, 6);
    A2.block<3,3>(0,0) = CtbM;  // Gyro bias to attitude error
    A2.block<3,3>(3,3) = CtbM;  // Accel bias to velocity error
    
    // Assemble state transition matrix
    A.block<9,9>(0,0) = A1;
    A.block<9,6>(0,9) = A2;
    
    return A;
}

MatrixXd NavigationCore::computeMeasurementMatrix(double roll,
                                                  double yaw,
                                                  double pitch,
                                                  double Latitude,
                                                  double h,
                                                  double Rx,
                                                  double Ry) {
    // Initialize measurement matrix (6x15)
    MatrixXd H = MatrixXd::Zero(6, 15);

    // Convert Euler angles to radians
    double roll_rad = roll * M_PI / 180.0;
    double pitch_rad = pitch * M_PI / 180.0;
    double yaw_rad = yaw * M_PI / 180.0;

    // Precompute trigonometric values
    double sin_roll = sin(roll_rad);
    double cos_roll = cos(roll_rad);
    double sin_pitch = sin(pitch_rad);
    double cos_pitch = cos(pitch_rad);
    double sin_yaw = sin(yaw_rad);
    double cos_yaw = cos(yaw_rad);

    // Compute H_m2 (velocity error to measurement)
    MatrixXd H_m2(3,3);
    double a = cos_roll * cos_yaw - sin_roll * sin_pitch * sin_yaw;
    double b = cos_roll * cos_yaw + sin_roll * sin_pitch * cos_yaw;
    double c = -cos_pitch * sin_yaw;
    double d = sin_roll * cos_yaw - cos_roll * sin_pitch * sin_yaw;

    H_m2 << 
        1 + a * a,             a * b,              0,
        c * a,                1 + c * a,           0,
        d * a,                d * b,              1;

    // Compute H_m1 (position error to measurement)
    MatrixXd H_m1(3,3);
    H_m1 << 
        Ry + h, 0, 0,
        0, (Rx + h) * cos(Latitude * M_PI / 180.0), 0,
        0, 0, 1;

    // Map velocity errors to measurements (first 3 measurements)
    H.block<3,3>(0,3) = H_m2;
    // Map position errors to measurements (last 3 measurements)
    H.block<3,3>(3,6) = H_m1;

    return H;
}

void NavigationCore::kalmanFilterStep(const VectorXd& X_prev,
                                     const MatrixXd& P_prev,
                                     const VectorXd& Z,
                                     const MatrixXd& H,
                                     const MatrixXd& R,
                                     const MatrixXd& Q,
                                     double T,
                                     const MatrixXd& A,
                                     const MatrixXd& B,
                                     VectorXd& X_new,
                                     MatrixXd& P_new) {
    // Approximate state transition matrix using Taylor series
    MatrixXd I = MatrixXd::Identity(A.rows(), A.cols());
    MatrixXd one_step_tran = I + A * T + (A * A * T * T) / 2.0 + 
                           (A * A * A * T * T * T) / 6.0 + 
                           (A * A * A * A * T * T * T * T) / 24.0;
    
    // Approximate system noise covariance matrix
    MatrixXd qq1 = B * Q * B.transpose() * T;
    MatrixXd sys_noise_drive = qq1;
    for (int kk = 2; kk <= 10; kk++) {
        MatrixXd qq2 = A * qq1;
        qq1 = (qq2 + qq2.transpose()) * T / kk;
        sys_noise_drive += qq1;
    }
    
    // Predict state and covariance
    MatrixXd one_step_P = one_step_tran * P_prev * one_step_tran.transpose() + sys_noise_drive;
    VectorXd one_step_X = one_step_tran * X_prev;
    
    // Compute Kalman gain
    MatrixXd S = H * one_step_P * H.transpose() + R;
    MatrixXd K_filter_gain = one_step_P * H.transpose() * S.inverse();
    
    // Update state and covariance
    X_new = one_step_X + K_filter_gain * (Z - H * one_step_X);
    MatrixXd I_KH = I - K_filter_gain * H;
    P_new = I_KH * one_step_P * I_KH.transpose() + K_filter_gain * R * K_filter_gain.transpose();
}
