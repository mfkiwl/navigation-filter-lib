/**
 * @file UkfNavigation.cpp
 * @brief Implementation of UKF-based navigation system
 *
 * @author peanut-nav
 * @date Created: 2025-08-10
 * @last Modified: 2025-08-10
 * @version 0.3.3
 */

#include "NavigationFactory.hpp"
#include "MathUtils.hpp"
#include <iostream>
#include <iomanip>
#include <cmath>

using namespace Eigen;
using namespace std;

static inline double deg2rad(double d){ return d * M_PI / 180.0; }
static inline double rad2deg(double r){ return r * 180.0 / M_PI; }

// ================== Initialization ==================
/**
 * @brief Initialize UKF navigation system
 */
void UkfNavigation::initialize(const NavParamsBase& base_params, 
                              NavigationState& state) {
    // Convert to UKF specific parameters
    const UkfParams& params = dynamic_cast<const UkfParams&>(base_params);
    params_ = params;
    ukf_ = params.ukf_params;  // Initialize UKF parameters
    state_ = state;            // Set initial navigation state
    
    // Calculate measurement update interval
    measurement_interval_ = params_.imu_rate / params_.gps_rate;
    
    // Initialize Earth rotation rate in navigation frame
    double lat0 = state_.Latitude[0] * M_PI / 180.0;
    wie_n_ << 0, 
             params_.earth_params.W_ie * cos(lat0),
             params_.earth_params.W_ie * sin(lat0);
    
    // Compute transport rate in tangent frame
    wet_t_ << -state_.Velocity[0](1)/(params_.earth_params.Ry + state_.Altitude[0]), 
             state_.Velocity[0](0)/(params_.earth_params.Rx + state_.Altitude[0]), 
             state_.Velocity[0](0) * tan(lat0)/(params_.earth_params.Rx + state_.Altitude[0]);
    
    // Total angular rate in tangent frame and convert to body frame
    Vector3d wit_t = wie_n_ + wet_t_;
    wit_b_ = state_.CbtM * wit_t;
    
    // Current Earth curvature radii
    current_Rx_ = params_.earth_params.Rx;
    current_Ry_ = params_.earth_params.Ry;

    // UKF 权重
    ukf_.lambda = 3.0 - ukf_.Nk;                 // To
    ukf_.w = VectorXd::Zero(2*ukf_.Nk + 1);
    ukf_.w(0) = ukf_.lambda / (ukf_.lambda + ukf_.Nk);
    for (int i=1;i<ukf_.w.size();++i)
        ukf_.w(i) = 1.0 / (2.0*(ukf_.lambda + ukf_.Nk));
    
    // 初次 f0（IMU 第一行不积分）
    Vector3d aibb0(0,0,0);
    aibb0 << 0,0,0;
    Vector3d aibn0 = state_.CtbM * aibb0;
    double rxn0 = params_.earth_params.Rx + state_.Altitude[0];
    double ryn0 = params_.earth_params.Ry + state_.Altitude[0];
    ukf_.f0 = errorDynamics(
        state_.CtbM,                 // Cnb（这里用 CtbM 即 n<-b）
        state_.Velocity[0], aibn0,
        params_.earth_params.W_ie,
        deg2rad(state_.Latitude[0]), rxn0, ryn0
    );
    
    // Initialize state vectors to zero
    int totalPoints = state_.Latitude.size();
    state_.Velocity.resize(totalPoints, Vector3d::Zero());
    state_.Pitch.resize(totalPoints);
    state_.Roll.resize(totalPoints);
    state_.Yaw.resize(totalPoints);
    
    // Initialize UKF parameters
    ukf_.Fai = MatrixXd::Identity(15, 15);  // Discrete state matrix
    ukf_.Q = MatrixXd::Zero(15, 15);      // Discrete process noise covariance
    ukf_.X_prop.resize(ukf_.Nk, 2*ukf_.Nk + 1);
    ukf_.X_pred.setZero(ukf_.Nk);
    ukf_.P_pred.setZero(ukf_.Nk, ukf_.Nk);
}

Eigen::Matrix<double,15,15> UkfNavigation::errorDynamics(
    const Eigen::Matrix3d& Cnb,
    const Eigen::Vector3d& vn0,   // [Ve, Vn, Vu]
    const Eigen::Vector3d& aibn,  // f in n-frame
    double wie, double lat, double rxn, double ryn)
{
    using Mat = Eigen::Matrix<double,15,15>;
    Mat f0 = Mat::Zero();

    f0(0,1) =  wie*std::sin(lat) + vn0(0)/rxn*std::tan(lat);               // L3
    f0(0,2) = -wie*std::cos(lat) - vn0(0)/rxn;                             // L4
    f0(0,4) = -1.0/ryn;                                                    // L5
    f0(0,8) =  vn0(1)/(ryn*ryn);                                           // L6

    f0(1,0) = -wie*std::sin(lat) - vn0(0)/rxn*std::tan(lat);               // L7
    f0(1,2) = -vn0(1)/ryn;                                                 // L8
    f0(1,3) =  1.0/rxn;                                                    // L9
    f0(1,6) = -wie*std::sin(lat);                                          // L10
    f0(1,8) = -vn0(0)/(rxn*rxn);                                           // L11

    f0(2,0) =  wie*std::cos(lat) + vn0(0)/rxn;                             // L12
    f0(2,1) =  vn0(1)/ryn;                                                 // L13
    f0(2,3) =  1.0/rxn*std::tan(lat);                                      // L14
    f0(2,6) =  wie*std::cos(lat) + vn0(0)/rxn/(std::cos(lat)*std::cos(lat));// L15
    f0(2,8) = -vn0(0)/(rxn*rxn)*std::tan(lat);                             // L16

    f0(3,1) = -aibn(2);                                                    // L17
    f0(3,2) =  aibn(1);                                                    // L18
    f0(3,3) =  1.0/rxn*(vn0(1)*std::tan(lat)-vn0(2));                      // L19
    f0(3,4) =  2*wie*std::sin(lat)+vn0(0)/rxn*std::tan(lat);               // L20
    f0(3,5) = -2*wie*std::cos(lat)-vn0(0)/rxn;                             // L21
    f0(3,6) =  2*wie*std::cos(lat)*vn0(1)+2*wie*std::sin(lat)*vn0(2)
              +vn0(0)*vn0(1)/rxn/(std::cos(lat)*std::cos(lat));            // L22
    f0(3,8) = (vn0(0)*vn0(2)-vn0(0)*vn0(1)*std::tan(lat))/(rxn*rxn);       // L23

    f0(4,0) =  aibn(2);                                                    // L24
    f0(4,2) = -aibn(0);                                                    // L25
    f0(4,3) = -2*wie*std::sin(lat)-2*vn0(0)/rxn*std::tan(lat);             // L26
    f0(4,4) = -vn0(2)/ryn;                                                 // L27
    f0(4,5) = -vn0(1)/ryn;                                                 // L28
    f0(4,6) = -vn0(0)*(2*wie*std::cos(lat)+vn0(0)/rxn/(std::cos(lat)*std::cos(lat))); // L29
    f0(4,8) = (vn0(1)*vn0(2)+vn0(0)*vn0(0)*std::tan(lat))/(rxn*rxn);       // L30

    f0(5,0) = -aibn(1);                                                    // L31
    f0(5,1) =  aibn(0);                                                    // L32
    f0(5,3) =  2*wie*std::cos(lat)+2*vn0(0)/rxn;                           // L33
    f0(5,4) =  2*vn0(1)/ryn;                                               // L34
    f0(5,6) = -2*wie*std::sin(lat)*vn0(0);                                 // L35
    f0(5,8) = -(vn0(0)*vn0(0)+vn0(1)*vn0(1))/(rxn*rxn);                    // L36

    f0(6,4) =  1.0/ryn;                                                    // L37
    f0(6,8) = -vn0(1)/(ryn*ryn);                                           // L38
    f0(7,3) =  1.0/rxn/std::cos(lat);                                      // L39
    f0(7,6) =  std::tan(lat)/rxn/std::cos(lat)*vn0(0);                     // L19
    f0(7,8) = -vn0(0)/(rxn*rxn)/std::cos(lat);                             // L20
    f0(8,5) =  1.0;                                                        // L21

    // sensor bias mapping blocks
    for (int i=0;i<3;i++){
        for (int j=9;j<12;j++){
            f0(i,  j) = Cnb(i, j-9);                                       // L24
            f0(i+3,j+3)= Cnb(i, j-9);                                      // L25
        }
    }
    return f0;
}

// ================== Strapdown Inertial Navigation Update ==================
/**
 * @brief Perform strapdown inertial navigation update
 */
void UkfNavigation::updateStrapdown(const IMUData& imu, int i) {
    // Convert gyro data from deg/h to rad/s and update attitude
    Vector3d wtb_b(imu.gx[i], imu.gy[i], imu.gz[i]);
    wtb_b = wtb_b * (M_PI/(3600.0*180.0));

    strapdownAttitudeUpdate(wtb_b, wit_b_, state_.Quaternion, params_.imu_rate, state_.CbtM);
    state_.CtbM = state_.CbtM.transpose();  // Update inverse transformation
    
    // Convert accelerometer data to navigation frame
    Vector3d f_b(imu.ax[i], imu.ay[i], imu.az[i]);
    last_f_INSt_ = state_.CtbM * f_b;  // Save for UsKF prediction
    
    // Compute gravity at current position
    double g = computeGravity(state_.Latitude[i], state_.Altitude[i], params_.earth_params);
    
    // Update Earth angular rates based on current position
    double lat_i_rad = deg2rad(state_.Latitude[i]);
    wie_n_ << 0, 
             params_.earth_params.W_ie * cos(lat_i_rad),
             params_.earth_params.W_ie * sin(lat_i_rad);
    
    // Update transport rate
    wet_t_ << -state_.Velocity[i](1)/(current_Ry_ + state_.Altitude[i]),
              state_.Velocity[i](0)/(current_Rx_ + state_.Altitude[i]),
              state_.Velocity[i](0) * tan(lat_i_rad)/(current_Rx_ + state_.Altitude[i]);
    
    // Update velocity and position
    Vector3d V_new;
    double Lat_new, Lon_new, h_new, Rx_new, Ry_new;
    
    strapdownVelocityPositionUpdate(
        last_f_INSt_, state_.Velocity[i],
        state_.Latitude[i], state_.Longitude[i], state_.Altitude[i],
        wie_n_, wet_t_, g, params_.imu_rate,
        params_.earth_params.Re, params_.earth_params.e,
        current_Rx_, current_Ry_,
        V_new, Lat_new, Lon_new, h_new, Rx_new, Ry_new
    );

    // Update navigation state
    state_.Velocity[i+1] = V_new;
    state_.Latitude[i+1] = Lat_new;
    state_.Longitude[i+1] = Lon_new;
    state_.Altitude[i+1] = h_new;
    current_Rx_ = Rx_new;
    current_Ry_ = Ry_new;
    
    // Update total angular rate in body frame
    Vector3d wit_t = wie_n_ + wet_t_;
    wit_b_ = state_.CbtM * wit_t;
    
    // Compute Euler angles from direction cosine matrix
    NavigationUtils::calculateEulerAngles(state_.CbtM, state_.Pitch[i+1], state_.Roll[i+1], state_.Yaw[i+1]);
    
    // Display progress
    if (i % 1000 == 0) {
        cout << "Processing: " << i << "/" << (state_.Latitude.size() - 1)
             << " (" << fixed << setprecision(1) 
             << (static_cast<double>(i)/(state_.Latitude.size()-1)*100) << "%)" << endl;
    }
}

/**
 * @brief Update attitude using quaternion integration
 */
void UkfNavigation::strapdownAttitudeUpdate(const Vector3d& wtb_b,
                                          const Vector3d& wit_b,
                                          Vector4d& quat,
                                          int IMUrate,
                                          Matrix3d& CbtM) {
    // Compute relative angular rate
    Vector3d delta_w = wtb_b - wit_b;
    double sita0 = delta_w.norm() * (1.0/IMUrate);
    
    // Construct quaternion update matrix
    Matrix4d sita = Matrix4d::Zero();
    sita << 0,          -delta_w(0), -delta_w(1), -delta_w(2),
            delta_w(0),  0,           delta_w(2), -delta_w(1),
            delta_w(1), -delta_w(2),  0,           delta_w(0),
            delta_w(2),  delta_w(1), -delta_w(0),  0;
    sita /= IMUrate;
    
    // Compute rotation quaternion
    Matrix4d I = Matrix4d::Identity();
    Matrix4d rotation = cos(sita0/2) * I + (sin(sita0/2)/sita0 * sita);
    
    // Update quaternion
    quat = rotation * quat;
    quat.normalize();
    
    // Convert quaternion to direction cosine matrix
    double q0 = quat(0), q1 = quat(1), q2 = quat(2), q3 = quat(3);
    CbtM << q0*q0 + q1*q1 - q2*q2 - q3*q3, 2*(q1*q2 + q0*q3),        2*(q1*q3 - q0*q2),
            2*(q1*q2 - q0*q3),        q0*q0 - q1*q1 + q2*q2 - q3*q3, 2*(q2*q3 + q0*q1),
            2*(q1*q3 + q0*q2),        2*(q2*q3 - q0*q1),        q0*q0 - q1*q1 - q2*q2 + q3*q3;
}
 
/**
 * @brief Update velocity and position using mechanization
 */
void UkfNavigation::strapdownVelocityPositionUpdate(const Vector3d& f_INSt,
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
    a += Vector3d(0, 0, -g);  // Add gravity
    
    // Update velocity using trapezoidal integration
    V_new = V_prev + (1.0/IMUrate) * a;
    
    // Update position
    double deltaT = 1.0/IMUrate;
    double Ry_h = Ry_prev + h_prev;
    double Rx_h = Rx_prev + h_prev;
    double cosLat = cos(Lat_prev * M_PI/180.0);
    
    // Latitude update (deg)
    Lat_new = Lat_prev + deltaT * (V_prev(1) + V_new(1)) * 180.0 / (2 * Ry_h * M_PI);
    
    // Longitude update (deg)
    Lon_new = Lon_prev + deltaT * (V_prev(0) + V_new(0)) * 180.0 / (2 * Rx_h * cosLat * M_PI);
    
    // Altitude update (m)
    h_new = h_prev + deltaT * (V_prev(2) + V_new(2)) / 2.0;
    
    // Update Earth curvature radii
    double sinLat = sin(Lat_new * M_PI/180.0);
    Rx = Re / (1 - e * sinLat * sinLat);  // Meridian radius
    Ry = Re / (1 + 2*e - 3*e * sinLat * sinLat);  // Transverse radius
}
 
/**
 * @brief Compute gravity magnitude
 * 
 * @return Gravity magnitude (m/s²)
 */
double UkfNavigation::computeGravity(double Latitude, double h, const NavigationParams& params) {
    double sinLat = sin(Latitude * M_PI/180.0);
    double sin2Lat = sinLat * sinLat;
    double denominator = sqrt(1 - params.gk2 * sin2Lat);
    double factor = (1 - 2*h/params.Re) / denominator;
    return params.g0 * (1 + params.gk1 * sin2Lat) * factor;
}
 
// ================== State Prediction ==================
/**
 * @brief Predict navigation state using UKF
 */
void UkfNavigation::predictState(int i) {
    // --- 1) 连续误差模型 f 的端点平均：f=(f0+f1)/2 ---
    const double rxn = current_Rx_ + state_.Altitude[i+1];
    const double ryn = current_Ry_ + state_.Altitude[i+1];

    Matrix<double,15,15> f1 = errorDynamics(
        state_.CtbM,                 // Cnb
        state_.Velocity[i+1], last_f_INSt_,
        params_.earth_params.W_ie,
        deg2rad(state_.Latitude[i+1]), rxn, ryn
    );
    Matrix<double,15,15> f = (ukf_.f0 + f1) * 0.5;
    ukf_.f0 = f1;

    // --- 2) 构造 Gz、离散化得到 Φ 和 Qd ---
    MatrixXd Gz = MatrixXd::Zero(ukf_.Nk, ukf_.Lk);
    buildGz(state_.CtbM, Gz);
    discretizeBySeries(f, ukf_.T /*量测周期*/, Gz, ukf_.Q0, ukf_.Fai, ukf_.Q);

    // --- 3) 生成 σ 点并线性传播：Xk|k-1 = Φ * χ ---
    const int L = 2*ukf_.Nk + 1;
    MatrixXd Xsig(ukf_.Nk, L);
    generateSigmaPoints(ukf_.X.col(ukf_.N_kalman-1), ukf_.P, ukf_.lambda, Xsig);

    ukf_.X_prop.resize(ukf_.Nk, L);
    for (int c=0;c<L;++c) ukf_.X_prop.col(c) = ukf_.Fai * Xsig.col(c);

    // --- 4) 求先验均值/协方差 ---
    ukf_.X_pred.setZero(ukf_.Nk);
    for (int c=0;c<L;++c) ukf_.X_pred += ukf_.w(c) * ukf_.X_prop.col(c);

    ukf_.P_pred = ukf_.Q; // 过程噪声起始
    for (int c=0;c<L;++c) {
        VectorXd d = ukf_.X_prop.col(c) - ukf_.X_pred;
        ukf_.P_pred += ukf_.w(c) * (d * d.transpose());
    }
}

void UkfNavigation::generateSigmaPoints(const VectorXd& x, const MatrixXd& P,
                                        double lambda, MatrixXd& Xsig) const
{
    const int L = 2*ukf_.Nk + 1;
    const double scale = std::sqrt(ukf_.Nk + lambda);

    Eigen::LLT<MatrixXd> llt(P);
    MatrixXd U = llt.matrixL(); // 下三角

    Xsig.resize(ukf_.Nk, L);
    Xsig.col(0) = x;
    for (int j=0;j<ukf_.Nk;++j){
        VectorXd col = U.col(j) * scale;
        Xsig.col(1+j)    = x + col;
        Xsig.col(1+ukf_.Nk+j)  = x - col;
    }
}

void UkfNavigation::buildGz(const Eigen::Matrix3d& CtbM, MatrixXd& Gz) const {
    Gz.setZero(15,6);
    Gz.block<3,3>(0,0) = CtbM;
    Gz.block<3,3>(3,3) = CtbM;
}

void UkfNavigation::discretizeBySeries(const MatrixXd& f, double Tao,
                                       const MatrixXd& Gz, const MatrixXd& Q0,
                                       MatrixXd& Fai, MatrixXd& Q) const
{
    const int n = f.rows();
    MatrixXd I = MatrixXd::Identity(n,n);

    // Φ = I + Σ_{i=1..10} ((Tao*f)^i / i!)
    Fai = I;
    MatrixXd TF = MatrixXd::Identity(n,n);
    for (int ii=1; ii<=10; ++ii){
        TF = TF * (Tao * f) / double(ii);
        Fai += TF;
    }

    MatrixXd Q1 = Gz * Q0 * Gz.transpose();
    MatrixXd M1 = Tao * Q1;
    Q = M1;
    for (int m=2; m<=30; ++m){
        MatrixXd M2 = f * M1;
        M1 = Tao * (M2 + M2.transpose()) / double(m);
        Q += M1;
        if (M1.norm() < 1e-12) break;
    }
}

// ================== Measurement Update ==================
/**
 * @brief Update navigation state using GPS measurement
 */
void UkfNavigation::updateMeasurement(const GPSData& gps, int i) {
    int gps_index = static_cast<int>(round(static_cast<double>(i + 1) / measurement_interval_) - 1);
    if (gps_index >= gps.time.size()) gps_index = gps.time.size() - 1;

    // Measurement residual (innovation)
    VectorXd Z(6);
    Z << state_.Velocity[i+1](0) - gps.vx[gps_index],
         state_.Velocity[i+1](1) - gps.vy[gps_index],
         state_.Velocity[i+1](2) - gps.vz[gps_index],
         // Convert position differences from degrees to meters:
         (state_.Latitude[i+1] - gps.lat[gps_index]) * M_PI * (current_Ry_ + state_.Altitude[i+1]) / 180.0,
         (state_.Longitude[i+1] - gps.lon[gps_index]) * M_PI * (current_Rx_ + state_.Altitude[i+1]) * 
             cos(state_.Latitude[i+1] * M_PI/180.0) / 180.0,
         state_.Altitude[i+1] - gps.alt[gps_index];
    
    MatrixXd H = buildMeasurementMatrix(state_.Roll[i+1], state_.Pitch[i+1], state_.Yaw[i+1],
                                state_.Latitude[i+1], state_.Altitude[i+1],
                                current_Rx_, current_Ry_);

    // 2) σ 点到观测域：γ_i = H * χ_i
    const int L = 2*ukf_.Nk + 1;
    MatrixXd Yprop(ukf_.Mk, L);
    for (int c=0;c<L;++c) Yprop.col(c) = H * ukf_.X_prop.col(c);

    // 3) 观测均值/协方差与互协方差
    VectorXd y_pred = VectorXd::Zero(ukf_.Mk);
    for (int c=0;c<L;++c) y_pred += ukf_.w(c) * Yprop.col(c);

    MatrixXd Pyy = params_.ukf_params.R;  // +R
    MatrixXd Pxy = MatrixXd::Zero(ukf_.Nk, ukf_.Mk);
    for (int c=0;c<L;++c){
        VectorXd dy = Yprop.col(c) - y_pred;
        VectorXd dx = ukf_.X_prop.col(c) - ukf_.X_pred;
        Pyy += ukf_.w(c) * (dy * dy.transpose());
        Pxy += ukf_.w(c) * (dx * dy.transpose());
    }

     // 4) 卡尔曼增益、创新、后验
    MatrixXd K = Pxy * Pyy.inverse();
    VectorXd innovation = Z - y_pred;

    VectorXd x_new = ukf_.X_pred + K * innovation;
    MatrixXd P_new = ukf_.P_pred - K * Pyy * K.transpose();

    // 保存
    const int k = ukf_.N_kalman;
    ukf_.X.col(k)      = x_new;
    ukf_.Xsave.col(k)  = x_new;
    ukf_.P             = P_new;     // 当前后验 P
    for (int j=0;j<15;++j) ukf_.P_mean_square(j,k) = std::sqrt(std::max(0.0, P_new(j,j)));

    // Reset error states for next cycle
    ukf_.X.block(0,k,9,1).setZero();

    // 下一时刻
    ukf_.N_kalman++;
}
 
// ================== Error Correction ==================
/**
 * @brief Correct navigation state errors
 */
void UkfNavigation::correctErrors(int i) {
    const int k = ukf_.N_kalman - 1;
    const VectorXd& Xcorr = ukf_.Xsave.col(k);
    
    // 位置
    state_.Latitude[i+1]  -= rad2deg(Xcorr(6));
    state_.Longitude[i+1] -= rad2deg(Xcorr(7));
    state_.Altitude[i+1]  -= Xcorr(8);

    // 速度
    state_.Velocity[i+1]  -= Xcorr.segment<3>(3);

    // 姿态：失准角 -> 误差方向余弦，左乘修正
    double E_err = rad2deg(Xcorr(0));
    double N_err = rad2deg(Xcorr(1));
    double U_err = rad2deg(Xcorr(2));
    Matrix3d C_errT = NavigationUtils::bodyToNavigationDCM(E_err, N_err, U_err);
    Matrix3d C_err  = C_errT.transpose();
    state_.CtbM = C_err * state_.CtbM;
    
    // Orthogonalize using Gram-Schmidt process
    Vector3d col0 = state_.CtbM.col(0);
    Vector3d col1 = state_.CtbM.col(1) - state_.CtbM.col(1).dot(col0) * col0 / col0.squaredNorm();
    Vector3d col2 = state_.CtbM.col(2) 
                - state_.CtbM.col(2).dot(col0) * col0 / col0.squaredNorm()
                - state_.CtbM.col(2).dot(col1) * col1 / col1.squaredNorm();
    state_.CtbM.col(0) = col0.normalized();
    state_.CtbM.col(1) = col1.normalized();
    state_.CtbM.col(2) = col2.normalized();
    state_.CbtM = state_.CtbM.transpose();  // Update inverse transformation
    
    // Recompute Euler angles after correction
    NavigationUtils::calculateEulerAngles(state_.CbtM, state_.Pitch[i+1], state_.Roll[i+1], state_.Yaw[i+1]);
    
    // Update Earth parameters based on corrected position
    double lat_rad = deg2rad(state_.Latitude[i+1]);
    double sinLat  = std::sin(lat_rad);
    current_Rx_ = params_.earth_params.Re / (1 - params_.earth_params.e * sinLat * sinLat);
    current_Ry_ = params_.earth_params.Re / (1 + 2*params_.earth_params.e - 3*params_.earth_params.e * sinLat * sinLat);
    wie_n_ << 0, 
            params_.earth_params.W_ie * cos(lat_rad),
            params_.earth_params.W_ie * sin(lat_rad);
    wet_t_ << -state_.Velocity[i+1](1)/(current_Ry_ + state_.Altitude[i+1]),
             state_.Velocity[i+1](0)/(current_Rx_ + state_.Altitude[i+1]),
             state_.Velocity[i+1](0) * tan(lat_rad)/(current_Rx_ + state_.Altitude[i+1]);
    Vector3d wit_t = wie_n_ + wet_t_;
    wit_b_ = state_.CbtM * wit_t;
    
    // Update quaternion from corrected Euler angles
    state_.Quaternion = NavigationUtils::eulerToQuaternion(state_.Pitch[i+1], state_.Roll[i+1], state_.Yaw[i+1]);
 
    // ========== Record History for RTS Smoother ==========
    RtsSmoother::FilterHistory h;
    
    // Save filtered state (posterior)
    h.state = ukf_.Xsave.col(k);
    
    h.predicted_state = ukf_.X_pred;
    h.covariance = ukf_.P;
    h.predicted_covariance = ukf_.P_pred;
    
    // Save transition matrix (discretized)
    h.transition_matrix = ukf_.Fai;

    // Save current navigation state snapshot
    h.nav_state.Latitude  = {state_.Latitude[i+1]};
    h.nav_state.Longitude = {state_.Longitude[i+1]};
    h.nav_state.Altitude  = {state_.Altitude[i+1]};
    h.nav_state.Velocity  = {state_.Velocity[i+1]};
    h.nav_state.Pitch     = {state_.Pitch[i+1]};
    h.nav_state.Roll      = {state_.Roll[i+1]};
    h.nav_state.Yaw       = {state_.Yaw[i+1]};
    h.nav_state.CbtM = state_.CbtM;
    h.nav_state.CtbM = state_.CtbM;
    rts_smoother_.addHistoryItem(h);
}

// ================== Main Run Loop ==================
/**
 * @brief Check if current step is measurement update step
 * 
 * @return true if measurement update should be performed
 */
bool UkfNavigation::isMeasurementStep(int i) const {
    return (i + 1) % measurement_interval_ == 0;
}

/**
 * @brief Execute full navigation processing sequence
 */
void UkfNavigation::run(const IMUData& imu, const GPSData& gps) {
    const int NavEnd = static_cast<int>(imu.index.size()) - 1;
    
    for (int i = 0; i < NavEnd; i++) {
        // Strapdown inertial navigation update
        updateStrapdown(imu, i);
        
        // Perform UKF steps at measurement intervals
        if (isMeasurementStep(i)) {
            // State prediction
            predictState(i);
            
            // Measurement update
            updateMeasurement(gps, i);
            
            // Error correction
            correctErrors(i);
        }
    }
}
 
// ================== UKF Specific Algorithms ==================

/**
 * @brief Build measurement matrix
 * 
 * @return Measurement matrix
 */
Eigen::MatrixXd UkfNavigation::buildMeasurementMatrix(double roll, double pitch, double yaw,
                                                     double lat, double h, double Rx, double Ry) {
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
 
    // Compute velocity error mapping matrix
    MatrixXd H_m2(3,3);
    double a = cos_roll * cos_yaw - sin_roll * sin_pitch * sin_yaw;
    double b = cos_roll * cos_yaw + sin_roll * sin_pitch * cos_yaw;
    double c = -cos_pitch * sin_yaw;
    double d = sin_roll * cos_yaw - cos_roll * sin_pitch * sin_yaw;
 
    H_m2 << 
        1 + a * a,             a * b,              0,
        c * a,                1 + c * a,           0,
        d * a,                d * b,              1;
 
    // Compute position error mapping matrix
    MatrixXd H_m1(3,3);
    H_m1 << 
        Ry + h, 0, 0,
        0, (Rx + h) * cos(lat * M_PI / 180.0), 0,
        0, 0, 1;
 
    // Assemble measurement matrix
    H.block<3,3>(0,3) = H_m2;  // Velocity error mapping
    H.block<3,3>(3,6) = H_m1;  // Position error mapping
    
    return H;
}
