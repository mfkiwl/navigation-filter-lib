/**
 * @file UkfInitializer.hpp
 * @brief UKF system initializer
 *
 * @author peanut-nav
 * @date Created: 2025-08-10
 * @last Modified: 2025-08-10
 * @version 0.3.3
 */

#pragma once
#include "ISystemInitializer.hpp"
#include "../params/UkfParams.hpp"

class UkfInitializer : public ISystemInitializer {
public:
    UkfInitializer(int IMUrate, int GPSrate, int simTime);
    
    void initialize_params(NavParamsBase& base_params, 
                          const std::string& dataDir) override;
    
    void initialize_state(NavigationState& state, 
                         int totalPoints) override;

    void initialize_kalman(NavParamsBase& base_params, 
                          int totalPoints) override;

private:
    int IMUrate_;
    int GPSrate_;
    int simTime_;
    
    double lat_rad_ = 0.0;
    double lon_rad_ = 0.0;
    double h0_ = 0.0;
    double v_east_ = 0.0;
    double v_north_ = 0.0;
    double v_up_ = 0.0;
    double fai_rad_ = 0.0;
    double st_rad_ = 0.0;
    double r_rad_ = 0.0;
    
    void loadInitialNavData(const std::string& filePath);
    Eigen::MatrixXd setupMeasurementNoise() const;
    Eigen::MatrixXd setupProcessNoise() const;
    Eigen::MatrixXd initializeCovarianceMatrix(double lat) const;
};
