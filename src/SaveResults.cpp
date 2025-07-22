/**
 * @file SaveResults.cpp
 * @brief Implementation of results saving utilities
 *
 * @author peanut-nav
 * @date Created: 2025-07-22
 * @last Modified: 2025-07-22
 * @version 0.1
 */

#include "SaveResults.hpp"
#include <iostream>

/**
 * @brief Save navigation results to an output file
 * 
 * Writes navigation results to a formatted text file for analysis.
 */
void SaveResults::saveNavigationResults(const NavigationState& state, const IMUData& imu) {
    // Open output file
    std::ofstream outFile("../data/KF_navoutQ.dat");
    if (!outFile) {
        std::cerr << "Error: Unable to open output file: KF_navoutQ.dat" << std::endl;
        return;
    }
    
    // Set high precision for numerical output
    outFile << std::setprecision(15);
    
    // Write navigation results for each time step
    for (size_t i = 0; i < imu.index.size(); i++) {
        outFile << std::setw(8) << i + 1 << "   "              // Index
                << std::setw(20) << state.Latitude[i] << "  "   // Latitude (degrees)
                << std::setw(20) << state.Longitude[i] << "  "  // Longitude (degrees)
                << std::setw(20) << state.Altitude[i] << "  "   // Altitude (meters)
                << std::setw(20) << state.Velocity[i][0] << "  "  // East velocity (m/s)
                << std::setw(20) << state.Velocity[i][1] << "  "  // North velocity (m/s)
                << std::setw(20) << state.Velocity[i][2] << "  "  // Up velocity (m/s)
                << std::setw(20) << state.Yaw[i] << "  "        // Yaw angle (degrees)
                << std::setw(20) << state.Pitch[i] << "  "      // Pitch angle (degrees)
                << std::setw(20) << state.Roll[i] << "\n";      // Roll angle (degrees)
    }
    
    // Close file and notify user
    outFile.close();
    std::cout << "Navigation results saved to KF_navoutQ.dat" << std::endl;
}
