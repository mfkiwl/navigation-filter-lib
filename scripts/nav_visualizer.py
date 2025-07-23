#!/usr/bin/env python3
"""
nav_visualizer.py - Navigation Results Visualizer

This script generates comprehensive visualization of navigation results by processing
KF_navoutQ.dat (navigation output) and track.dat (reference trajectory) files. It produces
7 figures showing position, velocity, attitude measurements and errors, along with 
performance statistics.

Features:
- Correct time axis generation based on IMU rate and simulation duration
- Professional visualization of 7 distinct figures with consistent styling
- Position error conversion from degrees to meters using Earth radius
- Performance statistics calculation and reporting
- Flexible command-line interface with output directory support

Author: peanut-nav
Date: 2025-07-23
Version: 0.1.0
"""

import os
import sys
import argparse
import numpy as np
import matplotlib.pyplot as plt

# Configure plotting aesthetics for professional presentation
plt.style.use('seaborn-whitegrid')
plt.rcParams['figure.dpi'] = 150
plt.rcParams['savefig.dpi'] = 300
plt.rcParams['font.size'] = 10
plt.rcParams['axes.titlesize'] = 12
plt.rcParams['axes.labelsize'] = 10
plt.rcParams['xtick.labelsize'] = 8
plt.rcParams['ytick.labelsize'] = 8

def load_navoutq(file_path, imu_rate=200, sim_time=600):
    """
    Load navigation results from KF_navoutQ.dat file
    
    Parameters:
    file_path (str): Path to navigation results file
    imu_rate (float): IMU sampling frequency in Hz
    sim_time (float): Total simulation duration in seconds
    
    Returns:
    dict: Dictionary containing time series of navigation states
    """
    print(f"Loading navigation results from: {file_path}")
    data = np.loadtxt(file_path)
    
    # Parse data structure according to saveNavigationResults C++ function
    # Column mapping: 
    # 0: index, 1: latitude(deg), 2: longitude(deg), 3: altitude(m), 
    # 4: east_velocity(m/s), 5: north_velocity(m/s), 6: up_velocity(m/s),
    # 7: yaw(deg), 8: pitch(deg), 9: roll(deg)
    n_points = data.shape[0]
    
    # Generate time vector from simulation parameters
    time_vector = np.linspace(0, sim_time, n_points)
    
    return {
        'time': time_vector,
        'latitude': data[:, 1],
        'longitude': data[:, 2],
        'height': data[:, 3],
        'velocity_east': data[:, 4],
        'velocity_north': data[:, 5],
        'velocity_up': data[:, 6],
        'yaw': data[:, 7],
        'pitch': data[:, 8],
        'roll': data[:, 9]
    }

def load_track(file_path):
    """
    Load reference trajectory data from track.dat file
    
    Parameters:
    file_path (str): Path to trajectory data file
    
    Returns:
    dict: Dictionary containing time series of reference trajectory states
    """
    print(f"Loading reference trajectory from: {file_path}")
    data = np.loadtxt(file_path)
    
    # Conversion factor from radians to degrees
    rad2deg = 180.0 / np.pi
    
    # Process yaw: normalize negative values to [0, 2π) and convert to degrees
    yaw_rad = data[:, 1].copy()
    yaw_rad[yaw_rad < 0] += 2 * np.pi
    ref_yaw = yaw_rad * rad2deg
    
    # Apply heading adjustment: 360 - yaw (consistent with C++ implementation)
    ref_yaw = 360.0 - ref_yaw
    
    return {
        'time': data[:, 0],
        'ref_yaw': ref_yaw,
        'ref_pitch': data[:, 2] * rad2deg,
        'ref_roll': data[:, 3] * rad2deg,
        'ref_velocity_east': data[:, 4],
        'ref_velocity_north': data[:, 5],
        'ref_velocity_up': data[:, 6],
        'ref_latitude': data[:, 7] * rad2deg,
        'ref_longitude': data[:, 8] * rad2deg,
        'ref_height': data[:, 9]
    }

def visualize_results(state, track, output_dir=None, imu_rate=200, sim_time=600):
    """
    Generate comprehensive visualization of navigation results
    
    Parameters:
    state (dict): Navigation state data
    track (dict): Reference trajectory data
    output_dir (str): Directory to save figures (None for display)
    imu_rate (float): IMU sampling frequency in Hz
    sim_time (float): Total simulation duration in seconds
    """
    # Earth radius constant (WGS-84 compatible value)
    Re = 6378135.072
    
    # Time vector for all plots
    t1 = state['time']
    
    # ===== Figure 1: Position and Velocity Measurements =====
    fig1 = plt.figure(1, figsize=(12, 8))
    fig1.suptitle('Position and Velocity Measurements', fontsize=14)
    
    # Latitude subplot
    plt.subplot(2, 3, 1)
    plt.plot(t1, state['latitude'])
    plt.title('Latitude Measurement')
    plt.xlabel('Time (s)')
    plt.ylabel('Latitude (°)')
    plt.grid(True)
    
    # Longitude subplot
    plt.subplot(2, 3, 2)
    plt.plot(t1, state['longitude'])
    plt.title('Longitude Measurement')
    plt.xlabel('Time (s)')
    plt.ylabel('Longitude (°)')
    plt.grid(True)
    
    # Height subplot
    plt.subplot(2, 3, 3)
    plt.plot(t1, state['height'])
    plt.title('Height Measurement')
    plt.xlabel('Time (s)')
    plt.ylabel('Height (m)')
    plt.grid(True)
    
    # East velocity subplot
    plt.subplot(2, 3, 4)
    plt.plot(t1, state['velocity_east'])
    plt.title('East Velocity Measurement')
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity (m/s)')
    plt.grid(True)
    
    # North velocity subplot
    plt.subplot(2, 3, 5)
    plt.plot(t1, state['velocity_north'])
    plt.title('North Velocity Measurement')
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity (m/s)')
    plt.grid(True)
    
    # Up velocity subplot
    plt.subplot(2, 3, 6)
    plt.plot(t1, state['velocity_up'])
    plt.title('Up Velocity Measurement')
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity (m/s)')
    plt.grid(True)
    
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    if output_dir:
        plt.savefig(os.path.join(output_dir, 'position_velocity_results.png'))
    
    # ===== Figure 2: Attitude Estimation Results =====
    fig2 = plt.figure(2, figsize=(8, 10))
    fig2.suptitle('Attitude Estimation Results', fontsize=14)
    
    # Pitch angle estimation
    plt.subplot(3, 1, 1)
    plt.plot(t1, state['pitch'])
    plt.title('Pitch Angle Estimation')
    plt.xlabel('Time (s)')
    plt.ylabel('Pitch Angle (°)')
    plt.grid(True)
    
    # Roll angle estimation
    plt.subplot(3, 1, 2)
    plt.plot(t1, state['roll'])
    plt.title('Roll Angle Estimation')
    plt.xlabel('Time (s)')
    plt.ylabel('Roll Angle (°)')
    plt.grid(True)
    
    # Yaw angle estimation
    plt.subplot(3, 1, 3)
    plt.plot(t1, state['yaw'])
    plt.title('Yaw Angle Estimation')
    plt.xlabel('Time (s)')
    plt.ylabel('Yaw Angle (°)')
    plt.grid(True)
    
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    if output_dir:
        plt.savefig(os.path.join(output_dir, 'attitude_estimation.png'))
    
    # ===== Figure 3: Position Errors =====
    fig3 = plt.figure(3, figsize=(8, 10))
    fig3.suptitle('Position Errors', fontsize=14)
    
    # Latitude error (converted from degrees to meters)
    plt.subplot(3, 1, 1)
    lat_error = (state['latitude'] - track['ref_latitude']) * (np.pi/180) * Re
    plt.plot(t1, lat_error)
    plt.title('Latitude Error')
    plt.xlabel('Time (s)')
    plt.ylabel('Error (m)')
    plt.grid(True)
    
    # Longitude error (converted from degrees to meters)
    plt.subplot(3, 1, 2)
    lon_error = (state['longitude'] - track['ref_longitude']) * (np.pi/180) * Re
    plt.plot(t1, lon_error)
    plt.title('Longitude Error')
    plt.xlabel('Time (s)')
    plt.ylabel('Error (m)')
    plt.grid(True)
    
    # Height error
    plt.subplot(3, 1, 3)
    alt_error = state['height'] - track['ref_height']
    plt.plot(t1, alt_error)
    plt.title('Height Error')
    plt.xlabel('Time (s)')
    plt.ylabel('Error (m)')
    plt.grid(True)
    
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    if output_dir:
        plt.savefig(os.path.join(output_dir, 'position_errors.png'))
    
    # ===== Figure 4: Velocity Errors =====
    fig4 = plt.figure(4, figsize=(8, 10))
    fig4.suptitle('Velocity Errors', fontsize=14)
    
    # East velocity error
    plt.subplot(3, 1, 1)
    v_east_error = state['velocity_east'] - track['ref_velocity_east']
    plt.plot(t1, v_east_error, 'r')
    plt.title('East Velocity Error')
    plt.xlabel('Time (s)')
    plt.ylabel('Error (m/s)')
    plt.grid(True)
    
    # North velocity error
    plt.subplot(3, 1, 2)
    v_north_error = state['velocity_north'] - track['ref_velocity_north']
    plt.plot(t1, v_north_error, 'r')
    plt.title('North Velocity Error')
    plt.xlabel('Time (s)')
    plt.ylabel('Error (m/s)')
    plt.grid(True)
    
    # Up velocity error
    plt.subplot(3, 1, 3)
    v_up_error = state['velocity_up'] - track['ref_velocity_up']
    plt.plot(t1, v_up_error, 'r')
    plt.title('Up Velocity Error')
    plt.xlabel('Time (s)')
    plt.ylabel('Error (m/s)')
    plt.grid(True)
    
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    if output_dir:
        plt.savefig(os.path.join(output_dir, 'velocity_errors.png'))
    
    # ===== Figure 6: Attitude Errors =====
    fig6 = plt.figure(6, figsize=(8, 10))
    fig6.suptitle('Attitude Errors', fontsize=14)
    
    # Pitch angle error
    plt.subplot(3, 1, 1)
    pitch_error = state['pitch'] - track['ref_pitch']
    plt.plot(t1, pitch_error)
    plt.title('Pitch Angle Error')
    plt.xlabel('Time (s)')
    plt.ylabel('Error (°)')
    plt.grid(True)
    
    # Roll angle error
    plt.subplot(3, 1, 2)
    roll_error = state['roll'] - track['ref_roll']
    plt.plot(t1, roll_error)
    plt.title('Roll Angle Error')
    plt.xlabel('Time (s)')
    plt.ylabel('Error (°)')
    plt.grid(True)
    
    # Yaw angle error
    plt.subplot(3, 1, 3)
    yaw_error = state['yaw'] - track['ref_yaw']
    plt.plot(t1, yaw_error)
    plt.title('Yaw Angle Error')
    plt.xlabel('Time (s)')
    plt.ylabel('Error (°)')
    plt.grid(True)
    
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    if output_dir:
        plt.savefig(os.path.join(output_dir, 'attitude_errors.png'))
    
    # ===== Figure 7: Reference Trajectory =====
    fig7 = plt.figure(7, figsize=(10, 8))
    plt.plot(track['ref_longitude'], track['ref_latitude'])
    plt.title('Reference Trajectory')
    plt.xlabel('Longitude (°)')
    plt.ylabel('Latitude (°)')
    plt.grid(True)
    plt.tight_layout()
    if output_dir:
        plt.savefig(os.path.join(output_dir, 'reference_trajectory.png'))
    
    # ===== Performance Statistics Calculation =====
    z_up = 20000
    z_down = min(120000, len(t1))
    
    if z_down > z_up:
        # Convert position errors to meters for RMS calculation
        lat_error = (state['latitude'][z_up:z_down] - track['ref_latitude'][z_up:z_down]) * (np.pi/180) * Re
        lon_error = (state['longitude'][z_up:z_down] - track['ref_longitude'][z_up:z_down]) * (np.pi/180) * Re
        
        # Direct errors for height and attitude
        alt_error = state['height'][z_up:z_down] - track['ref_height'][z_up:z_down]
        yaw_error = state['yaw'][z_up:z_down] - track['ref_yaw'][z_up:z_down]
        pitch_error = state['pitch'][z_up:z_down] - track['ref_pitch'][z_up:z_down]
        roll_error = state['roll'][z_up:z_down] - track['ref_roll'][z_up:z_down]
        
        # Calculate RMS errors
        z1 = np.sqrt(np.sum(lat_error**2) / (z_down - z_up))
        z2 = np.sqrt(np.sum(lon_error**2) / (z_down - z_up))
        z3 = np.sqrt(np.sum(alt_error**2) / (z_down - z_up))
        z4 = np.sqrt(np.sum(yaw_error**2) / (z_down - z_up))
        z5 = np.sqrt(np.sum(pitch_error**2) / (z_down - z_up))
        z6 = np.sqrt(np.sum(roll_error**2) / (z_down - z_up))
        
        # Print performance summary
        print('\n===== Performance Statistics =====')
        print(f'Latitude RMS Error: {z1:.6f} m')
        print(f'Longitude RMS Error: {z2:.6f} m')
        print(f'Height RMS Error: {z3:.6f} m')
        print(f'Yaw Angle RMS Error: {z4:.6f} °')
        print(f'Pitch Angle RMS Error: {z5:.6f} °')
        print(f'Roll Angle RMS Error: {z6:.6f} °')
        
        # Save statistics to file if output directory specified
        if output_dir:
            stats_file = os.path.join(output_dir, 'performance_statistics.txt')
            with open(stats_file, 'w') as f:
                f.write("===== Performance Statistics =====\n")
                f.write(f"Latitude RMS Error: {z1:.6f} m\n")
                f.write(f"Longitude RMS Error: {z2:.6f} m\n")
                f.write(f"Height RMS Error: {z3:.6f} m\n")
                f.write(f"Yaw Angle RMS Error: {z4:.6f} °\n")
                f.write(f"Pitch Angle RMS Error: {z5:.6f} °\n")
                f.write(f"Roll Angle RMS Error: {z6:.6f} °\n")
            print(f"Performance statistics saved to: {stats_file}")
    
    # Display figures if not saving to directory
    if not output_dir:
        plt.show()
    elif output_dir:
        print(f"All figures saved to: {output_dir}")

if __name__ == "__main__":
    # Configure command-line argument parser
    parser = argparse.ArgumentParser(
        description='Navigation Results Visualization Tool',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument('--navoutq', type=str, default="../output/KF_navoutQ.dat",
                        help='Path to navigation results file')
    parser.add_argument('--track', type=str, default="../data/track.dat",
                        help='Path to reference trajectory file')
    parser.add_argument('--output', type=str, default=None,
                        help='Output directory for saving figures')
    parser.add_argument('--save', action='store_true',
                        help='Save figures without display (requires --output)')
    parser.add_argument('--imu-rate', type=float, default=200.0,
                        help='IMU sampling rate in Hz')
    parser.add_argument('--sim-time', type=float, default=600.0,
                        help='Total simulation time in seconds')
    
    # Parse command-line arguments
    args = parser.parse_args()
    
    # Validate and prepare output directory
    if args.output:
        if not os.path.exists(args.output):
            os.makedirs(args.output)
            print(f"Created output directory: {args.output}")
        elif not os.path.isdir(args.output):
            print(f"Error: Output path is not a directory: {args.output}")
            sys.exit(1)
    
    # Verify input file existence
    if not os.path.exists(args.navoutq):
        print(f"Error: Navigation results file not found: {args.navoutq}")
        sys.exit(1)
    
    if not os.path.exists(args.track):
        print(f"Error: Reference trajectory file not found: {args.track}")
        sys.exit(1)
    
    # Load navigation and reference data
    state = load_navoutq(args.navoutq, imu_rate=args.imu_rate, sim_time=args.sim_time)
    track = load_track(args.track)
    
    # Handle data length inconsistencies
    if len(state['time']) != len(track['time']):
        print(f"Warning: Data length mismatch ({len(state['time'])} vs {len(track['time'])} points)")
        min_len = min(len(state['time']), len(track['time']))
        print(f"Truncating to common length: {min_len} points")
        
        # Truncate datasets to minimum length
        for key in state:
            state[key] = state[key][:min_len]
        for key in track:
            track[key] = track[key][:min_len]
    
    # Generate and display/save visualizations
    print("Generating navigation visualizations...")
    visualize_results(state, track, 
                      output_dir=args.output if args.save else None,
                      imu_rate=args.imu_rate,
                      sim_time=args.sim_time)
    print("Visualization process completed")
