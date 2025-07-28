# Navigation Sensor Simulation Dataset

This dataset contains synthetically generated inertial navigation and GPS measurements with realistic error characteristics. All data represents simulated sensor outputs for navigation algorithm development and testing.

## Download

📦 You can download the complete dataset from the [GitHub Release Page](https://github.com/peanut-nav/navigation-filter-lib/releases/tag/v0.1.1-data).
→ Dataset version: **v0.1.0**, published on **2025-07-23**

## File Descriptions

### gpsdata.dat
- **Content**: GPS measurements with simulated errors
- **Data Format**:  
  `Index East_velocity(m/s) North_velocity(m/s) Up_velocity(m/s) Latitude(rad) Longitude(rad) Height(m)`
- **Error Characteristics** (1σ):
  - Position: 
    - Horizontal: ±3.0 m
    - Vertical: ±3.0 m
  - Velocity: ±0.01 m/s (per axis)

### track.dat
- **Content**: Ground truth trajectory without errors
- **Data Format**:  
  `Index Heading(rad) Pitch(rad) Roll(rad) East_velocity(m/s) North_velocity(m/s) Up_velocity(m/s) Latitude(rad) Longitude(rad) Height(m)`
- **Heading Convention**:
  - Range: [-π, π] radians (-180° to 180°)
  - Positive values: Clockwise rotation from North
  - Negative values: Counter-clockwise rotation

### navdata.dat
- **Content**: Ideal INS measurements (gyroscope and accelerometer outputs)
- **File Structure**:
  1. Sampling period (seconds)
  2. Initial position: Latitude(rad) Longitude(rad) Height(m)
  3. Initial velocity: East_velocity(m/s) North_velocity(m/s) Up_velocity(m/s)
  4. Initial attitude: Heading(rad) Pitch(rad) Roll(rad)
  5. Empty line
  6. Repeating blocks of:
     - Gyroscope measurements: x_increment(rad) y_increment(rad) z_increment(rad)
     - Accelerometer measurements: x_increment(m/s) y_increment(m/s) z_increment(m/s)

## Error Models

### GPS Measurement Noise
- **Position Error**:
  - Horizontal (latitude/longitude): 3.0 m (1σ)
  - Vertical (Altitude): 3.0 m (1σ)
- **Velocity Error**: 0.01 m/s (1σ per axis)

### IMU Measurement Noise
- **Gyroscope**:
  - Bias: 0.01 deg/h
  - White noise: 0.01 deg/h (1σ)
- **Accelerometer**:
  - Bias: 50 μg
  - White noise: 50μg (1σ)

## Key Notes
1. All angular quantities are in radians
2. `navdata.dat` contains **ideal (noise-free)** IMU measurements
3. **IMU noise is added to IMU measurements in the function** `DataLoader::addIMUNoise()` located in `DataLoader.cpp`. A fixed seed is used to ensure **reproducibility** across runs
4. Velocity increments represent integrated measurements (Δv)
5. Gyro measurements are angle increments (Δθ)
