
# Kalman Filtering for IMU – C++ Implementation

This repository contains a simple, single-axis (1D) Kalman Filter implementation in C++. It's designed for estimating orientation from noisy IMU (Inertial Measurement Unit) sensor data, particularly combining gyroscope rate measurements and accelerometer angle readings.

---

## File Overview

- **`kalmanFiltering.cpp`**  
  Contains the definition and usage of the `KalmanFilter` class, including:
  - Initialization with process and measurement noise
  - A Kalman filter update step using gyroscope and accelerometer data

---

## How It Works

The filter estimates the angle based on:
- **Gyroscope rate** (prone to drift over time)
- **Accelerometer angle** (noisy but absolute)

The Kalman filter fuses these sources to produce a more accurate and stable estimate by:
1. **Predicting** the new angle from the gyroscope.
2. **Correcting** the prediction using the accelerometer reading.
3. **Updating** internal estimates of uncertainty and bias.

---

## Usage

### 1. Compile

```bash
g++ -o kalmanFiltering kalmanFiltering.cpp
```

### 2. Run

```bash
./kalmanFiltering
```

The program includes a demo of filter behavior using dummy data (you can adapt this to real IMU input streams). This is under "Example IMU data (simulated)" in the main() function.

---

## Example Class API

```cpp
KalmanFilter kf(Q, R, P);  // Initialize with process noise, measurement noise, and estimation error

kf.update(gyroRate, dt, measuredAngle);
float angleEstimate = kf.getAngle();
```

- `Q`: Process noise covariance
- `R`: Measurement noise covariance
- `P`: Initial estimation error
- `gyroRate`: Angular velocity (from gyroscope)
- `dt`: Time delta
- `measuredAngle`: Angle from accelerometer
- `K`:  Kalman gain
- `angle`: Estimated angle
- `bias`: Estimated bias in the gyroscope

---

## Customization

You can tune the filter by adjusting:
- `Q` (trust in model prediction)
- `R` (trust in measurements)
- `P` (initial uncertainty)

Higher `Q` → smoother but slower response.  
Higher `R` → less responsive to noisy measurements.

---

## Notes

- This is a **scalar Kalman filter** (single state variable). For 2D/3D orientation or full-state estimation, an extended Kalman filter (EKF) or complementary filter may be required.
- Sensor input is assumed in consistent units (e.g., radians for angle and radians/sec for rate).
