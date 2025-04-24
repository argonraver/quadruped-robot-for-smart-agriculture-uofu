#include <iostream>
#include <cmath>

// Simple 1D Kalman Filter for IMU
class KalmanFilter {
public:
    KalmanFilter(float processNoise, float measurementNoise, float estimationError)
        : Q(processNoise), R(measurementNoise), P(estimationError), K(0), angle(0), bias(0) {}
        // Inputs:
            // processNoise - Process noise covariance
            // measurementNoise - Measurement noise covariance
            // estimationError - Estimation error covariance
        // Initialized Variables:
            // K=0 - Kalman gain
            // angle=0 - The estimated state (angle)
            // bias=0 - The estimated bias in the gyroscope

    void update(float gyroRate, float dt, float measuredAngle) {    // Prediction and update steps using sensor data
        // Inputs:
            // gyroRate - Angular velocity
            // dt - Time step
            // measuredAngle - Angle from accelerometer
        // Calculates:
            // K - Kalman Gain, the weight of the measured angle in the correction step
            // angle - The estimated state (angle)
            // P - Estimation error covariance
        
        // Prediction Step
        angle += dt * (gyroRate - bias);    // Estimating the new angle using the gyroscope rate and time step
        P += dt * (dt * Q - 2 * P * bias + R);    // Estimating error covariance, incorporating process noise and measurement noise

        // Update Step
        K = P / (P + R);    // Calculates the Kalman gain, determining the weight of the measured angle in the correction step
        angle += K * (measuredAngle - angle);   // Corrects the estimated angle using the measured angle and the Kalman gain
        P = (1 - K) * P;    // Updates the estimation error covariance after the correction step
    }

    float getAngle() const {
        return angle;
    }

private:
    float Q;  // Process noise covariance
    float R;  // Measurement noise covariance
    float P;  // Estimation error covariance
    float K;  // Kalman gain
    float angle;  // Estimated angle
    float bias;  // Estimated bias in the gyroscope
};

int main() {
    KalmanFilter kf(0.01, 0.1, 1); // Tune these values: Q, R, P (processNoise, measurementNoise, estimationError)

    ///////////////////// Get Sensor Data Here /////////////////////
    // Example IMU data (simulated)
    float dt = 0.01; // 10 ms
    float gyroRate = 0.2; // rad/s
    float measuredAngle = 0.15; // rad
    ////////////////////////////////////////////////////////////////
    
    // Update Estimated Angle 
    // kf.update(gyroRate, dt, measuredAngle);
    // std::cout << "Estimated Angle: " << kf.getAngle() << std::endl;

    // Simulated updating measured angle
    for (int i = 0; i < 100; ++i) {
        measuredAngle += 0.005; // Simulating angle increase
        kf.update(gyroRate, dt, measuredAngle);
        std::cout << "Estimated Angle: " << kf.getAngle() << std::endl;
    }

    return 0;
}


// Explanation of Steps
// 	1. Prediction Step (Using Gyroscope Data)
// 	    • The angle is predicted using the previous estimate and the gyroscope data:
//          theta_predicted = theta_prev + (omega - bias) * dt
// 	    • The estimation error covariance is updated to account for uncertainty:
//          P = P + dt * (dt * (Q - 2) * P * bias + R)
// 
// 	2. Update Step (Using Accelerometer Data)
// 	    • The Kalman gain (K) is calculated to determine how much the measured angle should correct the prediction:
//          K = P/(P + R)
// 	    • The estimated angle is then corrected using the accelerometer data:
//          theta = theta_predicted + K * (theta_measured - theta_predicted)
// 	    • The error covariance is updated to reflect the uncertainty reduction:
//          P = (1 - K) * P