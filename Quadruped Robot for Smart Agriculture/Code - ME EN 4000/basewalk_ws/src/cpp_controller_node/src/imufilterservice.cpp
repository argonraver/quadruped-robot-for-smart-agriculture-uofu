#include <chrono>
#include <functional>
#include <memory>

#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "control_interfaces/srv/imufilteredinterface.hpp"

// For some reason, these definitions need to be in here or else the private class implodes
double weightAccel_x = 0.2;
double weightAccel_y = 0.2;
double weightAccel_z = 0.2;

double weightGyro_x = 0.4;
double weightGyro_y = 0.4;
double weightGyro_z = 0.4;

// Processed speed after filtering (rad/s)
double w_x;
double w_y;
double w_z;

// Last time's rotational speed
double wLast_x = 0.0;
double wLast_y = 0.0;
double wLast_z = 0.0;

// Combo weight is the weight of the gyroscope added to the accelerometer data
double comboWeight_x = 0.2;
double comboWeight_y = 0.2;
double comboWeight_z = 0.2;

// Results
double roll;
double pitch;

// Current and Previous gyroscope angle
double Axz_last = 0.0;
double Ayz_last = 0.0;
double Axz = 0.0;
double Ayz = 0.0;

// Accelerometer data and magnitude of the acceleration vector post filtering
double gravMagnitude;
double accelFiltered_x;
double accelFiltered_y;
double accelFiltered_z;

double accelFilteredLast_x = 0.0;
double accelFilteredLast_y = 0.0;
double accelFilteredLast_z = 10.0;

// Initializing first estimate of total vector gyro+accel
double Rx_est_last = 0.0;
double Ry_est_last  = 0.0;
double Rz_est_last  = 1.0;
double Rx_est = 0.0;
double Ry_est = 0.0;
double Rz_est = 1.0;
double R;

double RxGyro = 0.0;
double RyGyro = 0.0;
double RzGyro = 0.0;

double RxAcc = 0.0;
double RyAcc = 0.0;
double RzAcc = 0.0;

void add(const std::shared_ptr<control_interfaces::srv::Imufilteredinterface::Request> imu_data,
          std::shared_ptr<control_interfaces::srv::Imufilteredinterface::Response>      orient_data)
{ 
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Filter Service: Data Received");

  // Setting last points for derivation
  Axz_last = atan2(Rx_est_last, Rz_est_last);
  Ayz_last = atan2(Ry_est_last, Rz_est_last);

  Rx_est_last = Rx_est;
  Ry_est_last = Ry_est;
  Rz_est_last = Rz_est;

  // Retrieve new acceleration data
  accelFiltered_x = imu_data->accel_x * weightAccel_x + accelFilteredLast_x*(1-weightAccel_x);
  accelFiltered_y = imu_data->accel_y * weightAccel_y + accelFilteredLast_y*(1-weightAccel_y);
  accelFiltered_z = imu_data->accel_z * weightAccel_z + accelFilteredLast_z*(1-weightAccel_z);
  gravMagnitude = sqrt(pow(accelFiltered_x, 2) + pow(accelFiltered_y, 2) + pow(accelFiltered_z, 2));

  // Normalize acceleration data to the domain of [-1, 1]
  RxAcc = accelFiltered_x/gravMagnitude;
  RyAcc = accelFiltered_y/gravMagnitude;
  RzAcc = accelFiltered_z/gravMagnitude;

  w_x = (imu_data->gyro_x*weightGyro_x + wLast_x*(1-weightGyro_x)); // Gyro X or YZ (pitch)
  w_y = (imu_data->gyro_y*weightGyro_y + wLast_y*(1-weightGyro_y)); // Gyro Y or XZ (roll)
  w_z = (imu_data->gyro_z*weightGyro_z + wLast_z*(1-weightGyro_z)); // Gyro Z or XY (yaw)
  Axz = (w_x * imu_data->time_inc) + Axz_last;  // Roll angle from gyro
  Ayz = Ayz_last + (w_y * imu_data->time_inc); // Pitch angle from gyro

  // If the rotation doesn't change, set the gyroscope measurement to the last estimate.
  // Else, normalize each vector attained by the gyroscope data
  if (abs(Rz_est_last) < 0.05){ // Raise threshold if error is met
    RxGyro = Rx_est;
    RyGyro = Ry_est;
    RzGyro = Rz_est;
  } else {
    RxGyro = sin(Axz)/sqrt(1 + pow(cos(Axz), 2) * pow(tan(Ayz), 2));
    RyGyro = sin(Ayz)/sqrt(1 + pow(cos(Ayz), 2) * pow(tan(Axz), 2));
    if (Rz_est_last < 0) {
        RzGyro = -sqrt(1 - pow(RxGyro, 2) - pow(RyGyro, 2));
    } else {
        RzGyro = sqrt(1 - pow(RxGyro, 2) - pow(RyGyro, 2));
        }
  }

  // Combine accelerometer and gyroscope values
  Rx_est = (RxAcc + RxGyro * comboWeight_x) / (1 + comboWeight_x);
  Ry_est = (RyAcc + RyGyro * comboWeight_y) / (1 + comboWeight_y);
  Rz_est = (RzAcc + RzGyro * comboWeight_z) / (1 + comboWeight_z);
  R = sqrt(pow(Rx_est,2) + pow(Ry_est,2) + pow(Rz_est,2));

  // Re-normalize values
  Rx_est = Rx_est/R;
  Ry_est = Ry_est/R;
  Rz_est = Rz_est/R;
  
  // Convert to angle
  pitch = atan2(Rx_est, Rz_est)*180/M_PI;
  roll = atan2(Ry_est, Rz_est)*180/M_PI;

  orient_data->roll = roll;
  orient_data->pitch = pitch;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Filter Service: Return Complete");
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("imufilter_server");

  rclcpp::Service<control_interfaces::srv::Imufilteredinterface>::SharedPtr service =
    node->create_service<control_interfaces::srv::Imufilteredinterface>("imufilter", &add);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Filter Service: Initialized - Ready to Receive IMU Data");

  rclcpp::spin(node);
  rclcpp::shutdown();
};
