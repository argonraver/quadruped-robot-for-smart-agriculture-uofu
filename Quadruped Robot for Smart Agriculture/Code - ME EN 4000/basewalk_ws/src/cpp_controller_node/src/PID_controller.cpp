#include <iostream>
#include <chrono>
#include <memory>
#include <cstdlib>
#include <cmath>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "control_interfaces/srv/pidinterface.hpp"

// Initialize Variables
// PID gains
double Kp = 1.5;  // Proportional gain
double Ki = 0.1;  // Integral gain
double Kd = 0.05; // Derivative gain

// PID control variables
double prev_error = 0;
double integral = 0;
double dt = 0.01;  // Time step in seconds Data

// PID Controller function
double PIDController(double des_val, double curr_val) {
    double output_val = 0.0;
    
    while (output_val != des_val) {
        // Calculate error
        double curr_error = des_val - curr_val;

        // Calculate integral term
        integral += curr_error * dt;

        // Calculate derivative term
        double derivative = (curr_error - prev_error) / dt;

        // Compute PID output
        output_val = (Kp * curr_error) + (Ki * integral) + (Kd * derivative);

        // Store previous error
        prev_error = curr_error;

        // Wait for next cycle (simulate real-time control loop)
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(dt * 1000)));
    }

    return output_val;
}

void pid_control(const std::shared_ptr<control_interfaces::srv::Pidinterface::Request> des_data,
          std::shared_ptr<control_interfaces::srv::Pidinterface::Response> out_data)
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "PID Service: Data Received");

    // Output angle
    double output_val = PIDController(des_data->des_val, des_data->curr_val);

    out_data->output_val = output_val;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "PID Service: Return Complete");

}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("pid_server");

    rclcpp::Service<control_interfaces::srv::Pidinterface>::SharedPtr service =
        node->create_service<control_interfaces::srv::Pidinterface>("pid_control", &pid_control);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "PID Control Service: Initialized - Ready to Receive Desired Angles");

    rclcpp::spin(node);
    rclcpp::shutdown();
};


