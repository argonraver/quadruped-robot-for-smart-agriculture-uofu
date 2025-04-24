#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "control_interfaces/msg/coords.hpp"
#include "control_interfaces/msg/joint_angles.hpp"

// Define certain variables up here so they can be used globally
std::string legNumber;

class LegNode : public rclcpp::Node {
    public:
        // Does count correspond to the number of nodes of this type?
        LegNode() : Node("leg_node") {
            // Get the correct axes for leg, to be controlled by launch file
            this->declare_parameter("leg_number", "leg_one");
	    legNumber = this->get_parameter("leg_number").as_string();

            // Publish to a leg
            legPublisher_ = this->create_publisher<control_interfaces::msg::JointAngles>(legNumber, 10);

	    // Set up subscription to correct target
	    this->declare_parameter("leg_sub", "leg_one_commands");
	    std::string legSubTopic = this->get_parameter("leg_sub").as_string();

            // A subscriber, calls back on received target. Should be fastest arrangement.
            subscription_ = this->create_subscription<control_interfaces::msg::Coords>(legSubTopic, 10, std::bind(&LegNode::topic_callback, this, std::placeholders::_1));

            std::string logMessage = this->get_parameter("leg_number").as_string().c_str();
	    logMessage = logMessage + " IK node up";
	    RCLCPP_INFO(this->get_logger(), "%s", logMessage.c_str());
	}


    private:
        // Define leg properties FIX ME
	// Inches
        const double a2 = 6.3; // length of upper leg
        const double a4 = 6.75; // length of lower leg

       void topic_callback(const control_interfaces::msg::Coords & msg) const {
            // Calculate the target angles
            // Added scaling for gearing, odrive expectations.
            // IK Returns radians, converted to degrees for ODrive then divided by 10 for gearing.
            double xCoord = msg.x;
            double yCoord = msg.y;

            double lowerAngle = 2 * std::atan2(std::sqrt(std::pow((a2+a4), 2.0) - (std::pow((xCoord), 2.0) + std::pow((yCoord), 2.0))), sqrt(std::pow((xCoord), 2.0) + std::pow((yCoord), 2.0)) - std::pow((a2 - a4), 2.0));

            // Choose the right elbow solution for each side
	    if(legNumber == "leg_two" || legNumber == "leg_four") {
              lowerAngle = lowerAngle * -1;
            }

	    double upperAngle = (std::atan2(yCoord, xCoord) - std::atan2(a4 * std::sin(lowerAngle), a2 + a4 * std::cos(lowerAngle)));

            // Generate output message and convert to ODrive units, gear ratio
            auto outputMessage = control_interfaces::msg::JointAngles();
            outputMessage.thetaone = (18 / M_PI) * upperAngle;
            outputMessage.thetatwo = (18 / M_PI) * lowerAngle;

            // Publish the message
            legPublisher_->publish(outputMessage);

            // Log input and output. Got busy with ramps so disabled.
            //RCLCPP_INFO(this->get_logger(), " Recieved Inputs: (%2f, %2f) \n Generated Outputs: (%2f, %2f)", msg.x, msg.y, upperAngle, lowerAngle);
        }
        rclcpp::Publisher<control_interfaces::msg::JointAngles>::SharedPtr legPublisher_;
        rclcpp::Subscription<control_interfaces::msg::Coords>::SharedPtr subscription_;
	
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LegNode>());
    rclcpp::shutdown();
    return 0;
    }
