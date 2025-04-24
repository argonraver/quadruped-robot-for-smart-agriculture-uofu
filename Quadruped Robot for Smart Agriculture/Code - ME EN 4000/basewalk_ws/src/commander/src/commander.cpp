#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "control_interfaces/msg/coords.hpp"
#include "control_interfaces/msg/primitiveinterface.hpp"
#include "std_msgs/msg/string.hpp"

// Will translate strings "stand" "sit" "forward" "stop" "left" "right"
class Commander : public rclcpp::Node {
	public:
		Commander() : Node("commander") {
			subscription_ = this->create_subscription<std_msgs::msg::String>("inputCommands", 10, std::bind(&Commander::topic_callback, this, std::placeholders::_1));
			commandPublisher_ = this->create_publisher<control_interfaces::msg::Primitiveinterface>("primitiv_topic", 10); // Convert to a matching topic
		}

	private:
		int stopPrimitive = 1;
		int standPrimitive = 2;
		int sitPrimitive = 3;
		int walkPrimitive = 4;
		int twistDemo = 5;
		int pushDemo = 6;

		void topic_callback(const std_msgs::msg::String & msg) const { // When a message is received...
			auto outputMessage = control_interfaces::msg::Primitiveinterface();
			
			outputMessage.primitive_cmd = stopPrimitive; // Default to stop command
			
			// If the received message matches something I know, swap the output to the relevant command.
			// If you were to implement an Xbox controller or keyboard controller I would swap the comparison messages here and have this node listen for those messages. 
			if(msg.data == "stand") {
				outputMessage.primitive_cmd = standPrimitive;
				RCLCPP_INFO(this->get_logger(), "Standing");
			} else if(msg.data == "sit") {
				outputMessage.primitive_cmd = sitPrimitive;
				RCLCPP_INFO(this->get_logger(), "Sitting");
			} else if(msg.data == "forward") {
				outputMessage.primitive_cmd = walkPrimitive;
				RCLCPP_INFO(this->get_logger(), "Walking");
			} else if(msg.data == "twist") {
				outputMessage.primitive_cmd = twistDemo;
				RCLCPP_INFO(this->get_logger(), "Walking");
			} else if(msg.data == "push") {
				outputMessage.primitive_cmd = pushDemo;
				RCLCPP_INFO(this->get_logger(), "Walking");
			} else {
				RCLCPP_INFO(this->get_logger(), "Stopping");
			}
			
			// Pass the command on to the rest of the system.
			commandPublisher_->publish(outputMessage);
		}
		// Include the pointers so ROS can find things
		rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
		rclcpp::Publisher<control_interfaces::msg::Primitiveinterface>::SharedPtr commandPublisher_;
};

int main(int argc, char * argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Commander>());
	rclcpp::shutdown();
	return 0;
}

