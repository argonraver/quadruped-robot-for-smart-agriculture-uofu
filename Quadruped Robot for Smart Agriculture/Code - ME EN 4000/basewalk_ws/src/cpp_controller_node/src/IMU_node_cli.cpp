// Subscribers:
  // primitives_topic
  // imu_topic

// Publishers:
  // filtered_imu_topic
  // To be added if not in client: leg_nodes

// Clients:
  // IMU filtering
  // Motor position PID controller
  // Torque PID controller
  // Neutral command

#include <chrono>
#include <cstdlib>
#include <memory>
#include "rclcpp/rclcpp.hpp"

#include "control_interfaces/srv/neutralcommandinterface.hpp"
#include "control_interfaces/srv/imufilteredinterface.hpp"

#include "control_interfaces/srv/pidinterface.hpp"
#include "control_interfaces/msg/primitiveinterface.hpp"

#include "control_interfaces/msg/imuinterface.hpp"
#include "control_interfaces/msg/filteredimuinterface.hpp"

#include "control_interfaces/msg/joint_angles.hpp"
#include "control_interfaces/msg/coords.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

bool is_standing = 0;
double standHeight = 11.5; // Inches
double sitHeight = 3.9; // Inches

double roll; // Define IMU data as global for easy access calculating primitives
double pitch;

// Subscribes to primitives_topic to get primitive inputs
  class PrimitivesRunner : public rclcpp::Node
  {
    public:
      PrimitivesRunner()
      : Node("primitives_topic")
      {
        primitivesubscription_ = this->create_subscription<control_interfaces::msg::Primitiveinterface>(
        "primitiv_topic", 10, std::bind(&PrimitivesRunner::topic_callback, this, _1));

	// Set parameters for which legs to publish to
	this->declare_parameter("first_leg_number", "leg_one_commands");
        std::string legNumOne = this->get_parameter("first_leg_number").as_string();

	this->declare_parameter("second_leg_number", "leg_two_commands");
        std::string legNumTwo = this->get_parameter("second_leg_number").as_string();

	this->declare_parameter("third_leg_number", "leg_three_commands");
        std::string legNumThr = this->get_parameter("third_leg_number").as_string();

	this->declare_parameter("fourth_leg_number", "leg_four_commands");
        std::string legNumFou = this->get_parameter("fourth_leg_number").as_string();

	// Then create publishers for those legs. If want variable leg number add a parameter for leg quantity, and switch on that
	legPubOne_ = this->create_publisher<control_interfaces::msg::Coords>(legNumOne, 10);
	legPubTwo_ = this->create_publisher<control_interfaces::msg::Coords>(legNumTwo, 10);
	legPubThr_ = this->create_publisher<control_interfaces::msg::Coords>(legNumThr, 10);
	legPubFou_ = this->create_publisher<control_interfaces::msg::Coords>(legNumFou, 10);

}

    private:
      void topic_callback(const control_interfaces::msg::Primitiveinterface & msg) const // When a move command is recieved, move
      {
        RCLCPP_INFO(this->get_logger(), "I heard: '%ld'", msg.primitive_cmd);
        if(msg.primitive_cmd == 1) { // Stop command

          // With current walking system, it always ends each step at a "stopped" position.

        } else if(msg.primitive_cmd == 2) { // Stand command

	  // If sitting, call service to stand, else do nothing
	  if(!is_standing) {
	    double maxRampPosition = standHeight;
	    double minRampPosition = sitHeight;
	    double rampSteps = 1000.0;
	    double standTime = 6000; // Time to stand in milliseconds

	    // Publish to each leg a message corresponding to the desired ramp positions
	    auto legMessage = control_interfaces::msg::Coords(); // Coordinates in inches

	    for(int i = 0; i <= rampSteps; i++) {
	      // Here is where we need to call the IMU
	      // Perform trig on roll/pitch to determine how far the legs have to move to level out the body.

	      // All legs match for standing up, ramp straight up and down:
	      legMessage.x = i * (maxRampPosition - minRampPosition) / rampSteps + minRampPosition;

              // Start from a slightly back position then rotate forward to zero y position
	      legMessage.y = 2.0;
	      if(i >= 2 * rampSteps/3) {
		legMessage.y = (i - 2*rampSteps/3) * -(2.0) / (rampSteps/3) + 2.0;
	      }

	      legPubTwo_->publish(legMessage);
	      legPubFou_->publish(legMessage);

	      legMessage.y = -1 * legMessage.y; // Odd legs have opposite y directions. See Inverse Kinematics.
	      legPubOne_->publish(legMessage);
	      legPubThr_->publish(legMessage);

	      // Delay command
	      std::this_thread::sleep_for(std::chrono::milliseconds{static_cast<long>(standTime/rampSteps)}); // This *should* be fine if the IMU is in a separate thread, but if it isn't do t - t_last and only change the command once complete.
	    }
	  is_standing = 1;
	  } else {
	    RCLCPP_INFO(this->get_logger(), "Already standing!");
	  }


	} else if(msg.primitive_cmd == 3) { // Sit command

	  // If standing and neutral, call service to sit, else do nothing
	  if(is_standing) { // If walking is changed to need a separate stop command, make sure this checks for that too.
	    double maxRampPosition = standHeight;
	    double minRampPosition = sitHeight;
	    double rampSteps = 1000.0;
	    double sitTime = 12000; // Time to sit in milliseconds.

            // Publish to each leg a message corresponding to the desired ramp angles
            auto legMessage = control_interfaces::msg::Coords(); // Coordinates in inches

            for(int i = rampSteps; i >= 0; i--) {
              // Here is where we need to call the IMU
	      // Perform trig on roll/pitch to determine how far the legs have to move to level out the body.
            
              // Starting at the standing position, ramp down to the sitting position
	      legMessage.x = i * (maxRampPosition - minRampPosition) / rampSteps + minRampPosition;
	      // Start in line with the joints
	      legMessage.y = 0.0;

              // Rotate back to the sitting position at the end.
	      if(i <= rampSteps/5) {
		legMessage.y = (rampSteps/5 - i) / (rampSteps/5) * 2;
	      }

	      legPubFou_->publish(legMessage);
              legPubTwo_->publish(legMessage);

	      legMessage.y = legMessage.y * -1; // Flip the y axis for the odd legs.
              legPubThr_->publish(legMessage);
              legPubOne_->publish(legMessage);

	      std::this_thread::sleep_for(std::chrono::milliseconds{static_cast<long>(sitTime/rampSteps)});
	     }
	  is_standing = 0;
          } else {
            RCLCPP_INFO(this->get_logger(), "Already sitting!");
          }


	} else if(msg.primitive_cmd == 4 && is_standing) { // Walk forward command. Must be standing to walk.

	  double rampSteps = 600.0;
	  double stepLength = 3.0; // Inches
	  double stepHeight = 1.0; // Inches
	  double slideDepth = 0.2; // Inches
	  double walkDuration = 1200; // Milliseconds, not including 500 ms delay between steps
	  int caseNumber = 1;

	  for(int j = 1; j <= rampSteps; j++) { // Take four steps

	    auto steppingLegMessage = control_interfaces::msg::Coords(); // Coordinates to use for stepping
	    auto initSlideLegMessage = control_interfaces::msg::Coords(); // Slide if you haven't stepped yet
	    auto postSlideLegMessage = control_interfaces::msg::Coords(); // Slide if step complete

	    // Init Slide (linear horizontal, quadratic depth):
	    initSlideLegMessage.y = -j / (rampSteps/2) * (stepLength);
	    initSlideLegMessage.x = standHeight + slideDepth * (4/std::pow(stepLength,2)) * (initSlideLegMessage.y) * ((initSlideLegMessage.y) + stepLength);

	    // Post Slide (linear horizontal, quadratic depth):
	    postSlideLegMessage.y = (stepLength) - (stepLength * (j - (rampSteps/2)) / (rampSteps/2));
	    postSlideLegMessage.x = standHeight + slideDepth * (4/std::pow(stepLength,2)) * (postSlideLegMessage.y) * ((postSlideLegMessage.y) - stepLength);


	    switch(caseNumber) { // Switch on which leg is stepping, then publish to each leg
	      case 1 : // Leg one, four step

		// Compute leg one ramp. Linearly move forward then parabolically rise and fall.
		steppingLegMessage.y = j/(rampSteps/2) * stepLength;
		steppingLegMessage.x = standHeight - (-(4*stepHeight/std::pow(stepLength,2)) * (steppingLegMessage.y) * ((steppingLegMessage.y) - stepLength));

		// Resolve conflict with negatives
		initSlideLegMessage.y = -1 * initSlideLegMessage.y;
		steppingLegMessage.y = -1 * steppingLegMessage.y;

                legPubTwo_->publish(initSlideLegMessage);
                legPubFou_->publish(steppingLegMessage);

                // Flip y axes for odd legs
		steppingLegMessage.y = steppingLegMessage.y * -1;
		initSlideLegMessage.y = initSlideLegMessage.y * -1;
		legPubOne_->publish(steppingLegMessage);
		legPubThr_->publish(initSlideLegMessage);

		if(j >= (rampSteps/2)) {
		  caseNumber++;
		  std::this_thread::sleep_for(std::chrono::milliseconds{500}); // Delay slightly in between steps to give time to stabilize
		}

	      break;
	      case 2 : // Leg two, three step

		// Compute leg four ramp. Linearly move forward then parabolically rise and fall.
		steppingLegMessage.y = - (stepLength) + (j-rampSteps/2)/(rampSteps/2) * stepLength;
		steppingLegMessage.x = standHeight - ((-4 * stepHeight/std::pow(stepLength,2)) * (steppingLegMessage.y) * (steppingLegMessage.y + stepLength));
		
		postSlideLegMessage.y = -1 * postSlideLegMessage.y;
		steppingLegMessage.y = -1 * steppingLegMessage.y;

                legPubTwo_->publish(steppingLegMessage);
                legPubFou_->publish(postSlideLegMessage);

		postSlideLegMessage.y = postSlideLegMessage.y * -1;
		steppingLegMessage.y = steppingLegMessage.y * -1;
                legPubOne_->publish(postSlideLegMessage);
                legPubThr_->publish(steppingLegMessage);

		if(j >= (rampSteps)) {
		  caseNumber++;
		}

	      break;
	      case 3 :
	      break;
	    }
	    std::this_thread::sleep_for(std::chrono::milliseconds{static_cast<long>(walkDuration/rampSteps)});
	  }
	} else if(msg.primitive_cmd == 5 && is_standing) { // Twist!
	  // Don't modify this one with IMU input...
	
	  double rampSteps = 1000.0;
	  double twistTime = 6000; // Time to stand in milliseconds
	  double twistDiff = 1.0; // inches
	  
	  auto twistMessageLeft = control_interfaces::msg::Coords();
	  auto twistMessageRight = control_interfaces::msg::Coords();
	  
	  twistMessageLeft.y = 0.0;
          twistMessageRight.y = 0.0;
	  
	  for(int i = 0; i <= rampSteps; i++) {
	    if(rampSteps < (rampSteps/4)) { // Twist one way
              
              twistMessageLeft.x = standHeight - (i * twistDiff/2) / (rampSteps/4);
              twistMessageRight.x = standHeight + (i * twistDiff/2) / (rampSteps/4);
              
	    } else if(rampSteps < (rampSteps/2)) { // Then to straight
	      
	      twistMessageLeft.x = (standHeight - twistDiff/2) + (twistDiff/2) * (i - rampSteps/4) / (rampSteps/4);
	      twistMessageRight.x = (standHeight + twistDiff/2) - (twistDiff/2) * (i - rampSteps/4) / (rampSteps/4);
	    
	    } else if(rampSteps < (3*rampSteps/4) { // Twist the other way
	    
	      twistMessageLeft.x = standHeight + (i - rampSteps/2) * (twistDiff/2) / (rampSteps/4);
              twistMessageRight.x = standHeight - (i - rampSteps/2) * (twistDiff/2) / (rampSteps/4);
	    
	    } else { then back to straight
	    
	      twistMessageLeft.x = (standHeight + twistDiff/2) - (twistDiff/2) * (i - 3*rampSteps/4) / (rampSteps/4);
	      twistMessageRight.x = (standHeight - twistDiff/2) + (twistDiff/2) * (i - 3*rampSteps/4) / (rampSteps/4);
	    
	    }
	    
	    legPubFou_->publish(twistMessageLeft);
            legPubTwo_->publish(twistMessageLeft);

            legPubThr_->publish(twistMessageRight);
            legPubOne_->publish(twistMessageRight);
	  
	    std::this_thread::sleep_for(std::chrono::milliseconds{static_cast<long>(twistTime/rampSteps)});
	  }

	} else if(msg.primitive_cmd == 6 && is_standing) { // Pushup thing!
          // Don't modify this one with IMU input
	}
      }
      // All the pointers because ROS
      rclcpp::Subscription<control_interfaces::msg::Primitiveinterface>::SharedPtr primitivesubscription_;
      rclcpp::Publisher<control_interfaces::msg::Coords>::SharedPtr legPubOne_;
      rclcpp::Publisher<control_interfaces::msg::Coords>::SharedPtr legPubTwo_;
      rclcpp::Publisher<control_interfaces::msg::Coords>::SharedPtr legPubThr_;
      rclcpp::Publisher<control_interfaces::msg::Coords>::SharedPtr legPubFou_;
    };


  // Subscribes to imu_topic to get raw imu data
  class IMUSubscriber : public rclcpp::Node
  {
    public:
      IMUSubscriber()
      : Node("imu_subscriber")
      {
        imusubscription_ = this->create_subscription<control_interfaces::msg::Imuinterface>(
        "raw_imu_topic", 10, std::bind(&IMUSubscriber::topic_callback, this, _1));
      }

    private:
      void topic_callback(const control_interfaces::msg::Imuinterface & imu_raw_data) const
      {
        RCLCPP_INFO(this->get_logger(), "x acceleration: '%f'", imu_raw_data.accel_x);
        RCLCPP_INFO(this->get_logger(), "y acceleration: '%f'", imu_raw_data.accel_y);
        RCLCPP_INFO(this->get_logger(), "z acceleration: '%f'", imu_raw_data.accel_z);
        
        RCLCPP_INFO(this->get_logger(), "x gyro: '%f'", imu_raw_data.gyro_x);
        RCLCPP_INFO(this->get_logger(), "y gyro: '%f'", imu_raw_data.gyro_y);
        RCLCPP_INFO(this->get_logger(), "z gyro: '%f'", imu_raw_data.gyro_z);

        RCLCPP_INFO(this->get_logger(), "Roll: '%f'", imu_raw_data.roll);
        RCLCPP_INFO(this->get_logger(), "Pitch: '%f'", imu_raw_data.pitch);

        RCLCPP_INFO(this->get_logger(), "Time Increment: '%f'", imu_raw_data.time_inc);
      }
      rclcpp::Subscription<control_interfaces::msg::Imuinterface>::SharedPtr imusubscription_;
    };


  // Publish filtered imu data to filtered_imu_topic so python script can send to odrives
  class FilteredIMUPublisher : public rclcpp::Node
  {
    public:

      FilteredIMUPublisher()
      : Node("FilteredIMU_publisher"), count_(0)
      {
        publisher_ = this->create_publisher<control_interfaces::msg::Filteredimuinterface>("filtered_imu_topic", 10);
        timer_ = this->create_wall_timer(
        500ms, std::bind(&FilteredIMUPublisher::timer_callback, this));
      }

    private:
      // float filtered_imu_data;

      void timer_callback()
      {
        auto filtered_imu_message = control_interfaces::msg::Filteredimuinterface();
        filtered_imu_message.roll = roll;
        filtered_imu_message.pitch = pitch;
        RCLCPP_INFO(this->get_logger(), "Publishing roll data: '%f'", filtered_imu_message.roll);
        RCLCPP_INFO(this->get_logger(), "Publishing pitch data: '%f'", filtered_imu_message.pitch);
        publisher_->publish(filtered_imu_message);
      }
      rclcpp::TimerBase::SharedPtr timer_;
      rclcpp::Publisher<control_interfaces::msg::Filteredimuinterface>::SharedPtr publisher_;
      size_t count_;
  };



int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  rclcpp::Node::SharedPtr primitiveSubNode = std::make_shared<PrimitivesRunner>();
  rclcpp::Node::SharedPtr imuSubNode = std::make_shared<IMUSubscriber>();

  rclcpp::executors::StaticSingleThreadedExecutor executor;

  // Simultaneously spin the primitives node and the IMU node
  executor.add_node(primitiveSubNode);
  executor.add_node(imuSubNode);
  executor.spin();

  if (argc != 3) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: IMU_node_cli");
      return 1;
  }

////// Anything after spin needs to be moved into a spun node or it will not be run //////
  // In general, I think the IMU sub node needs to be reworked to include all of the service filtering so it can be applied directly.
  // The IMU is read by a script also in this package, it would be best if that script were broken out into its own Python package.
  // I also strongly suspect that most/all of the code past this point and in the services is copied directly from the ROS examples and may not work in practice. Make sure you know what you wrote actually does. Don't trust ChatGPT to code a robot.
  
  
  // Create a client for the IMU filtering service
  std::shared_ptr<rclcpp::Node> imu_node = rclcpp::Node::make_shared("IMU_node_cli");
  rclcpp::Client<control_interfaces::srv::Imufilteredinterface>::SharedPtr imu_client =
    imu_node->create_client<control_interfaces::srv::Imufilteredinterface>("request_imu_filtering");

  auto imu_request = std::make_shared<control_interfaces::srv::Imufilteredinterface::Request>();
  imu_request->accel_x = atoll(argv[1]);
  imu_request->accel_y = atoll(argv[2]);
  imu_request->accel_z = atoll(argv[3]);
  imu_request->gyro_x = atoll(argv[4]);
  imu_request->gyro_y = atoll(argv[5]);
  imu_request->gyro_z = atoll(argv[6]);
  imu_request->time_inc = atoll(argv[7]);

  while (!imu_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the IMU filtering service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "IMU filtering service not available, waiting again...");
  }

  auto imu_result = imu_client->async_send_request(imu_request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(imu_node, imu_result) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    float filtered_imu_roll = imu_result.get()->roll;
    float filtered_imu_pitch = imu_result.get()->pitch;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Roll: '%f'", filtered_imu_roll);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Pitch: '%f'", filtered_imu_pitch);
  
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call IMU filtering service");
  }

  // Create a client for the motor position PID controller service
  std::shared_ptr<rclcpp::Node> motor_pid_node = rclcpp::Node::make_shared("IMU_node_cli");
  rclcpp::Client<control_interfaces::srv::Pidinterface>::SharedPtr motor_pid_client =
    motor_pid_node->create_client<control_interfaces::srv::Pidinterface>("request_motor_pid_control");

  auto motor_pid_request = std::make_shared<control_interfaces::srv::Pidinterface::Request>();
  motor_pid_request->des_val = atoll(argv[1]);
  motor_pid_request->curr_val = atoll(argv[2]);

  while (!motor_pid_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the motor PID control service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Motor PID control service not available, waiting again...");
  }

  auto motor_pid_result = motor_pid_client->async_send_request(motor_pid_request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(motor_pid_node, motor_pid_result) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Motor PID controller: %f", motor_pid_result.get()->output_val);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call motor PID control service");
  }

  // Create a client for the torque PID controller service
  std::shared_ptr<rclcpp::Node> torque_pid_node = rclcpp::Node::make_shared("IMU_node_cli");
  rclcpp::Client<control_interfaces::srv::Pidinterface>::SharedPtr torque_pid_client =
    torque_pid_node->create_client<control_interfaces::srv::Pidinterface>("request_torque_pid_control");

  auto torque_pid_request = std::make_shared<control_interfaces::srv::Pidinterface::Request>();
  torque_pid_request->des_val = atoll(argv[1]);
  torque_pid_request->curr_val = atoll(argv[2]);

  while (!torque_pid_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the torque PID control service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Torque PID control service not available, waiting again...");
  }

  auto torque_pid_result = torque_pid_client->async_send_request(torque_pid_request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(torque_pid_node, torque_pid_result) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Torque PID controller: %f", torque_pid_result.get()->output_val);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call torque PID control service");
  }

  // Create a client for the neutral command service
  std::shared_ptr<rclcpp::Node> neutral_node = rclcpp::Node::make_shared("IMU_node_cli");
  rclcpp::Client<control_interfaces::srv::Neutralcommandinterface>::SharedPtr neutral_client =
    neutral_node->create_client<control_interfaces::srv::Neutralcommandinterface>("request_neutral_cmd");

  auto neutral_request = std::make_shared<control_interfaces::srv::Neutralcommandinterface::Request>();
  neutral_request->neutral_command_request = atoll(argv[1]);

  while (!neutral_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the neutral service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Neutral service not available, waiting again...");
  }

  auto neutral_result = neutral_client->async_send_request(neutral_request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(neutral_node, neutral_result) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Neutral Command: %f", neutral_result.get()->neutral_command_response);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call neutral service");
  }

  rclcpp::shutdown();
  return 0;
}
