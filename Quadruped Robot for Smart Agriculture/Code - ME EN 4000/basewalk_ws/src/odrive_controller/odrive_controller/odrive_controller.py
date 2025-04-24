import rclpy
from rclpy.node import Node

from control_interfaces.msg import JointAngles

from odrive_can.msg import ControlMessage
from odrive_can.srv import AxisState

# This node acts as a translation layer between the C++ leg node and the ODrive CAN nodes, which could not be published to via C++ for unknown reasons.
class OdriveController(Node):
	def __init__(self):
		super().__init__('odrive_controller')
		# Establish launch file to control each leg and axes
		self.declare_parameter('upper_leg_axis', 'axis0')
		self.declare_parameter('lower_leg_axis', 'axis1')
		self.declare_parameter('leg_number', 'leg_one')
		
		# Set topics to publish to for each leg axis
		upperLegAxisTopic = "odrive_" + self.get_parameter('upper_leg_axis').get_parameter_value().string_value + "/control_message"
		lowerLegAxisTopic = "odrive_" + self.get_parameter('lower_leg_axis').get_parameter_value().string_value + "/control_message"
		
		self.publisher_upper_ = self.create_publisher(ControlMessage, upperLegAxisTopic, 10)
		self.publisher_lower_ = self.create_publisher(ControlMessage, lowerLegAxisTopic, 10)
		
		# Subscribe to releated leg ik node
		self.subscription = self.create_subscription(JointAngles, self.get_parameter('leg_number').get_parameter_value().string_value, self.listener_callback, 10)
		self.subscription # Prevent unused variable warnings
		
		### Ensure axis state is set correctly. Should run ONCE ###
		# This part does not seem to do anything. Worth investigating, as it would allow for recalibration on a whim.
		self.cli = self.create_client(AxisState, 'request_axis_state')
		self.req = AxisState.Request()

		# Log success
		self.get_logger().info("Initiated odrive " + self.get_parameter('leg_number').get_parameter_value().string_value)
		
		
	def listener_callback(self, msg):
	
		# Generate ODrive formatted messages from recieved IK values
		lowerMessage = ControlMessage()
		upperMessage = ControlMessage()

		lowerMessage.control_mode = 3 # Position control mode
		lowerMessage.input_mode = 3 # Filtered position control. There is a (discovered after primitives implemented) ramp mode that may be worth investigating.
		lowerMessage.input_pos = msg.thetaone
		lowerMessage.input_vel = 0.
		lowerMessage.input_torque = 0.

		upperMessage.control_mode = 3
		upperMessage.input_mode = 3
		upperMessage.input_pos = msg.thetatwo
		upperMessage.input_vel = 0.
		upperMessage.input_torque = 0.
		
		# Got busy with the ramps. Uncomment for troubleshooting.
		#self.get_logger().info("I published angles %.2f and " % lowerMessage.input_pos)
		#self.get_logger().info("%.2f" % msg.thetatwo)
		
		# Publish
		self.publisher_upper_.publish(upperMessage)
		self.publisher_lower_.publish(lowerMessage)
		
		
	# Set correct control mode on launch?
	def send_request(self, controlMode):
		self.req.axis_requested_state = controlMode
		return self.cli.call_async(self.req)
		
def main(args=None):
	rclpy.init(args=args)
	
	odrive_controller = OdriveController()
	
	setAxisVar = odrive_controller.send_request(8) # Set odrives to correct mode
	rclpy.spin(odrive_controller)
	
	odrive_controller().destroy_node()
	rclpy.shutdown()
	
if __name__ == '__main__':
	main()
