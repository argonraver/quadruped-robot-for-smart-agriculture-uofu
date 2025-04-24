#!/bin/bash

# Sets the legs to neutral mode so they stop drawing power. They need to be recalibrated to turn them back on.
source install/setup.bash

ros2 service call /odrive_axis0/request_axis_state odrive_can/srv/AxisState "{axis_requested_state: 1}" & ros2 service call /odrive_axis1/request_axis_state odrive_can/srv/AxisState "{axis_requested_state: 1}" & ros2 service call /odrive_axis2/request_axis_state odrive_can/srv/AxisState "{axis_requested_state: 1}" & ros2 service call /odrive_axis3/request_axis_state odrive_can/srv/AxisState "{axis_requested_state: 1}" & ros2 service call /odrive_axis4/request_axis_state odrive_can/srv/AxisState "{axis_requested_state: 1}" & ros2 service call /odrive_axis5/request_axis_state odrive_can/srv/AxisState "{axis_requested_state: 1}" & ros2 service call /odrive_axis6/request_axis_state odrive_can/srv/AxisState "{axis_requested_state: 1}" & ros2 service call /odrive_axis7/request_axis_state odrive_can/srv/AxisState "{axis_requested_state: 1}"
