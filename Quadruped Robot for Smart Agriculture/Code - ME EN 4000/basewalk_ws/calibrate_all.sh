#!/bin/bash

# Calibrates each leg, switches to closed loop position control mode, then goes to a sitting position.
source install/setup.bash

# One joint from each leg at a time calibrates
ros2 service call /odrive_axis0/request_axis_state odrive_can/srv/AxisState "{axis_requested_state: 3}" & ros2 service call /odrive_axis2/request_axis_state odrive_can/srv/AxisState "{axis_requested_state: 3}" & ros2 service call /odrive_axis4/request_axis_state odrive_can/srv/AxisState "{axis_requested_state: 3}" & ros2 service call /odrive_axis6/request_axis_state odrive_can/srv/AxisState "{axis_requested_state: 3}"

ros2 service call /odrive_axis1/request_axis_state odrive_can/srv/AxisState "{axis_requested_state: 3}" & ros2 service call /odrive_axis3/request_axis_state odrive_can/srv/AxisState "{axis_requested_state: 3}" & ros2 service call /odrive_axis5/request_axis_state odrive_can/srv/AxisState "{axis_requested_state: 3}" & ros2 service call /odrive_axis7/request_axis_state odrive_can/srv/AxisState "{axis_requested_state: 3}"

# Set all joints to closed loop control
ros2 service call /odrive_axis0/request_axis_state odrive_can/srv/AxisState "{axis_requested_state: 8}" & ros2 service call /odrive_axis1/request_axis_state odrive_can/srv/AxisState "{axis_requested_state: 8}" & ros2 service call /odrive_axis2/request_axis_state odrive_can/srv/AxisState "{axis_requested_state: 8}" & ros2 service call /odrive_axis3/request_axis_state odrive_can/srv/AxisState "{axis_requested_state: 8}" & ros2 service call /odrive_axis4/request_axis_state odrive_can/srv/AxisState "{axis_requested_state: 8}" & ros2 service call /odrive_axis5/request_axis_state odrive_can/srv/AxisState "{axis_requested_state: 8}" & ros2 service call /odrive_axis6/request_axis_state odrive_can/srv/AxisState "{axis_requested_state: 8}" & ros2 service call /odrive_axis7/request_axis_state odrive_can/srv/AxisState "{axis_requested_state: 8}"

# Commands the sitting position. Commanded multiple times because messages occasionally seem to get missed.
./ik_sit.sh
./ik_sit.sh
./ik_sit.sh
