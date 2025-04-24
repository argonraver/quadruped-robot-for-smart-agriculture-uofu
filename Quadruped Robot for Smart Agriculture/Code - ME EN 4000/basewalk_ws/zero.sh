#!/bin/bash

# Set two legs to zero angle
source install/setup.bash

for i in $(seq 1 3)
do
	ros2 topic pub /leg_one control_interfaces/msg/JointAngles "{thetaone: 0.0, thetatwo: 0.0}" -1 & ros2 topic pub /leg_two control_interfaces/msg/JointAngles "{thetaone: 0.0, thetatwo: 0.0}" -1 & ros2 topic pub /leg_three control_interfaces/msg/JointAngles "{thetaone: 0.0, thetatwo: 0.0}" -1 & ros2 topic pub /leg_four control_interfaces/msg/JointAngles "{thetaone: 0.0, thetatwo: 0.0}" -1
done
