#!/bin/bash

# Example of how to use the inverse kinematics message. Can be helpful to test individual legs.
source install/setup.bash

ros2 topic pub /leg_one_commands control_interfaces/msg/Coords "{x: 1.0, y: 1.0}" -1

