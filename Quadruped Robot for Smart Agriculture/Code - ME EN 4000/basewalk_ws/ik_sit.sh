#!/bin/bash

Directly commands all legs to the sitting position.
source install/setup.bash

ros2 topic pub /leg_one_commands control_interfaces/msg/Coords "{x: 3.9, y: -2.0}" -1 & ros2 topic pub /leg_two_commands control_interfaces/msg/Coords "{x: 3.9, y: 2.0}" -1 & ros2 topic pub /leg_three_commands control_interfaces/msg/Coords "{x: 3.9, y: -2.0}" -1 & ros2 topic pub /leg_four_commands control_interfaces/msg/Coords "{x: 3.9, y: 2.0}" -1
