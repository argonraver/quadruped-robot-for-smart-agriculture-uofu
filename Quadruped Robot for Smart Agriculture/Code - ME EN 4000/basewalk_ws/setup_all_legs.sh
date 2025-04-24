#!/bin/bash

# Runs the launch file that instantiates all four legs and their nodes, plus the controller nodes.
source install/setup.bash
ros2 launch control_interfaces full_launch.yaml

