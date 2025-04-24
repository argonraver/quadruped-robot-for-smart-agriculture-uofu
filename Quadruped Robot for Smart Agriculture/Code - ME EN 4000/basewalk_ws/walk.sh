#!/bin/bash

# Sends the walk forward message to the ssh node
source install/setup.bash

ros2 topic pub /inputCommands std_msgs/msg/String "{data: forward}" -1
