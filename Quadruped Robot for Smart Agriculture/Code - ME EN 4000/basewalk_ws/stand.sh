#!/bin/bash

# Sends the stand message to the ssh node
source install/setup.bash

ros2 topic pub /inputCommands std_msgs/msg/String "{data: stand}" -1
