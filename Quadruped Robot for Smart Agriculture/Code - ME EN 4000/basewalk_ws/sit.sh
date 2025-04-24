#!/bin/bash

# sends the sit message to the ssh node
source install/setup.bash

ros2 topic pub /inputCommands std_msgs/msg/String "{data: sit}" -1
