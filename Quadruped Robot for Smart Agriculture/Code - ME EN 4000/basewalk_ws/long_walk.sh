#!/bin/bash

# Take 10 steps consecutively, wait 1 second in between.
source install/setup.bash

for i in seq $(seq 1 10)
do
	ros2 topic pub /inputCommands std_msgs/msg/String "{data: forward}" -1
	sleep 1
done
