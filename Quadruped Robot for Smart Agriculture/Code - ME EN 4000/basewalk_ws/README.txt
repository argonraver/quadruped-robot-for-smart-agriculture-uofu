Execute the scripts in here with
./script_name.sh
This will send the robot various commands. setup_all_legs.sh will enable command of the robot, calibrate_all.sh calibrates and converts to closed loop control. When in this state, the monitor scripts will check in on their respective joints, and the other commands will be interpreted and executed. 

The src directory contains all of the code these scripts are using. Apply changes by running "colcon build" in this directory.

NOTE: This Drive download is one version behind what is on the robot. The robot has had the IMU broken out into its own script and filtering moved directly into the main control node on its own thread. Further, the IMU script has been added to the full launch file. However, the IMU does *not* get applied to the primitives.