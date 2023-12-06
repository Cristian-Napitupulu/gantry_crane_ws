#!/bin/bash

# Open first terminal and run commands
gnome-terminal --tab --title="Terminal 1" -- bash -c "source install/local_setup.bash; source gantry_crane_venv/bin/activate; ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0; exec bash"

# Open second terminal and run commands
gnome-terminal --tab --title="Terminal 2" -- bash -c "ros2 topic echo /trolley_position; exec bash"

# Open third terminal and run commands
gnome-terminal --tab --title="Terminal 3" -- bash -c "source install/local_setup.bash; ros2 launch realsense_camera_yolo realsense_camera_yolo_launch.py; exec bash"
