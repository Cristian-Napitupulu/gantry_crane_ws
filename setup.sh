#!/bin/bash

# Open first terminal and run commands
gnome-terminal --tab --title="Command Translator" -- bash -c "source install/local_setup.bash; source gantry_crane_venv/bin/activate; ros2 run controller_command_translator controller_command_translator; exec bash"

# Open second terminal and run commands
gnome-terminal --tab --title="Realsense Launcher" -- bash -c "source install/local_setup.bash; source gantry_crane_venv/bin/activate; ros2 launch realsense_camera_yolo realsense_camera_yolo_launch.py; exec bash"

# Open third terminal and run commands
gnome-terminal --tab --title="Micro ROS Connector" -- bash -c "source install/local_setup.bash; source gantry_crane_venv/bin/activate; ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0; exec bash"
