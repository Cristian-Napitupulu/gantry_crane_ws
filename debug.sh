#!/bin/bash

# Open first terminal and run commands
gnome-terminal --tab --title="Trolley Position" -- bash -c "source install/local_setup.bash; ros2 topic echo /trolley_position; exec bash"

# Open second terminal and run commands
gnome-terminal --tab --title="Cable Length" -- bash -c "source install/local_setup.bash; ros2 topic echo /cable_length; exec bash"

# Open third terminal and run commands
gnome-terminal --tab --title="Sway Angle" -- bash -c "source install/local_setup.bash; ros2 topic echo /sway_angle; exec bash"

# Open fourth terminal and run commands
gnome-terminal --tab --title="Trolley Motor Voltage" -- bash -c "source install/local_setup.bash; ros2 topic echo /trolley_motor_voltage; exec bash"

# Open fifth terminal and run commands
gnome-terminal --tab --title="Hoist Motor Voltage" -- bash -c "source install/local_setup.bash; ros2 topic echo /hoist_motor_voltage; exec bash"