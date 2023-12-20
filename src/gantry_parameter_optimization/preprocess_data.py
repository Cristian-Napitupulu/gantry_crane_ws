import numpy as np

import pandas as pd
import os

script_directory = os.path.dirname(os.path.abspath(__file__))

# Load data
data = pd.read_excel(
    script_directory + "/data/sliding_mode_controller_data.xlsx", sheet_name="data"
)

# Extract timestamp
timestamp = data["timestamp"].values

# Create new timestamp
increment = 0.005
new_timestamp = np.arange(0, timestamp.max() + increment, increment)

# Extract data
trolley_motor_voltage = data["trolley_motor_voltage"].values
hoist_motor_voltage = data["hoist_motor_voltage"].values
x_triple_dot = data["trolley_position_third_derivative"].values
l_triple_dot = data["cable_length_third_derivative"].values
x_double_dot = data["trolley_position_second_derivative"].values
l_double_dot = data["cable_length_second_derivative"].values
x_dot = data["trolley_position_first_derivative"].values
l_dot = data["cable_length_first_derivative"].values
x = data["trolley_position"].values
l = data["cable_length"].values
theta_double_dot = data["sway_angle_second_derivative"].values
theta_dot = data["sway_angle_first_derivative"].values
theta = data["sway_angle"].values

# Interpolate data
new_trolley_motor_voltage = np.interp(new_timestamp, timestamp, trolley_motor_voltage)
new_hoist_motor_voltage = np.interp(new_timestamp, timestamp, hoist_motor_voltage)
new_x_triple_dot = np.interp(new_timestamp, timestamp, x_triple_dot)
new_l_triple_dot = np.interp(new_timestamp, timestamp, l_triple_dot)
new_x_double_dot = np.interp(new_timestamp, timestamp, x_double_dot)
new_l_double_dot = np.interp(new_timestamp, timestamp, l_double_dot)
new_x_dot = np.interp(new_timestamp, timestamp, x_dot)
new_l_dot = np.interp(new_timestamp, timestamp, l_dot)
new_x = np.interp(new_timestamp, timestamp, x)
new_l = np.interp(new_timestamp, timestamp, l)
new_theta_double_dot = np.interp(new_timestamp, timestamp, theta_double_dot)
new_theta_dot = np.interp(new_timestamp, timestamp, theta_dot)
new_theta = np.interp(new_timestamp, timestamp, theta)

# print size of new data
print("new_trolley_motor_voltage size: ", new_trolley_motor_voltage.size)
print("new_hoist_motor_voltage size: ", new_hoist_motor_voltage.size)
print("new_timestamp size: ", new_timestamp.size)
print("new_x_triple_dot size: ", new_x_triple_dot.size)
print("new_l_triple_dot size: ", new_l_triple_dot.size)
print("new_x_double_dot size: ", new_x_double_dot.size)
print("new_l_double_dot size: ", new_l_double_dot.size)
print("new_x_dot size: ", new_x_dot.size)
print("new_l_dot size: ", new_l_dot.size)
print("new_x size: ", new_x.size)
print("new_l size: ", new_l.size)
print("new_theta_double_dot size: ", new_theta_double_dot.size)
print("new_theta_dot size: ", new_theta_dot.size)
print("new_theta size: ", new_theta.size)



# Create new data frame
new_data = pd.DataFrame(
    {
        "timestamp": new_timestamp,
        "trolley_motor_voltage": new_trolley_motor_voltage,
        "hoist_motor_voltage": new_hoist_motor_voltage,
        "trolley_position_third_derivative": new_x_triple_dot,
        "cable_length_third_derivative": new_l_triple_dot,
        "trolley_position_second_derivative": new_x_double_dot,
        "cable_length_second_derivative": new_l_double_dot,
        "trolley_position_first_derivative": new_x_dot,
        "cable_length_first_derivative": new_l_dot,
        "trolley_position": new_x,
        "cable_length": new_l,
        "sway_angle_second_derivative": new_theta_double_dot,
        "sway_angle_first_derivative": new_theta_dot,
        "sway_angle": new_theta,
    }
)

# Save new data frame
with pd.ExcelWriter(
    script_directory + "/data/preprocess_sliding_mode_controller_data.xlsx"
) as writer:
    new_data.to_excel(writer, sheet_name="preprocess_data", index=False)