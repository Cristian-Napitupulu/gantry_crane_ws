#!/usr/bin/env python3

from sliding_mode_controller import main


import rclpy

LOG_FOLDER_PATH = '/home/roboy/roboy/logs/gantry_crane_controller/'

VARIABLE_TO_LOG = {
    "timestamp",
    "control_signal_trolley",
    "control_signal_hoist",
    "pwm_trolley_motor",
    "pwm_hoist_motor",
    "trolley_motor_voltage",
    "hoist_motor_voltage",
    "trolley_position",
    "cable_length",
    "sway_angle",
    "trolley_position_first_derivative",
    "cable_length_first_derivative",
    "sway_angle_first_derivative",
    "trolley_position_second_derivative",
    "cable_length_second_derivative",
    "sway_angle_second_derivative",
    "trolley_position_third_derivative",
    "cable_length_third_derivative",
    "sway_angle_third_derivative",
}
class Logger():
    def __init__(self, parent_folder_path):
        self.parent_folder_path = parent_folder_path


def main():
    rclpy.init()
    gantry_crane = GantryCraneConnector()
    gantry_crane_logger = Logger(gantry_crane.parent_folder_path)


if __name__ == '__main__':
    main()