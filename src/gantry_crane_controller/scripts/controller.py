#!/usr/bin/env python3

from gantry_crane_lib.logger import Logger

from sliding_mode_controller import main

import os

import numpy as np

import rclpy

LOG_FOLDER_PATH = '/home/icodes/Documents/gantry_crane_ws/src/gantry_crane_controller/log/'

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

DESIRED_TROLLEY_POSITION = 1.25
DESIRED_CABLE_LENGTH = 0.4

if __name__ == '__main__':
    gantry_crane_logger = Logger(LOG_FOLDER_PATH)