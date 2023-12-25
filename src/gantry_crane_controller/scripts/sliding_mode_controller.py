#!/usr/bin/env python3

import rclpy
from gantry_crane_lib.ros_connector import GantryCraneConnector
from gantry_crane_lib.ros_connector import GantryControlModes

from gantry_crane_lib.logger import Logger

import numpy as np
import json
import time

import pandas as pd
import matplotlib.pyplot as plt

import signal

MODEL_NAME = "non_linear_model"
CONTROLLER_NAME = "sliding_mode_controller"

GANTRY_CRANE_MODEL_PARAMETERS_JSON_PATH = "/home/icodes/Documents/gantry_crane_ws/src/gantry_crane_controller/parameters/gantry_crane_parameters.json"

SLIDING_MODE_CONTROLLER_PARAMETERS_JSON_PATH = "/home/icodes/Documents/gantry_crane_ws/src/gantry_crane_controller/parameters/sliding_mode_controller_parameters.json"

LOG_FOLDER_PATH = (
    "/home/icodes/Documents/gantry_crane_ws/src/gantry_crane_controller/data/"
)

DATA_SAVE_PATH = (
    "/home/icodes/Documents/gantry_crane_ws/src/gantry_crane_controller/data/"
)


class GantryCraneModel:
    def __init__(self, model_name, parameter_path):
        self.name = model_name

        self.get_parameter(parameter_path)
        self.initialize_variables()

    def get_name(self):
        return self.name

    def attach_connector(self, gantry_crane_connector):
        # Get connector to get variables
        self.gantry_crane_object = gantry_crane_connector

    def initialize_variables(self):
        # Initialize variables
        self.matrix_A = np.matrix([[0.0, 0.0], [0.0, 0.0]])
        self.matrix_B = np.matrix([[0.0, 0.0], [0.0, 0.0]])
        self.matrix_C = np.matrix([[0.0, 0.0], [0.0, 0.0]])
        self.matrix_D = np.matrix([[0.0], [0.0]])
        self.matrix_E = np.matrix([[0.0], [0.0]])
        self.matrix_F = np.matrix([[0.0], [0.0]])

    def get_parameter(self, parameter_json_path):
        with open(parameter_json_path, "r") as file:
            parameters = json.load(file)

        self.mt = parameters["gantry_crane_system_model"]["parameters"]["trolley_mass"][
            "value"
        ]
        self.mc = parameters["gantry_crane_system_model"]["parameters"][
            "container_mass"
        ]["value"]
        self.bt = parameters["gantry_crane_system_model"]["parameters"][
            "trolley_damping_coefficient"
        ]["value"]
        self.br = parameters["gantry_crane_system_model"]["parameters"][
            "cable_damping_coefficient"
        ]["value"]
        self.g = parameters["gantry_crane_system_model"]["parameters"][
            "gravity_acceleration"
        ]["value"]
        self.L1 = parameters["gantry_crane_system_model"]["parameters"][
            "trolley_motor_inductance"
        ]["value"]
        self.R1 = parameters["gantry_crane_system_model"]["parameters"][
            "trolley_motor_resistance"
        ]["value"]
        self.J1 = parameters["gantry_crane_system_model"]["parameters"][
            "trolley_motor_rotator_inertia"
        ]["value"]
        self.b1 = parameters["gantry_crane_system_model"]["parameters"][
            "trolley_motor_damping_coefficient"
        ]["value"]
        self.rp1 = parameters["gantry_crane_system_model"]["parameters"][
            "trolley_motor_pulley_radius"
        ]["value"]
        self.Kt1 = parameters["gantry_crane_system_model"]["parameters"][
            "trolley_motor_torque_constant"
        ]["value"]
        self.Ke1 = parameters["gantry_crane_system_model"]["parameters"][
            "trolley_motor_back_emf_constant"
        ]["value"]
        self.L2 = parameters["gantry_crane_system_model"]["parameters"][
            "hoist_motor_inductance"
        ]["value"]
        self.R2 = parameters["gantry_crane_system_model"]["parameters"][
            "hoist_motor_resistance"
        ]["value"]
        self.J2 = parameters["gantry_crane_system_model"]["parameters"][
            "hoist_motor_rotator_inertia"
        ]["value"]
        self.b2 = parameters["gantry_crane_system_model"]["parameters"][
            "hoist_motor_damping_coefficient"
        ]["value"]
        self.rp2 = parameters["gantry_crane_system_model"]["parameters"][
            "hoist_motor_pulley_radius"
        ]["value"]
        self.Kt2 = parameters["gantry_crane_system_model"]["parameters"][
            "hoist_motor_torque_constant"
        ]["value"]
        self.Ke2 = parameters["gantry_crane_system_model"]["parameters"][
            "hoist_motor_back_emf_constant"
        ]["value"]

    def update_matrix_A(self):
        # Update matrix A
        self.matrix_A[0, 0] = self.L1 * self.rp1 * (
            self.mt
            + self.mc
            * np.sin(self.gantry_crane_object.variables_value["sway_angle"]) ** 2
        ) / self.Kt1 + self.J1 * self.L1 / (self.Kt1 * self.rp1)
        self.matrix_A[0, 1] = (
            -self.L1
            * self.mc
            * self.rp1
            * np.sin(self.gantry_crane_object.variables_value["sway_angle"])
            / self.Kt1
        )
        self.matrix_A[1, 0] = (
            -self.L2
            * self.mc
            * self.rp2
            * np.sin(self.gantry_crane_object.variables_value["sway_angle"])
            / self.Kt2
        )
        self.matrix_A[
            1, 1
        ] = self.L2 * self.mc * self.rp2 / self.Kt2 + self.J2 * self.L2 / (
            self.Kt2 * self.rp2
        )

    def update_matrix_B(self):
        # Update matrix B
        self.matrix_B[0, 0] = (
            +self.L1
            * self.mc
            * self.rp1
            * np.sin(2 * self.gantry_crane_object.variables_value["sway_angle"])
            * self.gantry_crane_object.variables_first_derivative["sway_angle"]
            / self.Kt1
            + self.R1
            * self.rp1
            * (
                self.mt
                + self.mc
                * np.sin(self.gantry_crane_object.variables_value["sway_angle"]) ** 2
            )
            / self.Kt1
            + self.L1 * self.bt * self.rp1 / self.Kt1
            + self.J1 * self.R1 / (self.Kt1 * self.rp1)
            + self.L1 * self.b1 / (self.Kt1 * self.rp1)
        )
        self.matrix_B[0, 1] = (
            -self.L1
            * self.mc
            * self.rp1
            * np.cos(self.gantry_crane_object.variables_value["sway_angle"])
            * self.gantry_crane_object.variables_first_derivative["sway_angle"]
            / self.Kt1
            - self.R1
            * self.mc
            * self.rp1
            * np.sin(self.gantry_crane_object.variables_value["sway_angle"])
            / self.Kt1
        )
        self.matrix_B[1, 0] = (
            -self.L2
            * self.mc
            * self.rp2
            * np.cos(self.gantry_crane_object.variables_value["sway_angle"])
            * self.gantry_crane_object.variables_first_derivative["sway_angle"]
            / self.Kt2
            - self.R2
            * self.mc
            * self.rp2
            * np.sin(self.gantry_crane_object.variables_value["sway_angle"])
            / self.Kt2
        )
        self.matrix_B[1, 1] = (
            +self.L2 * self.br * self.rp2 / self.Kt2
            + self.R2 * self.mc * self.rp2 / self.Kt2
            + self.J2 * self.R2 / (self.Kt2 * self.rp2)
            + self.L2 * self.b2 / (self.Kt2 * self.rp2)
        )

    def update_matrix_C(self):
        # Update matrix C
        self.matrix_C[0, 0] = (
            self.R1 * self.bt * self.rp1 / self.Kt1
            + self.Ke1 / self.rp1
            + self.R1 * self.b1 / (self.Kt1 * self.rp1)
        )
        self.matrix_C[1, 1] = (
            self.R2 * self.br * self.rp2 / self.Kt2
            + self.Ke2 / self.rp2
            + self.R2 * self.b2 / (self.Kt2 * self.rp2)
        )

    def update_matrix_D(self):
        # Update matrix D
        self.matrix_D[0, 0] = (
            2
            * self.L1
            * self.mc
            * self.rp1
            * self.gantry_crane_object.variables_value["cable_length"]
            * np.sin(self.gantry_crane_object.variables_value["sway_angle"])
            * self.gantry_crane_object.variables_first_derivative["sway_angle"]
            / self.Kt1
        )
        self.matrix_D[1, 0] = (
            -2
            * self.L2
            * self.mc
            * self.rp2
            * self.gantry_crane_object.variables_value["cable_length"]
            * self.gantry_crane_object.variables_first_derivative["sway_angle"]
            / self.Kt2
        )

    def update_matrix_E(self):
        # Update matrix E
        self.matrix_E[0, 0] = (
            self.L1
            * self.mc
            * self.rp1
            * self.gantry_crane_object.variables_value["cable_length"]
            * np.cos(self.gantry_crane_object.variables_value["sway_angle"])
            * self.gantry_crane_object.variables_first_derivative["sway_angle"] ** 2
            / self.Kt1
            + self.L1
            * self.g
            * self.mc
            * self.rp1
            * np.cos(2 * self.gantry_crane_object.variables_value["sway_angle"])
            / self.Kt1
            + self.L1
            * self.mc
            * self.rp1
            * np.sin(self.gantry_crane_object.variables_value["sway_angle"])
            * self.gantry_crane_object.variables_first_derivative["cable_length"]
            * self.gantry_crane_object.variables_first_derivative["sway_angle"]
            / self.Kt1
            + self.R1
            * self.mc
            * self.rp1
            * self.gantry_crane_object.variables_value["cable_length"]
            * np.sin(self.gantry_crane_object.variables_value["sway_angle"])
            * self.gantry_crane_object.variables_first_derivative["sway_angle"]
            / self.Kt1
        )
        self.matrix_E[1, 0] = (
            +self.L2
            * self.g
            * self.mc
            * self.rp2
            * np.sin(self.gantry_crane_object.variables_value["sway_angle"])
            / self.Kt2
            - self.L2
            * self.mc
            * self.rp2
            * self.gantry_crane_object.variables_first_derivative["cable_length"]
            * self.gantry_crane_object.variables_first_derivative["sway_angle"]
            / self.Kt2
            - self.R2
            * self.mc
            * self.rp2
            * self.gantry_crane_object.variables_value["cable_length"]
            * self.gantry_crane_object.variables_first_derivative["sway_angle"]
            / self.Kt2
        )

    def update_matrix_F(self):
        # Update matrix F
        self.matrix_F[0, 0] = (
            self.R1
            * self.g
            * self.mc
            * self.rp1
            * np.sin(self.gantry_crane_object.variables_value["sway_angle"])
            * np.cos(self.gantry_crane_object.variables_value["sway_angle"])
            / self.Kt1
        )
        self.matrix_F[1, 0] = (
            -self.R2
            * self.g
            * self.mc
            * self.rp2
            * np.cos(self.gantry_crane_object.variables_value["sway_angle"])
            / self.Kt2
        )

    def update_model(self):
        self.update_matrix_A()
        self.update_matrix_B()
        self.update_matrix_C()
        self.update_matrix_D()
        self.update_matrix_E()
        self.update_matrix_F()


class Controller:
    def __init__(self, controller_name, parameter_path):
        self.name = controller_name
        # Initialize controller parameters
        self.get_parameter(parameter_path)

        self.initialize_variables()

    def initialize_variables(self):
        self.control_signal_trolley = 0.0
        self.control_signal_hoist = 0.0
        self.pwm_trolley_motor = 0
        self.pwm_hoist_motor = 0

    def get_name(self):
        return self.name

    def attach_connector(self, gantry_crane_connector):
        # Get connector to get variables
        self.gantry_crane_object = gantry_crane_connector

    def attach_model(self, non_linear_motor_model):
        # Get model to get matrices for control input calculation
        self.model_gantry_crane = non_linear_motor_model

    def get_parameter(self, parameter_json_path):
        # Get controller parameters
        with open(parameter_json_path, "r") as file:
            parameter_file = json.load(file)
        """
        Edit this function to change the controller parameters as you want.
        For good practice, make sure you are not hardcoding the parameters.
        Use json file to store the parameters.
        So, the parameters can be changed easily without recompiling the code.
        """
        # Sliding mode controller parameters
        self.matrix_alpha = np.matrix(
            parameter_file["sliding_mode_controller"]["parameters"]["matrix_alpha"][
                "value"
            ]
        )
        self.matrix_beta = np.matrix(
            parameter_file["sliding_mode_controller"]["parameters"]["matrix_beta"][
                "value"
            ]
        )
        self.matrix_lambda = np.matrix(
            parameter_file["sliding_mode_controller"]["parameters"]["matrix_lambda"][
                "value"
            ]
        )
        self.matrix_k = np.matrix(
            parameter_file["sliding_mode_controller"]["parameters"]["matrix_k"]["value"]
        )
        self.matrix_gamma = np.matrix(
            parameter_file["sliding_mode_controller"]["parameters"]["matrix_gamma"][
                "value"
            ]
        )

    def get_control_signal(self, desired_trolley_position, desired_cable_length=None):
        """
        Edit this function to change the control input calculation method as you want.
        This function get the desired trolley position and cable length.
        And return the control input for trolley and hoist motor.
        """
        desired_cable_length_ = desired_cable_length
        if desired_cable_length is None:
            desired_cable_length_ = self.gantry_crane_object.variables_value[
                "cable_length"
            ]

        desiredStateVector = np.matrix(
            [[desired_trolley_position], [desired_cable_length_]]
        )
        # State vector
        stateVector = np.matrix(
            [
                [self.gantry_crane_object.variables_value["trolley_position"]],
                [self.gantry_crane_object.variables_value["cable_length"]],
            ]
        )
        print(
            "Desired state vector: {}, {}. State vector: {}, {}.".format(
                desiredStateVector[0, 0],
                desiredStateVector[1, 0],
                stateVector[0, 0],
                stateVector[1, 0],
            )
        )
        stateVectorFirstDerivative = np.matrix(
            [
                [
                    self.gantry_crane_object.variables_first_derivative[
                        "trolley_position"
                    ]
                ],
                [self.gantry_crane_object.variables_first_derivative["cable_length"]],
            ]
        )

        stateVectorSecondDerivative = np.matrix(
            [
                [
                    self.gantry_crane_object.variables_second_derivative[
                        "trolley_position"
                    ]
                ],
                [self.gantry_crane_object.variables_second_derivative["cable_length"]],
            ]
        )
        print(
            "State vector first derivative: {}, {}. State vector second derivative: {}, {}.".format(
                stateVectorFirstDerivative[0, 0],
                stateVectorFirstDerivative[1, 0],
                stateVectorSecondDerivative[0, 0],
                stateVectorSecondDerivative[1, 0],
            )
        )

        # Sliding surface
        slidingSurface = np.matrix([[0.0], [0.0]])
        slidingSurface = (
            np.matmul(self.matrix_alpha, stateVector - desiredStateVector)
            + np.matmul(self.matrix_beta, stateVectorFirstDerivative)
            + stateVectorSecondDerivative /100000
            + self.matrix_lambda
            * self.gantry_crane_object.variables_value["sway_angle"]
        )
        # print("Sliding surface: ", slidingSurface)

        control_now = np.matrix([[0.0], [0.0]])
        control_now = (
            np.matmul(
                (
                    self.model_gantry_crane.matrix_B
                    - np.matmul(self.model_gantry_crane.matrix_A, self.matrix_beta)
                ),
                stateVectorSecondDerivative,
            )
            + np.matmul(
                (
                    self.model_gantry_crane.matrix_C
                    - np.matmul(self.model_gantry_crane.matrix_A, self.matrix_alpha)
                ),
                stateVectorFirstDerivative,
            )
            + self.model_gantry_crane.matrix_D
            * self.gantry_crane_object.variables_second_derivative["sway_angle"]
            + (
                self.model_gantry_crane.matrix_E
                - np.matmul(self.model_gantry_crane.matrix_A, self.matrix_lambda)
            )
            * self.gantry_crane_object.variables_first_derivative["sway_angle"]
            + self.model_gantry_crane.matrix_F
        )

        # Simplified control
        control_now = control_now - np.multiply(
            self.matrix_k, np.tanh(np.multiply(self.matrix_gamma, slidingSurface))
        )

        print("Control now: {}, {}.".format(control_now[0, 0], control_now[1, 0]))

        self.control_signal_trolley = control_now[0, 0]
        self.control_signal_hoist = control_now[1, 0]

        if desired_cable_length is None:
            self.control_signal_hoist = 0.0

        return self.control_signal_trolley, self.control_signal_hoist

    def convert_to_pwm(self, trolley_control_input, hoist_control_input=None):
        """
        Function to converts the control input to PWM linearly.
        This function used to support non-symmetric PWM and deadzone.
        So, you need to specify the PWM range based on the deadzone of each motor.
        Also, you need to specify the control input range.
        """
        # Trolley Deadzone PWM
        TROLLEY_PWM_MAX = 1023
        TROLLEY_PWM_MIN = 0
        pwm_trolley = self.linear_interpolation(
            trolley_control_input,
            0.0,
            TROLLEY_PWM_MIN,
            12,
            TROLLEY_PWM_MAX,
        )

        # Hoist Deadzone PWM
        HOIST_PWM_MAX = 1023
        HOIST_PWM_MIN = 0
        if hoist_control_input is None:
            pwm_hoist = 0
        else:
            pwm_hoist = self.linear_interpolation(
                hoist_control_input,
                0.0,
                HOIST_PWM_MIN,
                12,
                HOIST_PWM_MAX,
            )

        self.pwm_trolley_motor = pwm_trolley
        self.pwm_hoist_motor = pwm_hoist

        return self.pwm_trolley_motor, self.pwm_hoist_motor

    def linear_interpolation(self, x, x1, y1, x2, y2):
        return y1 + (x - x1) * (y2 - y1) / (x2 - x1)
    
    def get_control_pwm(self, desired_trolley_position, desired_cable_length=None):
        trolley_control_input, hoist_control_input = self.get_control_signal(
            desired_trolley_position, desired_cable_length
        )
        trolley_pwm, hoist_pwm = self.convert_to_pwm(
            trolley_control_input, hoist_control_input
        )
        return trolley_pwm, hoist_pwm


def collect_data(
    time_now,
    trolley_motor_control_input,
    hoist_motor_control_input,
    trolley_motor_pwm,
    hoist_motor_pwm,
    gantry_crane,
):
    # Timestamp
    gantry_crane.data_collection_buffer["timestamp"].append(time_now)

    # Control input from controller
    gantry_crane.data_collection_buffer["trolley_control_input"].append(
        trolley_motor_control_input
    )
    gantry_crane.data_collection_buffer["hoist_control_input"].append(
        hoist_motor_control_input
    )

    # PWM from controller
    gantry_crane.data_collection_buffer["trolley_control_pwm"].append(trolley_motor_pwm)
    gantry_crane.data_collection_buffer["hoist_control_pwm"].append(hoist_motor_pwm)

    # Motor voltage
    gantry_crane.data_collection_buffer["trolley_motor_voltage"].append(
        gantry_crane.variables_value["trolley_motor_voltage"]
    )
    gantry_crane.data_collection_buffer["hoist_motor_voltage"].append(
        gantry_crane.variables_value["hoist_motor_voltage"]
    )

    # Variables
    gantry_crane.data_collection_buffer["trolley_position_third_derivative"].append(
        gantry_crane.variables_third_derivative["trolley_position"]
    )
    gantry_crane.data_collection_buffer["cable_length_third_derivative"].append(
        gantry_crane.variables_third_derivative["cable_length"]
    )

    gantry_crane.data_collection_buffer["trolley_position_second_derivative"].append(
        gantry_crane.variables_second_derivative["trolley_position"]
    )
    gantry_crane.data_collection_buffer["cable_length_second_derivative"].append(
        gantry_crane.variables_second_derivative["cable_length"]
    )

    gantry_crane.data_collection_buffer["trolley_position_first_derivative"].append(
        gantry_crane.variables_first_derivative["trolley_position"]
    )
    gantry_crane.data_collection_buffer["cable_length_first_derivative"].append(
        gantry_crane.variables_first_derivative["cable_length"]
    )

    gantry_crane.data_collection_buffer["trolley_position"].append(
        gantry_crane.variables_value["trolley_position"]
    )
    gantry_crane.data_collection_buffer["cable_length"].append(
        gantry_crane.variables_value["cable_length"]
    )

    # Sway angle
    gantry_crane.data_collection_buffer["sway_angle_second_derivative"].append(
        gantry_crane.variables_second_derivative["sway_angle"]
    )
    gantry_crane.data_collection_buffer["sway_angle_first_derivative"].append(
        gantry_crane.variables_first_derivative["sway_angle"]
    )
    gantry_crane.data_collection_buffer["sway_angle"].append(
        gantry_crane.variables_value["sway_angle"]
    )


non_linear_motor_model = GantryCraneModel(
    MODEL_NAME, GANTRY_CRANE_MODEL_PARAMETERS_JSON_PATH
)
sliding_mode_controller = Controller(
    CONTROLLER_NAME, SLIDING_MODE_CONTROLLER_PARAMETERS_JSON_PATH
)