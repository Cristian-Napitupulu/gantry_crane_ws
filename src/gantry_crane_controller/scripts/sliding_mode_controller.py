#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import UInt32

import numpy as np
import json
import time

import pandas as pd

import signal


GANTRY_CRANE_NODE_NAME = "gantry_crane_control_system"
CONTROLLER_NODE_NAME = "sliding_mode_controller"

TROLLEY_POSITION_TOPIC_NAME = "trolley_position"
CABLE_LENGTH_TOPIC_NAME = "cable_length"
SWAY_ANGLE_TOPIC_NAME = "sway_angle"

TROLLEY_MOTOR_VOLTAGE_TOPIC_NAME = "trolley_motor_voltage"
HOIST_MOTOR_VOLTAGE_TOPIC_NAME = "hoist_motor_voltage"

MOTOR_PWM_TOPIC_NAME = "motor_pwm"

# Mode
IDLE_MODE = 0x00
MOVE_TO_ORIGIN_MODE = 0x1F
MOVE_TO_MIDDLE_MODE = 0x2F
MOVE_TO_END_MODE = 0x3F
LOCK_CONTAINER_MODE = 0x4F
UNLOCK_CONTAINER_MODE = 0x5F
COLLECT_DATA_MODE = 0xFF
CONTROL_MODE = 0x6F

DESIRED_TROLLEY_POSITION = 1.0
DESIRED_CABLE_LENGTH = 0.5

MAX_PWM = 1023

BRAKE_COMMAND = 0x7FF

GANTRY_CRANE_MODEL_PARAMETERS_JSON_PATH = "/home/icodes/Documents/gantry_crane_ws/src/sliding_mode_controller/scripts/gantry_crane_parameters.json"

SLIDING_MODE_CONTROLLER_PARAMETERS_JSON_PATH = "/home/icodes/Documents/gantry_crane_ws/src/sliding_mode_controller/scripts/sliding_mode_controller_parameters.json"

DATA_SAVE_PATH = (
    "/home/icodes/Documents/gantry_crane_ws/src/gantry_parameter_optimization/data/"
)

COLLECTING_DATA = [False]


# Create a class for moving average filter
# The data is streamed in real time
class MovingAverageFilter:
    def __init__(self, window_size):
        self.window_size = window_size
        self.data = []
        self.sum = 0.0

    def add_data(self, new_data):
        if len(self.data) == self.window_size:
            self.sum -= self.data[0]
            self.data.pop(0)

        self.data.append(new_data)
        self.sum += new_data

    def get_average(self):
        return self.sum / len(self.data)


class GantryCraneSystem(Node):
    def __init__(self):
        super().__init__(GANTRY_CRANE_NODE_NAME)

        self.subscribers = {}
        self.initialize_subscribers()

        self.parameter(GANTRY_CRANE_MODEL_PARAMETERS_JSON_PATH)

        self.moving_average_filters = {}

        self.initialize_variables()

        print("Gantry crane system initialized.")

    def initialize_subscribers(self):
        # Initialize subscriber
        self.subscribers["trolley_position"] = self.create_subscription(
            Float32, TROLLEY_POSITION_TOPIC_NAME, self.trolley_position_callback, 10
        )
        self.subscribers["cable_length"] = self.create_subscription(
            Float32, CABLE_LENGTH_TOPIC_NAME, self.cable_length_callback, 10
        )
        self.subscribers["sway_angle"] = self.create_subscription(
            Float32, SWAY_ANGLE_TOPIC_NAME, self.sway_angle_callback, 10
        )
        self.subscribers["trolley_motor_voltage"] = self.create_subscription(
            Float32,
            TROLLEY_MOTOR_VOLTAGE_TOPIC_NAME,
            self.trolley_motor_voltage_callback,
            10,
        )
        self.subscribers["hoist_motor_voltage"] = self.create_subscription(
            Float32,
            HOIST_MOTOR_VOLTAGE_TOPIC_NAME,
            self.hoist_motor_voltage_callback,
            10,
        )

    def reset(self):
        self.variables_value = {
            "trolley_position": 0.0,
            "cable_length": 0.0,
            "sway_angle": 0.0,
            "trolley_motor_voltage": 0.0,
            "hoist_motor_voltage": 0.0,
        }

        self.data_collection_buffer = {
            "timestamp": [],
            "trolley_control_input": [],
            "trolley_control_pwm": [],
            "hoist_control_input": [],
            "hoist_control_pwm": [],
            "trolley_motor_voltage": [],
            "hoist_motor_voltage": [],
            "trolley_position_third_derivative": [],
            "cable_length_third_derivative": [],
            "trolley_position_second_derivative": [],
            "cable_length_second_derivative": [],
            "trolley_position_first_derivative": [],
            "cable_length_first_derivative": [],
            "trolley_position": [],
            "cable_length": [],
            "sway_angle_second_derivative": [],
            "sway_angle_first_derivative": [],
            "sway_angle": [],
        }

    def initialize_variables(self):
        # Initialize variables
        self.variables_value = {
            "trolley_position": None,
            "cable_length": None,
            "sway_angle": None,
            "trolley_motor_voltage": None,
            "hoist_motor_voltage": None,
        }

        self.last_variables_value = {
            "trolley_position": 0.0,
            "cable_length": 0.0,
            "sway_angle": 0.0,
            "trolley_motor_voltage": 0.0,
            "hoist_motor_voltage": 0.0,
        }

        self.variables_first_derivative = {
            "trolley_position": 0.0,
            "cable_length": 0.0,
            "sway_angle": 0.0,
            "trolley_motor_voltage": 0.0,
            "hoist_motor_voltage": 0.0,
        }

        self.last_variables_first_derivative = {
            "trolley_position": 0.0,
            "cable_length": 0.0,
            "sway_angle": 0.0,
            "trolley_motor_voltage": 0.0,
            "hoist_motor_voltage": 0.0,
        }

        self.variables_second_derivative = {
            "trolley_position": 0.0,
            "cable_length": 0.0,
            "sway_angle": 0.0,
            "trolley_motor_voltage": 0.0,
            "hoist_motor_voltage": 0.0,
        }

        self.last_variables_second_derivative = {
            "trolley_position": 0.0,
            "cable_length": 0.0,
            "sway_angle": 0.0,
            "trolley_motor_voltage": 0.0,
            "hoist_motor_voltage": 0.0,
        }

        self.variables_third_derivative = {
            "trolley_position": 0.0,
            "cable_length": 0.0,
            "sway_angle": 0.0,
            "trolley_motor_voltage": 0.0,
            "hoist_motor_voltage": 0.0,
        }

        self.matrix_A = np.matrix([[0.0, 0.0], [0.0, 0.0]])
        self.matrix_B = np.matrix([[0.0, 0.0], [0.0, 0.0]])
        self.matrix_C = np.matrix([[0.0, 0.0], [0.0, 0.0]])
        self.matrix_D = np.matrix([[0.0], [0.0]])
        self.matrix_E = np.matrix([[0.0], [0.0]])
        self.matrix_F = np.matrix([[0.0], [0.0]])

        self.last_variables_message_timestamp = {
            "trolley_position": time.time(),
            "cable_length": time.time(),
            "sway_angle": time.time(),
            "trolley_motor_voltage": time.time(),
            "hoist_motor_voltage": time.time(),
        }

    def process_message(self, variable_name, message):
        timestamp = time.time()
        delta_time = timestamp - self.last_variables_message_timestamp[variable_name]

        value = message.data
        if variable_name == "trolley_position":
            value = round(value, 5)

        if variable_name == "cable_length":
            value = round(value, 3)

        if variable_name == "sway_angle":
            value = round(value, 3) * np.pi / 180.0

        # self.moving_average_filters[variable_name].add_data(value)
        self.variables_value[variable_name] = value

        self.variables_first_derivative[variable_name] = self.calculate_derivative(
            self.variables_value[variable_name],
            self.last_variables_value[variable_name],
            delta_time,
        )

        self.variables_second_derivative[variable_name] = self.calculate_derivative(
            self.variables_first_derivative[variable_name],
            self.last_variables_first_derivative[variable_name],
            delta_time,
        )

        self.variables_third_derivative[variable_name] = self.calculate_derivative(
            self.variables_second_derivative[variable_name],
            self.last_variables_second_derivative[variable_name],
            delta_time,
        )

        self.last_variables_value[variable_name] = self.variables_value[variable_name]
        self.last_variables_first_derivative[
            variable_name
        ] = self.variables_first_derivative[variable_name]
        self.last_variables_second_derivative[
            variable_name
        ] = self.variables_second_derivative[variable_name]

        self.last_variables_message_timestamp[variable_name] = time.time()
        return delta_time

    def trolley_position_callback(self, message):
        # self.get_logger().info("Trolley position: {:.5f}".format(message.data))
        message_period = self.process_message("trolley_position", message)
        # print("Trolley position: {:.5f}, First derivative: {:.5f}, Second derivative: {:.5f}. \t Message period (ms): ".format(self.variables_value["trolley_position"], self.variables_first_derivative["trolley_position"], self.variables_second_derivative["trolley_position"]), round(1000 * (message_period), 2))

    def cable_length_callback(self, message):
        # self.get_logger().info("Cable length: {:.3f}".format(message.data))
        message_period = self.process_message("cable_length", message)
        # print("Cable length: {:.3f}, First derivative: {:.3f}, Second derivative: {:.3f}. \t Message period (ms): ".format(self.variables_value["cable_length"], self.variables_first_derivative["cable_length"], self.variables_second_derivative["cable_length"]), round(1000 * (message_period), 2))

    def sway_angle_callback(self, message):
        # self.get_logger().info("Sway angle: {:.5f}".format(message.data))
        message_period = self.process_message("sway_angle", message)
        # print("Sway angle (rad): {:.5f}, \t First derivative: {:.5f}, \t Second derivative: {:.5f}. \t Message period (ms): ".format(self.variables_value["sway_angle"], self.variables_first_derivative["sway_angle"], self.variables_second_derivative["sway_angle"]), round(1000 * (message_period), 2))

    def trolley_motor_voltage_callback(self, message):
        # self.get_logger().info("Trolley motor voltage: {:.5f}".format(message.data))
        message_period = self.process_message("trolley_motor_voltage", message)
        # print("Trolley motor voltage: {:.5f}, First derivative: {:.5f}, Second derivative: {:.5f}. \t Message period (ms): ".format(self.variables_value["trolley_motor_voltage"], self.variables_first_derivative["trolley_motor_voltage"], self.variables_second_derivative["trolley_motor_voltage"]), round(1000 * (message_period), 2))

    def hoist_motor_voltage_callback(self, message):
        # self.get_logger().info("Hoist motor voltage: {:.5f}".format(message.data))
        message_period = self.process_message("hoist_motor_voltage", message)
        # print("Hoist motor voltage: {:.5f}, First derivative: {:.5f}, Second derivative: {:.5f}. \t Message period (ms): ".format(self.variables_value["hoist_motor_voltage"], self.variables_first_derivative["hoist_motor_voltage"], self.variables_second_derivative["hoist_motor_voltage"]), round(1000 * (message_period), 2))

    def calculate_derivative(self, current_value, last_value, delta_time):
        return (current_value - last_value) / delta_time

    def parameter(self, parameter_json_path):
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
            self.mt + self.mc * np.sin(self.variables_value["sway_angle"]) ** 2
        ) / self.Kt1 + self.J1 * self.L1 / (self.Kt1 * self.rp1)
        self.matrix_A[0, 1] = (
            -self.L1
            * self.mc
            * self.rp1
            * np.sin(self.variables_value["sway_angle"])
            / self.Kt1
        )
        self.matrix_A[1, 0] = (
            -self.L2
            * self.mc
            * self.rp2
            * np.sin(self.variables_value["sway_angle"])
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
            * np.sin(2 * self.variables_value["sway_angle"])
            * self.variables_first_derivative["sway_angle"]
            / self.Kt1
            + self.R1
            * self.rp1
            * (self.mt + self.mc * np.sin(self.variables_value["sway_angle"]) ** 2)
            / self.Kt1
            + self.L1 * self.bt * self.rp1 / self.Kt1
            + self.J1 * self.R1 / (self.Kt1 * self.rp1)
            + self.L1 * self.b1 / (self.Kt1 * self.rp1)
        )
        self.matrix_B[0, 1] = (
            -self.L1
            * self.mc
            * self.rp1
            * np.cos(self.variables_value["sway_angle"])
            * self.variables_first_derivative["sway_angle"]
            / self.Kt1
            - self.R1
            * self.mc
            * self.rp1
            * np.sin(self.variables_value["sway_angle"])
            / self.Kt1
        )
        self.matrix_B[1, 0] = (
            -self.L2
            * self.mc
            * self.rp2
            * np.cos(self.variables_value["sway_angle"])
            * self.variables_first_derivative["sway_angle"]
            / self.Kt2
            - self.R2
            * self.mc
            * self.rp2
            * np.sin(self.variables_value["sway_angle"])
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
            * self.variables_value["cable_length"]
            * np.sin(self.variables_value["sway_angle"])
            * self.variables_first_derivative["sway_angle"]
            / self.Kt1
        )
        self.matrix_D[1, 0] = (
            -2
            * self.L2
            * self.mc
            * self.rp2
            * self.variables_value["cable_length"]
            * self.variables_first_derivative["sway_angle"]
            / self.Kt2
        )

    def update_matrix_E(self):
        # Update matrix E
        self.matrix_E[0, 0] = (
            self.L1
            * self.mc
            * self.rp1
            * self.variables_value["cable_length"]
            * np.cos(self.variables_value["sway_angle"])
            * self.variables_first_derivative["sway_angle"] ** 2
            / self.Kt1
            + self.L1
            * self.g
            * self.mc
            * self.rp1
            * np.cos(2 * self.variables_value["sway_angle"])
            / self.Kt1
            + self.L1
            * self.mc
            * self.rp1
            * np.sin(self.variables_value["sway_angle"])
            * self.variables_first_derivative["cable_length"]
            * self.variables_first_derivative["sway_angle"]
            / self.Kt1
            + self.R1
            * self.mc
            * self.rp1
            * self.variables_value["cable_length"]
            * np.sin(self.variables_value["sway_angle"])
            * self.variables_first_derivative["sway_angle"]
            / self.Kt1
        )
        self.matrix_E[1, 0] = (
            +self.L2
            * self.g
            * self.mc
            * self.rp2
            * np.sin(self.variables_value["sway_angle"])
            / self.Kt2
            - self.L2
            * self.mc
            * self.rp2
            * self.variables_first_derivative["cable_length"]
            * self.variables_first_derivative["sway_angle"]
            / self.Kt2
            - self.R2
            * self.mc
            * self.rp2
            * self.variables_value["cable_length"]
            * self.variables_first_derivative["sway_angle"]
            / self.Kt2
        )

    def update_matrix_F(self):
        # Update matrix F
        self.matrix_F[0, 0] = (
            self.R1
            * self.g
            * self.mc
            * self.rp1
            * np.sin(self.variables_value["sway_angle"])
            * np.cos(self.variables_value["sway_angle"])
            / self.Kt1
        )
        self.matrix_F[1, 0] = (
            -self.R2
            * self.g
            * self.mc
            * self.rp2
            * np.cos(self.variables_value["sway_angle"])
            / self.Kt2
        )

    def update_matrices(self):
        self.update_matrix_A()
        self.update_matrix_B()
        self.update_matrix_C()
        self.update_matrix_D()
        self.update_matrix_E()
        self.update_matrix_F()


class Controller(Node):
    def __init__(self):
        # Initialize node
        super().__init__(CONTROLLER_NODE_NAME)

        # Initialize publisher
        self.motor_pwm_publisher = self.create_publisher(
            UInt32, MOTOR_PWM_TOPIC_NAME, 10
        )

        # Initialize variables
        self.parameter(SLIDING_MODE_CONTROLLER_PARAMETERS_JSON_PATH)

    def packValues(self, mode, pwm_trolley, pwm_hoist):
        # Ensure the values are within the specified ranges
        mode = max(min(mode, 255), 0)

        # Convert negative values to two's complement representation
        if pwm_trolley < 0:
            pwm_trolley = 0xFFF + pwm_trolley + 1
        if pwm_hoist < 0:
            pwm_hoist = 0xFFF + pwm_hoist + 1

        # Pack the values into a 32-bit integer
        packed_value = (
            (mode & 0xFF) | ((pwm_trolley & 0xFFF) << 8) | ((pwm_hoist & 0xFFF) << 20)
        )
        return packed_value

    def publish_motor_pwm(self, gantry_mode, trolley_motor_pwm, hoist_motor_pwm):
        message = UInt32()
        message.data = self.packValues(gantry_mode, trolley_motor_pwm, hoist_motor_pwm)
        # self.get_logger().info("Publishing: {}. Mode: {}. Troley motor PWM: {}. Hoist motor PWM: {}".format(message.data, gantry_mode, trolley_motor_pwm, hoist_motor_pwm))
        self.motor_pwm_publisher.publish(message)

    def parameter(self, parameter_json_path):
        with open(parameter_json_path, "r") as file:
            parameter_file = json.load(file)

        # Sliding mode controller parameters
        alpa1 = parameter_file["sliding_mode_controller"]["parameters"]["alpa1"][
            "value"
        ]
        alpa2 = parameter_file["sliding_mode_controller"]["parameters"]["alpa2"][
            "value"
        ]
        beta1 = parameter_file["sliding_mode_controller"]["parameters"]["beta1"][
            "value"
        ]
        beta2 = parameter_file["sliding_mode_controller"]["parameters"]["beta2"][
            "value"
        ]
        lambda1 = parameter_file["sliding_mode_controller"]["parameters"]["lambda1"][
            "value"
        ]
        lambda2 = parameter_file["sliding_mode_controller"]["parameters"]["lambda2"][
            "value"
        ]
        k1 = parameter_file["sliding_mode_controller"]["parameters"]["k1"]["value"]
        k2 = parameter_file["sliding_mode_controller"]["parameters"]["k2"]["value"]
        print(
            "Sliding mode controller's parameters last updated: ",
            parameter_file["sliding_mode_controller"]["last_update"],
        )

        self.matrix_lambda = np.matrix([[lambda1], [lambda2]])

        self.matrix_alpha = np.matrix([[alpa1, 0.0], [0.0, alpa2]])

        self.matrix_beta = np.matrix([[beta1, 0.0], [0.0, beta2]])

        self.k = [[k1], [k2]]

    def linear_interpolation(self, x, x1, y1, x2, y2):
        return y1 + (x - x1) * (y2 - y1) / (x2 - x1)

    def get_control_input(
        self, gantry_crane, desired_trolley_position, desired_cable_length
    ):
        # Modify as needed

        desiredStateVector = np.matrix(
            [[desired_trolley_position], [desired_cable_length]]
        )
        # State vector
        stateVector = np.matrix(
            [
                [gantry_crane.variables_value["trolley_position"]],
                [gantry_crane.variables_value["cable_length"]],
            ]
        )

        print(
            "Desired state vector: {}, State vector: {}".format(
                desiredStateVector, stateVector
            )
        )

        stateVectorFirstDerivative = np.matrix(
            [
                [gantry_crane.variables_first_derivative["trolley_position"]],
                [gantry_crane.variables_first_derivative["cable_length"]],
            ]
        )

        stateVectorSecondDerivative = np.matrix(
            [
                [gantry_crane.variables_second_derivative["trolley_position"]],
                [gantry_crane.variables_second_derivative["cable_length"]],
            ]
        )
        # print(
        #     "State vector first derivative: {}, State vector second derivative: {}".format(
        #         stateVectorFirstDerivative, stateVectorSecondDerivative
        #     )
        # )

        # Sliding surface
        # slidingSurface = np.matrix([[0.0], [0.0]])
        # slidingSurface = (
        #     np.matmul(self.matrix_alpha, stateVector - desiredStateVector)
        #     + np.matmul(self.matrix_beta, stateVectorFirstDerivative)
        #     + stateVectorSecondDerivative
        #     + self.matrix_lambda * gantry_crane.variables_value["sway_angle"]
        # )

        # Simplified sliding surface
        slidingSurface = (
            np.matmul(self.matrix_alpha, stateVector - desiredStateVector)
            + np.matmul(self.matrix_beta, stateVectorFirstDerivative)
            # + stateVectorSecondDerivative
            + self.matrix_lambda * gantry_crane.variables_value["sway_angle"]
        )
        # print("Sliding surface: ", slidingSurface)

        control_now = np.matrix([[0.0], [0.0]])
        # control_now = (
        #     np.matmul(
        #         (
        #             gantry_crane.matrix_B
        #             - np.matmul(gantry_crane.matrix_A, self.matrix_beta)
        #         ),
        #         stateVectorSecondDerivative,
        #     )
        #     + np.matmul(
        #         (
        #             gantry_crane.matrix_C
        #             - np.matmul(gantry_crane.matrix_A, self.matrix_alpha)
        #         ),
        #         stateVectorFirstDerivative,
        #     )
        #     + gantry_crane.matrix_D
        #     * gantry_crane.variables_second_derivative["sway_angle"]
        #     + (
        #         gantry_crane.matrix_E
        #         - np.matmul(gantry_crane.matrix_A, self.matrix_lambda)
        #     )
        #     * gantry_crane.variables_first_derivative["sway_angle"]
        #     + gantry_crane.matrix_F
        # )

        multiplier = np.array([[0.13], [1.0]])
        # Simplified control
        control_now = -np.multiply(
            self.k, np.tanh(np.multiply(multiplier, slidingSurface))
        )

        print("Control now: ", control_now)

        control_input1 = control_now[0, 0]
        control_input2 = control_now[1, 0]

        return control_input1, control_input2

    def convert_to_pwm(self, voltage):
        FORWARD_DEADZONE_PWM = 575
        REVERSE_DEADZONE_PWM = 575
        if voltage > 0:
            pwm = int(self.linear_interpolation(voltage, 0.0, FORWARD_DEADZONE_PWM, 12, MAX_PWM))
        elif voltage < 0:
            pwm = int(self.linear_interpolation(voltage, 0.0, -REVERSE_DEADZONE_PWM, -12, -MAX_PWM))
        else:
            pwm = 0
        return pwm


DURATION = 60.0
MAX_VOLTAGE = 10.0


def collect_data(
    gantry_crane,
    trolley_motor_control_input,
    hoist_motor_control_input,
    time_now,
    trolley_motor_pwm,
    hoist_motor_pwm,
):
    gantry_crane.data_collection_buffer["timestamp"].append(time_now)
    gantry_crane.data_collection_buffer["trolley_control_input"].append(
        trolley_motor_control_input
    )
    gantry_crane.data_collection_buffer["trolley_control_pwm"].append(trolley_motor_pwm)
    gantry_crane.data_collection_buffer["hoist_control_input"].append(
        hoist_motor_control_input
    )
    gantry_crane.data_collection_buffer["hoist_control_pwm"].append(hoist_motor_pwm)
    gantry_crane.data_collection_buffer["trolley_motor_voltage"].append(
        gantry_crane.variables_value["trolley_motor_voltage"]
    )
    gantry_crane.data_collection_buffer["hoist_motor_voltage"].append(
        gantry_crane.variables_value["hoist_motor_voltage"]
    )
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
    gantry_crane.data_collection_buffer["sway_angle_second_derivative"].append(
        gantry_crane.variables_second_derivative["sway_angle"]
    )
    gantry_crane.data_collection_buffer["sway_angle_first_derivative"].append(
        gantry_crane.variables_first_derivative["sway_angle"]
    )
    gantry_crane.data_collection_buffer["sway_angle"].append(
        gantry_crane.variables_value["sway_angle"]
    )


if __name__ == "__main__":
    rclpy.init()
    gantry_crane = GantryCraneSystem()
    slidingModeController = Controller()

    try:
        while True:
            # print(gantry_crane.variables_value["trolley_position"])

            if (
                gantry_crane.variables_value["trolley_position"] is not None
                and gantry_crane.variables_value["cable_length"] is not None
            ):
                data_incoming = True
                print("Data incoming...")
                gantry_crane.reset()
                break
            rclpy.spin_once(gantry_crane, timeout_sec=0.01)

        start_time = time.time()
        while True:
            if time.time() - start_time > 5:
                break
            rclpy.spin_once(gantry_crane, timeout_sec=0.01)

        trolley_motor_control_input = 0.0
        hoist_motor_control_input = 0.0
        start_time = time.time()
        while True:
            time_now = time.time() - start_time

            # Update matrices
            gantry_crane.update_matrices()

            rclpy.spin_once(gantry_crane, timeout_sec=0.01)

            # # Get control input
            (
                trolley_motor_control_input,
                hoist_motor_control_input,
            ) = slidingModeController.get_control_input(
                gantry_crane, DESIRED_TROLLEY_POSITION, DESIRED_CABLE_LENGTH
            )

            # Convert control input to PWM
            trolley_motor_pwm = slidingModeController.convert_to_pwm(
                trolley_motor_control_input
            )
            hoist_motor_pwm = slidingModeController.convert_to_pwm(
                hoist_motor_control_input
            )

            # Publish motor PWM
            gantryMode = CONTROL_MODE
            slidingModeController.publish_motor_pwm(
                gantryMode, trolley_motor_pwm, hoist_motor_pwm
            )

            rclpy.spin_once(slidingModeController, timeout_sec=0.01)

            print("slide and hoist... Time now: ", time_now)
            # collect_data(
            #     gantry_crane,
            #     trolley_motor_control_input,
            #     hoist_motor_control_input,
            #     time_now,
            #     trolley_motor_pwm,
            #     hoist_motor_pwm,
            # )

            if time_now > DURATION:
                break
        
        start_time = time.time()
        while True:
            time_now = time.time() - start_time

            # Update matrices
            gantry_crane.update_matrices()

            rclpy.spin_once(gantry_crane, timeout_sec=0.01)

            trolley_motor_control_input = 0

            hoist_motor_control_input = 0

            # Convert control input to PWM
            trolley_motor_pwm = slidingModeController.convert_to_pwm(
                trolley_motor_control_input
            )
            hoist_motor_pwm = slidingModeController.convert_to_pwm(
                hoist_motor_control_input
            )

            # Publish motor PWM
            gantryMode = CONTROL_MODE
            slidingModeController.publish_motor_pwm(
                gantryMode, trolley_motor_pwm, hoist_motor_pwm
            )

            rclpy.spin_once(slidingModeController, timeout_sec=0.01)

            # print("idle 1... Time now: ", time_now)
            # collect_data(
            #     gantry_crane,
            #     trolley_motor_control_input,
            #     hoist_motor_control_input,
            #     time_now,
            #     trolley_motor_pwm,
            #     hoist_motor_pwm,
            # )

            if time_now > 5:
                break

        data = pd.DataFrame(gantry_crane.data_collection_buffer)
        data.to_excel(
            DATA_SAVE_PATH + "sliding_mode_controller_data.xlsx",
            sheet_name="data",
            index=False,
            float_format="%.7f",
        )

        start_time = time.time()
        while True:
            time_now = time.time() - start_time

            # Update matrices
            gantry_crane.update_matrices()

            rclpy.spin_once(gantry_crane, timeout_sec=0.01)

            if gantry_crane.variables_value["trolley_position"] > 0.5:
                trolley_motor_control_input -= 0.005
            else:
                trolley_motor_control_input = 0

            if gantry_crane.variables_value["cable_length"] < 0.6:
                hoist_motor_control_input += 0.005
            else:
                hoist_motor_control_input = 0

            # Convert control input to PWM
            trolley_motor_pwm = slidingModeController.convert_to_pwm(
                trolley_motor_control_input
            )
            hoist_motor_pwm = slidingModeController.convert_to_pwm(
                hoist_motor_control_input
            )

            # Publish motor PWM
            gantryMode = CONTROL_MODE
            slidingModeController.publish_motor_pwm(
                gantryMode, trolley_motor_pwm, hoist_motor_pwm
            )

            rclpy.spin_once(slidingModeController, timeout_sec=0.01)

            if time_now > 15:
                break

        for i in range(10):
            gantryMode = MOVE_TO_ORIGIN_MODE
            slidingModeController.publish_motor_pwm(gantryMode, 0, 0)
            rclpy.spin_once(slidingModeController, timeout_sec=0.015)
            time.sleep(0.05)

    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    signal.setitimer(signal.ITIMER_REAL, 0)
