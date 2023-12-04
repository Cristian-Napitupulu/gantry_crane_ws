#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import Int32

import numpy as np
import json
import time

GANTRY_CRANE_NODE_NAME = "gantry_crane_control_system"
CONTROLLER_NODE_NAME = "sliding_mode_controller"

TROLLEY_POSITION_TOPIC_NAME = "trolley_position"
CABLE_LENGTH_TOPIC_NAME = "cable_length"
SWAY_ANGLE_TOPIC_NAME = "sway_angle"

MOTOR_PWM_TOPIC_NAME = "motor_pwm"

DESIRED_TROLLEY_POSITION = 1.0
DESIRED_CABLE_LENGTH = 0.4

GANTRY_CRANE_MODEL_PARAMETERS_JSON_PATH = "/home/icodes/Documents/gantry_crane_ws/src/sliding_mode_controller/scripts/gantry_crane_parameters.json"

SLIDING_MODE_CONTROLLER_PARAMETERS_JSON_PATH = "/home/icodes/Documents/gantry_crane_ws/src/sliding_mode_controller/scripts/sliding_mode_controller_parameters.json"


class GantryCraneSystem(Node):
    def __init__(self):
        super().__init__(GANTRY_CRANE_NODE_NAME)

        self.subscribers = {}
        self.initialize_subscribers()

        self.parameter(GANTRY_CRANE_MODEL_PARAMETERS_JSON_PATH)

        self.initialize_variables()

        self.update_matrices()

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

    def initialize_variables(self):
        # Initialize variables
        self.variables_value = {
            "trolley_position": 0.0,
            "cable_length": 0.0,
            "sway_angle": 0.0,
        }

        self.last_variables_value = {
            "trolley_position": 0.0,
            "cable_length": 0.0,
            "sway_angle": 0.0,
        }

        self.variables_first_derivative = {
            "trolley_position": 0.0,
            "cable_length": 0.0,
            "sway_angle": 0.0,
        }

        self.last_variables_first_derivative = {
            "trolley_position": 0.0,
            "cable_length": 0.0,
            "sway_angle": 0.0,
        }

        self.variables_second_derivative = {
            "trolley_position": 0.0,
            "cable_length": 0.0,
            "sway_angle": 0.0,
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
        }

    def process_message(self, variable_name, message):
        timestamp = time.time()
        delta_time = timestamp - self.last_variables_message_timestamp[variable_name]
        self.variables_value[variable_name] = message.data

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

        self.last_variables_value[variable_name] = self.variables_value[variable_name]
        self.last_variables_first_derivative[
            variable_name
        ] = self.variables_first_derivative[variable_name]
        self.last_variables_message_timestamp[variable_name] = time.time()

    def trolley_position_callback(self, message):
        # self.get_logger().info("Trolley position: {:.7f}".format(message.data))
        self.process_message("trolley_position", message)
        # print("Trolley position: {:.7f}, First derivative: {:.7f}, Second derivative: {:.7f}".format(self.variables_value["trolley_position"], self.variables_first_derivative["trolley_position"], self.variables_second_derivative["trolley_position"]))

    def cable_length_callback(self, message):
        # self.get_logger().info("Cable length: {:.7f}".format(message.data))
        self.process_message("cable_length", message)
        # print("Cable length: {:.7f}, First derivative: {:.7f}, Second derivative: {:.7f}".format(self.variables_value["cable_length"], self.variables_first_derivative["cable_length"], self.variables_second_derivative["cable_length"]))

    def sway_angle_callback(self, message):
        # self.get_logger().info("Sway angle: {:.7f}".format(message.data))
        self.process_message("sway_angle", message)
        # print("Sway angle: {:.7f}, First derivative: {:.7f}, Second derivative: {:.7f}".format(self.variables_value["sway_angle"], self.variables_first_derivative["sway_angle"], self.variables_second_derivative["sway_angle"]))

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
            Int32, MOTOR_PWM_TOPIC_NAME, 10
        )

        # Initialize variables
        self.parameter(SLIDING_MODE_CONTROLLER_PARAMETERS_JSON_PATH)

    def combineInt16toInt32(self, lower_int16, upper_int16):
        # Convert to unsigned int16
        if lower_int16 < 0:
            lower_int16 = 0x10000 + lower_int16

        if upper_int16 < 0:
            upper_int16 = 0x10000 + upper_int16

        # Use bitwise OR to combine the two int16 values into one int32 value
        combined_int32 = (upper_int16 << 16) | (lower_int16 & 0xFFFF)

        # Convert to signed int32
        if (
            combined_int32 & 0x80000000
        ):  # Check if the most significant bit is set (indicating a negative number)
            combined_int32 = -((combined_int32 ^ 0xFFFFFFFF) + 1)

        return combined_int32

    def publish_motor_pwm(self, trolley_motor_pwm, hoist_motor_pwm):
        message = Int32()
        message.data = self.combineInt16toInt32(trolley_motor_pwm, hoist_motor_pwm)
        # self.get_logger().info("Publishing: {}. Troley motor PWM: {}. Hoist motor PWM: {}".format(message.data, trolley_motor_pwm, hoist_motor_pwm))
        self.motor_pwm_publisher.publish(message)

    def parameter(self, parameter_json_path):
        with open(parameter_json_path, "r") as file:
            parameter_file = json.load(file)

        # Sliding mode controller parameters
        a1 = parameter_file["sliding_mode_controller"]["parameters"]["alpa1"]["value"]
        a2 = parameter_file["sliding_mode_controller"]["parameters"]["alpa2"]["value"]
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

        self.matrix_alpha = np.matrix([[a1, 0.0], [0.0, a2]])

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
        # print("Desired state vector: ", desiredStateVector)
        # State vector
        stateVector = np.matrix(
            [
                [gantry_crane.variables_value["trolley_position"]],
                [gantry_crane.variables_value["cable_length"]],
            ]
        )
        # print("State vector: ", stateVector)
        stateVectorFirstDerivative = np.matrix(
            [
                [gantry_crane.variables_first_derivative["trolley_position"]],
                [gantry_crane.variables_first_derivative["cable_length"]],
            ]
        )
        # print("State vector first derivative: ", stateVectorFirstDerivative)
        stateVectorSecondDerivative = np.matrix(
            [
                [gantry_crane.variables_second_derivative["trolley_position"]],
                [gantry_crane.variables_second_derivative["cable_length"]],
            ]
        )
        # print("State vector second derivative: ", stateVectorSecondDerivative)

        # Sliding surface
        slidingSurface = np.matrix([[0.0], [0.0]])
        slidingSurface = (
            np.matmul(self.matrix_alpha, stateVector - desiredStateVector)
            + np.matmul(self.matrix_beta, stateVectorFirstDerivative)
            + stateVectorSecondDerivative
            + self.matrix_lambda * gantry_crane.variables_value["sway_angle"]
        )
        # print("Sliding surface: ", slidingSurface)

        control_now = np.matrix([[0.0], [0.0]])
        control_now = (
            np.matmul(
                (
                    gantry_crane.matrix_B
                    - np.matmul(gantry_crane.matrix_A, self.matrix_beta)
                ),
                stateVectorSecondDerivative,
            )
            + np.matmul(
                (
                    gantry_crane.matrix_C
                    - np.matmul(gantry_crane.matrix_A, self.matrix_alpha)
                ),
                stateVectorFirstDerivative,
            )
            + gantry_crane.matrix_D
            * gantry_crane.variables_second_derivative["sway_angle"]
            + (
                gantry_crane.matrix_E
                - np.matmul(gantry_crane.matrix_A, self.matrix_lambda)
            )
            * gantry_crane.variables_first_derivative["sway_angle"]
            + gantry_crane.matrix_F
            - np.multiply(self.k, np.sign(slidingSurface))
        )
        print("Control now: ", control_now)

        control_input1 = int(
            self.linear_interpolation(control_now[0, 0], -12.0, -255, 12, 255)
        )
        control_input2 = int(
            self.linear_interpolation(control_now[1, 0], -12.0, -255, 12, 255)
        )

        # print("Control input 1: ", control_input1)
        # print("Control input 2: ", control_input2)

        if control_input1 > 255:
            control_input1 = 255
        elif control_input1 < -255:
            control_input1 = -255

        if control_input2 > 255:
            control_input2 = 255
        elif control_input2 < -255:
            control_input2 = -255

        return control_input1, control_input2


if __name__ == "__main__":
    rclpy.init()
    gantry_crane = GantryCraneSystem()
    slidingModeController = Controller()
    try:
        while 1:
            # Update matrices
            gantry_crane.update_matrices()
            (
                trolley_motor_pwm,
                hoist_motor_pwm,
            ) = slidingModeController.get_control_input(
                gantry_crane, DESIRED_TROLLEY_POSITION, DESIRED_CABLE_LENGTH
            )
            slidingModeController.publish_motor_pwm(trolley_motor_pwm, hoist_motor_pwm)

            rclpy.spin_once(gantry_crane, timeout_sec=0.02)
            rclpy.spin_once(slidingModeController, timeout_sec=0.02)

    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
