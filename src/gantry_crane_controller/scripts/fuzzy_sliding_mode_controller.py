#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import UInt32

import numpy as np
import pandas as pd
import time

from pyit2fls import (
    IT2FS_Gaussian_UncertStd,
    IT2FLS,
    min_t_norm,
    product_t_norm,
    max_s_norm,
    IT2FS_plot,
)
from numpy import array, linspace

GANTRY_CRANE_NODE_NAME = "gantry_crane_control_system"
CONTROLLER_NODE_NAME = "sliding_mode_controller"

TROLLEY_POSITION_TOPIC_NAME = "trolley_position"
CABLE_LENGTH_TOPIC_NAME = "cable_length"
SWAY_ANGLE_TOPIC_NAME = "sway_angle"

MOTOR_PWM_TOPIC_NAME = "controller_command"

# Mode
IDLE_MODE = 0x0F
MOVE_TO_ORIGIN_MODE = 0x1F
MOVE_TO_MIDDLE_MODE = 0x2F
MOVE_TO_END_MODE = 0x3F
LOCK_CONTAINER_MODE = 0x4F
UNLOCK_CONTAINER_MODE = 0x5F
COLLECT_DATA_MODE = 0xFF
CONTROL_MODE = 0x6F

DESIRED_TROLLEY_POSITION = 1.0
DESIRED_CABLE_LENGTH = 0.5

DATA_SAVE_PATH = (
    "/home/icodes/Documents/gantry_crane_ws/src/gantry_crane_controller/data/"
)


class GantryCraneSystem(Node):
    def __init__(self):
        super().__init__(GANTRY_CRANE_NODE_NAME)

        self.subscribers = {}
        self.initialize_subscribers()

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

        self.matrix_A = np.matrix(
            [
                [0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            ]
        )

        self.matrix_B = np.matrix(
            [[0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0]]
        )

        self.matrix_G = np.matrix([[0.0], [0.0], [0.0], [0.0], [0.0], [0.0]])

        self.last_variables_message_timestamp = {
            "trolley_position": time.time(),
            "cable_length": time.time(),
            "sway_angle": time.time(),
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

    # Update Matrix A
    def update_matrix_A(self, mc, mt, bt, br):
        self.matrix_A[1, 1] = -bt / mt
        self.matrix_A[1, 3] = -br * np.sin(self.variables_value["sway_angle"]) / mt
        self.matrix_A[3, 1] = -bt * np.sin(self.variables_value["sway_angle"]) / mt
        self.matrix_A[3, 3] = -(
            mc * (np.sin(self.variables_value["sway_angle"])) ** 2 / mt + 1
        ) * (br / mt)
        self.matrix_A[3, 5] = (
            self.variables_value["cable_length"]
            * self.variables_first_derivative["sway_angle"]
        )
        self.matrix_A[5, 1] = (
            -bt
            * np.cos(self.variables_value["sway_angle"])
            / (mt * self.variables_value["cable_length"])
        )
        self.matrix_A[5, 3] = (
            -br
            * np.sin(self.variables_value["sway_angle"])
            * np.cos(self.variables_value["sway_angle"])
            / (mt * self.variables_value["cable_length"])
        )
        self.matrix_A[5, 5] = (
            -2
            * self.variables_first_derivative["cable_length"]
            / self.variables_value["cable_length"]
        )

    # Update Matrix B
    def update_matrix_B(self, mc, mt):
        self.matrix_B[1, 0] = 1 / mt
        self.matrix_B[1, 1] = np.sin(self.variables_value["sway_angle"]) / mt
        self.matrix_B[3, 0] = np.sin(self.variables_value["sway_angle"]) / mt
        self.matrix_B[3, 1] = (
            mc * (np.sin(self.variables_value["sway_angle"])) ** 2 / mt + 1
        ) / mc
        self.matrix_B[5, 0] = np.cos(self.variables_value["sway_angle"]) / (
            mt * self.variables_value["cable_length"]
        )
        self.matrix_B[5, 1] = (
            np.sin(self.variables_value["sway_angle"])
            * np.cos(self.variables_value["sway_angle"])
            / (mt * self.variables_value["cable_length"])
        )

    # Update Matrix G
    def update_matrix_G(self, g):
        self.matrix_G[1, 0] = 0.0
        self.matrix_G[3, 0] = g * np.cos(self.variables_value["sway_angle"])
        self.matrix_G[5, 0] = (
            -g
            * np.sin(self.variables_value["sway_angle"])
            / self.variables_value["cable_length"]
        )

    def update_matrices(self):
        mc = 1
        mt = 2
        g = 9.81
        bt = 2
        br = 2
        self.update_matrix_A(mc, mt, bt, br)
        self.update_matrix_B(mc, mt)
        self.update_matrix_G(g)


class Controller(Node):
    def __init__(self):
        # Initialize node
        super().__init__(CONTROLLER_NODE_NAME)

        # Initialize publisher
        self.motor_pwm_publisher = self.create_publisher(
            UInt32, MOTOR_PWM_TOPIC_NAME, 10
        )

        # Initialize variables
        self.parameter()

        # fuzzyT2 Inisialisasi
        self.domain = linspace(-1.5, 1.5, 101)
        NB = IT2FS_Gaussian_UncertStd(self.domain, [-1, 0.2, 0.1, 1.0])
        NS = IT2FS_Gaussian_UncertStd(self.domain, [-0.5, 0.13, 0.1, 1.0])
        Z = IT2FS_Gaussian_UncertStd(self.domain, [0.0, 0.12, 0.05, 1.0])
        PS = IT2FS_Gaussian_UncertStd(self.domain, [0.5, 0.13, 0.1, 1.0])
        PB = IT2FS_Gaussian_UncertStd(self.domain, [1, 0.2, 0.1, 1.0])
        # IT2FS_plot(NB,NS, Z, PS, PB,legends=["Negative Big","Negative Small", "Zero", "Positive Small", "Positive Big"])

        self.domain_k = linspace(-1, 4, 101)
        Zk = IT2FS_Gaussian_UncertStd(self.domain_k, [0.0, 0.08, 0.08, 1.0])
        PSk = IT2FS_Gaussian_UncertStd(self.domain_k, [1, 0.4, 0.3, 1.0])
        PMk = IT2FS_Gaussian_UncertStd(self.domain_k, [2, 0.3, 0.3, 1.0])
        PBk = IT2FS_Gaussian_UncertStd(self.domain_k, [3, 0.3, 0.3, 1.0])
        # IT2FS_plot(Zk, PSk, PMk, PBk,legends=["Zero", "Positive Small", "Positive Medium", "Positive Big"])

        self.it2fls = IT2FLS()
        self.it2fls.add_input_variable("S")
        self.it2fls.add_input_variable("S_dot")
        self.it2fls.add_output_variable("K")

        # K Rule
        self.it2fls.add_rule([("S", NB), ("S_dot", NB)], [("K", PBk)])
        self.it2fls.add_rule([("S", NB), ("S_dot", NS)], [("K", PBk)])
        self.it2fls.add_rule([("S", NB), ("S_dot", Z)], [("K", PSk)])
        self.it2fls.add_rule([("S", NB), ("S_dot", PS)], [("K", PMk)])
        self.it2fls.add_rule([("S", NB), ("S_dot", PB)], [("K", PMk)])

        self.it2fls.add_rule([("S", NS), ("S_dot", NB)], [("K", PBk)])
        self.it2fls.add_rule([("S", NS), ("S_dot", NS)], [("K", PMk)])
        self.it2fls.add_rule([("S", NS), ("S_dot", Z)], [("K", PSk)])
        self.it2fls.add_rule([("S", NS), ("S_dot", PS)], [("K", PSk)])
        self.it2fls.add_rule([("S", NS), ("S_dot", PB)], [("K", PMk)])

        self.it2fls.add_rule([("S", Z), ("S_dot", NB)], [("K", PBk)])
        self.it2fls.add_rule([("S", Z), ("S_dot", NS)], [("K", PSk)])
        self.it2fls.add_rule([("S", Z), ("S_dot", Z)], [("K", Zk)])
        self.it2fls.add_rule([("S", Z), ("S_dot", PS)], [("K", PSk)])
        self.it2fls.add_rule([("S", Z), ("S_dot", PB)], [("K", PBk)])

        self.it2fls.add_rule([("S", PS), ("S_dot", NB)], [("K", PMk)])
        self.it2fls.add_rule([("S", PS), ("S_dot", NS)], [("K", PSk)])
        self.it2fls.add_rule([("S", PS), ("S_dot", Z)], [("K", PSk)])
        self.it2fls.add_rule([("S", PS), ("S_dot", PS)], [("K", PMk)])
        self.it2fls.add_rule([("S", PS), ("S_dot", PB)], [("K", PBk)])

        self.it2fls.add_rule([("S", PB), ("S_dot", NB)], [("K", PMk)])
        self.it2fls.add_rule([("S", PB), ("S_dot", NS)], [("K", PMk)])
        self.it2fls.add_rule([("S", PB), ("S_dot", Z)], [("K", PSk)])
        self.it2fls.add_rule([("S", PB), ("S_dot", PS)], [("K", PBk)])
        self.it2fls.add_rule([("S", PB), ("S_dot", PB)], [("K", PBk)])

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

    def u_fuzzy(self, St):
        S = max(-1.5, min(1.5, St[0]))
        S_dot = max(-1.5, min(1.5, St[1]))
        c, TR = self.it2fls.evaluate(
            {"S": S, "S_dot": S_dot},
            product_t_norm,
            max_s_norm,
            self.domain,
            method="Centroid",
            algorithm="EKM",
        )
        K = (TR["K"][0] + TR["K"][1]) / 2
        
        return K

    def parameter(self):
        # [k, lambda1, lambda2, alpha1, alpha2] = [
        #     1.85048618e-01,
        #     0.5,
        #     4.34629608e-01,
        #     2.53926722e00,
        #     9.96405942e-07,
        # ]
        [k, lambda1, lambda2, alpha1, alpha2] = [
            1.85048618e-01,
            0.5,
            4.34629608e-01,
            2.5,
            9.96405942e-07,
        ]
        
        self.matrix_lambda = np.matrix([[lambda1, 0.0], [0.0, lambda2]])
        self.matrix_alpha = np.matrix([[alpha1], [alpha2]])
        self.matrix_I = np.matrix([[1.0, 0.0], [0.0, 1.0]])

        k1 = k
        k2 = k
        self.k = [[k1], [k2]]

        self.sliding_surface_last = np.matrix([[0], [0]])

    def sign_matrix(self, X):
        Y = np.zeros(X.shape)
        for i in range(X.shape[0]):
            for j in range(X.shape[1]):
                if X[i, j] > 0:
                    Y[i, j] = 1
                elif X[i, j] < 0:
                    Y[i, j] = -1
                else:
                    Y[i, j] = 0
        return Y

    def linear_interpolation(self, x, x1, y1, x2, y2):
        return y1 + (x - x1) * (y2 - y1) / (x2 - x1)

    def get_control_input(
        self, gantry_crane, desired_trolley_position, desired_cable_length
    ):
        # Modify as needed

        desiredStateVector = np.matrix(
            [[desired_trolley_position], 
             [desired_cable_length]]
        )
        # print("Desired state vector: ", desiredStateVector)

        # State vector
        stateVector = np.matrix(
            [
                [gantry_crane.variables_value["trolley_position"]],
                [gantry_crane.variables_value["cable_length"]],
            ]
        )
        if stateVector[0, 0] == 0.0 or stateVector[1, 0] == 0.0:
            return 0, 0

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

        # Matrix A, B, G cap
        matrix_A1_cap = np.matrix(
            [
                [gantry_crane.matrix_A[1, 1], gantry_crane.matrix_A[1, 3]],
                [gantry_crane.matrix_A[3, 1], gantry_crane.matrix_A[3, 3]],
            ]
        )
        matrix_A2_cap = np.matrix(
            [
                [gantry_crane.matrix_A[1, 5]],
                [gantry_crane.matrix_A[3, 5]],
            ]
        )
        matrix_B_cap = np.matrix(
            [
                [gantry_crane.matrix_B[1, 0], gantry_crane.matrix_B[1, 1]],
                [gantry_crane.matrix_B[3, 0], gantry_crane.matrix_B[3, 1]],
            ]
        )

        matrix_G_cap = np.matrix(
            [[gantry_crane.matrix_G[1, 0]], [gantry_crane.matrix_G[3, 0]]]
        )

        # Create dummy matrix for sliding surface
        # sliding_surface_now = (
        #     self.matrix_lambda * (desiredStateVector - stateVector)
        #     - self.matrix_I * stateVectorFirstDerivative
        #     - self.matrix_alpha * gantry_crane.variables_value["sway_angle"]
        # )
        sliding_surface_now = (
            self.matrix_lambda * (desiredStateVector - stateVector)
            - self.matrix_I * stateVectorFirstDerivative
            - self.matrix_alpha * gantry_crane.variables_value["sway_angle"]
        )
    
        sliding_surface_dot = sliding_surface_now - self.sliding_surface_last
        self.sliding_surface_last = sliding_surface_now
        print("SSurface:",sliding_surface_now)
        SSx = [sliding_surface_now.item(0), sliding_surface_dot.item(0)]
        SSl = [sliding_surface_now.item(1), sliding_surface_dot.item(1)]
        kx = self.u_fuzzy(SSx)
        kl = self.u_fuzzy(SSl)
        print(f"Kx: {kx},SSx:{SSx}, Kl: {kl},SSl:{SSx}")
        k = np.matrix([[kx], [kl]])

        control_now = np.linalg.inv(matrix_B_cap) * (
            -np.matmul((matrix_A1_cap + self.matrix_lambda), stateVectorFirstDerivative)
            - matrix_G_cap
            - (matrix_A2_cap + self.matrix_alpha)
            * gantry_crane.variables_value["sway_angle"]
            + np.multiply(k, self.sign_matrix(sliding_surface_now))
        )

        

        # time.sleep(0.2)

        # s_trolley_position = (
        #     gantry_crane.variables_value["trolley_position"]
        #     - desired_trolley_position
        # )

        # s_cable_length = gantry_crane.variables_value["cable_length"] - desired_cable_length

        # control_input1 = - 120 * np.tanh(s_trolley_position)
        # if gantry_crane.variables_value["trolley_position"] > 1.0:
        #     control_input1 = 0
        # else:
        control_input1 = control_now[0, 0]
        control_input2 = control_now[1, 0]

        # time.sleep(1)

        # print("Control input 1: ", control_input1)
        # print("Control input 2: ", control_input2)
     
        print("Control now: ", control_input1, control_input2)
        return control_input1, control_input2

    def convert_to_pwm(self, voltage):
        MAX_PWM = 750
        FORWARD_DEADZONE_PWM = 575
        REVERSE_DEADZONE_PWM = 575
        if voltage > 0:
            pwm = int(
                self.linear_interpolation(
                    voltage, 0.0, FORWARD_DEADZONE_PWM, 4, MAX_PWM
                )
            )
        elif voltage < 0:
            pwm = int(
                self.linear_interpolation(
                    voltage, 0.0, -REVERSE_DEADZONE_PWM, -5, -MAX_PWM
                )
            )
        else:
            pwm = 0
        return pwm


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


DURATION = 15
if __name__ == "__main__":
    rclpy.init()
    gantry_crane = GantryCraneSystem()
    slidingModeController = Controller()
    try:
        while True:
            print("Waiting for data...")
            if (
                gantry_crane.variables_value["trolley_position"] is not None
                and gantry_crane.variables_value["cable_length"] is not None
            ):
                data_incoming = True
                print("Data incoming...")
                gantry_crane.reset()
                break
            rclpy.spin_once(gantry_crane, timeout_sec=0.01)

        for i in range(10):
            gantryMode = IDLE_MODE
            slidingModeController.publish_motor_pwm(gantryMode, 0, 0)
            rclpy.spin_once(slidingModeController, timeout_sec=0.015)
            time.sleep(0.05)

        gantry_crane.reset()
        start_time = time.time()
        while True:
            print(
                "Trolley position: {}, cable length: {}, sway angle: {}.".format(
                    gantry_crane.variables_value["trolley_position"],
                    gantry_crane.variables_value["cable_length"],
                    gantry_crane.variables_value["sway_angle"],
                )
            )
            if time.time() - start_time > 1:
                break
            rclpy.spin_once(gantry_crane, timeout_sec=0.001)

        trolley_motor_control_input = 0.0
        hoist_motor_control_input = 0.0
        trolley_motor_pwm = 0
        hoist_motor_pwm = 0
        start_time = time.time()
        while True:
            time_now = time.time() - start_time
            # Update matrices
            # gantry_crane.update_matrices()
            # (
            #     trolley_motor_control_input,
            #     hoist_motor_control_input,
            # ) = slidingModeController.get_control_input(
            #     gantry_crane, DESIRED_TROLLEY_POSITION, DESIRED_CABLE_LENGTH
            # )

            # trolley_motor_pwm = slidingModeController.convert_to_pwm(trolley_motor_control_input)
            # print("trolley_motor_pwm: ",trolley_motor_pwm)
            # hoist_motor_pwm = slidingModeController.convert_to_pwm(hoist_motor_control_input)
            gantryMode = CONTROL_MODE
            #Nilai pwm kecepatan trolley 0 674-675
            t_pwm=850
            sp_pos=0.2
            if gantry_crane.variables_value["trolley_position"] <sp_pos:
                trolley_motor_pwm = t_pwm
            elif gantry_crane.variables_value["trolley_position"] >sp_pos+0.03:
                trolley_motor_pwm = -t_pwm
            else:
                trolley_motor_pwm = 0
            #Nilai pwm kecepatan hoist 0 naik 785 turun 490
            if gantry_crane.variables_value["cable_length"] <0.65:
                hoist_motor_pwm = 200
            elif gantry_crane.variables_value["cable_length"] >0.68:
                hoist_motor_pwm = -200
            else:
                hoist_motor_pwm = 0
            hoist_motor_pwm = hoist_motor_pwm
            trolley_motor_pwm = trolley_motor_pwm
            

            slidingModeController.publish_motor_pwm(
                gantryMode, trolley_motor_pwm, hoist_motor_pwm
            )

            # print(
            #     "Trolley position: {} /t cable length: {}, sway angle: {}.".format(
            #         gantry_crane.variables_value["trolley_position"],
            #         gantry_crane.variables_value["cable_length"],
            #         gantry_crane.variables_value["sway_angle"],
            #     )
            # )

            print(
                "Trolley position: {} \t troley motor pwm: {}.".format(
                    gantry_crane.variables_value["trolley_position"],
                    trolley_motor_pwm,
                )
            )

            # print(f"trolley_motor_pwm: {trolley_motor_pwm}, hoist_motor_pwm: {hoist_motor_pwm}")

            rclpy.spin_once(gantry_crane, timeout_sec=0.025)
            rclpy.spin_once(slidingModeController, timeout_sec=0.025)

            collect_data(
                gantry_crane,
                trolley_motor_control_input,
                hoist_motor_control_input,
                time_now,
                trolley_motor_pwm,
                hoist_motor_pwm,
            )

            if time_now > DURATION:
                break

        data = pd.DataFrame(gantry_crane.data_collection_buffer)
        data.to_excel(
            DATA_SAVE_PATH + "sliding_mode_controller_data.xlsx",
            sheet_name="data",
            index=False,
            float_format="%.7f",
        )

        # for i in range(10):
        #     gantryMode = MOVE_TO_ORIGIN_MODE
        #     slidingModeController.publish_motor_pwm(gantryMode, 0, 0)
        #     rclpy.spin_once(slidingModeController, timeout_sec=0.015)
        #     time.sleep(0.05)

    except KeyboardInterrupt:
        pass

    for i in range(10):
        gantryMode = IDLE_MODE
        slidingModeController.publish_motor_pwm(gantryMode, 0, 0)
        rclpy.spin_once(slidingModeController, timeout_sec=0.025)
        time.sleep(0.05)
    
    rclpy.shutdown()
