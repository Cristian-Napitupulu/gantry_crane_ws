#!/usr/bin/env python3

from gantry_crane_lib.ros_connector import GantryCraneConnector, GantryControlModes
from gantry_crane_lib.logger import Logger

import rclpy
import time

import json

import numpy as np
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

LOG_FOLDER_PATH = (
    "/home/icodes/Documents/gantry_crane_ws/src/gantry_crane_controller/log"
)

MODEL_NAME = "force_control_model"
CONTROLLER_NAME = "fuzzy_sliding_mode_controller"

VARIABLE_TO_LOG = [
    "timestamp", # s
    "control_signal_trolley",
    "control_signal_hoist",
    "pwm_trolley_motor",    # PWM
    "pwm_hoist_motor",    # PWM
    "trolley_motor_voltage",    # Volt
    "hoist_motor_voltage",  # Volt
    "trolley_position", # meter
    "cable_length", # meter
    "sway_angle",   # Radiant
    "trolley_position_first_derivative",    # meter/s
    "cable_length_first_derivative",    # meter/s
    "sway_angle_first_derivative", # Rad/s
    "trolley_position_second_derivative",   # meter/s^2
    "cable_length_second_derivative",   # meter/s^2
    "sway_angle_second_derivative", # Rad/s^2
    "trolley_position_third_derivative",    # meter/s^3
    "cable_length_third_derivative",    # meter/s^3
    "sway_angle_third_derivative", # Rad/s^3
]

class GantryCraneModel:
    def __init__(self, model_name, parameter_path=None):
        self.name = model_name

        self.get_parameter(parameter_path)
        self.initialize_variables()

    def get_name(self):
        return self.name
    
    def attach_connector(self, gantry_crane_connector):
        self.connector = gantry_crane_connector

    def initialize_variables(self):
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

    def get_parameter(self, parameter_json_path=None):
        if parameter_json_path is not None:
            with open(parameter_json_path, "r") as file:
                parameters = json.load(file)

        self.mc = 1
        self.mt = 2
        self.g = 9.81
        self.bt = 2
        self.br = 2

    # Update Matrix A
    def update_matrix_A(self):
        self.matrix_A[1, 1] = -self.bt / self.mt
        self.matrix_A[1, 3] = -self.br * np.sin(self.connector.variables_value["sway_angle"]) / self.mt
        self.matrix_A[3, 1] = -self.bt * np.sin(self.connector.variables_value["sway_angle"]) / self.mt
        self.matrix_A[3, 3] = -(
            self.mc * (np.sin(self.connector.variables_value["sway_angle"])) ** 2 / self.mt + 1
        ) * (self.br / self.mt)
        self.matrix_A[3, 5] = (
            self.connector.variables_value["cable_length"]
            * self.connector.variables_first_derivative["sway_angle"]
        )
        self.matrix_A[5, 1] = (
            -self.bt
            * np.cos(self.connector.variables_value["sway_angle"])
            / (self.mt * self.connector.variables_value["cable_length"])
        )
        self.matrix_A[5, 3] = (
            -self.br
            * np.sin(self.connector.variables_value["sway_angle"])
            * np.cos(self.connector.variables_value["sway_angle"])
            / (self.mt * self.connector.variables_value["cable_length"])
        )
        self.matrix_A[5, 5] = (
            -2
            * self.connector.variables_first_derivative["cable_length"]
            / self.connector.variables_value["cable_length"]
        )

    # Update Matrix B
    def update_matrix_B(self):
        self.matrix_B[1, 0] = 1 / self.mt
        self.matrix_B[1, 1] = np.sin(self.connector.variables_value["sway_angle"]) / self.mt
        self.matrix_B[3, 0] = np.sin(self.connector.variables_value["sway_angle"]) / self.mt
        self.matrix_B[3, 1] = (
            self.mc * (np.sin(self.connector.variables_value["sway_angle"])) ** 2 / self.mt + 1
        ) / self.mc
        self.matrix_B[5, 0] = np.cos(self.connector.variables_value["sway_angle"]) / (
            self.mt * self.connector.variables_value["cable_length"]
        )
        self.matrix_B[5, 1] = (
            np.sin(self.connector.variables_value["sway_angle"])
            * np.cos(self.connector.variables_value["sway_angle"])
            / (self.mt * self.connector.variables_value["cable_length"])
        )

    # Update Matrix self.g
    def update_matrix_G(self):
        self.matrix_G[1, 0] = 0.0
        self.matrix_G[3, 0] = self.g * np.cos(self.connector.variables_value["sway_angle"])
        self.matrix_G[5, 0] = (
            -self.g
            * np.sin(self.connector.variables_value["sway_angle"])
            / self.connector.variables_value["cable_length"]
        )

    def update_model(self):
        self.update_matrix_A()
        self.update_matrix_B()
        self.update_matrix_G()
        

class Controller:
    def __init__(self, controller_name, parameter_path=None):
        self.name = controller_name

        self.get_parameter(parameter_path)

        self.initialize_variables()

    def get_name(self):
        return self.name
    
    def attach_connector(self, gantry_crane_connector):
        self.connector = gantry_crane_connector

    def attach_model(self, gantry_crane_model):
        self.model = gantry_crane_model

    def initialize_variables(self):
        self.control_signal_trolley = 0.0
        self.control_signal_hoist = 0.0
        self.pwm_trolley_motor = 0
        self.pwm_hoist_motor = 0

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

    def get_parameter(self, parameter_json_path=None):
        if parameter_json_path is not None:
            with open(parameter_json_path, "r") as file:
                parameters = json.load(file)

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

    def get_control_signal(self, desired_trolley_position, desired_cable_length=None):
        # Modify as needed
        desired_cable_length_ = desired_cable_length
        if desired_cable_length is None:
            desired_cable_length_ = self.connector.variables_value[
                "cable_length"
            ]

        desiredStateVector = np.matrix(
            [[desired_trolley_position], 
             [desired_cable_length_]]
        )
        # print("Desired state vector: ", desiredStateVector)

        # State vector
        stateVector = np.matrix(
            [
                [self.connector.variables_value["trolley_position"]],
                [self.connector.variables_value["cable_length"]],
            ]
        )

        # print("State vector: ", stateVector)

        stateVectorFirstDerivative = np.matrix(
            [
                [self.connector.variables_first_derivative["trolley_position"]],
                [self.connector.variables_first_derivative["cable_length"]],
            ]
        )
        # print("State vector first derivative: ", stateVectorFirstDerivative)

        stateVectorSecondDerivative = np.matrix(
            [
                [self.connector.variables_second_derivative["trolley_position"]],
                [self.connector.variables_second_derivative["cable_length"]],
            ]
        )

        # Matrix A, B, self.g cap
        matrix_A1_cap = np.matrix(
            [
                [self.model.matrix_A[1, 1], self.model.matrix_A[1, 3]],
                [self.model.matrix_A[3, 1], self.model.matrix_A[3, 3]],
            ]
        )
        matrix_A2_cap = np.matrix(
            [
                [self.model.matrix_A[1, 5]],
                [self.model.matrix_A[3, 5]],
            ]
        )
        matrix_B_cap = np.matrix(
            [
                [self.model.matrix_B[1, 0], self.model.matrix_B[1, 1]],
                [self.model.matrix_B[3, 0], self.model.matrix_B[3, 1]],
            ]
        )

        matrix_G_cap = np.matrix(
            [[self.model.matrix_G[1, 0]], [self.model.matrix_G[3, 0]]]
        )

        # Create dummy matrix for sliding surface
        # sliding_surface_now = (
        #     self.matrix_lambda * (desiredStateVector - stateVector)
        #     - self.matrix_I * stateVectorFirstDerivative
        #     - self.matrix_alpha * self.connector.variables_value["sway_angle"]
        # )
        sliding_surface_now = (
            self.matrix_lambda * (desiredStateVector - stateVector)
            - self.matrix_I * stateVectorFirstDerivative
            - self.matrix_alpha * self.connector.variables_value["sway_angle"]
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
            * self.connector.variables_value["sway_angle"]
            + np.multiply(k, np.sign(sliding_surface_now))
        )

        # time.sleep(0.2)

        # s_trolley_position = (
        #     self.connector.variables_value["trolley_position"]
        #     - desired_trolley_position
        # )

        # s_cable_length = self.connector.variables_value["cable_length"] - desired_cable_length

        # control_input1 = - 120 * np.tanh(s_trolley_position)
        # if self.connector.variables_value["trolley_position"] > 1.0:
        #     control_input1 = 0
        # else:

        self.control_signal_trolley = control_now[0, 0]
        self.control_signal_hoist = control_now[1, 0]

        # time.sleep(1)
     
        print("Control now: {}, {}.".format(self.control_signal_trolley, self.control_signal_hoist))
        if desired_cable_length is None:
            self.control_signal_hoist = 0.0

        return self.control_signal_trolley, self.control_signal_hoist

    def convert_to_pwm(self, trolley_control_input, hoist_control_input=None):
        MAX_PWM = 750
        FORWARD_DEADZONE_PWM = 575
        REVERSE_DEADZONE_PWM = 575

        if trolley_control_input > 0:
            pwm_trolley = int(
                self.linear_interpolation(
                    trolley_control_input, 0.0, FORWARD_DEADZONE_PWM, 4, MAX_PWM
                )
            )

        elif trolley_control_input < 0:
            pwm_trolley = int(
                self.linear_interpolation(
                    trolley_control_input, 0.0, -REVERSE_DEADZONE_PWM, -5, -MAX_PWM
                )
            )

        else:
            pwm_trolley = 0

        if hoist_control_input is None:
            pwm_hoist = 0
        elif hoist_control_input > 0:
            pwm_hoist = int(
                self.linear_interpolation(
                    hoist_control_input, 0.0, FORWARD_DEADZONE_PWM, 4, MAX_PWM
                )
            )
        elif hoist_control_input < 0:
            pwm_hoist = int(
                self.linear_interpolation(
                    hoist_control_input, 0.0, -REVERSE_DEADZONE_PWM, -5, -MAX_PWM
                )
            )
        else:
            pwm_hoist = 0

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
    

Gantry_Crane_Connector = GantryCraneConnector()

gantry_crane_logger = Logger(LOG_FOLDER_PATH)

force_control_model = GantryCraneModel(MODEL_NAME)

fuzzy_sliding_mode_controller = Controller(CONTROLLER_NAME)

gantry_crane_model = force_control_model
gantry_crane_controller = fuzzy_sliding_mode_controller

def collect_data():
    '''
    Ini adalah fungsi global untuk mengumpulkan data.
    Data yang dikumpulkan adalah data yang ada di dalam buffer milik logger.
    Jika ingin mengumpulkan data baru, silahkan menambahkan fungsi add_to_buffer baru.
    Cek "VARIABLE_TO_LOG" untuk melihat variabel apa saja yang bisa dikumpulkan.
    '''
    gantry_crane_logger.add_to_buffer("timestamp", gantry_crane_logger.get_time_sec())

    gantry_crane_logger.add_to_buffer(
        "control_signal_trolley", gantry_crane_controller.control_signal_trolley
    )

    gantry_crane_logger.add_to_buffer(
        "control_signal_hoist", gantry_crane_controller.control_signal_hoist
    )

    gantry_crane_logger.add_to_buffer(
        "pwm_trolley_motor", Gantry_Crane_Connector.control_pwm["trolley_motor"]
    )

    gantry_crane_logger.add_to_buffer(
        "pwm_hoist_motor", Gantry_Crane_Connector.control_pwm["hoist_motor"]
    )

    gantry_crane_logger.add_to_buffer(
        "trolley_motor_voltage",
        Gantry_Crane_Connector.variables_value["trolley_motor_voltage"],
    )

    gantry_crane_logger.add_to_buffer(
        "hoist_motor_voltage",
        Gantry_Crane_Connector.variables_value["hoist_motor_voltage"],
    )

    gantry_crane_logger.add_to_buffer(
        "trolley_position", Gantry_Crane_Connector.variables_value["trolley_position"]
    )

    gantry_crane_logger.add_to_buffer(
        "cable_length", Gantry_Crane_Connector.variables_value["cable_length"]
    )

    gantry_crane_logger.add_to_buffer(
        "sway_angle", Gantry_Crane_Connector.variables_value["sway_angle"]
    )

    gantry_crane_logger.add_to_buffer(
        "trolley_position_first_derivative",
        Gantry_Crane_Connector.variables_first_derivative["trolley_position"],
    )

    gantry_crane_logger.add_to_buffer(
        "cable_length_first_derivative",
        Gantry_Crane_Connector.variables_first_derivative["cable_length"],
    )

    gantry_crane_logger.add_to_buffer(
        "sway_angle_first_derivative",
        Gantry_Crane_Connector.variables_first_derivative["sway_angle"],
    )

    gantry_crane_logger.add_to_buffer(
        "trolley_position_second_derivative",
        Gantry_Crane_Connector.variables_second_derivative["trolley_position"],
    )

    gantry_crane_logger.add_to_buffer(
        "cable_length_second_derivative",
        Gantry_Crane_Connector.variables_second_derivative["cable_length"],
    )

    gantry_crane_logger.add_to_buffer(
        "sway_angle_second_derivative",
        Gantry_Crane_Connector.variables_second_derivative["sway_angle"],
    )

    gantry_crane_logger.add_to_buffer(
        "trolley_position_third_derivative",
        Gantry_Crane_Connector.variables_third_derivative["trolley_position"],
    )

    gantry_crane_logger.add_to_buffer(
        "cable_length_third_derivative",
        Gantry_Crane_Connector.variables_third_derivative["cable_length"],
    )

    gantry_crane_logger.add_to_buffer(
        "sway_angle_third_derivative",
        Gantry_Crane_Connector.variables_third_derivative["sway_angle"],
    )

def create_plot():
    '''
    Ini adalah fungsi global untuk membuat plot.
    Plot yang dibuat adalah plot yang ada di dalam buffer milik logger.
    Jika ingin membuat plot baru, silahkan menambahkan fungsi create_plot baru.
    Cek "VARIABLE_TO_LOG" untuk melihat variabel apa saja yang bisa diplot.
    '''
    gantry_crane_logger.create_plot(
        "control_signal_trolley", "timestamp", "control_signal_trolley"
    )
    gantry_crane_logger.create_plot(
        "control_signal_hoist", "timestamp", "control_signal_hoist"
    )

    gantry_crane_logger.create_plot(
        "pwm_trolley_motor", "timestamp", "pwm_trolley_motor"
    )
    gantry_crane_logger.create_plot("pwm_hoist_motor", "timestamp", "pwm_hoist_motor")

    gantry_crane_logger.create_plot(
        "trolley_motor_voltage", "timestamp", "trolley_motor_voltage"
    )
    gantry_crane_logger.create_plot(
        "hoist_motor_voltage", "timestamp", "hoist_motor_voltage"
    )

    gantry_crane_logger.create_plot("trolley_position", "timestamp", "trolley_position")
    gantry_crane_logger.create_plot("cable_length", "timestamp", "cable_length")
    gantry_crane_logger.create_plot("sway_angle", "timestamp", "sway_angle")

    gantry_crane_logger.create_plot(
        "trolley_position_first_derivative",
        "timestamp",
        "trolley_position_first_derivative",
    )

    gantry_crane_logger.create_plot(
        "cable_length_first_derivative",
        "timestamp",
        "cable_length_first_derivative",
    )

    gantry_crane_logger.create_plot(
        "sway_angle_first_derivative",
        "timestamp",
        "sway_angle_first_derivative",
    )

    gantry_crane_logger.create_plot(
        "trolley_position_second_derivative",
        "timestamp",
        "trolley_position_second_derivative",
    )

    gantry_crane_logger.create_plot(
        "cable_length_second_derivative",
        "timestamp",
        "cable_length_second_derivative",
    )

    gantry_crane_logger.create_plot(
        "trolley_motor_voltage vs trolley_position",
        "trolley_motor_voltage",
        "trolley_position",
    )

    gantry_crane_logger.create_plot(
        "hoist_motor_voltage vs cable_length",
        "hoist_motor_voltage",
        "cable_length",
    )

    gantry_crane_logger.create_plot(
        "troley_motor_voltage vs trolley_speed",
        "trolley_motor_voltage",
        "trolley_position_first_derivative",
    )

    gantry_crane_logger.create_plot(
        "hoist_motor_voltage vs cable_speed",
        "hoist_motor_voltage",
        "cable_length_first_derivative",
    )

def send_command_and_collect_data(
    mode=GantryControlModes.IDLE_MODE, trolley_pwm=0, hoist_pwm=0
):
    '''
    Ini adalah fungsi global untuk mengirimkan command ke gantry crane dan mengumpulkan data.
    Data yang dikumpulkan adalah data yang ada di dalam buffer milik logger.
    '''
    # Print current PWM
    print("trolley motor pwm: {}, hoist motor pwm: {}".format(trolley_pwm, hoist_pwm))

    # Publish motor PWM
    Gantry_Crane_Connector.publish_command(mode, trolley_pwm, hoist_pwm)

    # Collect data
    collect_data()

def control_gantry_crane(
    timeout_sec=15.0,
    desired_trolley_position=1.0,
    desired_cable_length=None,
    max_trolley_position_steady_state_error=0.1,
    max_cable_length_steady_state_error=0.1,
):
    '''
    Ini adalah fungsi global untuk mengontrol gantry crane.
    Fungsi ini akan mengontrol gantry crane sampai gantry crane mencapai posisi yang diinginkan atau timeout.
    Fungsinya akan mengembalikan string yang berisi hasil dari kontrol gantry crane.
    '''
    result = None
    inside_max_error = False
    time_inside_max_error = 0.0
    start_time = time.time()
    while True:
        # Update matrices
        gantry_crane_model.update_model()

        # Generate control signal and convert control signal to PWM
        trolley_motor_pwm, hoist_motor_pwm = gantry_crane_controller.get_control_pwm(
            desired_trolley_position, desired_cable_length
        )

        send_command_and_collect_data(
            GantryControlModes.CONTROL_MODE, trolley_motor_pwm, hoist_motor_pwm
        )

        # Check if timeout
        if time.time() - start_time > timeout_sec:
            result = "Timeout. Final error: {}, {}.".format(
                abs(
                    Gantry_Crane_Connector.variables_value["trolley_position"]
                    - desired_trolley_position
                ),
                abs(
                    Gantry_Crane_Connector.variables_value["cable_length"] - desired_cable_length
                ),
            )
            break

        # Check if gantry crane has reached desired position and cable length for 5 seconds
        if inside_max_error and time.time() - time_inside_max_error > 5:
            result = "Desired position reached. Final error: {}, {}.".format(
                abs(
                    Gantry_Crane_Connector.variables_value["trolley_position"]
                    - desired_trolley_position
                ),
                abs(
                    Gantry_Crane_Connector.variables_value["cable_length"] - desired_cable_length
                ),
            )
            break

        # Handle if desired_cable_length is None (only control trolley position)
        if desired_cable_length is None:
            # Check if gantry crane has reached desired position
            if (
                abs(
                    Gantry_Crane_Connector.variables_value["trolley_position"]
                    - desired_trolley_position
                )
                < max_trolley_position_steady_state_error
            ):
                inside_max_error = True
                time_inside_max_error = time.time()

            else:
                inside_max_error = False
        
        # Handle if desired_cable_length is not None (control trolley position and cable length)
        else:
            # Check if gantry crane has reached desired position and cable length
            if (
                abs(
                    Gantry_Crane_Connector.variables_value["trolley_position"]
                    - desired_trolley_position
                )
                < max_trolley_position_steady_state_error
                and abs(
                    Gantry_Crane_Connector.variables_value["cable_length"] - desired_cable_length
                )
                < max_cable_length_steady_state_error
            ):
                inside_max_error = True
                time_inside_max_error = time.time()

            else:
                inside_max_error = False

    return result


DURATION = 20
DESIRED_TROLLEY_POSITION = 1.0
DESIRED_CABLE_LENGTH = 0.55

if __name__ == "__main__":
    gantry_crane_model.attach_connector(Gantry_Crane_Connector)
    print("Gantry crane model initialized: {}".format(gantry_crane_model.get_name()))

    gantry_crane_controller.attach_connector(Gantry_Crane_Connector)
    gantry_crane_controller.attach_model(gantry_crane_model)
    print("Controller initialized: {}".format(gantry_crane_controller.get_name()))

    Gantry_Crane_Connector.begin()
    time.sleep(1)
    try:
        '''
        Fungsi reset_timer akan mengatur ulang timer di dalam logger.
        Ini untuk memastikan bahwa timestamp di dalam logger dimulai dari 0.
        '''
        gantry_crane_logger.reset_timer()

        '''
        Untuk mengontrol gantry crane, gunakan fungsi control_gantry_crane.
        Fungsi ini akan mengontrol gantry crane sampai gantry crane mencapai posisi yang diinginkan atau timeout.
        Input dari fungsi ini adalah:
        - timeout_sec: durasi maksimal untuk mengontrol gantry crane.
        - desired_trolley_position: posisi trolley yang diinginkan.
        - desired_cable_length: panjang kabel yang diinginkan.
        - max_trolley_position_steady_state_error: toleransi error posisi trolley.
        - max_cable_length_steady_state_error: toleransi error panjang kabel.
        Jika desired_cable_length adalah None, maka fungsi ini hanya akan mengontrol posisi trolley.
        Jika dalam durasi timeout_sec, gantry crane tidak mencapai posisi yang diinginkan, maka fungsi ini akan mengembalikan string "timeout".
        '''
        result = control_gantry_crane(
            timeout_sec=DURATION,
            desired_trolley_position=0.375,
            desired_cable_length=0.6,
            max_trolley_position_steady_state_error=0.01,
            max_cable_length_steady_state_error=0.01,
        )

        '''Print result'''
        print("First position reached. Result: {}".format(result))

        '''
        Untuk menambah target posisi, tambahkan fungsi control_gantry_crane baru dengan posisi yang diinginkan.
        '''
        result = control_gantry_crane(
            timeout_sec=DURATION,
            desired_trolley_position=1.0,
            desired_cable_length=0.4,
            max_trolley_position_steady_state_error=0.01,
            max_cable_length_steady_state_error=0.01,
        )

        '''Print result'''
        print("Second position reached. Result: {}".format(result))

        '''
        Fungsi write_buffers_to_excel akan menuliskan data yang ada di dalam buffer milik logger ke dalam file excel.
        Setiap pemanggilan akan menghasilkan file excel baru.
        Silahkan mengganti nama file excel yang dihasilkan dengan nama yang sesuai.
        Silahkan cek folder LOG_FOLDER_PATH untuk melihat file excel yang dihasilkan.
        '''
        gantry_crane_logger.write_buffers_to_excel(
            gantry_crane_controller.get_name() + "_data.xlsx"
        )

        create_plot()

        gantry_crane_logger.reset_timer()
        start_time = time.time()
        while True:
            time_now = time.time() - start_time
            # Update matrices
            # gantry_crane_model.update_model()
            # (
            #     trolley_motor_control_input,
            #     hoist_motor_control_input,
            # ) = gantry_crane_controller.get_control_signal(
            #     DESIRED_TROLLEY_POSITION, DESIRED_CABLE_LENGTH
            # )

            # trolley_motor_pwm, hoist_motor_pwm = gantry_crane_controller.convert_to_pwm(
            #     trolley_motor_control_input, hoist_motor_control_input
            # )

            if Gantry_Crane_Connector.variables_value["trolley_position"] <0.65:
                trolley_motor_pwm = 400
            elif Gantry_Crane_Connector.variables_value["trolley_position"] >1.5:
                trolley_motor_pwm = -400
            else:
                trolley_motor_pwm = 0
            if Gantry_Crane_Connector.variables_value["cable_length"] <0.5:
                hoist_motor_pwm = 590
            elif Gantry_Crane_Connector.variables_value["cable_length"] >0.6:
                hoist_motor_pwm = -650
            else:
                hoist_motor_pwm = 0

            hoist_motor_pwm = 0
            trolley_motor_pwm = 0
            gantryMode = GantryControlModes.CONTROL_MODE

            Gantry_Crane_Connector.publish_command(
                gantryMode, trolley_motor_pwm, hoist_motor_pwm
            )

            collect_data()

            rclpy.spin_once(Gantry_Crane_Connector, timeout_sec=0.01)

            if time_now > DURATION:
                break

        gantry_crane_logger.write_buffers_to_excel(
            gantry_crane_controller.get_name() + "_data.xlsx"
        )

        create_plot()

    except KeyboardInterrupt:
        pass

    '''
    Fungsi ini digunakan untuk mengembalikan gantry crane ke posisi awal.
    '''
    Gantry_Crane_Connector.move_trolley_to_origin()
    Gantry_Crane_Connector.hoist_container_to_middle()

    rclpy.shutdown()
