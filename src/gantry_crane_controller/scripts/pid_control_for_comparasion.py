#!/usr/bin/env python3

import rclpy
from gantry_crane_lib.ros_connector import GantryCraneConnector
from gantry_crane_lib.ros_connector import GantryControlModes

# from dnn_model import DNN

from gantry_crane_lib.logger import Logger

import numpy as np
import json
import time

import pandas as pd
import matplotlib.pyplot as plt

import signal

import torch
from collections import OrderedDict

MODEL_NAME = "non_linear_model"
CONTROLLER_NAME = "pinn_controller"

GANTRY_CRANE_MODEL_PARAMETERS_JSON_PATH = "/home/icodes/Documents/gantry_crane_ws/src/gantry_crane_controller/parameters/gantry_crane_parameters.json"

SLIDING_MODE_CONTROLLER_PARAMETERS_JSON_PATH = "/home/icodes/Documents/gantry_crane_ws/src/gantry_crane_controller/parameters/sliding_mode_controller_parameters.json"

LOG_FOLDER_PATH = (
    "/home/icodes/Documents/gantry_crane_ws/src/gantry_crane_controller/data/"
)

DATA_SAVE_PATH = (
    "/home/icodes/Documents/gantry_crane_ws/src/gantry_crane_controller/data/"
)

KPx = 120
KIx = 150
KDx = 10
Opx = 485
Onx = -450
KPl = 200
KIl = 100
KDl = 2.2
Opl = 705
Onl = -835
MAX_PWM = 850

Apx = 130
Bpx = 482
Anx = 130
Bnx = -482
FEED_FORWARD_X = 90
Apl = 150
Bpl = 705
Anl = 150
Bnl = -835
FEED_FORWARD_L = 0

DAMPING_X = 6.544
DAMPING_L = 2.473155
DAMPING_TETA = 0.12

# Apx = 100
# Bpx = 600
# Anx = 100
# Bnx = -600
# Apl = 50.0
# Bpl = 750
# Anl = 50.0
# Bnl = -600


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
        self.matrix_A[1, 1] = (
            self.L2 * self.mc * self.rp2 / self.Kt2
            + self.J2 * self.L2 / (self.Kt2 * self.rp2)
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
        self.integral_x = 0.0
        self.integral_l = 0.0
        self.prev_error_x = 0.0
        self.prev_error_l = 0.0
        self.control_signal_trolley = 0.0
        self.control_signal_hoist = 0.0
        self.prev_time = 0.0


        # the deep neural network


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

        # Define state
        sway_angle = self.gantry_crane_object.variables_value["sway_angle"]
        trolley_position = self.gantry_crane_object.variables_value["trolley_position"]
        cable_length = self.gantry_crane_object.variables_value["cable_length"]

        # if abs(sway_angle) < 0.01 :
        #     sway_angle = 0.00

        if desired_cable_length is None:
            cable_length = 0.4
            desired_cable_length = cable_length


        # print(
        #     float(sway_angle),
        #     float(trolley_position),
        #     float(cable_length),
        #     float(sway_momentum),
        #     float(trolley_momentum),
        #     float(cable_momentum),
        # )
        # print(stateVector)
        error_x = desired_trolley_position - trolley_position
        error_l = desired_cable_length - cable_length


        if(self.prev_time):
            self.integral_x += error_x * (time.time() - self.prev_time)
            self.integral_l += error_l * (time.time() - self.prev_time)

        self.prev_time = time.time()

        prop_x = KPx * error_x
        prop_l = KPl * error_l

        integral_bound_x = MAX_PWM - prop_x
        integral_bound_l = MAX_PWM - prop_l
        # integral_bound_l = MAX_PWM
        # integral_bound_x = MAX_PWM
        self.integral_x = np.clip(self.integral_x, -integral_bound_x, integral_bound_x)
        self.integral_l = np.clip(self.integral_l, -integral_bound_l, integral_bound_l)

        derivative_x = KDx * (error_x - self.prev_error_x)
        derivative_l = KDl * (error_l - self.prev_error_l)

        self.prev_error_x = error_x
        self.prev_error_l = error_l

        if error_x > 0:
            Ox = Opx
        else:
            Ox = Onx
        
        if error_l > 0:
            Ol = Opl
        else:
            Ol = Onl

        self.pwm_trolley_motor = Ox+prop_x + KIx * self.integral_x + derivative_x
        self.pwm_hoist_motor = Ol+prop_l + KIl * self.integral_l + derivative_l
        self.control_signal_trolley = self.pwm_trolley_motor
        self.control_signal_hoist = self.pwm_hoist_motor

        return self.pwm_trolley_motor, self.pwm_hoist_motor

    def get_momentum(self):
        sway_angle = self.gantry_crane_object.variables_value["sway_angle"]
        trolley_position = self.gantry_crane_object.variables_value["trolley_position"]
        cable_length = self.gantry_crane_object.variables_value["cable_length"]

        # if abs(sway_angle) < 0.01 :
        #     sway_angle = 0.00

        sway_angle_first_derivative = (
            self.gantry_crane_object.variables_first_derivative["sway_angle"]
        )
        trolley_position_first_derivative = (
            self.gantry_crane_object.variables_first_derivative["trolley_position"]
        )
        cable_length_first_derivative = (
            self.gantry_crane_object.variables_first_derivative["cable_length"]
        )

        sway_momentum = self.mp * float(
            cable_length
        ) ** 2 * sway_angle_first_derivative + self.mp * cable_length * trolley_position_first_derivative * np.cos(
            sway_angle
        )
        troley_momentum = (
            self.mp * cable_length * sway_angle_first_derivative * np.cos(sway_angle)
            + (self.mp + self.mt) * trolley_position_first_derivative
            + self.mp * sway_angle_first_derivative * np.sin(sway_angle)
        )
        cable_momentum = (
            self.mp * trolley_position_first_derivative * np.sin(sway_angle)
            + self.mp * cable_length_first_derivative
        )
        # SWAY_MOMENTUM_MAX = SWAY_MOMENTUM_MAX.to(self.device)
        # TROLLEY_MOMENTUM_MAX = TROLLEY_MOMENTUM_MAX.to(self.device)
        # CABLE_MOMENTUM_MAX = CABLE_MOMENTUM_MAX.to(self.device)
        sway_momentum = torch.clip(
            sway_momentum, -self.SWAY_MOMENTUM_MAX, self.SWAY_MOMENTUM_MAX
        )
        troley_momentum = torch.clip(
            troley_momentum, -self.TROLLEY_MOMENTUM_MAX, self.TROLLEY_MOMENTUM_MAX
        )
        cable_momentum = torch.clip(
            cable_momentum, -self.CABLE_MOMENTUM_MAX, self.CABLE_MOMENTUM_MAX
        )

        return sway_momentum, troley_momentum, cable_momentum

    def parsingIDA(self, output_NN):

        R1 = torch.clone(output_NN[1])
        R2 = torch.clone(output_NN[2])

        Ja12 = torch.clone(output_NN[3])
        Ja13 = torch.clone(output_NN[4])
        Ja14 = torch.clone(output_NN[5])

        Ja23 = torch.clone(output_NN[6])
        Ja24 = torch.clone(output_NN[7])

        Ja34 = torch.clone(output_NN[8])

        Ra = torch.zeros((4, 4)).float().to(self.device)
        Ja = torch.zeros((4, 4)).float().to(self.device)

        Ja[0, :] = (
            torch.tensor(
                [
                    0.0,
                    Ja12,
                    Ja13,
                    Ja14,
                ]
            )
            .float()
            .to(self.device)
        )
        Ja[1, :] = torch.tensor([-Ja12, 0.0, Ja23, Ja24]).float().to(self.device)
        Ja[2, :] = torch.tensor([-Ja13, -Ja23, 0.0, Ja34]).float().to(self.device)
        Ja[3, :] = torch.tensor([-Ja14, -Ja24, -Ja34, 0.0]).float().to(self.device)

        Ra[2, 2] = R1
        Ra[3, 3] = R2

        return Ja, Ra

    def convert_force_to_pwm(self, control_signal, desired_trolley_position):
        # print(float(control_signal[4]), float(control_signal[5]))

        interval = time.time() - self.control_timer
        trolley_position_first_derivative = (
            self.gantry_crane_object.variables_first_derivative["trolley_position"]
        )
        cable_length = self.gantry_crane_object.variables_value["cable_length"]
        trolley_position = self.gantry_crane_object.variables_value["trolley_position"]

        if self.control_timer == 0:
            interval = 0.0
        # self.control_signal_trolley += float(control_signal[4]) * interval
        # self.control_signal_hoist += float(control_signal[5]) * interval
        self.control_signal_trolley = float(control_signal[3])
        # self.control_signal_hoist = float(control_signal[5])
        self.control_timer = time.time()
        if self.control_signal_trolley > 0:
            pwm_troley = Apx * self.control_signal_trolley + Bpx
        else:
            pwm_troley = Anx * self.control_signal_trolley + Bnx

        # if self.control_signal_hoist > 0:
        #     pwm_hoist = (
        #         Apl * self.control_signal_hoist
        #         + Bpl
        #         + FEED_FORWARD_L * (0.65 - cable_length)
        #     )
        # else:
        #     pwm_hoist = (
        #         Anl * self.control_signal_hoist
        #         + Bnl
        #         - FEED_FORWARD_L * (0.65 - cable_length)
        #     )
        if (
            abs(trolley_position - desired_trolley_position) > 0.03
        ) and trolley_position_first_derivative == 0:
            pwm_troley += pwm_troley / abs(pwm_troley) * FEED_FORWARD_X

        return float(pwm_troley), float(0.0)

    def linear_interpolation(self, x, x1, y1, x2, y2):
        return y1 + (x - x1) * (y2 - y1) / (x2 - x1)

    def get_control_pwm(self, desired_trolley_position, desired_cable_length=None):
        trolley_pwm, hoist_pwm = self.get_control_signal(
            desired_trolley_position, desired_cable_length
        )
        # trolley_pwm, hoist_pwm = self.convert_force_to_pwm(
        #     trolley_control_input, hoist_control_input
        # )
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
pid_controller = Controller(
    CONTROLLER_NAME, SLIDING_MODE_CONTROLLER_PARAMETERS_JSON_PATH
)
