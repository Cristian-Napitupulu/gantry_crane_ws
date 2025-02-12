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

MODEL_PATH = "src/gantry_crane_controller/pinn_models/Model FINAL 18 jun.pt"

Apx = 130
Bpx = 485
Anx = 130
Bnx = -485
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

        self.initialize_NN()

        self.get_parameter(parameter_path)

        self.initialize_variables()

    def initialize_variables(self):
        self.control_signal_trolley = 0.0
        self.control_signal_hoist = 0.0
        self.pwm_trolley_motor = 0
        self.pwm_hoist_motor = 0
        self.control_timer = 0.0

        # the deep neural network

    def initialize_NN(self):

        self.device = torch.device("cpu")
        if torch.cuda.is_available():
            self.device = torch.device("cuda")

        self.layers = [8, 20, 20, 20, 19]
        # self.PINN_model = DNN(self.layers)

        self.PINN_model = torch.jit.load(MODEL_PATH, map_location=self.device)

        self.SWAY_MOMENTUM_MAX = torch.tensor(1.5).to(self.device)
        self.TROLLEY_MOMENTUM_MAX = torch.tensor(2.5).to(self.device)
        self.CABLE_MOMENTUM_MAX = torch.tensor(2).to(self.device)

    def get_name(self):
        return self.name

    def attach_connector(self, gantry_crane_connector):
        # Get connector to get variables
        self.gantry_crane_object = gantry_crane_connector

    def attach_model(self, non_linear_motor_model):
        # Get model to get matrices for control input calculation
        self.model_gantry_crane = non_linear_motor_model

    def Hamiltonian(self, Xf):

        tet = Xf[0]
        x = Xf[1]
        l = Xf[2]
        ptet = Xf[3]
        px = Xf[4]
        pl = Xf[5]

        M00 = self.mp * torch.square(l)
        M00 = M00.reshape(1)
        M01 = self.mp * torch.mul(l, torch.cos(tet))
        M01 = M01.reshape(1)
        M02 = torch.zeros(tet.shape).float().to(self.device)
        M02 = M02.reshape(1)

        M10 = self.mp * torch.mul(l, torch.cos(tet))
        # M10=torch.reshape(M10,(tet.shape[0],1))
        M11 = torch.Tensor(self.mt + self.mp)
        # M11=torch.reshape(M11,(tet.shape[0],1))
        M12 = self.mp * torch.sin(tet)
        # M12=torch.reshape(M12,(tet.shape[0],1))

        M20 = torch.zeros(tet.shape).float().to(self.device)
        # M20=torch.reshape(M20,(tet.shape[0],1))
        M21 = self.mp * torch.sin(tet)
        # M21=torch.reshape(M21,(tet.shape[0],1))
        M22 = torch.Tensor(self.mp)
        # M22=torch.reshape(M22,(tet.shape[0],1))
        M10 = M10.reshape(1)
        M11 = M11.reshape(1)
        M12 = M12.reshape(1)
        M20 = M20.reshape(1)
        M21 = M21.reshape(1)
        M22 = M22.reshape(1)

        # print(M00.shape)
        # Mass Matrix and Its inverse

        M0 = torch.cat([M00, M01, M02])
        M1 = torch.cat([M10, M11, M12])
        M2 = torch.cat([M20, M21, M22])
        M = torch.cat([M0, M1, M2])
        M = torch.reshape(M, (3, 3))

        Minv = torch.linalg.inv(M)
        ptet = ptet.reshape(1)
        px = px.reshape(1)
        pl = pl.reshape(1)
        p = torch.cat([ptet, px, pl], 0)
        p = torch.reshape(p, (1, 3))

        pT = torch.t(p)
        # print(Minv)
        # print(p)

        Hm = self.hlf * torch.matmul(
            (torch.matmul(p, Minv)), pT
        ) + self.mp * self.gr * l * (self.uni - torch.cos(tet))

        return Hm

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
        self.R = torch.zeros((6, 6)).float().to(self.device)
        self.R[3:3] = DAMPING_TETA
        self.R[4:4] = DAMPING_X
        self.R[5:5] = DAMPING_L

        self.J = torch.zeros((6, 6), dtype=torch.float32).to(self.device)
        self.J[0, 3] = torch.Tensor([1.0])
        self.J[1, 4] = torch.Tensor([1.0])
        self.J[2, 5] = torch.Tensor([1.0])
        self.J[3, 0] = torch.Tensor([-1.0])
        self.J[4, 1] = torch.Tensor([-1.0])
        self.J[5, 2] = torch.Tensor([-1.0])

        self.mp = torch.tensor(1.128).to(self.device)
        self.mt = torch.tensor(2.0).to(self.device)
        self.gr = torch.tensor(9.8).to(self.device)
        self.zr = torch.tensor(0.0).to(self.device)
        self.uni = torch.tensor(1.0).to(self.device)
        self.hlf = torch.tensor(0.5).to(self.device)

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
        sway_momentum, trolley_momentum, cable_momentum = self.get_momentum()

        # if abs(sway_angle) < 0.01 :
        #     sway_angle = 0.00

        if desired_cable_length is None:
            cable_length = 0.4
            desired_cable_length = cable_length

        desiredStateVector = torch.tensor(
            [desired_trolley_position, desired_cable_length]
        ).to(self.device)

        # State vector
        # sway_angle = 0.0
        # trolley_momentum = 0.0
        # sway_momentum = 0.0
        stateVector = torch.tensor(
            [
                sway_angle,
                trolley_position,
                cable_length,
                sway_momentum,
                trolley_momentum,
                cable_momentum,
            ]
        ).to(self.device)
        # print(
        #     float(sway_angle),
        #     float(trolley_position),
        #     float(cable_length),
        #     float(sway_momentum),
        #     float(trolley_momentum),
        #     float(cable_momentum),
        # )
        # print(stateVector)
        neural_networks_input = torch.cat([stateVector, desiredStateVector], 0)
        neural_networks_input.requires_grad = True

        neural_networks_output = self.PINN_model(neural_networks_input)

        Hamiltonian_plant = self.Hamiltonian(neural_networks_input[0:6])

        virtual_potential = torch.square(
            neural_networks_input[1] - neural_networks_input[6]
        ) + torch.square(neural_networks_input[2] - neural_networks_input[7])

        Hamiltonian_desired = (
            Hamiltonian_plant + neural_networks_output[0] + virtual_potential
        )

        gradient_of_hamiltonian_plant = torch.autograd.grad(
            Hamiltonian_plant,
            neural_networks_input,
            grad_outputs=torch.ones_like(Hamiltonian_plant),
            retain_graph=True,
            create_graph=True,
        )[0]

        gradient_of_hamiltonian_desired = torch.autograd.grad(
            Hamiltonian_desired,
            neural_networks_input,
            grad_outputs=torch.ones_like(Hamiltonian_desired),
            retain_graph=True,
            create_graph=True,
        )[0]

        interconnection_desired, damping_desired = self.parsingIDA(
            neural_networks_output
        )

        control_signal = torch.matmul(
            (interconnection_desired + self.J - damping_desired + self.R),
            gradient_of_hamiltonian_desired[0:6],
        ) - torch.matmul((self.J - self.R), gradient_of_hamiltonian_plant[0:6])
        # print(gradient_of_hamiltonian_plant, gradient_of_hamiltonian_desired)

        self.pwm_trolley_motor, self.pwm_hoist_motor = self.convert_force_to_pwm(
            control_signal, desired_trolley_position
        )

        if desired_cable_length == cable_length:
            self.pwm_hoist_motor = 0

        # if trolley_position > 1.45:
        #     self.pwm_trolley_motor = 0

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
        R3 = torch.clone(output_NN[3])

        Ja12 = torch.clone(output_NN[4])
        Ja13 = torch.clone(output_NN[5])
        Ja14 = torch.clone(output_NN[6])
        Ja15 = torch.clone(output_NN[7])
        Ja16 = torch.clone(output_NN[8])

        Ja23 = torch.clone(output_NN[9])
        Ja24 = torch.clone(output_NN[10])
        Ja25 = torch.clone(output_NN[11])
        Ja26 = torch.clone(output_NN[12])

        Ja34 = torch.clone(output_NN[13])
        Ja35 = torch.clone(output_NN[14])
        Ja36 = torch.clone(output_NN[15])

        Ja45 = torch.clone(output_NN[16])
        Ja46 = torch.clone(output_NN[17])

        Ja56 = torch.clone(output_NN[18])

        Ra = torch.zeros((6, 6)).float().to(self.device)
        Ja = torch.zeros((6, 6)).float().to(self.device)

        Ja[0, :] = (
            torch.tensor([0.0, Ja12, Ja13, Ja14, Ja15, Ja16]).float().to(self.device)
        )
        Ja[1, :] = (
            torch.tensor([-Ja12, 0.0, Ja23, Ja24, Ja25, Ja26]).float().to(self.device)
        )
        Ja[2, :] = (
            torch.tensor([-Ja13, -Ja23, 0.0, Ja34, Ja35, Ja36]).float().to(self.device)
        )
        Ja[3, :] = (
            torch.tensor([-Ja14, -Ja24, -Ja34, 0.0, Ja45, Ja46]).float().to(self.device)
        )
        Ja[4, :] = (
            torch.tensor([-Ja15, -Ja25, -Ja35, -Ja45, 0.0, Ja56])
            .float()
            .to(self.device)
        )
        Ja[5, :] = (
            torch.tensor([-Ja16, -Ja26, -Ja36, -Ja46, -Ja56, 0.0])
            .float()
            .to(self.device)
        )

        Ra[3, 3] = R1
        Ra[4, 4] = R2
        Ra[5, 5] = R3

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
        self.control_signal_trolley = float(control_signal[4])
        self.control_signal_hoist = float(control_signal[5])
        self.control_timer = time.time()
        if self.control_signal_trolley > 0:
            pwm_troley = Apx * self.control_signal_trolley + Bpx
        else:
            pwm_troley = Anx * self.control_signal_trolley + Bnx

        if self.control_signal_hoist > 0:
            pwm_hoist = (
                Apl * self.control_signal_hoist
                + Bpl
                + FEED_FORWARD_L * (0.65 - cable_length)
            )
        else:
            pwm_hoist = (
                Anl * self.control_signal_hoist
                + Bnl
                - FEED_FORWARD_L * (0.65 - cable_length)
            )
        if (
            abs(trolley_position - desired_trolley_position) > 0.03
        ) and trolley_position_first_derivative == 0:
            pwm_troley += pwm_troley / abs(pwm_troley) * FEED_FORWARD_X

        return float(pwm_troley), float(pwm_hoist)

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
pinn_controller = Controller(
    CONTROLLER_NAME, SLIDING_MODE_CONTROLLER_PARAMETERS_JSON_PATH
)
