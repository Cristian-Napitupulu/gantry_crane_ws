#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import UInt32

import numpy as np
import json
import time

from control import *
from control.matlab import *
from control.matlab import tf as trf

import pandas as pd

from pyit2fls import TSK, IT2FS_Gaussian_UncertStd, IT2FS_plot, \
                     product_t_norm, max_s_norm

from numpy import array, linspace

from gantry_crane_lib.ros_connector import GantryCraneConnector, GantryControlModes
from gantry_crane_lib.logger import Logger

DESIRED_TROLLEY_POSITION = 1.0
DESIRED_CABLE_LENGTH = 0.5


LOG_FOLDER_PATH = (
    "/home/icodes/Documents/gantry_crane_ws/src/gantry_crane_controller/log"
)

MODEL_NAME = "force_control_model"
CONTROLLER_NAME = "lyapunov_mode_controller"

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
        self.matrix_A=None
        self.matrix_B=None
        self.matrix_G=None
    def get_parameter(self, parameter_json_path=None):
        if parameter_json_path is not None:
            with open(parameter_json_path, "r") as file:
                parameters = json.load(file)

        self.mc = 1
        self.mt = 2
        self.g = 9.81
        self.bt = 2
        self.br = 2

    def update_matrix_A(self):
        self.matrix_A=None
    def update_matrix_B(self):
        self.matrix_B=None
    def update_matrix_G(self):
        self.matrix_G=None 
    def update_model(self):
        self.update_matrix_A()
        self.update_matrix_B()
        self.update_matrix_G() 

def fuzzytype2(u,e,tet,u0=575):
    domain1 = linspace(0., 5., 100)
    Z1 = IT2FS_Gaussian_UncertStd(domain1, [0, 0.4, 0.2, 1.])
    S1 = IT2FS_Gaussian_UncertStd(domain1, [1.5, 0.3, 0.2, 1.])
    B1 = IT2FS_Gaussian_UncertStd(domain1, [5, 1.5, 0.3, 1.])
    
    domain2=linspace(0,1,100)
    Z2 = IT2FS_Gaussian_UncertStd(domain2, [0, 0.003, 0.001, 1.])
    S2 = IT2FS_Gaussian_UncertStd(domain2, [0.4, 0.2, 0.1, 1.])
    B2 = IT2FS_Gaussian_UncertStd(domain2, [1, 0.2, 0.1, 1.])

    domain3 = linspace(0., 1.0, 100)
    Z3 = IT2FS_Gaussian_UncertStd(domain3, [0, 0.004, 0.001, 1.])
    B3 = IT2FS_Gaussian_UncertStd(domain3, [1, 0.5, 0.1, 1.])

    myIT2FLS = TSK(product_t_norm, max_s_norm)
    myIT2FLS.add_input_variable("u")
    myIT2FLS.add_input_variable("e")
    myIT2FLS.add_input_variable("tet")
    myIT2FLS.add_output_variable("un")

    myIT2FLS.add_rule([("u", B1)], [("un", {"const":u0, "u":25., "e":0.,"tet":0})])
    myIT2FLS.add_rule([("u", S1)], [("un", {"const":u0, "u":25., "e":0.,"tet":0})])
    myIT2FLS.add_rule([("u", Z1), ("e", S2)], [("un", {"const":u0, "u":25., "e":0.,"tet":0})])
    myIT2FLS.add_rule([("u", Z1), ("e", B2)], [("un", {"const":u0, "u":25., "e":0.,"tet":0})])
    myIT2FLS.add_rule([("u", Z1), ("tet", B3)], [("un", {"const":u0, "u":25., "e":0.,"tet":0})])
    myIT2FLS.add_rule([("u", Z1), ("e", Z2), ("tet", Z3)], [("un", {"const":0, "u":0., "e":0.,"tet":0})])

    e = max(0, min(1, abs(e)))
    tet = max(0, min(1, abs(tet)))
    u = max(0, min(5, abs(u)))
    res=myIT2FLS.evaluate({"u":u, "e":e, "tet":tet})
    return res['un']

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

    def get_parameter(self, parameter_json_path=None):
        if parameter_json_path is not None:
            with open(parameter_json_path, "r") as file:
                parameters = json.load(file)
        self.Kpx = 7.36#7.36  # 9.89
        self.Kdx = 5.01#5.01  # 2.87
        self.Kpl = 14.18  # 10
        self.Kdl = 8.98  # 3.13

    def get_control_signal(self, desired_trolley_position, desired_cable_length=None):
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

        self.ex=self.connector.variables_value["trolley_position"]-desired_trolley_position
        dex=self.connector.variables_first_derivative["trolley_position"]
        self.el=self.connector.variables_value["cable_length"]-desired_cable_length_
        dell=self.connector.variables_first_derivative["cable_length"]
        tetha=self.connector.variables_value["sway_angle"]

        # Control Signal Pos
        CS1 = -self.Kpx * self.ex - self.Kdx * dex 
        print(f"Control_X: {CS1},e_x: {self.ex}, tet={tetha}")

        # Control Signal CLength
        CS2 = -self.Kpl * self.el - self.Kdl * dell

        self.control_signal_trolley = CS1
        self.control_signal_hoist = CS2

        # time.sleep(1)
     
        print("Control now: {}, {}.".format(self.control_signal_trolley, self.control_signal_hoist))
        if desired_cable_length is None:
            self.control_signal_hoist = 0.0

        return self.control_signal_trolley, self.control_signal_hoist

    def convert_to_pwm(self, trolley_control_input, hoist_control_input=None):
        tetha=self.connector.variables_value["sway_angle"]

        ufuzzx=fuzzytype2(trolley_control_input,self.ex,tetha,578)      
        pwm_trolley= np.sign(trolley_control_input)*ufuzzx

        if np.sign(hoist_control_input)==-1:
            ufuzzl=fuzzytype2(hoist_control_input,self.el,tetha,650) 
        else:
            ufuzzl=fuzzytype2(hoist_control_input,self.el,tetha,520)   
        pwm_hoist= np.sign(hoist_control_input)*ufuzzl
        
        
        self.pwm_trolley_motor = pwm_trolley
        self.pwm_hoist_motor = pwm_hoist

        return self.pwm_trolley_motor, self.pwm_hoist_motor
    
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

lyapunov_controller = Controller(CONTROLLER_NAME)

gantry_crane_model = force_control_model
gantry_crane_controller = lyapunov_controller


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
    timeout_sec=50.0,
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


DURATION = 50
DESIRED_TROLLEY_POSITION = 1.0
DESIRED_CABLE_LENGTH = 0.4

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
            #desired_cable_length=None,
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
            #desired_cable_length=None,
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
        # result = control_gantry_crane(
        #     timeout_sec=DURATION,
        #     desired_trolley_position=1.25,
        #     desired_cable_length=0.5,
        #     max_trolley_position_steady_state_error=0.01,
        #     max_cable_length_steady_state_error=0.01,
        # )

        # result = control_gantry_crane(
        #     timeout_sec=DURATION,
        #     desired_trolley_position=0.25,
        #     desired_cable_length=0.3,
        #     max_trolley_position_steady_state_error=0.01,
        #     max_cable_length_steady_state_error=0.01,
        # )

        gantry_crane_logger.write_buffers_to_excel(
            gantry_crane_controller.get_name() + "_data.xlsx"
        )

        create_plot()

        # gantry_crane_logger.reset_timer()
        # start_time = time.time()
        # while True:
        #     time_now = time.time() - start_time
        #     # Update matrices
        #     gantry_crane_model.update_model()
        #     (
        #         trolley_motor_control_input,
        #         hoist_motor_control_input,
        #     ) = gantry_crane_controller.get_control_signal(
        #         DESIRED_TROLLEY_POSITION, DESIRED_CABLE_LENGTH
        #     )

        #     trolley_motor_pwm, hoist_motor_pwm = gantry_crane_controller.convert_to_pwm(
        #         trolley_motor_control_input, hoist_motor_control_input
        #     )

        #     if Gantry_Crane_Connector.variables_value["trolley_position"] <0.95:
        #         trolley_motor_pwm = 600
        #     elif Gantry_Crane_Connector.variables_value["trolley_position"] >1.5:
        #         trolley_motor_pwm = -600
        #     else:
        #         trolley_motor_pwm = 0
        #     if Gantry_Crane_Connector.variables_value["cable_length"] <0.5:
        #         hoist_motor_pwm = 590
        #     elif Gantry_Crane_Connector.variables_value["cable_length"] >0.6:
        #         hoist_motor_pwm = -650
        #     else:
        #         hoist_motor_pwm = 0

        #     hoist_motor_pwm = 0
        #     trolley_motor_pwm = 0
        #     gantryMode = GantryControlModes.CONTROL_MODE

        #     Gantry_Crane_Connector.publish_command(
        #         gantryMode, trolley_motor_pwm, hoist_motor_pwm
        #     )

        #     collect_data()

        #     rclpy.spin_once(Gantry_Crane_Connector, timeout_sec=0.01)

        #     if time_now > DURATION:
        #         break

        # gantry_crane_logger.write_buffers_to_excel(
        #     gantry_crane_controller.get_name() + "_data.xlsx"
        # )

        # create_plot()

    except KeyboardInterrupt:
        pass

    '''
    Fungsi ini digunakan untuk mengembalikan gantry crane ke posisi awal.
    '''
    Gantry_Crane_Connector.move_trolley_to_origin()
    Gantry_Crane_Connector.hoist_container_to_middle()

    rclpy.shutdown()
