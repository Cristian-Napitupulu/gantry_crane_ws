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


GANTRY_CRANE_NODE_NAME = "gantry_crane_control_system"
CONTROLLER_NODE_NAME = "sliding_mode_controller"

TROLLEY_POSITION_TOPIC_NAME = "trolley_position"
CABLE_LENGTH_TOPIC_NAME = "cable_length"
SWAY_ANGLE_TOPIC_NAME = "sway_angle"

MOTOR_PWM_TOPIC_NAME = "motor_pwm"

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

MAX_PWM = 1023


class GantryCraneSystem(Node):
    def __init__(self):
        super().__init__(GANTRY_CRANE_NODE_NAME)

        self.subscribers = {}
        self.initialize_subscribers()
        self.initialize_variables()

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

        self.last_variables_message_timestamp = {
            "trolley_position": time.time(),
            "cable_length": time.time(),
            "sway_angle": time.time(),
        }

        self.deltatime = {
            "trolley_position": 0,
            "cable_length": 0,
            "sway_angle": 0,
        }

        self.inittime = {
            "trolley_position": time.time(),
            "cable_length": time.time(),
            "sway_angle": time.time(),
        }

        self.list_variables_value = {
            "trolley_position": [],
            "cable_length": [],
            "sway_angle": [],
        }
        self.list_dvariables_value = {
            "trolley_position": [],
            "cable_length": [],
            "sway_angle": [],
        }
        self.list_d2variables_value = {
            "trolley_position": [],
            "cable_length": [],
            "sway_angle": [],
        }
        self.list_time_value = {
            "trolley_position": [],
            "cable_length": [],
            "sway_angle": [],
        }

    def process_message(self, variable_name, message):
        timestamp = time.time()
        delta_time = timestamp - self.last_variables_message_timestamp[variable_name]
        if variable_name == "trolley_position":
            value = round(message.data, 4)

        if variable_name == "cable_length":
            value = round(message.data, 2)

        if variable_name == "sway_angle":
            value = message.data * np.pi / 180.0
            value = -1 * round(value, 4)

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

        self.last_variables_value[variable_name] = self.variables_value[variable_name]
        self.last_variables_first_derivative[
            variable_name
        ] = self.variables_first_derivative[variable_name]
        self.last_variables_message_timestamp[variable_name] = time.time()
        self.deltatime[variable_name] = delta_time

        self.list_variables_value[variable_name].append(value)
        self.list_dvariables_value[variable_name].append(
            self.variables_first_derivative[variable_name]
        )
        self.list_d2variables_value[variable_name].append(
            self.variables_second_derivative[variable_name]
        )
        timenow = timestamp - self.inittime[variable_name]
        self.list_time_value[variable_name].append(timenow)

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

    def calculate_derivative(self, current_value, last_value, delta_time):
        return (current_value - last_value) / delta_time


class Controller(Node):
    def __init__(self):
        # Initialize node
        super().__init__(CONTROLLER_NODE_NAME)

        # Initialize publisher
        self.motor_pwm_publisher = self.create_publisher(
            UInt32, MOTOR_PWM_TOPIC_NAME, 10
        )
        self.posIntErr = 0
        self.tetIntErr = 0

        self.inittime = time.time()
        self.listtimenow = []
        self.listCS1 = []
        self.listCS2 = []
        self.listPWM_pos = []
        self.listPWM_length = []
        self.listxref = []
        self.listlref = []

        self.sCS1=0
        # Initialize variables

    def packValues(self, mode, pwm_trolley, pwm_hoist):
        # Ensure the values are within the specified ranges
        pwm_trolley = max(min(pwm_trolley, MAX_PWM), -MAX_PWM)
        pwm_hoist = max(min(pwm_hoist, MAX_PWM), -MAX_PWM)
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

    def linear_interpolation(self, x, x1, y1, x2, y2):
        return y1 + (x - x1) * (y2 - y1) / (x2 - x1)
    
    def tanh_interpolation(self, x, x1, y1, x2, y2):
        a=1 #constant on 1, move the graph upward
        s=0.5 #constan on 1 for rescale to 1 maximum
        r=1.8 # 2.2 greater value: move the graph to right
        k= 1 # 0.75#grater value:Increase the steepnes

        tanhres=s*(np.tanh(k*x-r)+a)
        fres=self.linear_interpolation(tanhres,0,y1,1,y2)
        return fres
   
    def get_PID_control_input(self, gantry_crane, x_ref, t, tnow):
        # Kpx = 11.7
        # Kix = 2.5#2.41
        # Kdx = 18.2
        # Kptet = 1.28
        # Kitet = -2.03
        # Kdtet = 3.57

        Kpx = 11.7
        Kix = 0.5#2.41
        Kdx = 1.2
        Kptet = 1.28
        Kitet = -2.03
        Kdtet = 3.57

        x = gantry_crane.variables_value["trolley_position"]
        l = gantry_crane.variables_value["cable_length"]
        tetha = gantry_crane.variables_value["sway_angle"]
        xref_t = x_ref[np.argmin(np.abs(t - tnow))]
        print("xref_t: ", xref_t)
        ex = xref_t - x
        etet = tetha
        dex = gantry_crane.variables_first_derivative["trolley_position"] 
        detet = gantry_crane.variables_first_derivative["sway_angle"] 
        self.posIntErr = self.posIntErr + (
            self.posIntErr * gantry_crane.deltatime["trolley_position"] 
        )
        self.tetIntErr = self.tetIntErr + (
            self.tetIntErr * gantry_crane.deltatime["sway_angle"] 
        )
        # Control Signal Pos
        CS1 = Kpx * ex + Kix * self.posIntErr + Kdx * dex
        # Control Signal Sway
        CS2 = Kptet * etet + Kitet * self.tetIntErr + Kdtet * detet
        control_now = CS1 + CS2
        print("Control now: ", control_now)
        """
        control_input1 = int(
            self.linear_interpolation(control_now, -12.0, -MAX_PWM, 12, MAX_PWM)
        )
        
        if CS1 > 0:
            control_input1 = int(self.tanh_interpolation(control_now, 0, 570, 12, 700))
        elif CS1<0:
            control_input1 = -1 * (int(self.tanh_interpolation(-control_now, 0, 570, 12, 700)))
        else:
            control_input1=0
        """

        if CS1 > 0:
            control_input1 = int(self.linear_interpolation(control_now, 0, 580, 4, 700))
        elif CS1<0:
            control_input1 = -1 * (int(self.linear_interpolation(-control_now, 0, 580, 4, 700)))
        else:
            control_input1=0
        
        if control_input1 > MAX_PWM:
            control_input1 = MAX_PWM
        elif control_input1 < -MAX_PWM:
            control_input1 = -MAX_PWM

        # control_input1 = 0
        control_input2 = 0
        print("Control input 1: ", control_input1)

        self.listtimenow.append(time.time() - self.inittime)
        self.listCS1.append(control_now)
        self.listCS2.append(0)  # tanpa pengontrol l
        self.listPWM_pos.append(control_input1)
        self.listPWM_length.append(control_input2)
        self.listxref.append(xref_t)
        self.listlref.append(0)  # tanpa pengontrol l

        return int(control_input1), int(control_input2)

    def get_lyapunov_control_input(self, gantry_crane, x_ref, l_ref):
        Kpx = 7.36#7.36  # 9.89
        Kdx = 3.71#5.01  # 2.87
        Kpl = 14.18  # 10
        Kdl = 8.98  # 3.13
       

        x = gantry_crane.variables_value["trolley_position"]
        l = gantry_crane.variables_value["cable_length"]
        tetha = gantry_crane.variables_value["sway_angle"]

        ex = x - x_ref
        etet = tetha
        el = l - l_ref

        dex = gantry_crane.variables_first_derivative["trolley_position"] 
        detet = gantry_crane.variables_first_derivative["sway_angle"] 
        dell = gantry_crane.variables_first_derivative["cable_length"]

        d2ex = gantry_crane.variables_second_derivative["trolley_position"] 

        # if abs(ex) < 0.01:
        #    Kdx=0.0005 

        print("kdx: ", Kdx)
        # if abs(ex)<0.05:
        #     CS1=abs(CS1)
        # Control Signal Pos
        CS1 = -Kpx * ex - Kdx * dex 
        # Control Signal CLength
        CS2 = -Kpl * el - Kdl * dell

        

        print(f"Control_X: {CS1}, Control_L={CS2}")
        """
        control_input1 = int(
            self.linear_interpolation(control_now, -12.0, -MAX_PWM, 12, MAX_PWM)
        )
        """
        if (abs(CS1-self.sCS1) < 0.4) and (np.sign(CS1)!=np.sign(self.sCS1)):
            CS1=np.sign(self.sCS1)*abs(CS1)
        if CS1 > 0:
            control_input1 = int(self.tanh_interpolation(CS1, 0, 570, 10, 700))
            #control_input1 = int(self.linear_interpolation(CS1, 0, 570, 20, 700))
        elif CS1<0:
            control_input1 = -1 * (int(self.tanh_interpolation(-CS1, 0, 570, 10, 700)))
            #control_input1 = -1*int(self.linear_interpolation(-CS1, 0, 570, 20, 700))
        else:
            control_input1=0
        
        self.sCS1=CS1

        # if abs(control_input1) > 570 and abs(control_input1) <= 577:
        #     control_input1 = np.sign(control_input1) * 565
            #control_input1 = int(self.linear_interpolation(CS1, 0, 570, 20, 700))
        

        if CS2 > 0:
            control_input2 = int(self.tanh_interpolation(CS2, 0, 250, 5, 500))
        else:
            control_input2 = -1 * (int(self.tanh_interpolation(-CS2, 0, 250, 5, 500)))

        if control_input1 > MAX_PWM:
            control_input1 = MAX_PWM
        elif control_input1 < -MAX_PWM:
            control_input1 = -MAX_PWM

        if control_input2 > MAX_PWM:
            control_input2 = MAX_PWM
        elif control_input2 < -MAX_PWM:
            control_input2 = -MAX_PWM

        # control_input1 = 0
        # control_input2=0
        print(f"Control input 1: {control_input1}, Control input 2: {control_input2}")

        self.listtimenow.append(time.time() - self.inittime)
        self.listCS1.append(CS1)
        self.listCS2.append(CS2)
        self.listPWM_pos.append(control_input1)
        self.listPWM_length.append(control_input2)
        self.listxref.append(x_ref)
        self.listlref.append(l_ref)
        return int(control_input1), int(control_input2)


def x_reference():
    # x_ref model 3th order ITAE
    # still editable and needs some check and validation
    damp_rat = 0.5  # damping ratio
    Ts = 3.5
    w = 4 / (damp_rat * Ts)
    num = w**3
    den = [1, 1.75 * w, 2.15 * w**2, 1.5 * w**3]

    x_ref_tf = trf(num, den)

    x_ref_sys = feedback(x_ref_tf, -1)

    t_sim = np.arange(0, 50, 0.001)
    [x_ref, t] = step(0.5 * x_ref_sys, t_sim)
    teta_ref = np.zeros(len(x_ref))
    return x_ref, teta_ref, t


if __name__ == "__main__":
    x_ref, teta_ref, t = x_reference()
    rclpy.init()
    gantry_crane = GantryCraneSystem()
    slidingModeController = Controller()
    timestamp0 = time.time()
    pengontrol = "PSO_Lyapunov"
    try:
        while 1:
            # PID Robust
            timestamp1 = time.time()
            tnow = timestamp1 - timestamp0
            # (
            #     trolley_motor_pwm,
            #     hoist_motor_pwm,
            # ) = slidingModeController.get_PID_control_input(
            #     gantry_crane, x_ref, t,tnow
            # )

            # Lyapunov Control Action
            x_ref = 0.25
            l_ref = 0.5
            (
                trolley_motor_pwm,
                hoist_motor_pwm,
            ) = slidingModeController.get_lyapunov_control_input(
                gantry_crane, x_ref, l_ref
            )

            gantryMode = CONTROL_MODE
            slidingModeController.publish_motor_pwm(
                gantryMode, trolley_motor_pwm, hoist_motor_pwm
            )

            rclpy.spin_once(gantry_crane, timeout_sec=0.02)
            rclpy.spin_once(slidingModeController, timeout_sec=0.02)

            if tnow > 30:
                break

        datapos = pd.DataFrame(
            {
                "timestamp": gantry_crane.list_time_value["trolley_position"],
                "trolley_position": gantry_crane.list_variables_value[
                    "trolley_position"
                ],
                "trolley_velocity": gantry_crane.list_dvariables_value[
                    "trolley_position"
                ],
                "trolley_acceleration": gantry_crane.list_d2variables_value[
                    "trolley_position"
                ],
            }
        )
        datalcab = pd.DataFrame(
            {
                "timestamp": gantry_crane.list_time_value["cable_length"],
                "cable_length": gantry_crane.list_variables_value["cable_length"],
                "cable_velocity": gantry_crane.list_dvariables_value["cable_length"],
                "cable_acceleration": gantry_crane.list_d2variables_value[
                    "cable_length"
                ],
            }
        )
        datasway = pd.DataFrame(
            {
                "timestamp": gantry_crane.list_time_value["sway_angle"],
                "sway_angle": gantry_crane.list_variables_value["sway_angle"],
                "sway_velocity": gantry_crane.list_dvariables_value["sway_angle"],
                "sway_acceleration": gantry_crane.list_d2variables_value["sway_angle"],
            }
        )

        datapos.to_excel(pengontrol + "position.xlsx", sheet_name="trolley_position")
        datalcab.to_excel(pengontrol + "cable_length.xlsx", sheet_name="cable_length")
        datasway.to_excel(pengontrol + "sway_angle.xlsx", sheet_name="sway_angle")

        timesig = slidingModeController.listtimenow
        Cpos = slidingModeController.listCS1
        Clen = slidingModeController.listCS2
        CPWM_pos = slidingModeController.listPWM_pos
        CPWM_length = slidingModeController.listPWM_length
        dataxref = slidingModeController.listxref
        datalref = slidingModeController.listlref
        datacontrol = {
            "timesig": timesig,
            "Cpos": Cpos,
            "Clen": Clen,
            "CPWM_pos": CPWM_pos,
            "CPWM_length": CPWM_length,
            "dataxref": dataxref,
            "datalref": datalref,
        }
        datacontrol = pd.DataFrame(datacontrol)
        datacontrol.to_excel(pengontrol + "control.xlsx", sheet_name="datacontrol")

        for i in range(20):
            gantryMode = IDLE_MODE
            slidingModeController.publish_motor_pwm(gantryMode, 0, 0)
            rclpy.spin_once(slidingModeController, timeout_sec=0.001)
            time.sleep(0.05)

    except KeyboardInterrupt:
        pass

    rclpy.shutdown()