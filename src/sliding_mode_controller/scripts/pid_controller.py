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
from  control.matlab import tf as trf


GANTRY_CRANE_NODE_NAME = "gantry_crane_control_system"
CONTROLLER_NODE_NAME = "sliding_mode_controller"

TROLLEY_POSITION_TOPIC_NAME = "trolley_position"
CABLE_LENGTH_TOPIC_NAME = "cable_length"
SWAY_ANGLE_TOPIC_NAME = "sway_angle"

MOTOR_PWM_TOPIC_NAME = "motor_pwm"

# Mode
IDLE_MODE = 0x00
MOVE_TO_ORIGIN_MODE = 0x1F
MOVE_TO_MIDDLE_MODE = 0x2F
MOVE_TO_END_MODE = 0x3F
LOCK_CONTAINER_MODE = 0x4F
UNLOCK_CONTAINER_MODE = 0x5F
COLLECT_DATA_MODE = 0xFF
CONTROL_MODE=0x6F

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



    def process_message(self, variable_name, message):
        timestamp = time.time()
        delta_time = timestamp - self.last_variables_message_timestamp[variable_name]
        if variable_name == "trolley_position":
            value = round(message.data, 4)

        if variable_name == "cable_length":
            value = round(message.data, 2)

        if variable_name == "sway_angle":
            value = - message.data * np.pi / 180.0
            value = round(value, 4)

        
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
        self.deltatime[variable_name]=delta_time
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
        self.posIntErr=0
        self.tetIntErr=0
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

    def get_PID_control_input(self,gantry_crane, x_ref, t,tnow):
        Kpx= 49					
        Kix= 56.7
        Kdx= 7.02
        Kptet= 3.68
        Kitet=-0.792
        Kdtet=1.49
        
        x=gantry_crane.variables_value["trolley_position"]
        l=gantry_crane.variables_value["cable_length"]
        tetha=gantry_crane.variables_value["sway_angle"]
        xref_t=x_ref[np.argmin(np.abs(t-tnow))]
        ex=xref_t-x
        print("ex: ", ex)
        etet=tetha
        dex=gantry_crane.variables_first_derivative["trolley_position"]/100000
        detet=gantry_crane.variables_first_derivative["sway_angle"]/100000
        self.posIntErr=self.posIntErr+(self.posIntErr*gantry_crane.deltatime["trolley_position"]/100000)
        self.tetIntErr=self.tetIntErr+(self.tetIntErr*gantry_crane.deltatime["sway_angle"]/100000)
        #Control Signal Pos
        CS1=Kpx*ex + Kix*self.posIntErr + Kdx*dex
        #Control Signal Sway
        CS2=Kptet*etet + Kitet*self.tetIntErr +Kdtet*detet
        control_now=CS1 + CS2
        print("Control now: ", control_now)
        control_input1 = int(
            self.linear_interpolation(control_now, -12.0, -MAX_PWM, 12, MAX_PWM)
        )
        if control_input1 > MAX_PWM:
            control_input1 = MAX_PWM
        elif control_input1 < -MAX_PWM:
            control_input1 = -MAX_PWM

        control_input1 = 400
        control_input2=0
        print("Control input 1: ", control_input1)
        return int(control_input1), int(control_input2)

        
def x_reference():
        #x_ref model 3th order ITAE
        #still editable and needs some check and validation
        damp_rat = 0.5         #damping ratio
        Ts = 3.5                   
        w = 4/(damp_rat*Ts)
        num = w**3
        den = [1,1.75*w, 2.15*w**2, 1.5*w**3]

        x_ref_tf = trf(num,den)

        x_ref_sys = feedback(x_ref_tf,-1)

        t_sim = np.arange(0,50,0.001)
        [x_ref,t] = step(0.5*x_ref_sys,t_sim)
        teta_ref=np.zeros(len(x_ref))
        return x_ref,teta_ref, t

if __name__ == "__main__":
    x_ref,teta_ref, t=x_reference()
    rclpy.init()
    gantry_crane = GantryCraneSystem()
    slidingModeController = Controller()
    timestamp0=time.time()
    try:
        while 1:
            # Update matrices
            #gantry_crane.update_matrices()
            timestamp1=time.time()
            tnow=timestamp1-timestamp0
            (
                trolley_motor_pwm,
                hoist_motor_pwm,
            ) = slidingModeController.get_PID_control_input(
                gantry_crane, x_ref, t,tnow
            )

            gantryMode = CONTROL_MODE
            slidingModeController.publish_motor_pwm(
                gantryMode, trolley_motor_pwm, hoist_motor_pwm
            )

            rclpy.spin_once(gantry_crane, timeout_sec=0.02)
            rclpy.spin_once(slidingModeController, timeout_sec=0.02)

            if tnow>15:
                break
        
        slidingModeController.publish_motor_pwm(
            IDLE_MODE, 0, 0
        )

    except KeyboardInterrupt:
        pass

    rclpy.shutdown()