#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import UInt32

import pandas as pd
import numpy as np
import time

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


class GantryCraneAnalysis(Node):
    def __init__(self):
        super().__init__("gantry_crane_analysis")
        self.get_logger().info("Gantry Crane Analysis Node has been started")

        self.subscribers = {}
        self.initialize_subscribers()

        self.initialize_variable_buffers()

        self.initialize_pwm_publisher()

        while (
            len(self.variable_buffers["trolley_position"]) < 1000
            or len(self.variable_buffers["cable_length"]) < 1000
            or len(self.variable_buffers["sway_angle"]) < 1000
            or len(self.variable_buffers["trolley_motor_voltage"]) < 1000
            or len(self.variable_buffers["hoist_motor_voltage"]) < 1000
        ):
            time.sleep(0.1)

        self.calculate_variable_derivatives()

    def initialize_subscribers(self):
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

    def trolley_position_callback(self, message):
        data = round(message.data, 5)
        self.process_message(data, "trolley_position")

    def cable_length_callback(self, message):
        data = round(message.data, 3)
        self.process_message(data, "cable_length")

    def sway_angle_callback(self, message):
        data = round(message.data * np.pi / 180, 3)
        self.process_message(data, "sway_angle")

    def trolley_motor_voltage_callback(self, message):
        data = round(message.data, 3)
        self.process_message(data, "trolley_motor_voltage")

    def hoist_motor_voltage_callback(self, message):
        data = round(message.data, 3)
        self.process_message(data, "hoist_motor_voltage")

    def calculate_derivative(self, current_value, last_value, delta_time):
        return (current_value - last_value) / delta_time

    def process_message(self, value, variable_name):
        self.variable_buffers[variable_name].append(value)
        self.variable_timestamp_buffers[variable_name].append(time.time())

    def initialize_variable_buffers(self):
        self.variable_buffers = {
            "trolley_position": [],
            "cable_length": [],
            "sway_angle": [],
            "trolley_motor_voltage": [],
            "hoist_motor_voltage": [],
        }
        self.variable_timestamp_buffers = {
            "trolley_position": [],
            "cable_length": [],
            "sway_angle": [],
            "trolley_motor_voltage": [],
            "hoist_motor_voltage": [],
        }

    def calculate_variable_derivatives(self):
        self.variable_first_derivatives = {
            "trolley_position": [],
            "cable_length": [],
            "sway_angle": [],
            "trolley_motor_voltage": [],
            "hoist_motor_voltage": [],
        }
        self.variable_second_derivatives = {
            "trolley_position": [],
            "cable_length": [],
            "sway_angle": [],
            "trolley_motor_voltage": [],
            "hoist_motor_voltage": [],
        }
        self.variable_third_derivatives = {
            "trolley_position": [],
            "cable_length": [],
            "sway_angle": [],
            "trolley_motor_voltage": [],
            "hoist_motor_voltage": [],
        }

        for variable_name in self.variable_buffers:
            for i in range(1, len(self.variable_buffers[variable_name])):
                current_value = self.variable_buffers[variable_name][i]
                last_value = self.variable_buffers[variable_name][i - 1]
                delta_time = (
                    self.variable_timestamp_buffers[variable_name][i]
                    - self.variable_timestamp_buffers[variable_name][i - 1]
                )
                self.variable_first_derivatives[variable_name].append(
                    self.calculate_derivative(current_value, last_value, delta_time)
                )

            for i in range(1, len(self.variable_first_derivatives[variable_name])):
                current_value = self.variable_first_derivatives[variable_name][i]
                last_value = self.variable_first_derivatives[variable_name][i - 1]
                delta_time = (
                    self.variable_timestamp_buffers[variable_name][i]
                    - self.variable_timestamp_buffers[variable_name][i - 1]
                )
                self.variable_second_derivatives[variable_name].append(
                    self.calculate_derivative(current_value, last_value, delta_time)
                )

            for i in range(1, len(self.variable_second_derivatives[variable_name])):
                current_value = self.variable_second_derivatives[variable_name][i]
                last_value = self.variable_second_derivatives[variable_name][i - 1]
                delta_time = (
                    self.variable_timestamp_buffers[variable_name][i]
                    - self.variable_timestamp_buffers[variable_name][i - 1]
                )
                self.variable_third_derivatives[variable_name].append(
                    self.calculate_derivative(current_value, last_value, delta_time)
                )

    def write_variable_buffers_to_excel(self):
        data = {
            "trolley_position_timestamp": self.variable_timestamp_buffers[
                "trolley_position"
            ],
            "trolley_position": self.variable_buffers["trolley_position"],
            "trolley_position_first_derivative": self.variable_first_derivatives[
                "trolley_position"
            ],
            "trolley_position_second_derivative": self.variable_second_derivatives[
                "trolley_position"
            ],
            "trolley_position_third_derivative": self.variable_third_derivatives[
                "trolley_position"
            ],
            "cable_length_timestamp": self.variable_timestamp_buffers["cable_length"],
            "cable_length": self.variable_buffers["cable_length"],
            "cable_length_first_derivative": self.variable_first_derivatives[
                "cable_length"
            ],
            "cable_length_second_derivative": self.variable_second_derivatives[
                "cable_length"
            ],
            "cable_length_third_derivative": self.variable_third_derivatives[
                "cable_length"
            ],
            "sway_angle_timestamp": self.variable_timestamp_buffers["sway_angle"],
            "sway_angle": self.variable_buffers["sway_angle"],
            "sway_angle_first_derivative": self.variable_first_derivatives[
                "sway_angle"
            ],
            "sway_angle_second_derivative": self.variable_second_derivatives[
                "sway_angle"
            ],
            "sway_angle_third_derivative": self.variable_third_derivatives[
                "sway_angle"
            ],
            "trolley_motor_voltage_timestamp": self.variable_timestamp_buffers[
                "trolley_motor_voltage"
            ],
            "trolley_motor_voltage": self.variable_buffers["trolley_motor_voltage"],
            "trolley_motor_voltage_first_derivative": self.variable_first_derivatives[
                "trolley_motor_voltage"
            ],
            "trolley_motor_voltage_second_derivative": self.variable_second_derivatives[
                "trolley_motor_voltage"
            ],
            "trolley_motor_voltage_third_derivative": self.variable_third_derivatives[
                "trolley_motor_voltage"
            ],
            "hoist_motor_voltage_timestamp": self.variable_timestamp_buffers[
                "hoist_motor_voltage"
            ],
            "hoist_motor_voltage": self.variable_buffers["hoist_motor_voltage"],
            "hoist_motor_voltage_first_derivative": self.variable_first_derivatives[
                "hoist_motor_voltage"
            ],
            "hoist_motor_voltage_second_derivative": self.variable_second_derivatives[
                "hoist_motor_voltage"
            ],
            "hoist_motor_voltage_third_derivative": self.variable_third_derivatives[
                "hoist_motor_voltage"
            ],
        }
        df = pd.DataFrame(data)
        df.to_excel(
            "gantry_crane_analysis.xlsx",
            sheet_name="gantry_crane_analysis",
            index=False,
        )

    def initialize_pwm_publisher(self):
        self.pwm_publisher = self.create_publisher(UInt32, MOTOR_PWM_TOPIC_NAME, 10)

    def packValues(self, mode, pwm_trolley, pwm_hoist):
        # Ensure the values are within the specified ranges
        pwm_trolley = max(min(pwm_trolley, 255), -255)
        pwm_hoist = max(min(pwm_hoist, 255), -255)
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
    
    def publish_pwm(self, mode, pwm_trolley, pwm_hoist):
        message = UInt32()
        message.data = self.packValues(mode, pwm_trolley, pwm_hoist)
        self.pwm_publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    gantry_crane_analysis = GantryCraneAnalysis()
    rclpy.spin(gantry_crane_analysis)
    gantry_crane_analysis.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
