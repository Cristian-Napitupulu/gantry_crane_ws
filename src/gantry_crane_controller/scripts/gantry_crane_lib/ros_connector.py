import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import UInt32

import time

import numpy as np

GANTRY_CRANE_NODE_NAME = "gantry_crane_node"

TROLLEY_POSITION_TOPIC_NAME = "trolley_position"
CABLE_LENGTH_TOPIC_NAME = "cable_length"
SWAY_ANGLE_TOPIC_NAME = "sway_angle"
TROLLEY_MOTOR_VOLTAGE_TOPIC_NAME = "trolley_motor_voltage"
HOIST_MOTOR_VOLTAGE_TOPIC_NAME = "hoist_motor_voltage"

MOTOR_PWM_TOPIC_NAME = "motor_pwm"


class GantryControlModes():
    IDLE_MODE = 0x00  # Idle mode, do nothing
    MOVE_TO_ORIGIN_MODE = 0x1F  # Move to origin mode, move trolley to origin
    MOVE_TO_MIDDLE_MODE = 0x2F  # Move to middle mode, move trolley to middle
    MOVE_TO_END_MODE = 0x3F  # Move to end mode, move trolley to end
    LOCK_CONTAINER_MODE = 0x4F  # Lock container mode, move servo to lock container
    UNLOCK_CONTAINER_MODE = (
        0x5F  # Unlock container mode, move servo to unlock container
    )
    CONTROL_MODE = 0xFF  # Control mode, control trolley and hoist motors
    BRAKE_COMMAND = 0x7FF  # Brake command, flag to stop either trolley or hoist motor


class GantryCraneConnector(Node):
    def __init__(self):
        super().__init__(GANTRY_CRANE_NODE_NAME)

        self.subscribers = {}
        self.initialize_subscribers()

        # Initialize publisher
        self.motor_pwm_publisher = self.create_publisher(
            UInt32, MOTOR_PWM_TOPIC_NAME, 10
        )

        self.initialize_variables()

        self.get_logger().info("Gantry crane system initialized.")

    def initialize_subscribers(self):
        # Create subscriber for trolley position topic
        self.subscribers["trolley_position"] = self.create_subscription(
            Float32, TROLLEY_POSITION_TOPIC_NAME, self.trolley_position_callback, 10
        )

        # Create subscriber for cable length topic
        self.subscribers["cable_length"] = self.create_subscription(
            Float32, CABLE_LENGTH_TOPIC_NAME, self.cable_length_callback, 10
        )

        # Create subscriber for sway angle topic
        self.subscribers["sway_angle"] = self.create_subscription(
            Float32, SWAY_ANGLE_TOPIC_NAME, self.sway_angle_callback, 10
        )

        # Create subscriber for trolley motor voltage topic
        self.subscribers["trolley_motor_voltage"] = self.create_subscription(
            Float32,
            TROLLEY_MOTOR_VOLTAGE_TOPIC_NAME,
            self.trolley_motor_voltage_callback,
            10,
        )

        # Create subscriber for hoist motor voltage topic
        self.subscribers["hoist_motor_voltage"] = self.create_subscription(
            Float32,
            HOIST_MOTOR_VOLTAGE_TOPIC_NAME,
            self.hoist_motor_voltage_callback,
            10,
        )

    # Create a function to block until all subscribers are ready and has received at least one message
    def wait_for_subscribers(self):
        self.get_logger().info("Waiting for subscribers...")

    def trolley_position_callback(
        self, message
    ):  # Callback function for trolley position topic
        self.process_message("trolley_position", message)

    def cable_length_callback(
        self, message
    ):  # Callback function for cable length topic
        self.process_message("cable_length", message)

    def sway_angle_callback(self, message):  # Callback function for sway angle topic
        self.process_message("sway_angle", message)

    def trolley_motor_voltage_callback(
        self, message
    ):  # Callback function for trolley motor voltage topic
        self.process_message("trolley_motor_voltage", message)

    def hoist_motor_voltage_callback(
        self, message
    ):  # Callback function for hoist motor voltage topic
        self.process_message("hoist_motor_voltage", message)

    def process_message(self, variable_name, message):
        timestamp = time.time()
        delta_time = timestamp - self.last_variables_message_timestamp[variable_name]

        value = message.data
        if variable_name == "trolley_position":
            value = round(
                value, 5
            )  # Round to 5 decimal places: resolution of the encoder

        if variable_name == "cable_length":
            value = round(
                value, 3
            )  # Round to 3 decimal places: resolution of the LIDAR

        if variable_name == "sway_angle":
            value = round(
                value * np.pi / 180.0, 3
            )  # Convert to radians and round to 3 decimal places, resolution of the LIDAR

        if variable_name == "trolley_motor_voltage":
            value = round(value, 5)  # Round to 5 decimal places: resolution of the DAC

        if variable_name == "hoist_motor_voltage":
            value = round(value, 5)  # Round to 5 decimal places: resolution of the DAC

        self.variables_value[variable_name] = value  # Update value

        # Calculate first derivative
        self.variables_first_derivative[variable_name] = self.calculate_derivative(
            self.variables_value[variable_name],
            self.last_variables_value[variable_name],
            delta_time,
        )

        # Calculate second derivative
        self.variables_second_derivative[variable_name] = self.calculate_derivative(
            self.variables_first_derivative[variable_name],
            self.last_variables_first_derivative[variable_name],
            delta_time,
        )

        # Calculate third derivative
        self.variables_third_derivative[variable_name] = self.calculate_derivative(
            self.variables_second_derivative[variable_name],
            self.last_variables_second_derivative[variable_name],
            delta_time,
        )

        # Update last values
        self.last_variables_value[variable_name] = self.variables_value[variable_name]

        # Update last first derivatives
        self.last_variables_first_derivative[
            variable_name
        ] = self.variables_first_derivative[variable_name]

        # Update last second derivatives
        self.last_variables_second_derivative[
            variable_name
        ] = self.variables_second_derivative[variable_name]

        # Update timestamp
        self.last_variables_message_timestamp[variable_name] = time.time()

    def calculate_derivative(
        self, current_value, last_value, delta_time
    ):  # Function to calculate derivative
        return (current_value - last_value) / delta_time

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
            "trolley_position": None,
            "cable_length": None,
            "sway_angle": None,
            "trolley_motor_voltage": None,
            "hoist_motor_voltage": None,
        }

        self.last_variables_first_derivative = {
            "trolley_position": 0.0,
            "cable_length": 0.0,
            "sway_angle": 0.0,
            "trolley_motor_voltage": 0.0,
            "hoist_motor_voltage": 0.0,
        }

        self.variables_second_derivative = {
            "trolley_position": None,
            "cable_length": None,
            "sway_angle": None,
            "trolley_motor_voltage": None,
            "hoist_motor_voltage": None,
        }

        self.last_variables_second_derivative = {
            "trolley_position": 0.0,
            "cable_length": 0.0,
            "sway_angle": 0.0,
            "trolley_motor_voltage": 0.0,
            "hoist_motor_voltage": 0.0,
        }

        self.variables_third_derivative = {
            "trolley_position": None,
            "cable_length": None,
            "sway_angle": None,
            "trolley_motor_voltage": None,
            "hoist_motor_voltage": None,
        }

        self.last_variables_message_timestamp = {
            "trolley_position": None,
            "cable_length": None,
            "sway_angle": None,
            "trolley_motor_voltage": None,
            "hoist_motor_voltage": None,
        }

        self.control_input = {
            "trolley_motor": None,
            "hoist_motor": None,
        }

        self.control_pwm = {
            "trolley_motor": None,
            "hoist_motor": None,
        }

    def reset_variable(self):
        self.variables_value["trolley_position"] = 0.0
        self.variables_value["cable_length"] = 0.0
        self.variables_value["sway_angle"] = 0.0
        self.variables_value["trolley_motor_voltage"] = 0.0
        self.variables_value["hoist_motor_voltage"] = 0.0

        self.variables_first_derivative["trolley_position"] = 0.0
        self.variables_first_derivative["cable_length"] = 0.0
        self.variables_first_derivative["sway_angle"] = 0.0
        self.variables_first_derivative["trolley_motor_voltage"] = 0.0
        self.variables_first_derivative["hoist_motor_voltage"] = 0.0

        self.variables_second_derivative["trolley_position"] = 0.0
        self.variables_second_derivative["cable_length"] = 0.0
        self.variables_second_derivative["sway_angle"] = 0.0
        self.variables_second_derivative["trolley_motor_voltage"] = 0.0
        self.variables_second_derivative["hoist_motor_voltage"] = 0.0

        self.variables_third_derivative["trolley_position"] = 0.0
        self.variables_third_derivative["cable_length"] = 0.0
        self.variables_third_derivative["sway_angle"] = 0.0
        self.variables_third_derivative["trolley_motor_voltage"] = 0.0
        self.variables_third_derivative["hoist_motor_voltage"] = 0.0

        self.last_variables_message_timestamp["trolley_position"] = time.time()
        self.last_variables_message_timestamp["cable_length"] = time.time()
        self.last_variables_message_timestamp["sway_angle"] = time.time()
        self.last_variables_message_timestamp["trolley_motor_voltage"] = time.time()
        self.last_variables_message_timestamp["hoist_motor_voltage"] = time.time()

        self.control_input["trolley_motor"] = 0.0
        self.control_input["hoist_motor"] = 0.0

        self.control_pwm["trolley_motor"] = 0.0
        self.control_pwm["hoist_motor"] = 0.0

    def publish_PWM(self, gantry_mode, input_pwm_trolley_motor, input_pwm_hoist_motor):
        message = UInt32()
        message.data = self.packValues(
            gantry_mode, input_pwm_trolley_motor, input_pwm_hoist_motor
        )
        self.motor_pwm_publisher.publish(message)

    def packValues(
        self, mode=GantryControlModes.IDLE_MODE, trolley_motor_pwm=0, pwm_hoist_motor=0
    ):
        """
        We're using this function to pack the values into a 32-bit integer
        so that we can send them over a single topic
        The 32-bit integer is split into three parts:
        1. Mode (8 bits)
        2. Trolley motor PWM (12 bits)
        3. Hoist motor PWM (12 bits)
        Actual PWM range in the microcontroller is -1023 to 1023
        So, make sure the values are within this range
        For braking, set the PWM to 0x7FF: 2047 (BRAKE_COMMAND)
        """
        # Ensure the values are within the specified ranges
        mode = max(min(int(mode), 255), 0)
        trolley_motor_pwm = int(trolley_motor_pwm)
        pwm_hoist_motor = int(pwm_hoist_motor)

        if trolley_motor_pwm != GantryControlModes.BRAKE_COMMAND:
            trolley_motor_pwm = max(min(trolley_motor_pwm, 1023), -1023)
        if pwm_hoist_motor != GantryControlModes.BRAKE_COMMAND:
            pwm_hoist_motor = max(min(pwm_hoist_motor, 1023), -1023)

        # Convert negative values to two's complement representation
        if trolley_motor_pwm < 0:
            trolley_motor_pwm = 0xFFF + trolley_motor_pwm + 1
        if pwm_hoist_motor < 0:
            pwm_hoist_motor = 0xFFF + pwm_hoist_motor + 1

        # Pack the values into a 32-bit integer
        packed_value = (
            (mode & 0xFF)
            | ((trolley_motor_pwm & 0xFFF) << 8)
            | ((pwm_hoist_motor & 0xFFF) << 20)
        )
        return packed_value
    
    def initialize(self):
        self.reset_variable()
        self.wait_for_subscribers()
        self.wait_for_messages()
        start_time = time.time()
        while time.time() - start_time < 1.0:
            gantryMode = GantryControlModes.IDLE_MODE
            self.publish_PWM(gantryMode, 0, 0)
            time.sleep(0.1)
            rclpy.spin_once(self)
        self.get_logger().info("Gantry crane system initialized.")

    def wait_for_subscribers(self):
        while self.motor_pwm_publisher.get_subscription_count() == 0:
            time.sleep(0.1)
            rclpy.spin_once(self)
        self.get_logger().info("All subscribers ready")

    def wait_for_messages(self):
        while (
            self.variables_value["trolley_position"] is None
            or self.variables_value["cable_length"] is None
            or self.variables_value["sway_angle"] is None
            or self.variables_value["trolley_motor_voltage"] is None
            or self.variables_value["hoist_motor_voltage"] is None
        ):
            time.sleep(0.1)
            rclpy.spin_once(self)
        self.get_logger().info("All messages received")

    def collect_data(self):
        pass