#!/usr/bin/env python3

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

CONTROLLER_COMMAND_TOPIC_NAME = "controller_command"

DEFAULT_SPIN_TIMEOUT = 0.025  # Default spin timeout in seconds

MAX_TROLLEY_POSITION = 1.5  # Maximum trolley position in meters
MIN_TROLLEY_POSITION = 0.0  # Minimum trolley position in meters

MAX_CABLE_LENGTH = 0.65  # Maximum cable length in meters
MIN_CABLE_LENGTH = 0.40  # Minimum cable length in meters

MODES_TIMEOUT = 7.5  # Timeout for modes in seconds

HOIST_RISE_PWM = 800  # PWM for hoist motor to rise the container
HOIST_LOWER_PWM = 800  # PWM for hoist motor to lower the container
HOLD_HOIST_PWM = -500

ROUNDING_DIGITS = 5  # Number of digits to round the values


class GantryControlModes:
    IDLE_MODE = 0x0F  # Idle mode, do nothing
    MOVE_TO_ORIGIN_MODE = 0x1F  # Move to origin mode, move trolley to origin
    MOVE_TO_MIDDLE_MODE = 0x2F  # Move to middle mode, move trolley to middle
    MOVE_TO_END_MODE = 0x3F  # Move to end mode, move trolley to end
    LOCK_CONTAINER_MODE = 0x4F  # Lock container mode, move servo to lock container
    UNLOCK_CONTAINER_MODE = (
        0x5F  # Unlock container mode, move servo to unlock container
    )
    CONTROL_MODE = 0x6F  # Control mode, control trolley and hoist motors
    BRAKE_MODE = 0x7F # Brake mode, stop either trolley or hoist motor
    PWM_BRAKE_FLAG = 0x7FF  # Brake command, flag to stop either trolley or hoist motor


class GantryCraneConnector(Node):
    def __init__(self):
        rclpy.init()
        super().__init__(GANTRY_CRANE_NODE_NAME)

        # Get node name
        self.name = self.get_name()

        self.subscribers = {}
        self.initialize_subscribers()

        # Initialize publisher
        self.motor_pwm_publisher = self.create_publisher(
            UInt32, CONTROLLER_COMMAND_TOPIC_NAME, 10
        )

        self.initialize_variables()

        self.spin_timeout = DEFAULT_SPIN_TIMEOUT

        self.get_logger().info("Gantry crane connector initialized: " + self.name)

    def set_spin_timeout(self, spin_timeout=DEFAULT_SPIN_TIMEOUT):
        self.spin_timeout = spin_timeout

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
            value = round(
                value, 5
            )  # Round to 5 decimal places: resolution of the DAC

        if variable_name == "hoist_motor_voltage":
            value = round(
                value, 5
            )  # Round to 5 decimal places: resolution of the DAC

        self.variables_value[variable_name] = value  # Update value

        # Calculate first derivative
        self.variables_first_derivative[variable_name] = round(
            self.calculate_derivative(
                self.variables_value[variable_name],
                self.last_variables_value[variable_name],
                delta_time,
            ),
            ROUNDING_DIGITS,
        )

        # Calculate second derivative
        self.variables_second_derivative[variable_name] = round(
            self.calculate_derivative(
                self.variables_first_derivative[variable_name],
                self.last_variables_first_derivative[variable_name],
                delta_time,
            ),
            ROUNDING_DIGITS,
        )

        # Calculate third derivative
        self.variables_third_derivative[variable_name] = round(
            self.calculate_derivative(
                self.variables_second_derivative[variable_name],
                self.last_variables_second_derivative[variable_name],
                delta_time,
            ),
            ROUNDING_DIGITS,
        )

        # Update last values
        self.last_variables_value[variable_name] = self.variables_value[variable_name]

        # Update last first derivatives
        self.last_variables_first_derivative[variable_name] = (
            self.variables_first_derivative[variable_name]
        )

        # Update last second derivatives
        self.last_variables_second_derivative[variable_name] = (
            self.variables_second_derivative[variable_name]
        )

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
            "trolley_position": 0,
            "cable_length": 0,
            "sway_angle": 0,
            "trolley_motor_voltage": 0,
            "hoist_motor_voltage": 0,
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
        self.get_logger().info("Resetting gantry crane variables...")

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

        self.get_logger().info("Gantry crane variables reset")

    def update(self):
        rclpy.spin_once(self, timeout_sec=self.spin_timeout)

    def publish_command(
        self, gantry_mode, input_pwm_trolley_motor, input_pwm_hoist_motor
    ):
        self.control_pwm["trolley_motor"] = input_pwm_trolley_motor
        self.control_pwm["hoist_motor"] = input_pwm_hoist_motor

        # message = ControllerCommandMessage()
        # message.mode = gantry_mode
        # message.trolley_motor_pwm = self.control_pwm["trolley_motor"]
        # message.hoist_motor_pwm = self.control_pwm["hoist_motor"]

        message = UInt32()
        message.data = self.packValues(
            gantry_mode,
            self.control_pwm["trolley_motor"],
            self.control_pwm["hoist_motor"],
        )

        self.motor_pwm_publisher.publish(message)
        self.update()

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
        For braking, set the PWM to 0x7FF: 2047 (PWM_BRAKE_FLAG)
        """
        # Ensure the values are within the specified ranges
        mode = max(min(int(mode), 255), 0)
        trolley_motor_pwm = int(trolley_motor_pwm)
        pwm_hoist_motor = int(pwm_hoist_motor)

        if trolley_motor_pwm != GantryControlModes.PWM_BRAKE_FLAG:
            trolley_motor_pwm = max(min(trolley_motor_pwm, 1023), -1023)
        if pwm_hoist_motor != GantryControlModes.PWM_BRAKE_FLAG:
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
    
    def initialized(self):
        self.get_logger().info("Initializing gantry crane connector...")
        self.wait_for_messages()
        self.wait_for_subscribers()
        self.reset_variable()
        self.get_logger().info("Gantry crane connector initialized")
        
        return True


    def begin(self, slide_to_position="origin", hoist_to_position="middle"):
        self.get_logger().info("Beginning gantry crane connector...")
        self.wait_for_messages()
        self.wait_for_subscribers()
        self.reset_variable()

        for i in range(10):
            self.update()

        slide_to_position = slide_to_position.lower()
        hoist_to_position = hoist_to_position.lower()

        if hoist_to_position == "top":
            self.hoist_container_to_top()
        elif hoist_to_position == "middle":
            self.hoist_container_to_middle()
        elif hoist_to_position == "bottom":
            self.hoist_container_to_bottom()
        else:
            self.get_logger().info("Invalid container position")
            return False

        if slide_to_position == "origin":
            self.move_trolley_to_origin()
        elif slide_to_position == "middle":
            self.move_trolley_to_middle()
        elif slide_to_position == "end":
            self.move_trolley_to_end()
        else:
            self.get_logger().info("Invalid trolley position")
            return False

        self.idle()

        self.get_logger().info("Gantry crane connector ready")
        time.sleep(2.0)

    def wait_for_subscribers(self):
        self.get_logger().info("Waiting for subscribers...")
        while self.motor_pwm_publisher.get_subscription_count() == 0:
            self.update()
        self.get_logger().info("All subscribers ready")

    def wait_for_messages(self):
        self.get_logger().info("Waiting for messages...")
        for variable_name in self.variables_value:
            if variable_name == "trolley_motor_voltage":
                break

            self.get_logger().info("Waiting for " + variable_name + " messages...")
            while self.variables_value[variable_name] is None:
                self.update()
                time.sleep(0.1)
            self.get_logger().info("..." + variable_name + " messages received")

        self.get_logger().info("All messages received")

    def move_trolley_to_origin(self):
        self.get_logger().info("Moving trolley to origin...")
        start_time = time.time()
        while (
            self.variables_value["trolley_position"] > MIN_TROLLEY_POSITION + 0.01
            or time.time() - start_time < MODES_TIMEOUT
        ):
            if (
                abs(self.variables_value["trolley_position"] - MIN_TROLLEY_POSITION)
                < 0.01
                and abs(self.variables_first_derivative["trolley_position"]) < 0.01
            ):
                break

            self.publish_command(GantryControlModes.MOVE_TO_ORIGIN_MODE, 0, 0)

        self.brake(brake_trolley=True, brake_hoist=False)

        return True

    def move_trolley_to_middle(self):
        self.get_logger().info("Moving trolley to middle...")
        start_time = time.time()
        while (
            abs(
                self.variables_value["trolley_position"]
                - (MAX_TROLLEY_POSITION + MIN_TROLLEY_POSITION) / 2
            )
            > 0.03
            or time.time() - start_time < MODES_TIMEOUT
        ):
            self.publish_command(GantryControlModes.MOVE_TO_MIDDLE_MODE, 0, 0)

        self.brake(brake_trolley=True, brake_hoist=False)

        return True

    def move_trolley_to_end(self):
        self.get_logger().info("Moving trolley to end...")
        start_time = time.time()
        while (
            self.variables_value["trolley_position"] < MAX_TROLLEY_POSITION - 0.01
            or time.time() - start_time < MODES_TIMEOUT
        ):
            if (
                abs(self.variables_value["trolley_position"] - MAX_TROLLEY_POSITION)
                < 0.01
                and abs(self.variables_first_derivative["trolley_position"]) < 0.01
            ):
                break

            self.publish_command(GantryControlModes.MOVE_TO_END_MODE, 0, 0)

        return True

    def lock_container(self):
        self.get_logger().info("Locking container...")
        start_time = time.time()
        while time.time() - start_time < MODES_TIMEOUT:
            self.publish_command(GantryControlModes.LOCK_CONTAINER_MODE, 0, 0)
            time.sleep(0.1)

        return True

    def unlock_container(self):
        self.get_logger().info("Unlocking container...")
        start_time = time.time()
        while time.time() - start_time < MODES_TIMEOUT:
            self.publish_command(GantryControlModes.UNLOCK_CONTAINER_MODE, 0, 0)
            time.sleep(0.1)

        return True

    def control(self, trolley_motor_pwm, hoist_motor_pwm):
        self.publish_command(
            GantryControlModes.CONTROL_MODE, trolley_motor_pwm, hoist_motor_pwm
        )

    def brake(self, brake_trolley=True, brake_hoist=True):
        self.get_logger().info("Braking...")
        for i in range(10):
            if brake_trolley:
                trolley_pwm_flag = GantryControlModes.PWM_BRAKE_FLAG
            else:
                trolley_pwm_flag = 0

            if brake_hoist:
                hoist_pwm_flag = GantryControlModes.PWM_BRAKE_FLAG
            else:
                hoist_pwm_flag = 0

            self.publish_command(
                GantryControlModes.BRAKE_MODE, trolley_pwm_flag, hoist_pwm_flag
            )

    def idle(self):
        self.get_logger().info("Idling...")
        for i in range(10):
            self.publish_command(GantryControlModes.IDLE_MODE, 0, 0)
        time.sleep(0.5)

    def hoist_container_to_top(self):
        self.initialize_variables()
        self.wait_for_messages()  # Wait for messages to make sure the cable length is updated
        self.get_logger().info(
            "Hoisting container to top... (" + str(MIN_CABLE_LENGTH) + " m)"
        )
        start_time = time.time()
        while (
            abs(self.variables_value["cable_length"] - MIN_CABLE_LENGTH) > 0.001
            or time.time() - start_time < MODES_TIMEOUT
        ):
            error = self.variables_value["cable_length"] - MIN_CABLE_LENGTH
            if error < 0:
                hoist_pwm = -HOIST_LOWER_PWM * np.sign(error)
            elif error > 0:
                hoist_pwm = -HOIST_RISE_PWM * np.sign(error)
            else:
                hoist_pwm = 0

            self.get_logger().debug(
                "Error : " + str(error) + " m. PWM : " + str(hoist_pwm)
            )

            self.publish_command(GantryControlModes.CONTROL_MODE, 0, hoist_pwm)

        for i in range(10):
            self.publish_command(GantryControlModes.CONTROL_MODE, 0, HOLD_HOIST_PWM)

        self.brake(brake_trolley=False, brake_hoist=True)

        return True

    def hoist_container_to_middle(self):
        self.initialize_variables()
        self.wait_for_messages()  # Wait for messages to make sure the cable length is updated
        self.get_logger().info(
            "Hoisting container to middle... ("
            + str((MAX_CABLE_LENGTH + MIN_CABLE_LENGTH) / 2)
            + " m)"
        )
        start_time = time.time()
        while (
            abs(
                self.variables_value["cable_length"]
                - (MAX_CABLE_LENGTH + MIN_CABLE_LENGTH) / 2
            )
            > 0.001
            or time.time() - start_time < MODES_TIMEOUT
        ):
            error = (
                self.variables_value["cable_length"]
                - (MAX_CABLE_LENGTH + MIN_CABLE_LENGTH) / 2
            )
            if error < 0:
                hoist_pwm = -HOIST_LOWER_PWM * np.sign(error)
            elif error > 0:
                hoist_pwm = -HOIST_RISE_PWM * np.sign(error)
            else:
                hoist_pwm = 0

            self.get_logger().debug(
                "Error : " + str(error) + " m. PWM : " + str(hoist_pwm)
            )
            self.publish_command(GantryControlModes.CONTROL_MODE, 0, hoist_pwm)

        for i in range(10):
            self.publish_command(GantryControlModes.CONTROL_MODE, 0, HOLD_HOIST_PWM)

        self.brake(brake_trolley=False, brake_hoist=True)

        return True

    def hoist_container_to_bottom(self):
        self.initialize_variables()
        self.wait_for_messages()  # Wait for messages to make sure the cable length is updated
        self.get_logger().info(
            "Hoisting container to bottom... (" + str(MAX_CABLE_LENGTH) + " m)"
        )
        start_time = time.time()
        while (
            abs(self.variables_value["cable_length"] - MAX_CABLE_LENGTH) > 0.001
            or time.time() - start_time < MODES_TIMEOUT
        ):
            error = self.variables_value["cable_length"] - MAX_CABLE_LENGTH
            if error < 0:
                hoist_pwm = -HOIST_LOWER_PWM * np.sign(error)
            elif error > 0:
                hoist_pwm = -HOIST_RISE_PWM * np.sign(error)
            else:
                hoist_pwm = 0

            self.get_logger().debug(
                "Error : " + str(error) + " m. PWM : " + str(hoist_pwm)
            )

            self.publish_command(GantryControlModes.CONTROL_MODE, 0, hoist_pwm)

        for i in range(10):
            self.publish_command(GantryControlModes.CONTROL_MODE, 0, HOLD_HOIST_PWM)

        self.brake(brake_trolley=False, brake_hoist=True)

        return True

    def end(self):
        self.get_logger().info("Stopping gantry crane connector...")
        self.brake(brake_trolley=True, brake_hoist=True)
        self.idle()
        self.get_logger().info("Gantry crane connector stopped")
        rclpy.shutdown()
        return True
