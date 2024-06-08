from gantry_crane_lib.ros_connector import GantryCraneConnector, GantryControlModes
from gantry_crane_lib.ros_connector import (
    MIN_TROLLEY_POSITION,
    MAX_TROLLEY_POSITION,
    MIN_CABLE_LENGTH,
    MAX_CABLE_LENGTH,
)
from gantry_crane_lib.logger import Logger

VARIABLE_TO_LOG = {
    "timestamp",
    "control_signal_trolley",
    "control_signal_hoist",
    "pwm_trolley_motor",
    "pwm_hoist_motor",
    "trolley_motor_voltage",
    "hoist_motor_voltage",
    "trolley_position",
    "cable_length",
    "sway_angle",
    "trolley_position_first_derivative",
    "cable_length_first_derivative",
    "sway_angle_first_derivative",
    "trolley_position_second_derivative",
    "cable_length_second_derivative",
    "sway_angle_second_derivative",
    "trolley_position_third_derivative",
    "cable_length_third_derivative",
    "sway_angle_third_derivative",
}


def collect_data(logger, connector, controller):
    logger.add_to_buffer("timestamp", logger.get_time_sec())

    logger.add_to_buffer("control_signal_trolley", controller.control_signal_trolley)

    logger.add_to_buffer("control_signal_hoist", controller.control_signal_hoist)

    logger.add_to_buffer("pwm_trolley_motor", connector.control_pwm["trolley_motor"])

    logger.add_to_buffer("pwm_hoist_motor", connector.control_pwm["hoist_motor"])

    logger.add_to_buffer(
        "trolley_motor_voltage",
        connector.variables_value["trolley_motor_voltage"],
    )

    logger.add_to_buffer(
        "hoist_motor_voltage",
        connector.variables_value["hoist_motor_voltage"],
    )

    logger.add_to_buffer(
        "trolley_position", connector.variables_value["trolley_position"]
    )

    logger.add_to_buffer("cable_length", connector.variables_value["cable_length"])

    logger.add_to_buffer("sway_angle", connector.variables_value["sway_angle"])

    logger.add_to_buffer(
        "trolley_position_first_derivative",
        connector.variables_first_derivative["trolley_position"],
    )

    logger.add_to_buffer(
        "cable_length_first_derivative",
        connector.variables_first_derivative["cable_length"],
    )

    logger.add_to_buffer(
        "sway_angle_first_derivative",
        connector.variables_first_derivative["sway_angle"],
    )

    logger.add_to_buffer(
        "trolley_position_second_derivative",
        connector.variables_second_derivative["trolley_position"],
    )

    logger.add_to_buffer(
        "cable_length_second_derivative",
        connector.variables_second_derivative["cable_length"],
    )

    logger.add_to_buffer(
        "sway_angle_second_derivative",
        connector.variables_second_derivative["sway_angle"],
    )

    logger.add_to_buffer(
        "trolley_position_third_derivative",
        connector.variables_third_derivative["trolley_position"],
    )

    logger.add_to_buffer(
        "cable_length_third_derivative",
        connector.variables_third_derivative["cable_length"],
    )

    logger.add_to_buffer(
        "sway_angle_third_derivative",
        connector.variables_third_derivative["sway_angle"],
    )


def create_plot(logger):
    logger.create_plot("control_signal_trolley", "timestamp", "control_signal_trolley")
    logger.create_plot("control_signal_hoist", "timestamp", "control_signal_hoist")

    logger.create_plot("pwm_trolley_motor", "timestamp", "pwm_trolley_motor")
    logger.create_plot("pwm_hoist_motor", "timestamp", "pwm_hoist_motor")

    logger.create_plot("trolley_motor_voltage", "timestamp", "trolley_motor_voltage")
    logger.create_plot("hoist_motor_voltage", "timestamp", "hoist_motor_voltage")

    logger.create_plot("trolley_position", "timestamp", "trolley_position")
    logger.create_plot("cable_length", "timestamp", "cable_length")
    logger.create_plot("sway_angle", "timestamp", "sway_angle")

    logger.create_plot(
        "trolley_position_first_derivative",
        "timestamp",
        "trolley_position_first_derivative",
    )

    logger.create_plot(
        "cable_length_first_derivative",
        "timestamp",
        "cable_length_first_derivative",
    )

    logger.create_plot(
        "sway_angle_first_derivative",
        "timestamp",
        "sway_angle_first_derivative",
    )

    logger.create_plot(
        "trolley_position_second_derivative",
        "timestamp",
        "trolley_position_second_derivative",
    )

    logger.create_plot(
        "cable_length_second_derivative",
        "timestamp",
        "cable_length_second_derivative",
    )

    logger.create_plot(
        "troley_motor_voltage vs trolley_speed",
        "trolley_motor_voltage",
        "trolley_position_first_derivative",
    )

    logger.create_plot(
        "hoist_motor_voltage vs cable_speed",
        "hoist_motor_voltage",
        "cable_length_first_derivative",
    )


class MovingAverage:
    def __init__(self, window_size=10):
        self.window_size = window_size
        self.data = []

    def add_data(self, data):
        self.data.append(data)
        if len(self.data) > self.window_size:
            self.data.pop(0)

    def get_average(self):
        return sum(self.data) / len(self.data)
