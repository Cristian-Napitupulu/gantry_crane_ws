#!/usr/bin/env python3

from gantry_crane_lib.ros_connector import GantryCraneConnector, GantryControlModes
from gantry_crane_lib.logger import Logger

from sliding_mode_controller import non_linear_motor_model, sliding_mode_controller

import rclpy
import time

LOG_FOLDER_PATH = (
    "/home/icodes/Documents/gantry_crane_ws/src/gantry_crane_controller/log"
)

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

gantry_crane_model = non_linear_motor_model
gantry_crane_controller = sliding_mode_controller

gantry_crane_logger = Logger(LOG_FOLDER_PATH)

rclpy.init()
gantry_crane = GantryCraneConnector()

PLAY_DURATION = 10.0
DESIRED_TROLLEY_POSITION = 1.25
DESIRED_CABLE_LENGTH = 0.4
MAX_ERROR_TROLLEY_POSITION = 0.05
MAX_ERROR_CABLE_LENGTH = 0.05


def collect_data():
    gantry_crane_logger.add_to_buffer(
        "timestamp", time.time() - gantry_crane_logger.start_time
    )

    gantry_crane_logger.add_to_buffer(
        "control_signal_trolley", gantry_crane_controller.control_signal_trolley
    )

    gantry_crane_logger.add_to_buffer(
        "control_signal_hoist", gantry_crane_controller.control_signal_hoist
    )

    gantry_crane_logger.add_to_buffer(
        "pwm_trolley_motor", gantry_crane_controller.pwm_trolley_motor
    )

    gantry_crane_logger.add_to_buffer(
        "pwm_hoist_motor", gantry_crane_controller.pwm_hoist_motor
    )

    gantry_crane_logger.add_to_buffer(
        "trolley_motor_voltage",
        gantry_crane.variables_value["trolley_motor_voltage"],
    )

    gantry_crane_logger.add_to_buffer(
        "hoist_motor_voltage",
        gantry_crane.variables_value["hoist_motor_voltage"],
    )

    gantry_crane_logger.add_to_buffer(
        "trolley_position", gantry_crane.variables_value["trolley_position"]
    )

    gantry_crane_logger.add_to_buffer(
        "cable_length", gantry_crane.variables_value["cable_length"]
    )

    gantry_crane_logger.add_to_buffer(
        "sway_angle", gantry_crane.variables_value["sway_angle"]
    )

    gantry_crane_logger.add_to_buffer(
        "trolley_position_first_derivative",
        gantry_crane.variables_first_derivative["trolley_position"],
    )

    gantry_crane_logger.add_to_buffer(
        "cable_length_first_derivative",
        gantry_crane.variables_first_derivative["cable_length"],
    )

    gantry_crane_logger.add_to_buffer(
        "sway_angle_first_derivative",
        gantry_crane.variables_first_derivative["sway_angle"],
    )

    gantry_crane_logger.add_to_buffer(
        "trolley_position_second_derivative",
        gantry_crane.variables_second_derivative["trolley_position"],
    )

    gantry_crane_logger.add_to_buffer(
        "cable_length_second_derivative",
        gantry_crane.variables_second_derivative["cable_length"],
    )

    gantry_crane_logger.add_to_buffer(
        "sway_angle_second_derivative",
        gantry_crane.variables_second_derivative["sway_angle"],
    )

    gantry_crane_logger.add_to_buffer(
        "trolley_position_third_derivative",
        gantry_crane.variables_third_derivative["trolley_position"],
    )

    gantry_crane_logger.add_to_buffer(
        "cable_length_third_derivative",
        gantry_crane.variables_third_derivative["cable_length"],
    )

    gantry_crane_logger.add_to_buffer(
        "sway_angle_third_derivative",
        gantry_crane.variables_third_derivative["sway_angle"],
    )


def create_plot():
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

def sweep_trolley_position():
    trolley_motor_pwm = 0
    hoist_motor_pwm = 0
    while True:
        # Update matrices
        gantry_crane_model.update_model()

        # Generate control input

        # Convert control input to PWM
        trolley_motor_pwm += 1
        hoist_motor_pwm += 1
        print(
            "trolley motor pwm: {}, hoist motor pwm: {}".format(
                trolley_motor_pwm, hoist_motor_pwm
            )
        )

        # Publish motor PWM
        gantryMode = GantryControlModes.CONTROL_MODE
        gantry_crane.publish_PWM(gantryMode, trolley_motor_pwm, hoist_motor_pwm)

        rclpy.spin_once(gantry_crane, timeout_sec=0.01)

        # Collect data
        collect_data()

        time.sleep(0.5)


def control_gantry_crane(
    timeout_sec=15.0,
    desired_trolley_position=1.0,
    desired_cable_length=None,
    max_error_trolley_position=0.1,
    max_error_cable_length=0.1,
):
    result = None
    inside_max_error = False
    time_inside_max_error = 0.0
    start_time = time.time()
    while True:
        # Update matrices
        gantry_crane_model.update_model()

        # Generate control input
        (
            trolley_motor_control_input,
            hoist_motor_control_input,
        ) = gantry_crane_controller.get_control_input(
            desired_trolley_position, desired_cable_length
        )

        # Convert control input to PWM
        trolley_motor_pwm, hoist_motor_pwm = gantry_crane_controller.convert_to_pwm(
            trolley_motor_control_input, hoist_motor_control_input
        )
        print(
            "trolley motor pwm: {}, hoist motor pwm: {}".format(
                trolley_motor_pwm, hoist_motor_pwm
            )
        )

        # Publish motor PWM
        gantryMode = GantryControlModes.CONTROL_MODE
        gantry_crane.publish_PWM(gantryMode, trolley_motor_pwm, hoist_motor_pwm)

        rclpy.spin_once(gantry_crane, timeout_sec=0.01)

        # Collect data
        collect_data()

        # Check if timeout
        if time.time() - start_time > timeout_sec:
            result = "timeout"
            break

        if inside_max_error and time.time() - time_inside_max_error > timeout_sec / 3:
            result = "desired trolley position reached"
            break

        if desired_cable_length is None:
            if (
                abs(
                    gantry_crane.variables_value["trolley_position"]
                    - desired_trolley_position
                )
                < max_error_trolley_position
            ):
                inside_max_error = True
                time_inside_max_error = time.time()

            else:
                inside_max_error = False
        else:
            if (
                abs(
                    gantry_crane.variables_value["trolley_position"]
                    - desired_trolley_position
                )
                < max_error_trolley_position
                and abs(
                    gantry_crane.variables_value["cable_length"] - desired_cable_length
                )
                < max_error_cable_length
            ):
                inside_max_error = True
                time_inside_max_error = time.time()
                # result = "desired trolley position and cable length reached"

            else:
                inside_max_error = False

    return result


if __name__ == "__main__":
    gantry_crane_model.attach_connector(gantry_crane)
    print("Gantry crane model initialized: {}".format(gantry_crane_model.get_name()))

    gantry_crane_controller.attach_connector(gantry_crane)
    gantry_crane_controller.attach_model(gantry_crane_model)
    print("Controller initialized: {}".format(gantry_crane_controller.get_name()))

    try:
        # Initialize gantry crane
        gantry_crane.begin()

        # Begin logging
        gantry_crane_logger.begin()

        # Control gantry crane to first position
        result = control_gantry_crane(
            timeout_sec=PLAY_DURATION,
            desired_trolley_position=1.0,
            desired_cable_length=0.4,
            max_error_trolley_position=MAX_ERROR_TROLLEY_POSITION,
            max_error_cable_length=MAX_ERROR_CABLE_LENGTH,
        )

        print("Result: {}".format(result))

        # time.sleep(5.0) # Wait for 5 seconds

        # Control gantry crane to second position
        result = control_gantry_crane(
            timeout_sec=PLAY_DURATION,
            desired_trolley_position=0.3,
            desired_cable_length=0.65,
            max_error_trolley_position=MAX_ERROR_TROLLEY_POSITION,
            max_error_cable_length=MAX_ERROR_CABLE_LENGTH,
        )

        print("Result: {}".format(result))

        gantry_crane_logger.write_buffers_to_excel(
            gantry_crane_controller.get_name() + "_data.xlsx"
        )

        create_plot()
        
        # gantry_crane.reset_position()

    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
