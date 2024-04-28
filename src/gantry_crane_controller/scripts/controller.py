#!/usr/bin/env python3

from gantry_crane_lib.ros_connector import GantryCraneConnector, GantryControlModes
from gantry_crane_lib.logger import Logger

from sliding_mode_controller import non_linear_motor_model, sliding_mode_controller

import rclpy
import time

import numpy as np

gantry_crane_connector = GantryCraneConnector()

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

gantry_crane_model = non_linear_motor_model  # Change this to change the model
gantry_crane_controller = (
    sliding_mode_controller  # Change this to change the controller
)

gantry_crane_logger = Logger(LOG_FOLDER_PATH)

PLAY_DURATION = 10.0
MAX_ERROR_TROLLEY_POSITION = 0.05
MAX_ERROR_CABLE_LENGTH = 0.05


def collect_data():
    gantry_crane_logger.add_to_buffer("timestamp", gantry_crane_logger.get_time_sec())

    gantry_crane_logger.add_to_buffer(
        "control_signal_trolley", gantry_crane_controller.control_signal_trolley
    )

    gantry_crane_logger.add_to_buffer(
        "control_signal_hoist", gantry_crane_controller.control_signal_hoist
    )

    gantry_crane_logger.add_to_buffer(
        "pwm_trolley_motor", gantry_crane_connector.control_pwm["trolley_motor"]
    )

    gantry_crane_logger.add_to_buffer(
        "pwm_hoist_motor", gantry_crane_connector.control_pwm["hoist_motor"]
    )

    gantry_crane_logger.add_to_buffer(
        "trolley_motor_voltage",
        gantry_crane_connector.variables_value["trolley_motor_voltage"],
    )

    gantry_crane_logger.add_to_buffer(
        "hoist_motor_voltage",
        gantry_crane_connector.variables_value["hoist_motor_voltage"],
    )

    gantry_crane_logger.add_to_buffer(
        "trolley_position", gantry_crane_connector.variables_value["trolley_position"]
    )

    gantry_crane_logger.add_to_buffer(
        "cable_length", gantry_crane_connector.variables_value["cable_length"]
    )

    gantry_crane_logger.add_to_buffer(
        "sway_angle", gantry_crane_connector.variables_value["sway_angle"]
    )

    gantry_crane_logger.add_to_buffer(
        "trolley_position_first_derivative",
        gantry_crane_connector.variables_first_derivative["trolley_position"],
    )

    gantry_crane_logger.add_to_buffer(
        "cable_length_first_derivative",
        gantry_crane_connector.variables_first_derivative["cable_length"],
    )

    gantry_crane_logger.add_to_buffer(
        "sway_angle_first_derivative",
        gantry_crane_connector.variables_first_derivative["sway_angle"],
    )

    gantry_crane_logger.add_to_buffer(
        "trolley_position_second_derivative",
        gantry_crane_connector.variables_second_derivative["trolley_position"],
    )

    gantry_crane_logger.add_to_buffer(
        "cable_length_second_derivative",
        gantry_crane_connector.variables_second_derivative["cable_length"],
    )

    gantry_crane_logger.add_to_buffer(
        "sway_angle_second_derivative",
        gantry_crane_connector.variables_second_derivative["sway_angle"],
    )

    gantry_crane_logger.add_to_buffer(
        "trolley_position_third_derivative",
        gantry_crane_connector.variables_third_derivative["trolley_position"],
    )

    gantry_crane_logger.add_to_buffer(
        "cable_length_third_derivative",
        gantry_crane_connector.variables_third_derivative["cable_length"],
    )

    gantry_crane_logger.add_to_buffer(
        "sway_angle_third_derivative",
        gantry_crane_connector.variables_third_derivative["sway_angle"],
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
    # Print current PWM
    print(
        "Collecting data... Current PWM: trolley: {}, hoist: {}".format(
            trolley_pwm, hoist_pwm
        )
    )

    # Publish motor PWM
    gantry_crane_connector.publish_command(mode, trolley_pwm, hoist_pwm)

    # Collect data
    collect_data()


def sweep_trolley_motor_pwm(pwm_range=[0, 1023], increment=25, timeout_sec=5.0):
    min_pwm = int(abs(pwm_range[0]))
    max_pwm = int(abs(pwm_range[1]) + abs(increment))

    increment = int(increment)
    if increment < 0:
        rising = False
    else:
        rising = True

    if rising:
        for pwm in range(min_pwm, max_pwm, increment):
            # print("Current PWM: {}".format(pwm))
            start_time = time.time()
            while (
                gantry_crane_connector.variables_value["trolley_position"] < 0.75
                and time.time() - start_time < timeout_sec
            ):
                send_command_and_collect_data(GantryControlModes.CONTROL_MODE, pwm, 0)

            if gantry_crane_connector.variables_value["trolley_position"] >= 0.75:
                gantry_crane_connector.brake()
                gantry_crane_connector.move_trolley_to_origin()

            if timeout_sec >= 1.0:
                timer_reset = time.time()
                while time.time() - timer_reset < 1.0:
                    send_command_and_collect_data(GantryControlModes.IDLE_MODE, 0, 0)
                gantry_crane_connector.move_trolley_to_origin()

    else:
        for pwm in range(-min_pwm, -max_pwm, increment):
            # print("Current PWM: {}".format(pwm))
            start_time = time.time()
            while (
                gantry_crane_connector.variables_value["trolley_position"] > 0.75
                and time.time() - start_time < timeout_sec
            ):
                send_command_and_collect_data(GantryControlModes.CONTROL_MODE, pwm, 0)

            if gantry_crane_connector.variables_value["trolley_position"] <= 0.75:
                gantry_crane_connector.brake()
                gantry_crane_connector.move_trolley_to_end()

            if timeout_sec >= 1.0:
                timer_reset = time.time()
                while time.time() - timer_reset < 1.0:
                    send_command_and_collect_data(GantryControlModes.IDLE_MODE, 0, 0)
                gantry_crane_connector.move_trolley_to_end()


def sweep_hoist_motor_pwm(pwm_range=[0, 1023], increment=25, timeout_sec=5.0):
    min_pwm = int(abs(pwm_range[0]))
    max_pwm = int(abs(pwm_range[1]) + abs(increment))

    increment = int(increment)
    if increment < 0:
        rising = False
    else:
        rising = True

    if rising:  # Lower container
        for pwm in range(min_pwm, max_pwm, increment):
            # print("Current PWM: {}".format(pwm))
            start_time = time.time()
            while (
                gantry_crane_connector.variables_value["cable_length"] < 0.60
                and time.time() - start_time < timeout_sec
            ):
                send_command_and_collect_data(GantryControlModes.CONTROL_MODE, 0, pwm)

            if gantry_crane_connector.variables_value["cable_length"] >= 0.60:
                gantry_crane_connector.brake()
                gantry_crane_connector.hoist_container_to_top()

            if timeout_sec >= 1.0:
                timer_reset = time.time()
                while time.time() - timer_reset < 1.0:
                    send_command_and_collect_data(GantryControlModes.IDLE_MODE, 0, 0)
                gantry_crane_connector.hoist_container_to_top()

    else:  # Raise container
        for pwm in range(-min_pwm, -max_pwm, increment):
            # print("Current PWM: {}".format(pwm))
            start_time = time.time()
            while (
                gantry_crane_connector.variables_value["cable_length"] > 0.40
                and time.time() - start_time < timeout_sec
            ):
                send_command_and_collect_data(GantryControlModes.CONTROL_MODE, 0, pwm)

            if gantry_crane_connector.variables_value["cable_length"] <= 0.40:
                gantry_crane_connector.brake()
                gantry_crane_connector.hoist_container_to_bottom()

            if timeout_sec >= 1.0:
                timer_reset = time.time()
                while time.time() - timer_reset < 1.0:
                    send_command_and_collect_data(GantryControlModes.IDLE_MODE, 0, 0)
                gantry_crane_connector.hoist_container_to_bottom()


def control_gantry_crane(
    timeout_sec=15.0,
    desired_trolley_position=1.0,
    desired_cable_length=None,
    max_trolley_position_steady_state_error=0.1,
    max_cable_length_steady_state_error=0.1,
):
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

        gantry_crane_connector.publish_command(
            GantryControlModes.CONTROL_MODE, trolley_motor_pwm, hoist_motor_pwm
        )
        send_command_and_collect_data(
            GantryControlModes.CONTROL_MODE, trolley_motor_pwm, hoist_motor_pwm
        )

        # Check if timeout
        if time.time() - start_time > timeout_sec:
            result = "timeout. Final error: {}, {}.".format(
                abs(
                    gantry_crane_connector.variables_value["trolley_position"]
                    - desired_trolley_position
                ),
                abs(
                    gantry_crane_connector.variables_value["cable_length"]
                    - desired_cable_length
                ),
            )
            break

        if inside_max_error and time.time() - time_inside_max_error > 5:
            result = "desired position reached. Final error: {}, {}.".format(
                abs(
                    gantry_crane_connector.variables_value["trolley_position"]
                    - desired_trolley_position
                ),
                abs(
                    gantry_crane_connector.variables_value["cable_length"]
                    - desired_cable_length
                ),
            )
            break

        if desired_cable_length is None:
            if (
                abs(
                    gantry_crane_connector.variables_value["trolley_position"]
                    - desired_trolley_position
                )
                < max_trolley_position_steady_state_error
            ):
                inside_max_error = True
                time_inside_max_error = time.time()

            else:
                inside_max_error = False
        else:
            if (
                abs(
                    gantry_crane_connector.variables_value["trolley_position"]
                    - desired_trolley_position
                )
                < max_trolley_position_steady_state_error
                and abs(
                    gantry_crane_connector.variables_value["cable_length"]
                    - desired_cable_length
                )
                < max_cable_length_steady_state_error
            ):
                inside_max_error = True
                time_inside_max_error = time.time()

            else:
                inside_max_error = False

    return result


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


if __name__ == "__main__":
    gantry_crane_model.attach_connector(gantry_crane_connector)
    print("Gantry crane model initialized: {}".format(gantry_crane_model.get_name()))

    gantry_crane_controller.attach_connector(gantry_crane_connector)
    gantry_crane_controller.attach_model(gantry_crane_model)
    print("Controller initialized: {}".format(gantry_crane_controller.get_name()))

    # Initialize gantry crane
    gantry_crane_connector.begin(slide_to_position="middle", hoist_to_position="top")
    time.sleep(5.0)

    try:
        # Begin logger timer
        gantry_crane_logger.reset_timer()

        for i in range(5):
            gantry_crane_connector.move_trolley_to_origin()
            gantry_crane_connector.idle()
            time.sleep(5.0)
            # Sweep trolley motor PWM
            gantry_crane_logger.reset_timer()
            sweep_trolley_motor_pwm(pwm_range=[0, 700], increment=1, timeout_sec=0.1)
            gantry_crane_logger.write_buffers_to_excel(
                gantry_crane_controller.get_name()
                + "_rising_sweep_trolley_data_1_increment.xlsx"
            )
            gantry_crane_logger.reset_buffers()

        # for i in range(5):
        #     gantry_crane_connector.move_trolley_to_end()
        #     gantry_crane_connector.idle()
        #     time.sleep(5.0)
        #     # Sweep trolley motor PWM
        #     gantry_crane_logger.reset_timer()
        #     sweep_trolley_motor_pwm(pwm_range=[0, -700], increment=-1, timeout_sec=0.1)
        #     gantry_crane_logger.write_buffers_to_excel(
        #         gantry_crane_controller.get_name()
        #         + "_falling_sweep_trolley_data_1_increment.xlsx"
        #     )
        #     gantry_crane_logger.reset_buffers()

        # for i in range(5):
        #     gantry_crane_connector.move_trolley_to_origin()
        #     gantry_crane_connector.idle()
        #     time.sleep(5.0)
        #     # Sweep trolley motor PWM
        #     gantry_crane_logger.reset_timer()
        #     sweep_trolley_motor_pwm(pwm_range=[0, 700], increment=10, timeout_sec=2.0)
        #     gantry_crane_logger.write_buffers_to_excel(
        #         gantry_crane_controller.get_name()
        #         + "_rising_sweep_trolley_data_10_increment.xlsx"
        #     )
        #     gantry_crane_logger.reset_buffers()

        # for i in range(5):
        #     gantry_crane_connector.move_trolley_to_end()
        #     gantry_crane_connector.idle()
        #     time.sleep(5.0)
        #     # Sweep trolley motor PWM
        #     gantry_crane_logger.reset_timer()
        #     sweep_trolley_motor_pwm(pwm_range=[0, -700], increment=-10, timeout_sec=2.0)
        #     gantry_crane_logger.write_buffers_to_excel(
        #         gantry_crane_controller.get_name()
        #         + "_falling_sweep_trolley_data_10_increment.xlsx"
        #     )
        #     gantry_crane_logger.reset_buffers()

        # gantry_crane_connector.move_trolley_to_middle()
        # for i in range(5):
        #     gantry_crane_connector.hoist_container_to_top()
        #     gantry_crane_connector.idle()
        #     time.sleep(5.0)
        #     # Sweep hoist motor PWM
        #     gantry_crane_logger.reset_timer()
        #     sweep_hoist_motor_pwm(pwm_range=[0, 800], increment=1, timeout_sec=0.1)
        #     gantry_crane_logger.write_buffers_to_excel(
        #         gantry_crane_controller.get_name()
        #         + "_rising_sweep_hoist_data_1_increment.xlsx"
        #     )
        #     gantry_crane_logger.reset_buffers()

        # gantry_crane_connector.move_trolley_to_middle()
        # for i in range(5):
        #     gantry_crane_connector.hoist_container_to_bottom()
        #     gantry_crane_connector.idle()
        #     time.sleep(5.0)
        #     # Sweep hoist motor PWM
        #     gantry_crane_logger.reset_timer()
        #     sweep_hoist_motor_pwm(pwm_range=[0, -800], increment=-1, timeout_sec=0.1)
        #     gantry_crane_logger.write_buffers_to_excel(
        #         gantry_crane_controller.get_name()
        #         + "_falling_sweep_hoist_data_1_increment.xlsx"
        #     )
        #     gantry_crane_logger.reset_buffers()

        # gantry_crane_connector.move_trolley_to_middle()
        # for i in range(5):
        #     gantry_crane_connector.hoist_container_to_top()
        #     gantry_crane_connector.idle()
        #     time.sleep(5.0)
        #     # Sweep hoist motor PWM
        #     gantry_crane_logger.reset_timer()
        #     sweep_hoist_motor_pwm(pwm_range=[0, 800], increment=10, timeout_sec=2.0)
        #     gantry_crane_logger.write_buffers_to_excel(
        #         gantry_crane_controller.get_name()
        #         + "_rising_sweep_hoist_data_10_increment.xlsx"
        #     )
        #     gantry_crane_logger.reset_buffers()

        # gantry_crane_connector.move_trolley_to_middle()
        # for i in range(5):
        #     gantry_crane_connector.hoist_container_to_bottom()
        #     gantry_crane_connector.idle()
        #     time.sleep(5.0)
        #     # Sweep hoist motor PWM
        #     gantry_crane_logger.reset_timer()
        #     sweep_hoist_motor_pwm(pwm_range=[0, -800], increment=-10, timeout_sec=2.0)
        #     gantry_crane_logger.write_buffers_to_excel(
        #         gantry_crane_controller.get_name()
        #         + "_falling_sweep_hoist_data_10_increment.xlsx"
        #     )
        #     gantry_crane_logger.reset_buffers()

        # Control gantry crane to first position
        result = control_gantry_crane(
            timeout_sec=PLAY_DURATION,
            desired_trolley_position=0.375,
            desired_cable_length=0.55,
            max_trolley_position_steady_state_error=MAX_ERROR_TROLLEY_POSITION,
            max_cable_length_steady_state_error=MAX_ERROR_CABLE_LENGTH,
        )
        print("First position reached. Result: {}".format(result))

        time.sleep(5.0)  # Wait for 5 seconds

        # Control gantry crane to second position
        result = control_gantry_crane(
            timeout_sec=PLAY_DURATION,
            desired_trolley_position=0.75,
            desired_cable_length=0.5,
            max_trolley_position_steady_state_error=MAX_ERROR_TROLLEY_POSITION,
            max_cable_length_steady_state_error=MAX_ERROR_CABLE_LENGTH,
        )
        print("Second position reached. Result: {}".format(result))

        time.sleep(5.0)  # Wait for 5 seconds

        # Control gantry crane to third position
        result = control_gantry_crane(
            timeout_sec=PLAY_DURATION,
            desired_trolley_position=1.125,
            desired_cable_length=0.4,
            max_trolley_position_steady_state_error=MAX_ERROR_TROLLEY_POSITION,
            max_cable_length_steady_state_error=MAX_ERROR_CABLE_LENGTH,
        )
        print("Third position reached. Result: {}".format(result))

        gantry_crane_logger.write_buffers_to_excel(
            gantry_crane_controller.get_name() + "_sweep_trolley_data.xlsx"
        )

        # create_plot()

    except KeyboardInterrupt:
        pass

    # gantry_crane_connector.move_trolley_to_middle()
    # gantry_crane_connector.hoist_container_to_top()
    rclpy.shutdown()
