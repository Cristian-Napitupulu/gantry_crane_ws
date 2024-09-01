#!/usr/bin/env python3

from gantry_crane_lib.ros_connector import GantryCraneConnector, GantryControlModes
from gantry_crane_lib.ros_connector import (
    MIN_TROLLEY_POSITION,
    MAX_TROLLEY_POSITION,
    MIN_CABLE_LENGTH,
    MAX_CABLE_LENGTH,
)
from gantry_crane_lib.logger import Logger

from sliding_mode_controller import non_linear_motor_model, sliding_mode_controller

from pinn_controller import pinn_controller

import random
import time

import utility

gantry_crane_connector = GantryCraneConnector()

LOG_FOLDER_PATH = (
    "/home/icodes/Documents/gantry_crane_ws/src/gantry_crane_controller/log"
)

gantry_crane_logger = Logger(LOG_FOLDER_PATH)

gantry_crane_model = non_linear_motor_model  # Change this to change the model
gantry_crane_controller = (
    sliding_mode_controller  # Change this to change the controller
)

PLAY_DURATION = 30.0
MAX_ERROR_TROLLEY_POSITION = 0.005
MAX_ERROR_CABLE_LENGTH = 0.01

current_time = time.time()


def send_command_and_collect_data(
    mode=GantryControlModes.IDLE_MODE, trolley_pwm=0, hoist_pwm=0
):
    # Print for debugging every 200 ms
    global current_time
    if (time.time() - current_time) * 1000 > 200:
        # Print current PWM
        print(
            "Collecting data... Current PWM: trolley: {}, hoist: {}".format(
                int(trolley_pwm), int(hoist_pwm)
            )
        )
        current_time = time.time()

    # Publish motor PWM
    gantry_crane_connector.publish_command(mode, trolley_pwm, hoist_pwm)

    # Collect data
    utility.collect_data(
        gantry_crane_logger, gantry_crane_connector, gantry_crane_controller
    )


def wait(duration=5.0):
    start_time = time.time()
    while time.time() - start_time < duration:
        send_command_and_collect_data(GantryControlModes.IDLE_MODE, 0, 0)


def sweep_trolley_motor_pwm(pwm_range=[0, 1023], increment=25, timeout_sec=5.0):
    min_pwm = int(abs(pwm_range[0]))
    max_pwm = int(abs(pwm_range[1]) + abs(increment))

    increment = int(increment)
    if increment < 0:
        rising = False
    else:
        rising = True

    if rising:
        timer_begin = time.time()
        while time.time() - timer_begin <= 1.0:
            gantry_crane_connector.idle()
        for pwm in range(min_pwm, max_pwm, increment):
            # print("Current PWM: {}".format(pwm))
            start_time = time.time()
            while (
                gantry_crane_connector.variables_value["trolley_position"]
                < MAX_TROLLEY_POSITION - 0.25
                and time.time() - start_time < timeout_sec
            ):
                send_command_and_collect_data(GantryControlModes.CONTROL_MODE, pwm, 0)

            if (
                gantry_crane_connector.variables_value["trolley_position"]
                >= MAX_TROLLEY_POSITION - 0.25
            ):
                gantry_crane_connector.brake()
                gantry_crane_connector.move_trolley_to_origin()
                time.sleep(1.0)

            if timeout_sec >= 5.0:
                timer_reset = time.time()
                while time.time() - timer_reset < 1.0:
                    send_command_and_collect_data(GantryControlModes.IDLE_MODE, 0, 0)
                gantry_crane_connector.move_trolley_to_origin()
                time.sleep(1.0)

    else:
        timer_begin = time.time()
        while time.time() - timer_begin <= 1.0:
            gantry_crane_connector.idle()
        for pwm in range(-min_pwm, -max_pwm, increment):
            # print("Current PWM: {}".format(pwm))
            start_time = time.time()
            while (
                gantry_crane_connector.variables_value["trolley_position"]
                > MIN_TROLLEY_POSITION + 0.25
                and time.time() - start_time < timeout_sec
            ):
                send_command_and_collect_data(GantryControlModes.CONTROL_MODE, pwm, 0)

            if (
                gantry_crane_connector.variables_value["trolley_position"]
                <= MIN_TROLLEY_POSITION + 0.25
            ):
                gantry_crane_connector.brake()
                gantry_crane_connector.move_trolley_to_end()
                time.sleep(1.0)

            if timeout_sec >= 1.0:
                timer_reset = time.time()
                while time.time() - timer_reset < 1.0:
                    send_command_and_collect_data(GantryControlModes.IDLE_MODE, 0, 0)
                gantry_crane_connector.move_trolley_to_end()
                time.sleep(1.0)


def sweep_hoist_motor_pwm(pwm_range=[0, 1023], increment=25, timeout_sec=5.0):
    min_pwm = int(abs(pwm_range[0]))
    max_pwm = int(abs(pwm_range[1]) + abs(increment))

    increment = int(increment)
    if increment < 0:
        rising = False
    else:
        rising = True

    if rising:  # Lower container
        timer_begin = time.time()
        while time.time() - timer_begin <= 1.0:
            gantry_crane_connector.idle()
        for pwm in range(min_pwm, max_pwm, increment):
            # print("Current PWM: {}".format(pwm))
            start_time = time.time()
            while (
                gantry_crane_connector.variables_value["cable_length"]
                < MAX_CABLE_LENGTH
                and time.time() - start_time < timeout_sec
            ):
                send_command_and_collect_data(GantryControlModes.CONTROL_MODE, 0, pwm)

            if (
                gantry_crane_connector.variables_value["cable_length"]
                >= MAX_CABLE_LENGTH
            ):
                gantry_crane_connector.brake()
                gantry_crane_connector.hoist_container_to_top()
                time.sleep(1.0)

            if timeout_sec >= 1.0:
                timer_reset = time.time()
                while time.time() - timer_reset < 1.0:
                    send_command_and_collect_data(GantryControlModes.IDLE_MODE, 0, 0)
                gantry_crane_connector.hoist_container_to_top()
                time.sleep(1.0)

    else:  # Raise container
        timer_begin = time.time()
        while time.time() - timer_begin <= 1.0:
            gantry_crane_connector.idle()
        for pwm in range(-min_pwm, -max_pwm, increment):
            # print("Current PWM: {}".format(pwm))
            start_time = time.time()
            while (
                gantry_crane_connector.variables_value["cable_length"]
                > MIN_CABLE_LENGTH
                and time.time() - start_time < timeout_sec
            ):
                send_command_and_collect_data(GantryControlModes.CONTROL_MODE, 0, pwm)

            if (
                gantry_crane_connector.variables_value["cable_length"]
                <= MIN_CABLE_LENGTH
            ):
                gantry_crane_connector.brake()
                gantry_crane_connector.hoist_container_to_bottom()
                time.sleep(1.0)

            if timeout_sec >= 1.0:
                timer_reset = time.time()
                while time.time() - timer_reset < 1.0:
                    send_command_and_collect_data(GantryControlModes.IDLE_MODE, 0, 0)
                gantry_crane_connector.hoist_container_to_bottom()
                time.sleep(1.0)


def step_response_trolley_motor_pwm(
    pwm=0, timeout_sec=20.0, enable_random=False, random_range=50
):
    if pwm < 0:
        rising = False
    else:
        rising = True

    if rising:
        timer_begin = time.time()
        while time.time() - timer_begin <= 1.0:
            gantry_crane_connector.idle()

        timer_operate = time.time()
        while time.time() - timer_operate < timeout_sec:
            pwm_input = pwm
            if enable_random:
                pwm_random_offset = int(random.uniform(-random_range, random_range))
                pwm_input = pwm + pwm_random_offset

            if (
                gantry_crane_connector.variables_value["trolley_position"]
                <= MAX_TROLLEY_POSITION - 0.25
            ):
                send_command_and_collect_data(
                    GantryControlModes.CONTROL_MODE, pwm_input, 0
                )
            else:
                send_command_and_collect_data(GantryControlModes.IDLE_MODE, 0, 0)

        timer_end = time.time()
        while time.time() - timer_end <= 1.0:
            send_command_and_collect_data(GantryControlModes.IDLE_MODE, 0, 0)
        time.sleep(1.0)

    else:
        timer_begin = time.time()
        while time.time() - timer_begin <= 1.0:
            gantry_crane_connector.idle()

        timer_operate = time.time()
        while time.time() - timer_operate < timeout_sec:
            pwm_input = pwm
            if enable_random:
                pwm_random_offset = int(random.uniform(-random_range, random_range))
                pwm_input = pwm + pwm_random_offset

            if (
                gantry_crane_connector.variables_value["trolley_position"]
                >= MIN_TROLLEY_POSITION + 0.25
            ):
                send_command_and_collect_data(
                    GantryControlModes.CONTROL_MODE, pwm_input, 0
                )
            else:
                send_command_and_collect_data(GantryControlModes.IDLE_MODE, 0, 0)

        timer_end = time.time()
        while time.time() - timer_end <= 1.0:
            send_command_and_collect_data(GantryControlModes.IDLE_MODE, 0, 0)
        time.sleep(1.0)


def step_response_hoist_motor_pwm(
    pwm=0, timeout_sec=20.0, enable_random=False, random_range=50
):
    if pwm < 0:
        extend = False
    else:
        extend = True

    if extend:
        timer_begin = time.time()
        while time.time() - timer_begin <= 1.0:
            gantry_crane_connector.idle()

        timer_operate = time.time()
        while time.time() - timer_operate < timeout_sec:
            pwm_input = pwm
            if enable_random:
                pwm_random_offset = int(random.uniform(-random_range, random_range))
                pwm_input = pwm + pwm_random_offset

            if (
                gantry_crane_connector.variables_value["cable_length"]
                <= MAX_CABLE_LENGTH
            ):
                send_command_and_collect_data(
                    GantryControlModes.CONTROL_MODE, 0, pwm_input
                )
            else:
                send_command_and_collect_data(GantryControlModes.IDLE_MODE, 0, 0)

        timer_end = time.time()
        while time.time() - timer_end <= 1.0:
            send_command_and_collect_data(GantryControlModes.IDLE_MODE, 0, 0)
        time.sleep(1.0)

    else:
        timer_begin = time.time()
        while time.time() - timer_begin <= 1.0:
            gantry_crane_connector.idle()

        timer_operate = time.time()
        while time.time() - timer_operate < timeout_sec:
            pwm_input = pwm
            if enable_random:
                pwm_random_offset = int(random.uniform(-random_range, random_range))
                pwm_input = pwm + pwm_random_offset

            if (
                gantry_crane_connector.variables_value["cable_length"]
                >= MIN_CABLE_LENGTH
            ):
                send_command_and_collect_data(
                    GantryControlModes.CONTROL_MODE, 0, pwm_input
                )
            else:
                send_command_and_collect_data(GantryControlModes.IDLE_MODE, 0, 0)

        timer_end = time.time()
        while time.time() - timer_end <= 1.0:
            send_command_and_collect_data(GantryControlModes.IDLE_MODE, 0, 0)
        time.sleep(1.0)


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

        send_command_and_collect_data(
            GantryControlModes.CONTROL_MODE, trolley_motor_pwm, hoist_motor_pwm
        )

        # Check if timeout
        if time.time() - start_time > timeout_sec:
            result = "Timeout. Final error: {}, {}.".format(
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
            result = "Desired position reached. Final error: {}, {}.".format(
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

    gantry_crane_connector.brake()
    return result


NUM_REPEAT = 3

if __name__ == "__main__":
    gantry_crane_model.attach_connector(gantry_crane_connector)
    print("Gantry crane model initialized: {}".format(gantry_crane_model.get_name()))

    gantry_crane_controller.attach_connector(gantry_crane_connector)
    gantry_crane_controller.attach_model(gantry_crane_model)
    print("Controller initialized: {}".format(gantry_crane_controller.get_name()))

    # Initialize gantry crane
    gantry_crane_connector.begin(hoist_to_position="bottom")
    time.sleep(15.0)
    try:
        """
        The following code block is used to test the gantry crane's motors.
        """

        # """ 1. Sweep trolley motor PWM """
        # for i in range(NUM_REPEAT):
        #     gantry_crane_connector.move_trolley_to_origin()
        #     time.sleep(10.0)
        #     gantry_crane_logger.reset_buffers()
        #     gantry_crane_logger.reset_timer()
        #     sweep_trolley_motor_pwm(pwm_range=[0, 451], increment=25, timeout_sec=10.0)
        #     gantry_crane_logger.write_buffers_to_excel(
        #         "trolley_motor_sweep_response.xlsx"
        #     )
        #     utility.create_plot(gantry_crane_logger)

        # """ 2. Sweep hoist motor PWM extend (lower container) """
        # gantry_crane_connector.move_trolley_to_middle()
        # for i in range(NUM_REPEAT):
        #     gantry_crane_connector.hoist_container_to_top()
        #     time.sleep(10.0)
        #     gantry_crane_logger.reset_buffers()
        #     gantry_crane_logger.reset_timer()
        #     sweep_hoist_motor_pwm(pwm_range=[0, 501], increment=10, timeout_sec=0.3)
        #     gantry_crane_connector.brake()
        #     gantry_crane_logger.write_buffers_to_excel(
        #         "hoist_motor_extend_sweep_response.xlsx"
        #     )
        #     utility.create_plot(gantry_crane_logger)

        # """ 3. Sweep hoist motor PWM shrink (raise container) """
        # gantry_crane_connector.move_trolley_to_middle()
        # for i in range(NUM_REPEAT):
        #     gantry_crane_connector.hoist_container_to_bottom()
        #     time.sleep(10.0)
        #     gantry_crane_logger.reset_buffers()
        #     gantry_crane_logger.reset_timer()
        #     sweep_hoist_motor_pwm(pwm_range=[-700, -901], increment=-40, timeout_sec=7.5)
        #     gantry_crane_connector.brake()
        #     gantry_crane_logger.write_buffers_to_excel(
        #         "hoist_motor_shrink_sweep_response.xlsx"
        #     )
        #     utility.create_plot(gantry_crane_logger)

        # """ 4. Step response trolley motor PWM """
        # for step_pwm in range(600, 751, 25):
        #     for i in range(NUM_REPEAT):
        #         gantry_crane_connector.move_trolley_to_origin()
        #         gantry_crane_connector.idle()
        #         time.sleep(10.0)
        #         gantry_crane_logger.reset_buffers()
        #         gantry_crane_logger.reset_timer()
        #         step_response_trolley_motor_pwm(
        #             pwm=step_pwm, timeout_sec=20, enable_random=True, random_range=50
        #         )
        #         gantry_crane_logger.write_buffers_to_excel(
        #             f"trolley_step_response_{step_pwm}_with_random.xlsx"
        #         )
        #         utility.create_plot(gantry_crane_logger)

        # """ 5. Step response hoist motor PWM extend (lower container) """
        # gantry_crane_connector.move_trolley_to_middle()
        # for step_pwm in range(200, 350, 25):
        #     for i in range(NUM_REPEAT):
        #         gantry_crane_connector.hoist_container_to_top()
        #         time.sleep(10.0)
        #         gantry_crane_logger.reset_buffers()
        #         gantry_crane_logger.reset_timer()
        #         step_response_hoist_motor_pwm(
        #             pwm=step_pwm, timeout_sec=20, enable_random=True, random_range=50
        #         )
        #         gantry_crane_logger.write_buffers_to_excel(
        #             f"hoist_extend_step_response_{step_pwm}_with_random.xlsx"
        #         )
        #         utility.create_plot(gantry_crane_logger)

        # """ 6. Step response hoist motor PWM shrink (raise container) """
        # gantry_crane_connector.move_trolley_to_middle()
        # for step_pwm in range(200, 350, 25):
        #     for i in range(NUM_REPEAT):
        #         gantry_crane_connector.hoist_container_to_bottom()
        #         time.sleep(10.0)
        #         gantry_crane_logger.reset_buffers()
        #         gantry_crane_logger.reset_timer()
        #         step_response_hoist_motor_pwm(
        #             pwm=-step_pwm, timeout_sec=20, enable_random=True, random_range=50
        #         )
        #         gantry_crane_logger.write_buffers_to_excel(
        #             f"hoist_shrink_step_response_{step_pwm}_with_random.xlsx"
        #         )
        #         utility.create_plot(gantry_crane_logger)

        gantry_crane_logger.reset_buffers()
        gantry_crane_logger.reset_timer()

        wait(5.0)

        """
        The following code block is used to control the gantry crane to move to three different positions.
        """
        # Control gantry crane to first position
        result = control_gantry_crane(
            timeout_sec=PLAY_DURATION,
            desired_trolley_position=1.0,
            desired_cable_length=0.4,
            max_trolley_position_steady_state_error=MAX_ERROR_TROLLEY_POSITION,
            max_cable_length_steady_state_error=MAX_ERROR_CABLE_LENGTH,
        )
        print("Target first position. Result: {}".format(result))

        wait(5.0)

        # Control gantry crane to second position
        result = control_gantry_crane(
            timeout_sec=PLAY_DURATION,
            desired_trolley_position=0.25,
            desired_cable_length=0.5,
            max_trolley_position_steady_state_error=MAX_ERROR_TROLLEY_POSITION,
            max_cable_length_steady_state_error=MAX_ERROR_CABLE_LENGTH,
        )
        print("Target second position. Result: {}".format(result))

        wait(5.0)

        # # Control gantry crane to third position
        # result = control_gantry_crane(
        #     timeout_sec=PLAY_DURATION,
        #     desired_trolley_position=1.35,
        #     desired_cable_length=0.65,
        #     max_trolley_position_steady_state_error=MAX_ERROR_TROLLEY_POSITION,
        #     max_cable_length_steady_state_error=MAX_ERROR_CABLE_LENGTH,
        # )
        # print("Target third position. Result: {}".format(result))

        gantry_crane_logger.write_buffers_to_excel(
            gantry_crane_controller.get_name() + "_control_gantry_crane_data_light.xlsx"
        )

    except KeyboardInterrupt:
        pass

    gantry_crane_connector.end()
