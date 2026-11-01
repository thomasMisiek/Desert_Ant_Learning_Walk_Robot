from roboclaw import Roboclaw

import os
import time

import robot_config as config
import robot_state as state


# ============================================================
# GENERAL FUNCTIONS
# ============================================================

def clamp(value, minimum, maximum):
    return max(
        minimum,
        min(
            maximum,
            value,
        ),
    )


def apply_deadzone(value):
    if abs(value) < config.DEADZONE:
        return 0.0

    return value


# ============================================================
# SHARED MOTOR COMMAND
# ============================================================

def set_motor_command(lin, strafe, rot):
    with state.motor_command_lock:
        state.latest_motor_command["lin"] = clamp(
            lin,
            -1.0,
            1.0,
        )

        state.latest_motor_command["strafe"] = clamp(
            strafe,
            -1.0,
            1.0,
        )

        state.latest_motor_command["rot"] = clamp(
            rot,
            -1.0,
            1.0,
        )

        state.latest_motor_command["timestamp"] = time.time()


def get_motor_command():
    with state.motor_command_lock:
        return (
            state.latest_motor_command["lin"],
            state.latest_motor_command["strafe"],
            state.latest_motor_command["rot"],
            state.latest_motor_command["timestamp"],
        )


def reset_motor_command():
    set_motor_command(
        0.0,
        0.0,
        0.0,
    )


# ============================================================
# ROBOCLAW CONNECTION MANAGEMENT
# ============================================================

def open_roboclaw_once(controller_port):
    controller = None

    try:
        controller = Roboclaw(
            controller_port,
            config.ROBOCLAW_BAUDRATE,
            retries=1,
        )

        if controller.Open() == 1:
            return controller

    except Exception:
        pass

    if controller is not None:
        try:
            controller.Close()
        except Exception:
            pass

    return None


def disconnect_controller(
    controller_name,
    expected_controller=None,
):
    controller = None

    with state.controllers_lock:
        current_controller = state.controllers.get(
            controller_name
        )

        if (
            expected_controller is not None
            and current_controller is not expected_controller
        ):
            return

        controller = current_controller
        state.controllers[controller_name] = None

    if controller is not None:
        try:
            controller.Close()
        except Exception:
            pass


def roboclaw_connection_manager():
    while not state.stop_event.is_set():
        for controller_name in (
            "front",
            "rear",
        ):
            controller_port = config.CONTROLLER_PORTS[
                controller_name
            ]

            with state.controllers_lock:
                controller = state.controllers.get(
                    controller_name
                )

            if (
                controller is not None
                and not os.path.exists(controller_port)
            ):
                disconnect_controller(
                    controller_name,
                    controller,
                )

                controller = None

            if (
                controller is None
                and os.path.exists(controller_port)
            ):
                new_controller = open_roboclaw_once(
                    controller_port
                )

                if new_controller is not None:
                    with state.controllers_lock:
                        if state.controllers[controller_name] is None:
                            state.controllers[
                                controller_name
                            ] = new_controller

                            # Debug RoboClaw connection:
                            # print("[ROBOCLAW] Connected:", controller_name, controller_port)
                        else:
                            try:
                                new_controller.Close()
                            except Exception:
                                pass

        time.sleep(
            config.DEVICE_CHECK_PERIOD
        )


# ============================================================
# MOTOR CONTROL
# ============================================================

def mecanum_mix(
    forward,
    strafe,
    rotation,
):
    wheel_commands = {
        "front_left": (
            forward
            + strafe
            + rotation
        ),
        "front_right": (
            forward
            - strafe
            - rotation
        ),
        "rear_left": (
            forward
            - strafe
            + rotation
        ),
        "rear_right": (
            forward
            + strafe
            - rotation
        ),
    }

    largest_command = max(
        1.0,
        max(
            abs(command)
            for command in wheel_commands.values()
        ),
    )

    for wheel_name in wheel_commands:
        wheel_commands[wheel_name] /= largest_command

    return wheel_commands


def calculate_controller_duties(
    forward,
    strafe,
    rotation,
):
    wheel_commands = mecanum_mix(
        forward,
        strafe,
        rotation,
    )

    controller_duties = {
        "front": {
            "M1": 0,
            "M2": 0,
        },
        "rear": {
            "M1": 0,
            "M2": 0,
        },
    }

    for wheel_name in wheel_commands:
        mapping = config.MOTOR_MAP[
            wheel_name
        ]

        duty = int(
            clamp(
                wheel_commands[wheel_name],
                -1.0,
                1.0,
            )
            * config.MAX_MOTOR_DUTY
            * mapping["direction"]
        )

        controller_duties[
            mapping["controller"]
        ][mapping["channel"]] = duty

    return controller_duties


def send_controller_command(
    controller_name,
    controller,
    duties,
):
    try:
        controller.DutyM1M2(
            config.ADDRESS,
            duties["M1"],
            duties["M2"],
        )

        # Optional acknowledgement verification remains disabled.
        # It must not be enabled when _writechecksum() no longer
        # waits for a RoboClaw acknowledgement.

        return True

    except Exception:
        disconnect_controller(
            controller_name,
            controller,
        )

        return False


def stop_motors():
    with state.controllers_lock:
        front_controller = state.controllers.get(
            "front"
        )

        rear_controller = state.controllers.get(
            "rear"
        )

    if front_controller is not None:
        try:
            front_controller.DutyM1M2(
                config.ADDRESS,
                0,
                0,
            )
        except Exception:
            pass

    if rear_controller is not None:
        try:
            rear_controller.DutyM1M2(
                config.ADDRESS,
                0,
                0,
            )
        except Exception:
            pass


# ============================================================
# SINGLE MOTOR OUTPUT THREAD
# ============================================================

def motor_output_loop():
    next_update = time.time()

    smooth_lin = 0.0
    smooth_strafe = 0.0
    smooth_rot = 0.0

    while not state.stop_event.is_set():
        lin, strafe, rot, command_time = (
            get_motor_command()
        )

        if (
            command_time <= 0.0
            or time.time() - command_time
            > config.MOTOR_WATCHDOG_TIMEOUT
        ):
            lin = 0.0
            strafe = 0.0
            rot = 0.0

        # Apply the same exponential smoothing to all commands,
        # including commands that request zero.
        smooth_lin += config.MOTOR_SMOOTHING_ALPHA * (
            lin
            - smooth_lin
        )

        smooth_strafe += config.MOTOR_SMOOTHING_ALPHA * (
            strafe
            - smooth_strafe
        )

        smooth_rot += config.MOTOR_SMOOTHING_ALPHA * (
            rot
            - smooth_rot
        )

        duties = calculate_controller_duties(
            smooth_lin,
            smooth_strafe,
            smooth_rot,
        )

        with state.motor_command_lock:
            state.latest_smoothed_motor_command["lin"] = smooth_lin
            state.latest_smoothed_motor_command["strafe"] = smooth_strafe
            state.latest_smoothed_motor_command["rot"] = smooth_rot

            state.latest_motor_duties["front_m1"] = duties["front"]["M1"]
            state.latest_motor_duties["front_m2"] = duties["front"]["M2"]
            state.latest_motor_duties["rear_m1"] = duties["rear"]["M1"]
            state.latest_motor_duties["rear_m2"] = duties["rear"]["M2"]

            state.latest_motor_duties["front_right"] = duties["front"]["M1"]
            state.latest_motor_duties["front_left"] = duties["front"]["M2"]
            state.latest_motor_duties["rear_right"] = duties["rear"]["M1"]
            state.latest_motor_duties["rear_left"] = duties["rear"]["M2"]

        # Debug requested and smoothed motor commands:
        # print("[MOTOR] requested:", lin, strafe, rot, "smoothed:", smooth_lin, smooth_strafe, smooth_rot)
        # print("[MOTOR DUTY] front M1/M2:", duties["front"]["M1"], duties["front"]["M2"], "rear M1/M2:", duties["rear"]["M1"], duties["rear"]["M2"])

        with state.controllers_lock:
            front_controller = state.controllers.get(
                "front"
            )

            rear_controller = state.controllers.get(
                "rear"
            )

        if front_controller is not None:
            send_controller_command(
                "front",
                front_controller,
                duties["front"],
            )

        if rear_controller is not None:
            send_controller_command(
                "rear",
                rear_controller,
                duties["rear"],
            )

        next_update += config.MOTOR_PERIOD

        sleep_time = (
            next_update
            - time.time()
        )

        if sleep_time > 0:
            time.sleep(
                sleep_time
            )
        else:
            next_update = time.time()

    stop_motors()
