import os
import select
import time

import evdev
import numpy as np

from cx import CXRing

import motor_system
import recording_system
import robot_config as config
import robot_state as state
import vision_system


# ============================================================
# JOYSTICK STATE HELPERS
# ============================================================

def reset_axis_values():
    state.axis_values[0] = 127
    state.axis_values[1] = 127
    state.axis_values[2] = 127
    state.axis_values[5] = 127


# ============================================================
# LOGITECH CONNECTION MANAGEMENT
# ============================================================

def find_logitech_once():
    for device_file in evdev.list_devices():
        device = None

        try:
            device = evdev.InputDevice(
                device_file
            )

            if "Logitech" in device.name:
                return device

        except Exception:
            pass

        if device is not None:
            try:
                device.close()
            except Exception:
                pass

    return None


def disconnect_logitech(
    expected_joystick=None
):
    old_joystick = None

    with state.joystick_lock:
        if (
            expected_joystick is not None
            and state.joystick is not expected_joystick
        ):
            return

        old_joystick = state.joystick
        state.joystick = None

    if old_joystick is not None:
        try:
            old_joystick.close()
        except Exception:
            pass

    reset_axis_values()

    state.current_mode = "idle"
    motor_system.reset_motor_command()


def logitech_connection_manager():
    while not state.stop_event.is_set():
        with state.joystick_lock:
            current_joystick = state.joystick

        if current_joystick is None:
            new_joystick = find_logitech_once()

            if new_joystick is not None:
                reset_axis_values()

                with state.joystick_lock:
                    if state.joystick is None:
                        state.joystick = new_joystick

                        print(
                            "[JOYSTICK] Connected:",
                            new_joystick.name,
                            "on",
                            new_joystick.fn
                        )
                    else:
                        try:
                            new_joystick.close()
                        except Exception:
                            pass

        time.sleep(
            config.DEVICE_CHECK_PERIOD
        )


def _clear_current_memory_directory_if_needed():
    with state.recording_lock:
        memory_directory = state.current_memory_directory

    if memory_directory is None:
        return

    try:
        for filename in os.listdir(
            memory_directory
        ):
            file_path = os.path.join(
                memory_directory,
                filename,
            )

            if os.path.isfile(
                file_path
            ):
                os.remove(
                    file_path
                )

    except Exception as error:
        print("[RESET DIRECTORY ERROR]", error)


# ============================================================
# JOYSTICK EVENT PROCESSING
# ============================================================

def update_joystick():
    with state.joystick_lock:
        current_joystick = state.joystick

    if current_joystick is None:
        return

    try:
        while True:
            readable, _, _ = select.select(
                [current_joystick],
                [],
                [],
                0,
            )

            if not readable:
                break

            events = current_joystick.read()

            # Debug joystick refresh:
            # print("[JOYSTICK REFRESH]")

            for event in events:
                if event.type == evdev.ecodes.EV_ABS:
                    state.axis_values[event.code] = event.value

                    # Debug individual joystick axis events:
                    # print("[JOYSTICK AXIS]", event.code, event.value)

                elif (
                    event.type == evdev.ecodes.EV_KEY
                    and event.value == 1
                ):
                    if event.code == config.BUTTON_USER_DRIVE:
                        state.current_mode = "user_drive"
                        print("User driving")

                    elif event.code == config.BUTTON_PI_RESET:
                        if state.cx is not None:
                            state.cx.cpu4_mem = np.zeros_like(
                                state.cx.cpu4_mem
                            )

                        print("Resetting PI")

                    elif event.code == config.BUTTON_PI_RETURN:
                        state.current_mode = "pi_return"
                        print("PI return")

                    elif event.code == config.BUTTON_RECORDING_START:
                        recording_system.start_csv_recording()

                    elif event.code == config.BUTTON_SPIRAL:
                        print("spiral")
                        state.current_mode = "spiral"

                        state.search_error_acc = 0.0
                        state.pirouette_error_acc = 0.0
                        state.pirouette_phase = 0

                        state.pirouette_start_heading = None
                        state.pirouette_has_left_start = False

                        state.last_spiral_snapshot_time = 0.0

                        state.spiral_sign = np.random.choice(
                            [-1, 1]
                        )

                    elif event.code == config.BUTTON_SNAPSHOT:
                        print("snapshot")

                        if state.current_mode == "user_drive":
                            vision_system.capture_and_learn(
                                state.ego_deg
                            )

                    elif event.code == config.BUTTON_VISUAL_RESET:
                        print("[RESET] Visual memory")

                        old_visual_compass = state.visual_compass

                        new_session = (
                            recording_system.rotate_session_and_queue_model(
                                old_visual_compass
                            )
                        )

                        # If the non-critical recording system cannot create
                        # a new folder, preserve the old reset behaviour by
                        # clearing the current snapshot directory.
                        if new_session is None:
                            _clear_current_memory_directory_if_needed()

                        state.visual_compass = (
                            vision_system.create_visual_compass()
                        )

                        state.cx_ring = CXRing(
                            update_gain=0.5
                        )

                        with state.snapshot_lock:
                            state.snapshot_id = 0

                    elif event.code == config.BUTTON_VISUAL_HOME:
                        state.visual_compass.nest_return()
                        state.current_mode = "visual_home"
                        print("visual homing")

    except (
        IOError,
        OSError,
    ):
        disconnect_logitech(
            current_joystick
        )

    except Exception:
        disconnect_logitech(
            current_joystick
        )


def get_axis(axis):
    return state.axis_values.get(
        axis,
        127,
    ) - 127
