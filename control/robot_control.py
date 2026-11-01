import time

import cv2
import numpy as np

import arduino_system
import joystick_system
import motor_system
import robot_config as config
import robot_state as state
import vision_system


# ============================================================
# CONTROL LOOP
# ============================================================

def control_loop():
    next_update = time.time()

    while not state.stop_event.is_set():
        try:
            joystick_system.update_joystick()

            left_x = joystick_system.get_axis(0)
            left_y = -joystick_system.get_axis(1)
            right_x = joystick_system.get_axis(2)
            right_y = -joystick_system.get_axis(5)

            forward = (
                left_y
                + right_y
            )

            heading_value, encoder_flow = (
                arduino_system.consume_arduino_data()
            )

            heading = np.deg2rad(
                heading_value
            )

            flow = (
                encoder_flow
                * config.TICKS_TO_FLOW
            )

            state.ego_deg = state.cx.update_and_decode_egocentric(
                heading,
                flow,
            )

            ego_rad = np.deg2rad(
                state.ego_deg
            )

            # -------------------------
            # USER DRIVE
            # -------------------------

            if state.current_mode == "user_drive":
                lin = motor_system.apply_deadzone(
                    motor_system.clamp(
                        forward / 127.0,
                        -1.0,
                        1.0,
                    )
                )

                strafe = motor_system.apply_deadzone(
                    motor_system.clamp(
                        -left_x / 127.0,
                        -1.0,
                        1.0,
                    )
                )

                rot = motor_system.apply_deadzone(
                    motor_system.clamp(
                        right_x / 127.0,
                        -1.0,
                        1.0,
                    )
                )

            # -------------------------
            # PI RETURN
            # -------------------------

            elif state.current_mode == "pi_return":
                error = ego_rad

                abs_deg = abs(
                    np.rad2deg(
                        error
                    )
                )

                if abs_deg > 20:
                    rot_mag = (
                        0.4
                        + (abs_deg / 180.0)
                        * 0.4
                    )

                    rot = (
                        rot_mag
                        * np.sign(
                            error
                        )
                    )

                    lin = 0.0

                else:
                    lin = 0.65
                    rot = 0.0

                strafe = 0.0

            # -------------------------
            # VISUAL HOMING
            # -------------------------

            elif state.current_mode == "visual_home":
                with state.cam_lock:
                    if state.latest_frame is None:
                        frame = None
                    else:
                        frame = state.latest_frame.copy()

                if frame is None:
                    lin = 0.0
                    strafe = 0.0
                    rot = 0.0

                else:
                    small = cv2.resize(
                        frame,
                        (360, 100),
                    )

                    features = state.vision_model.presentation(
                        small,
                        learning=False,
                    )

                    ego_vis = state.visual_compass.presentation(
                        features,
                        learning=False,
                    )

                    if isinstance(
                        ego_vis,
                        list,
                    ):
                        ego_vis = ego_vis[0]

                    heading_deg = heading_value

                    allocentric_vis = (
                        heading_deg
                        + ego_vis
                    ) % 360.0

                    ring_dir, ring_speed = state.cx_ring.presentation(
                        allocentric_vis
                    )

                    error_deg = (
                        (
                            ring_dir
                            - heading_deg
                            + 180.0
                        ) % 360.0
                    ) - 180.0

                    abs_deg = abs(
                        error_deg
                    )

                    rot_mag = (
                        abs_deg
                        / 180.0
                    )

                    rot = (
                        rot_mag
                        * np.sign(
                            error_deg
                        )
                    )

                    lin = (
                        0.7
                        * max(
                            0.0,
                            ring_speed,
                        )
                        * (
                            1.0
                            - abs_deg / 180.0
                        )
                    )

                    strafe = 0.0

            # -------------------------
            # SPIRAL SEARCH
            # -------------------------

            elif state.current_mode == "spiral":
                ego_vis = None
                abs_pi_vis_error = None

                # Use the same camera pathway as visual homing and learning.
                with state.cam_lock:
                    if state.latest_frame is not None:
                        frame = state.latest_frame.copy()
                    else:
                        frame = None

                if True:
                    small = cv2.resize(
                        frame,
                        (360, 100),
                    )

                    features = state.vision_model.presentation(
                        small,
                        learning=False,
                    )

                    ego_vis = state.visual_compass.presentation(
                        features,
                        learning=False,
                    )

                    if isinstance(
                        ego_vis,
                        list,
                    ):
                        ego_vis = ego_vis[0]

                    pi_vis_error_deg = (
                        ego_vis
                        - state.ego_deg
                        + 180.0
                    ) % 360.0 - 180.0

                    abs_pi_vis_error = abs(
                        pi_vis_error_deg
                    )

                # If PI and the visual compass agree, move away
                # from the PI nest direction.
                if (
                    abs_pi_vis_error is not None
                    and abs_pi_vis_error < 20.0
                ):
                    desired = (
                        ego_rad
                        + np.pi
                    )

                # Otherwise move around the PI nest direction
                # using the randomly selected spiral side.
                else:
                    desired = (
                        ego_rad
                        + state.spiral_sign
                        * np.deg2rad(
                            79.0
                        )
                    )

                desired = (
                    desired
                    + np.pi
                ) % (
                    2.0
                    * np.pi
                ) - np.pi

                now_snapshot = time.time()
                snapshot_interval = 1.0 / 5.0

                # -------------------------
                # PHASE 0: SEARCH MOTION
                # -------------------------

                if state.pirouette_phase == 0:
                    error = desired

                    abs_deg = abs(
                        np.rad2deg(
                            error
                        )
                    )

                    lin = 0.35
                    strafe = 0.0

                    rot_mag = (
                        0.4
                        + (abs_deg / 180.0)
                        * 0.45
                    )

                    rot = (
                        rot_mag
                        * np.sign(
                            error
                        )
                    )

                    state.search_error_acc += state.search_error
                    state.pirouette_error_acc += state.search_error

                    if state.pirouette_error_acc > 35.0:
                        state.pirouette_phase = 1

                        state.pirouette_start_heading = (
                            heading_value
                        )

                        state.pirouette_has_left_start = False

                        state.pirouette_sign = np.random.choice(
                            [-1, 1]
                        )

                        state.last_spiral_snapshot_time = 0.0

                # -------------------------
                # PHASE 1: 360° PIROUETTE
                # -------------------------

                elif state.pirouette_phase == 1:
                    if (
                        now_snapshot
                        - state.last_spiral_snapshot_time
                        >= snapshot_interval
                    ):
                        vision_system.capture_and_learn(
                            state.ego_deg
                        )

                        state.last_spiral_snapshot_time = (
                            now_snapshot
                        )

                    lin = 0.0
                    strafe = 0.0
                    rot = 0.55 * state.pirouette_sign

                    heading_from_start = (
                        heading_value
                        - state.pirouette_start_heading
                        + 180.0
                    ) % 360.0 - 180.0

                    abs_heading_from_start = abs(
                        heading_from_start
                    )

                    if abs_heading_from_start > 70.0:
                        state.pirouette_has_left_start = True

                    if (
                        state.pirouette_has_left_start
                        and abs_heading_from_start <= 25.0
                    ):
                        state.pirouette_phase = 0
                        state.pirouette_error_acc = 0.0

                        state.pirouette_start_heading = None
                        state.pirouette_has_left_start = False

                if state.search_error_acc > 180.0:
                    state.visual_compass.nest_return()
                    state.current_mode = "pi_return"

                if abs_pi_vis_error is not None:
                    print(
                        "[SPIRAL] PI=",
                        round(state.ego_deg, 2),
                        "VIS=",
                        round(ego_vis, 2),
                        "ERR=",
                        round(abs_pi_vis_error, 2),
                        "PHASE=",
                        state.pirouette_phase,
                        "DES=",
                        round(np.rad2deg(desired), 2)
                    )

                else:
                    print(
                        "[SPIRAL] No visual prediction. PHASE=",
                        state.pirouette_phase,
                        "DES=",
                        round(np.rad2deg(desired), 2)
                    )

            # -------------------------
            # IDLE
            # -------------------------

            else:
                lin = 0.0
                strafe = 0.0
                rot = 0.0

            state.lastlin = lin

            motor_system.set_motor_command(
                lin,
                strafe,
                rot,
            )

            # Debug each control-loop motor request:
            # print("[CONTROL COMMAND] mode:", state.current_mode, "lin:", lin, "strafe:", strafe, "rot:", rot)

        except Exception as error:
            print(
                "[CONTROL ERROR]",
                error
            )

            motor_system.reset_motor_command()

        next_update += config.CONTROL_PERIOD

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
