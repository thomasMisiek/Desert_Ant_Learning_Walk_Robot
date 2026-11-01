import glob
import os
import time

import cv2

from visual_compass import GreenVisualPipeline, MB_8_Nest

import robot_config as config
import robot_state as state


# ============================================================
# VISUAL MODEL INITIALISATION
# ============================================================

def create_visual_compass():
    return MB_8_Nest(
        n_kcs=20000,
        n_input=100,
        connections_per_kc=5,
        lr=1,
        sparsity=0.01,
    )


def initialize_visual_system():
    state.vision_model = GreenVisualPipeline(
        resolution=(10, 10),
        crop_bounds=(0, 360),
    )

    state.visual_compass = create_visual_compass()


# ============================================================
# CAMERA CONNECTION MANAGEMENT
# ============================================================

def find_working_camera_once():
    devices = sorted(
        glob.glob("/dev/video*")
    )

    for device in devices:
        capture = None

        try:
            capture = cv2.VideoCapture(
                device,
                cv2.CAP_V4L2,
            )

            if capture.isOpened():
                return (
                    device,
                    capture,
                )

        except Exception:
            pass

        if capture is not None:
            try:
                capture.release()
            except Exception:
                pass

    return (
        None,
        None,
    )


def disconnect_camera(
    expected_capture=None
):
    old_capture = None

    with state.camera_connection_lock:
        if (
            expected_capture is not None
            and state.camera_capture is not expected_capture
        ):
            return

        old_capture = state.camera_capture

        state.camera_capture = None
        state.camera_device = None
        state.camera_connected = False

    with state.cam_lock:
        state.latest_frame = None

    if old_capture is not None:
        try:
            old_capture.release()
        except Exception:
            pass


def camera_connection_manager():
    while not state.stop_event.is_set():
        with state.camera_connection_lock:
            current_capture = state.camera_capture
            current_device = state.camera_device

        if (
            current_capture is not None
            and current_device is not None
            and not os.path.exists(current_device)
        ):
            disconnect_camera(
                current_capture
            )

            current_capture = None

        if current_capture is None:
            new_device, new_capture = find_working_camera_once()

            if new_capture is not None:
                with state.camera_connection_lock:
                    if state.camera_capture is None:
                        state.camera_capture = new_capture
                        state.camera_device = new_device
                        state.camera_connected = True

                        print(
                            "[CAM] Connected on",
                            new_device
                        )

                    else:
                        try:
                            new_capture.release()
                        except Exception:
                            pass

        time.sleep(
            config.DEVICE_CHECK_PERIOD
        )


# ============================================================
# CAMERA FRAME THREAD
# ============================================================

def camera_thread():
    failed_reads = 0

    while not state.stop_event.is_set():
        with state.camera_connection_lock:
            current_capture = state.camera_capture

        if current_capture is None:
            failed_reads = 0

            time.sleep(
                0.01
            )

            continue

        try:
            ret, frame = current_capture.read()

            if ret:
                failed_reads = 0

                with state.cam_lock:
                    state.latest_frame = frame.copy()

                # Debug camera refresh:
                # print("[CAM FRAME]", frame.shape)

            else:
                failed_reads += 1

                if failed_reads >= 5:
                    disconnect_camera(
                        current_capture
                    )

                    failed_reads = 0

                time.sleep(
                    0.01
                )

        except Exception:
            disconnect_camera(
                current_capture
            )

            failed_reads = 0

            time.sleep(
                0.01
            )


# ============================================================
# VISUAL LEARNING
# ============================================================

def capture_and_learn(ego_pi_deg):
    try:
        with state.cam_lock:
            if state.latest_frame is None:
                print("[CAM] No frame yet")
                return False

            frame = state.latest_frame.copy()

        small = cv2.resize(
            frame,
            (360, 100),
        )

        with state.recording_lock:
            memory_directory = state.current_memory_directory
            debug_directory = state.current_debug_snapshot_directory

        if memory_directory is None:
            memory_directory = config.VISUAL_MEMORY_DIRECTORY_NAME

            os.makedirs(
                memory_directory,
                exist_ok=True,
            )

        with state.snapshot_lock:
            current_snapshot_id = state.snapshot_id

        filename = os.path.join(
            memory_directory,
            str(current_snapshot_id)
            + "_"
            + str(round(ego_pi_deg, 2))
            + ".png",
        )

        cv2.imwrite(
            filename,
            small,
        )

        # Optional 20x20 debugging snapshot.
        # Uncomment these lines to save it.
        #
        # if debug_directory is not None:
        #     debug_small = cv2.resize(frame, (20, 20))
        #     cv2.imwrite(
        #         os.path.join(debug_directory, str(current_snapshot_id) + ".png"),
        #         debug_small,
        #     )

        features = state.vision_model.presentation(
            small,
            learning=False,
        )

        state.visual_compass.presentation(
            features,
            learning=True,
            angle_to_truth=ego_pi_deg,
            angle_to_north=None,
            timestep=None,
        )

        print(
            "[SNAPSHOT]",
            filename
        )

        now = time.time()

        with state.snapshot_lock:
            state.snapshot_id += 1
            state.snapshot_counter += 1
            state.last_snapshot_time = now
            state.last_snapshot_filename = filename

        try:
            import arduino_system
            arduino_system.queue_bip()

        except Exception as error:
            print("[SNAP BIP ERROR]", error)

        return True

    except Exception as error:
        print(
            "[SNAP ERROR]",
            error
        )

        return False
