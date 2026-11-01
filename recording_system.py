import csv
import os
import pickle
import queue
import time

import robot_config as config
import robot_state as state


csv_event_queue = queue.Queue(
    maxsize=config.CSV_QUEUE_SIZE
)

# Model saving is rare. This queue is intentionally unbounded so a visual
# memory save request is never discarded because telemetry is busy.
model_save_queue = queue.Queue()


CSV_FIELDS = [
    "timestamp",
    "unix_time",
    "program_elapsed_s",
    "recording_elapsed_s",
    "recording_start_unix_time",
    "session_directory",
    "driving_mode",

    "compass_heading_deg",
    "ego_pi_deg",

    "encoder_front_right_delta",
    "encoder_front_left_delta",
    "encoder_rear_right_delta",
    "encoder_rear_left_delta",

    "encoder_front_right_total",
    "encoder_front_left_total",
    "encoder_rear_right_total",
    "encoder_rear_left_total",
    "encoder_combined_total",

    "motor_requested_lin",
    "motor_requested_strafe",
    "motor_requested_rot",

    "motor_smoothed_lin",
    "motor_smoothed_strafe",
    "motor_smoothed_rot",

    "motor_front_m1_duty",
    "motor_front_m2_duty",
    "motor_rear_m1_duty",
    "motor_rear_m2_duty",

    "motor_front_right_duty",
    "motor_front_left_duty",
    "motor_rear_right_duty",
    "motor_rear_left_duty",

    "snapshot_taken",
    "snapshot_count_delta",
    "snapshot_id",
    "snapshot_filename",
    "last_snapshot_time",

    "gnss_connected",
    "gnss_port",
    "gnss_data_age_s",
    "gnss_latitude",
    "gnss_longitude",
    "gnss_local_easting_m",
    "gnss_local_northing_m",
    "gnss_ellipsoid_height_m",
    "gnss_sea_level_height_m",
    "gnss_horizontal_accuracy_m",
    "gnss_vertical_accuracy_m",
    "gnss_speed_accuracy_m_s",
    "gnss_heading_accuracy_deg",
    "gnss_pdop",
    "gnss_valid_fix",
    "gnss_fix_type",
    "gnss_fix_type_text",
    "gnss_rtk_status",
    "gnss_rtk_status_text",
    "gnss_differential_corrections",
    "gnss_satellites",
    "gnss_ground_speed_m_s",
    "gnss_ground_speed_km_h",
    "gnss_heading_deg",
]


def _make_unique_session_directory():
    os.makedirs(
        config.RECORDING_ROOT,
        exist_ok=True,
    )

    base_name = time.strftime(
        "%Y-%m-%d_%H-%M-%S"
    )

    session_directory = os.path.join(
        config.RECORDING_ROOT,
        base_name,
    )

    suffix = 1

    while os.path.exists(session_directory):
        session_directory = os.path.join(
            config.RECORDING_ROOT,
            base_name + "_%02d" % suffix,
        )

        suffix += 1

    memory_directory = os.path.join(
        session_directory,
        config.VISUAL_MEMORY_DIRECTORY_NAME,
    )

    debug_directory = os.path.join(
        session_directory,
        config.DEBUG_SNAPSHOT_DIRECTORY_NAME,
    )

    os.makedirs(
        memory_directory
    )

    os.makedirs(
        debug_directory
    )

    return (
        session_directory,
        memory_directory,
        debug_directory,
    )


def start_initial_session():
    try:
        session_directory, memory_directory, debug_directory = (
            _make_unique_session_directory()
        )

        with state.recording_lock:
            state.current_session_directory = session_directory
            state.current_memory_directory = memory_directory
            state.current_debug_snapshot_directory = debug_directory

        with state.recording_state_lock:
            state.recording_enabled = False
            state.recording_start_time = None

        print("[RECORDING] New session:", session_directory)
        print("[RECORDING] CSV disabled until button 307 is pressed")

        return session_directory

    except Exception as error:
        print("[RECORDING ERROR] Could not create initial session:", error)

        with state.recording_lock:
            state.current_session_directory = None
            state.current_memory_directory = None
            state.current_debug_snapshot_directory = None

        with state.recording_state_lock:
            state.recording_enabled = False
            state.recording_start_time = None

        return None


def start_csv_recording():
    with state.recording_lock:
        session_directory = state.current_session_directory

    if session_directory is None:
        print("[RECORDING ERROR] No current session folder")
        return False

    with state.recording_state_lock:
        if state.recording_enabled:
            print("[RECORDING] Already active")
            return False

        state.recording_generation += 1
        state.recording_enabled = True
        state.recording_start_time = time.time()

        generation = state.recording_generation
        start_time = state.recording_start_time

    print(
        "[RECORDING] CSV started in",
        session_directory
    )

    # Debug recording generation and exact start time:
    # print("[RECORDING DEBUG] generation:", generation, "start:", start_time)

    return True


def stop_csv_recording():
    with state.recording_state_lock:
        old_generation = state.recording_generation
        was_enabled = state.recording_enabled

        state.recording_enabled = False
        state.recording_start_time = None
        state.recording_generation += 1

    try:
        csv_event_queue.put_nowait(
            {
                "_event": "close",
                "_recording_generation": old_generation,
            }
        )

    except queue.Full:
        print("[CSV WARNING] Queue full while requesting CSV close")

    if was_enabled:
        print("[RECORDING] CSV stopped")

    return was_enabled


def rotate_session_and_queue_model(model_to_save):
    """
    Stop CSV recording, switch immediately to a new empty session and queue
    the previous mushroom-body model for asynchronous pickling in the old
    session.

    CSV remains disabled in the new session until button 307 is pressed.
    """
    stop_csv_recording()

    old_session_directory = None

    try:
        session_directory, memory_directory, debug_directory = (
            _make_unique_session_directory()
        )

        with state.recording_lock:
            old_session_directory = state.current_session_directory

            state.current_session_directory = session_directory
            state.current_memory_directory = memory_directory
            state.current_debug_snapshot_directory = debug_directory

        with state.snapshot_lock:
            state.last_snapshot_filename = None
            state.last_snapshot_time = None

        print("[RECORDING] New session:", session_directory)
        print("[RECORDING] CSV disabled until button 307 is pressed")

    except Exception as error:
        print("[RECORDING ERROR] Could not create new session:", error)
        return None

    if (
        old_session_directory is not None
        and model_to_save is not None
    ):
        try:
            model_save_queue.put_nowait(
                {
                    "session_directory": old_session_directory,
                    "model": model_to_save,
                }
            )

        except Exception as error:
            print("[MODEL QUEUE ERROR]", error)

    return session_directory


def _empty_gnss_data():
    return {
        "connected": False,
        "port": None,
        "data_age": None,
        "latitude": None,
        "longitude": None,
        "local_easting": None,
        "local_northing": None,
        "ellipsoid_height": None,
        "sea_level_height": None,
        "horizontal_accuracy": None,
        "vertical_accuracy": None,
        "speed_accuracy": None,
        "heading_accuracy": None,
        "pdop": None,
        "valid_fix": False,
        "fix_type": None,
        "fix_type_text": "NO DATA",
        "rtk_status": None,
        "rtk_status_text": "NO DATA",
        "differential_corrections": False,
        "satellites": None,
        "ground_speed": None,
        "ground_speed_kmh": None,
        "heading": None,
    }


def _read_gnss_data():
    if state.gnss_reader is None:
        return _empty_gnss_data()

    try:
        return state.gnss_reader.get_current_measures()

    except Exception as error:
        print("[CSV GPS ERROR]", error)
        return _empty_gnss_data()


def _build_csv_row(
    previous_snapshot_counter,
    recording_start_time,
    recording_generation,
):
    now = time.time()

    with state.recording_lock:
        session_directory = state.current_session_directory

    with state.arduino_data_lock:
        compass_heading = state.latest_azimuth

        front_right_delta = state.latest_encoder_deltas[
            "front_right"
        ]
        front_left_delta = state.latest_encoder_deltas[
            "front_left"
        ]
        rear_right_delta = state.latest_encoder_deltas[
            "rear_right"
        ]
        rear_left_delta = state.latest_encoder_deltas[
            "rear_left"
        ]

        front_right_total = state.fracc
        front_left_total = state.flacc
        rear_right_total = state.rracc
        rear_left_total = state.rlacc

    encoder_combined_total = (
        front_right_total
        + front_left_total
        + rear_right_total
        + rear_left_total
    ) / 4.0

    with state.motor_command_lock:
        requested_lin = state.latest_motor_command["lin"]
        requested_strafe = state.latest_motor_command["strafe"]
        requested_rot = state.latest_motor_command["rot"]

        smoothed_lin = state.latest_smoothed_motor_command["lin"]
        smoothed_strafe = state.latest_smoothed_motor_command["strafe"]
        smoothed_rot = state.latest_smoothed_motor_command["rot"]

        front_m1 = state.latest_motor_duties["front_m1"]
        front_m2 = state.latest_motor_duties["front_m2"]
        rear_m1 = state.latest_motor_duties["rear_m1"]
        rear_m2 = state.latest_motor_duties["rear_m2"]

        front_right_duty = state.latest_motor_duties["front_right"]
        front_left_duty = state.latest_motor_duties["front_left"]
        rear_right_duty = state.latest_motor_duties["rear_right"]
        rear_left_duty = state.latest_motor_duties["rear_left"]

    with state.snapshot_lock:
        snapshot_counter = state.snapshot_counter
        snapshot_id = state.snapshot_id
        snapshot_filename = state.last_snapshot_filename
        last_snapshot_time = state.last_snapshot_time

    snapshot_count_delta = snapshot_counter - previous_snapshot_counter

    if snapshot_count_delta < 0:
        snapshot_count_delta = 0

    if snapshot_count_delta > 0:
        snapshot_taken = 1
    else:
        snapshot_taken = 0

    gnss = _read_gnss_data()

    row = {
        "timestamp": time.strftime(
            "%Y-%m-%d %H:%M:%S",
            time.localtime(now),
        ),
        "unix_time": now,
        "program_elapsed_s": now - state.program_start_time,
        "recording_elapsed_s": now - recording_start_time,
        "recording_start_unix_time": recording_start_time,
        "session_directory": session_directory,
        "driving_mode": state.current_mode,

        "compass_heading_deg": compass_heading,
        "ego_pi_deg": state.ego_deg,

        "encoder_front_right_delta": front_right_delta,
        "encoder_front_left_delta": front_left_delta,
        "encoder_rear_right_delta": rear_right_delta,
        "encoder_rear_left_delta": rear_left_delta,

        "encoder_front_right_total": front_right_total,
        "encoder_front_left_total": front_left_total,
        "encoder_rear_right_total": rear_right_total,
        "encoder_rear_left_total": rear_left_total,
        "encoder_combined_total": encoder_combined_total,

        "motor_requested_lin": requested_lin,
        "motor_requested_strafe": requested_strafe,
        "motor_requested_rot": requested_rot,

        "motor_smoothed_lin": smoothed_lin,
        "motor_smoothed_strafe": smoothed_strafe,
        "motor_smoothed_rot": smoothed_rot,

        "motor_front_m1_duty": front_m1,
        "motor_front_m2_duty": front_m2,
        "motor_rear_m1_duty": rear_m1,
        "motor_rear_m2_duty": rear_m2,

        "motor_front_right_duty": front_right_duty,
        "motor_front_left_duty": front_left_duty,
        "motor_rear_right_duty": rear_right_duty,
        "motor_rear_left_duty": rear_left_duty,

        "snapshot_taken": snapshot_taken,
        "snapshot_count_delta": snapshot_count_delta,
        "snapshot_id": snapshot_id,
        "snapshot_filename": snapshot_filename,
        "last_snapshot_time": last_snapshot_time,

        "gnss_connected": gnss.get("connected"),
        "gnss_port": gnss.get("port"),
        "gnss_data_age_s": gnss.get("data_age"),
        "gnss_latitude": gnss.get("latitude"),
        "gnss_longitude": gnss.get("longitude"),
        "gnss_local_easting_m": gnss.get("local_easting"),
        "gnss_local_northing_m": gnss.get("local_northing"),
        "gnss_ellipsoid_height_m": gnss.get("ellipsoid_height"),
        "gnss_sea_level_height_m": gnss.get("sea_level_height"),
        "gnss_horizontal_accuracy_m": gnss.get("horizontal_accuracy"),
        "gnss_vertical_accuracy_m": gnss.get("vertical_accuracy"),
        "gnss_speed_accuracy_m_s": gnss.get("speed_accuracy"),
        "gnss_heading_accuracy_deg": gnss.get("heading_accuracy"),
        "gnss_pdop": gnss.get("pdop"),
        "gnss_valid_fix": gnss.get("valid_fix"),
        "gnss_fix_type": gnss.get("fix_type"),
        "gnss_fix_type_text": gnss.get("fix_type_text"),
        "gnss_rtk_status": gnss.get("rtk_status"),
        "gnss_rtk_status_text": gnss.get("rtk_status_text"),
        "gnss_differential_corrections": gnss.get("differential_corrections"),
        "gnss_satellites": gnss.get("satellites"),
        "gnss_ground_speed_m_s": gnss.get("ground_speed"),
        "gnss_ground_speed_km_h": gnss.get("ground_speed_kmh"),
        "gnss_heading_deg": gnss.get("heading"),
    }

    event = {
        "_event": "row",
        "_recording_generation": recording_generation,
        "row": row,
    }

    return event, snapshot_counter


def csv_sampler_loop():
    next_update = time.time()
    previous_snapshot_counter = 0
    previous_generation = None

    while not state.stop_event.is_set():
        try:
            with state.recording_state_lock:
                recording_enabled = state.recording_enabled
                recording_start_time = state.recording_start_time
                recording_generation = state.recording_generation

            if not recording_enabled or recording_start_time is None:
                with state.snapshot_lock:
                    previous_snapshot_counter = state.snapshot_counter

                previous_generation = recording_generation

            else:
                if previous_generation != recording_generation:
                    with state.snapshot_lock:
                        previous_snapshot_counter = state.snapshot_counter

                    previous_generation = recording_generation

                event, previous_snapshot_counter = _build_csv_row(
                    previous_snapshot_counter,
                    recording_start_time,
                    recording_generation,
                )

                try:
                    csv_event_queue.put_nowait(
                        event
                    )

                    # Debug each 100 ms telemetry sample:
                    # print("[CSV SAMPLE]", event["row"]["driving_mode"], event["row"]["compass_heading_deg"], event["row"]["gnss_rtk_status_text"])

                except queue.Full:
                    print("[CSV WARNING] Telemetry queue full, row dropped")

        except Exception as error:
            print("[CSV SAMPLE ERROR]", error)

        next_update += config.CSV_SAMPLE_PERIOD

        sleep_time = next_update - time.time()

        if sleep_time > 0:
            time.sleep(
                sleep_time
            )
        else:
            next_update = time.time()


def csv_writer_loop():
    current_session_directory = None
    current_generation = None
    closed_generations = set()

    csv_file = None
    csv_writer = None
    last_flush_time = time.time()

    try:
        while (
            not state.stop_event.is_set()
            or not csv_event_queue.empty()
        ):
            try:
                event = csv_event_queue.get(
                    timeout=0.1
                )

            except queue.Empty:
                if (
                    csv_file is not None
                    and time.time() - last_flush_time
                    >= config.CSV_FLUSH_PERIOD
                ):
                    try:
                        csv_file.flush()
                    except Exception as error:
                        print("[CSV FLUSH ERROR]", error)

                    last_flush_time = time.time()

                continue

            try:
                event_type = event.get(
                    "_event"
                )

                event_generation = event.get(
                    "_recording_generation"
                )

                if event_type == "close":
                    closed_generations.add(
                        event_generation
                    )

                    if (
                        csv_file is not None
                        and current_generation == event_generation
                    ):
                        try:
                            csv_file.flush()
                            csv_file.close()
                        except Exception:
                            pass

                        csv_file = None
                        csv_writer = None
                        current_session_directory = None
                        current_generation = None

                    continue

                if event_type != "row":
                    continue

                if event_generation in closed_generations:
                    continue

                row = event.get(
                    "row"
                )

                if row is None:
                    continue

                row_session_directory = row.get(
                    "session_directory"
                )

                if row_session_directory is None:
                    continue

                if (
                    row_session_directory != current_session_directory
                    or event_generation != current_generation
                ):
                    if csv_file is not None:
                        try:
                            csv_file.flush()
                            csv_file.close()
                        except Exception:
                            pass

                    current_session_directory = row_session_directory
                    current_generation = event_generation

                    csv_path = os.path.join(
                        current_session_directory,
                        config.CSV_FILENAME,
                    )

                    file_exists = os.path.exists(
                        csv_path
                    )

                    file_has_data = (
                        file_exists
                        and os.path.getsize(csv_path) > 0
                    )

                    csv_file = open(
                        csv_path,
                        "a",
                        newline="",
                    )

                    csv_writer = csv.DictWriter(
                        csv_file,
                        fieldnames=CSV_FIELDS,
                        extrasaction="ignore",
                    )

                    if not file_has_data:
                        csv_writer.writeheader()

                    last_flush_time = time.time()

                csv_writer.writerow(
                    row
                )

                # Debug each row physically appended:
                # print("[CSV WRITE]", row["timestamp"], row["driving_mode"])

                if (
                    time.time() - last_flush_time
                    >= config.CSV_FLUSH_PERIOD
                ):
                    csv_file.flush()
                    last_flush_time = time.time()

            except Exception as error:
                print("[CSV WRITE ERROR]", error)

                if csv_file is not None:
                    try:
                        csv_file.close()
                    except Exception:
                        pass

                csv_file = None
                csv_writer = None
                current_session_directory = None
                current_generation = None

            finally:
                csv_event_queue.task_done()

    finally:
        if csv_file is not None:
            try:
                csv_file.flush()
                csv_file.close()
            except Exception:
                pass


def model_saver_loop():
    while (
        not state.stop_event.is_set()
        or not model_save_queue.empty()
    ):
        try:
            save_request = model_save_queue.get(
                timeout=0.1
            )

        except queue.Empty:
            continue

        temporary_path = None

        try:
            session_directory = save_request[
                "session_directory"
            ]

            model = save_request[
                "model"
            ]

            model_path = os.path.join(
                session_directory,
                config.VISUAL_MODEL_FILENAME,
            )

            temporary_path = model_path + ".tmp"

            with open(
                temporary_path,
                "wb",
            ) as model_file:
                pickle.dump(
                    model,
                    model_file,
                    protocol=pickle.HIGHEST_PROTOCOL,
                )

                model_file.flush()

            os.rename(
                temporary_path,
                model_path,
            )

            print("[MODEL SAVED]", model_path)

            try:
                import arduino_system
                arduino_system.queue_bip()

            except Exception as error:
                print("[MODEL BIP ERROR]", error)

        except Exception as error:
            print("[MODEL SAVE ERROR]", error)

            try:
                if (
                    temporary_path is not None
                    and os.path.exists(temporary_path)
                ):
                    os.remove(temporary_path)
            except Exception:
                pass

        finally:
            model_save_queue.task_done()
