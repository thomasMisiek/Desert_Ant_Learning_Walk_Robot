import threading
import time

import path_integration_precise as pi
from cx import CXRing

from nmea import NMEAReader

import arduino_system
import joystick_system
import motor_system
import recording_system
import robot_config as config
import robot_control
import robot_state as state
import vision_system


threads = []


# ============================================================
# CLEANUP
# ============================================================

def close_devices():
    # Request an immediate CSV flush/close before stopping the workers.
    try:
        recording_system.stop_csv_recording()
    except Exception as error:
        print("[RECORDING STOP ERROR]", error)

    state.stop_event.set()

    motor_system.reset_motor_command()
    motor_system.stop_motors()

    joystick_system.disconnect_logitech()
    arduino_system.disconnect_arduino()
    vision_system.disconnect_camera()

    motor_system.disconnect_controller(
        "front"
    )

    motor_system.disconnect_controller(
        "rear"
    )

    if state.gnss_reader is not None:
        try:
            state.gnss_reader.stop()
        except Exception as error:
            print("[GNSS STOP ERROR]", error)

    # Give the CSV writer, Arduino owner and model saver a brief opportunity
    # to close their files and ports. This occurs only during shutdown.
    deadline = time.time() + 2.0

    for thread in threads:
        remaining = deadline - time.time()

        if remaining <= 0.0:
            break

        try:
            thread.join(
                remaining
            )
        except Exception:
            pass


# ============================================================
# MAIN
# ============================================================

def main():
    global threads

    state.program_start_time = time.time()
    state.stop_event.clear()

    state.cx = pi.CX(
        0.0,
        0.0,
    )

    state.cx_ring = CXRing(
        update_gain=0.5
    )

    vision_system.initialize_visual_system()
    recording_system.start_initial_session()

    try:
        state.gnss_reader = NMEAReader(
            port=config.GNSS_PORT,
            baudrate=config.GNSS_BAUDRATE,
            serial_timeout=config.GNSS_SERIAL_TIMEOUT,
            reconnect_delay=config.GNSS_RECONNECT_DELAY,
            no_data_reconnect_time=config.GNSS_NO_DATA_RECONNECT_TIME,
            verbose=True,
        )

        state.gnss_reader.start()

    except Exception as error:
        state.gnss_reader = None
        print("[GNSS START ERROR]", error)

    threads = [
        threading.Thread(
            target=motor_system.roboclaw_connection_manager
        ),

        # One thread owns every Arduino open/read/write/close operation.
        threading.Thread(
            target=arduino_system.arduino_io_loop
        ),

        threading.Thread(
            target=joystick_system.logitech_connection_manager
        ),
        threading.Thread(
            target=vision_system.camera_connection_manager
        ),
        threading.Thread(
            target=vision_system.camera_thread
        ),
        threading.Thread(
            target=robot_control.control_loop
        ),
        threading.Thread(
            target=motor_system.motor_output_loop
        ),

        # Non-critical recording workers.
        threading.Thread(
            target=recording_system.csv_sampler_loop
        ),
        threading.Thread(
            target=recording_system.csv_writer_loop
        ),
        threading.Thread(
            target=recording_system.model_saver_loop
        ),
    ]

    for thread in threads:
        thread.daemon = True
        thread.start()

    print("[SYSTEM] Robot started")
    print("[SYSTEM] Devices reconnect automatically")
    print("[SYSTEM] Button 305: user drive")
    print("[SYSTEM] Button 304: reset PI")
    print("[SYSTEM] Button 306: PI return")
    print("[SYSTEM] Button 307: start CSV recording")
    print("[SYSTEM] Button 309: spiral")
    print("[SYSTEM] Button 308: save and reset visual memory")
    print("[SYSTEM] Button 310: capture visual snapshot")
    print("[SYSTEM] Button 311: visual homing")

    try:
        while not state.stop_event.is_set():
            time.sleep(
                0.2
            )

    except KeyboardInterrupt:
        print()
        print("[SYSTEM] Stopping")

    finally:
        close_devices()


if __name__ == "__main__":
    main()
