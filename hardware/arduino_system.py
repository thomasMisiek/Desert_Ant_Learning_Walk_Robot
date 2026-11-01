import os
import queue
import re
import threading
import time

import serial

import robot_config as config
import robot_state as state
import usb_port_manager


# ============================================================
# OPTIONAL ARDUINO OUTPUT QUEUE
# ============================================================

arduino_output_queue = queue.Queue(
    maxsize=config.ARDUINO_OUTPUT_QUEUE_SIZE
)

_arduino_disconnect_request = threading.Event()


# ============================================================
# EXACT EXISTING ARDUINO PACKET FORMAT
# ============================================================

ARDUINO_PATTERN = re.compile(
    r'DATA:HDG:([\d.]+)'
    r':ENC_FR_DELTA:(-?\d+)'
    r':ENC_FL_DELTA:(-?\d+)'
    r':ENC_RR_DELTA:(-?\d+)'
    r':ENC_RL_DELTA:(-?\d+)'
)


# ============================================================
# ARDUINO STATE / CONNECTION HELPERS
# ============================================================

def disconnect_arduino(
    expected_serial=None
):
    """
    Request a disconnect.

    The Arduino I/O thread remains the only thread that closes the serial
    object. This avoids closing the port while another Arduino thread is
    reading or writing it.
    """
    with state.arduino_connection_lock:
        if (
            expected_serial is not None
            and state.ser is not expected_serial
        ):
            return

    _arduino_disconnect_request.set()


def _publish_open_serial(serial_port):
    with state.arduino_connection_lock:
        state.ser = serial_port
        state.arduino_connected = False


def _clear_serial_reference(serial_port):
    with state.arduino_connection_lock:
        if state.ser is serial_port:
            state.ser = None
            state.arduino_connected = False


def _mark_arduino_connected(serial_port):
    first_connection = False

    with state.arduino_connection_lock:
        if state.ser is serial_port:
            if not state.arduino_connected:
                first_connection = True

            state.arduino_connected = True

    return first_connection


def _close_owned_serial(serial_port):
    _clear_serial_reference(
        serial_port
    )

    if serial_port is not None:
        try:
            serial_port.close()
        except Exception:
            pass

    usb_port_manager.release_port(
        config.ARDUINO_PORT,
        "arduino",
    )


# ============================================================
# EXACT EXISTING ARDUINO DATA UPDATE
# ============================================================

def _consume_received_line(line, serial_port):
    match = ARDUINO_PATTERN.fullmatch(
        line
    )

    if not match:
        return False

    heading_value = float(
        match.group(1)
    )

    fr = int(
        match.group(2)
    )

    fl = int(
        match.group(3)
    )

    rr = int(
        match.group(4)
    )

    rl = int(
        match.group(5)
    )

    combined_flow = (
        fr
        + fl
        + rr
        + rl
    ) / 4.0

    with state.arduino_data_lock:
        state.latest_azimuth = heading_value

        # Existing PI accumulator. Only consume_arduino_data()
        # resets this value.
        state.latest_encoder_flow += combined_flow

        # Independent LCD accumulator. The LCD code resets only
        # this duplicate value and never consumes PI data.
        state.encoder_display_flow += combined_flow

        state.latest_encoder_deltas["front_right"] = fr
        state.latest_encoder_deltas["front_left"] = fl
        state.latest_encoder_deltas["rear_right"] = rr
        state.latest_encoder_deltas["rear_left"] = rl

        state.fracc += fr
        state.flacc += fl
        state.rracc += rr
        state.rlacc += rl

        state.latest_arduino_packet_time = time.time()

    if _mark_arduino_connected(serial_port):
        print(
            "[ARDUINO] Connected on",
            config.ARDUINO_PORT
        )

    # Debug each accepted Arduino refresh:
    # print("[ARDUINO DATA] heading:", heading_value, "encoders:", fr, fl, rr, rl)

    return True


def consume_arduino_data():
    with state.arduino_data_lock:
        heading_value = state.latest_azimuth
        encoder_flow = state.latest_encoder_flow
        state.latest_encoder_flow = 0.0

    return (
        heading_value,
        encoder_flow,
    )


# ============================================================
# NON-BLOCKING OPTIONAL ARDUINO OUTPUT
# ============================================================

def queue_bip():
    try:
        arduino_output_queue.put_nowait(
            "BIP\n"
        )

    except queue.Full:
        print("[ARDUINO OUTPUT] BIP queue full")


def queue_lcd_text(text):
    try:
        lcd_text = str(text)
        lcd_text = lcd_text[:32].ljust(32)

        arduino_output_queue.put_nowait(
            "LCD:" + lcd_text + "\n"
        )

    except queue.Full:
        print("[ARDUINO OUTPUT] LCD queue full")

    except Exception as error:
        print("[ARDUINO OUTPUT ERROR]", error)


def _accuracy_text(horizontal_accuracy):
    if horizontal_accuracy is None:
        return "A----"

    try:
        value = float(horizontal_accuracy)

        if value < 0.0:
            return "A----"

        if value < 10.0:
            return "A%0.2f" % value

        if value < 100.0:
            return "A%0.1f" % value

        return "A>99m"

    except Exception:
        return "A----"


def _rtk_text(gnss_data):
    if not gnss_data:
        return "---"

    if not gnss_data.get("connected", False):
        return "---"

    rtk_status = gnss_data.get("rtk_status")

    if rtk_status == 2:
        return "FIX"

    if rtk_status == 1:
        return "FLT"

    if rtk_status == 0:
        return "NON"

    return "---"


def _satellite_text(satellites):
    if satellites is None:
        return "--"

    try:
        value = int(satellites)
        value = max(0, min(99, value))
        return "%02d" % value

    except Exception:
        return "--"


def build_lcd_text():
    with state.arduino_data_lock:
        encoder_value = state.encoder_display_flow
        state.encoder_display_flow = 0.0

        heading_value = state.latest_azimuth

    try:
        encoder_integer = int(round(encoder_value))
    except Exception:
        encoder_integer = 0

    encoder_integer = max(
        -99,
        min(
            99,
            encoder_integer,
        ),
    )

    try:
        heading_integer = int(round(heading_value)) % 360
    except Exception:
        heading_integer = 0

    gnss_data = None

    try:
        if state.gnss_reader is not None:
            gnss_data = state.gnss_reader.get_current_measures()

    except Exception as error:
        print("[LCD GPS ERROR]", error)
        gnss_data = None

    if gnss_data is None:
        horizontal_accuracy = None
        satellites = None
    else:
        horizontal_accuracy = gnss_data.get(
            "horizontal_accuracy"
        )
        satellites = gnss_data.get(
            "satellites"
        )

    encoder_text = "E%+03d" % encoder_integer
    heading_text = "C%03d" % heading_integer
    accuracy_text = _accuracy_text(
        horizontal_accuracy
    )

    row_1 = (
        encoder_text
        + " "
        + heading_text
        + " "
        + accuracy_text
    )[:16].ljust(16)

    row_2 = (
        "RTK:"
        + _rtk_text(gnss_data)
        + " SAT:"
        + _satellite_text(satellites)
    )[:16].ljust(16)

    return row_1 + row_2


def _write_command(serial_port, command):
    serial_port.write(
        command.encode("ascii")
    )

    # Debug BIP/LCD packets sent to Arduino:
    # print("[ARDUINO TX]", command.strip())


# ============================================================
# SINGLE ARDUINO SERIAL OWNER THREAD
# ============================================================

def arduino_io_loop():
    """
    This is the only thread that opens, reads, writes and closes the Arduino
    serial port.

    The exact DATA packet parser, encoder accumulation, LCD packet and BIP
    packet remain unchanged.
    """
    serial_port = None
    next_lcd_update = time.time()

    while not state.stop_event.is_set():
        if serial_port is None:
            _arduino_disconnect_request.clear()

            if not os.path.exists(config.ARDUINO_PORT):
                state.stop_event.wait(
                    config.DEVICE_CHECK_PERIOD
                )
                continue

            if not usb_port_manager.claim_port(
                config.ARDUINO_PORT,
                "arduino",
            ):
                state.stop_event.wait(
                    config.DEVICE_CHECK_PERIOD
                )
                continue

            try:
                serial_port = serial.Serial(
                    config.ARDUINO_PORT,
                    config.ARDUINO_BAUDRATE,
                    timeout=0.05,
                    write_timeout=0.05,
                )

                _publish_open_serial(
                    serial_port
                )

                # Opening the Arduino serial port resets the board.
                # This wait occurs only in this non-critical Arduino thread.
                if state.stop_event.wait(
                    config.ARDUINO_RESET_DELAY
                ):
                    break

                try:
                    serial_port.reset_input_buffer()
                except Exception:
                    pass

                next_lcd_update = time.time()

                # Debug exact Arduino port opening:
                # print("[ARDUINO OPEN]", config.ARDUINO_PORT)

            except Exception as error:
                print(
                    "[ARDUINO OPEN ERROR]",
                    error
                )

                _close_owned_serial(
                    serial_port
                )

                serial_port = None

                state.stop_event.wait(
                    config.DEVICE_CHECK_PERIOD
                )

                continue

        connection_failed = False

        try:
            if _arduino_disconnect_request.is_set():
                connection_failed = True

            if not os.path.exists(config.ARDUINO_PORT):
                connection_failed = True

            # Read a limited batch so outgoing BIP/LCD messages are not
            # starved by continuous incoming Arduino packets.
            lines_read = 0

            while (
                not connection_failed
                and serial_port.in_waiting > 0
                and lines_read < 20
            ):
                line = serial_port.readline().decode(
                    "ascii",
                    errors="ignore",
                ).strip()

                if line:
                    _consume_received_line(
                        line,
                        serial_port,
                    )

                lines_read += 1

            now = time.time()

            if (
                not connection_failed
                and now >= next_lcd_update
            ):
                lcd_text = build_lcd_text()

                _write_command(
                    serial_port,
                    "LCD:" + lcd_text + "\n",
                )

                # Debug each LCD refresh:
                # print("[LCD REFRESH]", lcd_text[:16], "|", lcd_text[16:])

                next_lcd_update += config.LCD_UPDATE_PERIOD

                if next_lcd_update < now:
                    next_lcd_update = (
                        now
                        + config.LCD_UPDATE_PERIOD
                    )

            commands_sent = 0

            while (
                not connection_failed
                and commands_sent < 10
            ):
                try:
                    command = arduino_output_queue.get_nowait()

                except queue.Empty:
                    break

                try:
                    _write_command(
                        serial_port,
                        command,
                    )

                finally:
                    arduino_output_queue.task_done()

                commands_sent += 1

        except (
            serial.SerialTimeoutException,
            serial.SerialException,
            OSError,
        ) as error:
            print(
                "[ARDUINO SERIAL ERROR]",
                error
            )

            connection_failed = True

        except Exception as error:
            print(
                "[ARDUINO I/O ERROR]",
                error
            )

            connection_failed = True

        if connection_failed:
            _close_owned_serial(
                serial_port
            )

            serial_port = None
            _arduino_disconnect_request.clear()

            state.stop_event.wait(
                config.DEVICE_CHECK_PERIOD
            )

            continue

        time.sleep(
            0.001
        )

    _close_owned_serial(
        serial_port
    )
