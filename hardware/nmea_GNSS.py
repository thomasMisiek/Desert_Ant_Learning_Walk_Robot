import math
import os
import struct
import threading
import time

import serial

import usb_port_manager


BAUDRATE = 38400
SERIAL_TIMEOUT = 0.2
RECONNECT_DELAY = 2.0
NO_DATA_RECONNECT_TIME = 5.0


def ubx_checksum(data):
    checksum_a = 0
    checksum_b = 0

    for value in data:
        checksum_a = (checksum_a + value) & 0xFF
        checksum_b = (checksum_b + checksum_a) & 0xFF

    return checksum_a, checksum_b


def fix_type_text(value):
    fix_types = {
        0: "NO FIX",
        1: "DEAD RECKONING",
        2: "2D FIX",
        3: "3D FIX",
        4: "GNSS + DEAD RECKONING",
        5: "TIME ONLY"
    }

    return fix_types.get(value, "UNKNOWN")


def rtk_status_text(value):
    statuses = {
        0: "NO RTK",
        1: "RTK FLOAT",
        2: "RTK FIXED"
    }

    return statuses.get(value, "UNKNOWN")


class NMEAReader:
    """
    Non-blocking u-blox UBX NAV-PVT reader.

    The exact configured /dev/serial/by-id path is the only port opened.
    Port opening, UBX validation, reading and reconnection run in this
    reader's daemon thread and never block the robot control loop.
    """

    def __init__(
        self,
        port,
        baudrate=BAUDRATE,
        serial_timeout=SERIAL_TIMEOUT,
        reconnect_delay=RECONNECT_DELAY,
        no_data_reconnect_time=NO_DATA_RECONNECT_TIME,
        verbose=True
    ):
        self.port = port
        self.baudrate = baudrate
        self.serial_timeout = serial_timeout
        self.reconnect_delay = reconnect_delay
        self.no_data_reconnect_time = no_data_reconnect_time
        self.verbose = verbose

        self.reference_latitude = None
        self.reference_longitude = None

        self._serial = None
        self._thread = None
        self._stop_event = threading.Event()
        self._lock = threading.Lock()
        self._buffer = b""

        self._measures = self._empty_measures()

    def _empty_measures(self):
        return {
            "connected": False,
            "port": self.port,
            "last_update_time": None,
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
            "heading": None
        }

    def start(self):
        if self._thread is not None and self._thread.is_alive():
            return

        self._stop_event.clear()

        self._thread = threading.Thread(
            target=self._connection_loop
        )

        self._thread.daemon = True
        self._thread.start()

    def stop(self):
        self._stop_event.set()

        if self._thread is not None and self._thread.is_alive():
            self._thread.join(1.0)

    def close(self):
        self.stop()

    def is_connected(self):
        with self._lock:
            return bool(self._measures["connected"])

    def get_current_measures(self):
        with self._lock:
            measures = dict(self._measures)

        if measures["last_update_time"] is not None:
            measures["data_age"] = max(
                0.0,
                time.time() - measures["last_update_time"]
            )
        else:
            measures["data_age"] = None

        return measures

    def get_measures(self):
        return self.get_current_measures()

    def reset_local_reference(
        self,
        latitude=None,
        longitude=None
    ):
        with self._lock:
            self.reference_latitude = latitude
            self.reference_longitude = longitude

    def print_current_measures(self):
        measures = self.get_current_measures()

        print("")
        print("GNSS CONNECTION")
        print("Connected:", measures["connected"])
        print("Port:", measures["port"])
        print("Data age:", measures["data_age"], "s")

        print("")
        print("GNSS POSITION")
        print("Latitude:", measures["latitude"])
        print("Longitude:", measures["longitude"])
        print("Local easting:", measures["local_easting"], "m")
        print("Local northing:", measures["local_northing"], "m")

        print("")
        print("ALTITUDE")
        print("Ellipsoid height:", measures["ellipsoid_height"], "m")
        print("Sea-level height:", measures["sea_level_height"], "m")

        print("")
        print("ACCURACY")
        print("Horizontal accuracy:", measures["horizontal_accuracy"], "m")
        print("Vertical accuracy:", measures["vertical_accuracy"], "m")
        print("Speed accuracy:", measures["speed_accuracy"], "m/s")
        print("Heading accuracy:", measures["heading_accuracy"], "degrees")
        print("PDOP:", measures["pdop"])

        print("")
        print("FIX")
        print("Valid fix:", measures["valid_fix"])
        print("Fix type:", measures["fix_type_text"])
        print("RTK status:", measures["rtk_status_text"])
        print("Differential corrections:", measures["differential_corrections"])
        print("Satellites:", measures["satellites"])

        print("")
        print("MOVEMENT")
        print("Ground speed:", measures["ground_speed"], "m/s")
        print("Ground speed:", measures["ground_speed_kmh"], "km/h")
        print("Heading:", measures["heading"], "degrees")

    def _connection_loop(self):
        while not self._stop_event.is_set():
            if not os.path.exists(self.port):
                self._set_disconnected()
                self._stop_event.wait(
                    self.reconnect_delay
                )
                continue

            if not usb_port_manager.claim_port(
                self.port,
                "gnss",
            ):
                self._set_disconnected()
                self._stop_event.wait(
                    self.reconnect_delay
                )
                continue

            try:
                opened = self._open_serial()

                if opened:
                    self._read_serial_until_disconnect()

            finally:
                self._close_serial()

            if not self._stop_event.is_set():
                self._stop_event.wait(
                    self.reconnect_delay
                )

    def _open_serial(self):
        try:
            self._serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.serial_timeout,
                write_timeout=self.serial_timeout
            )

            self._buffer = b""

            with self._lock:
                self._measures["port"] = self.port
                self._measures["connected"] = False

            if self.verbose:
                print("Trying GNSS port:", self.port)

            return True

        except Exception as error:
            self._serial = None
            self._set_disconnected()

            if self.verbose:
                print("Could not open GNSS port:", self.port, error)

            return False

    def _close_serial(self):
        serial_port = self._serial
        self._serial = None

        if serial_port is not None:
            try:
                if serial_port.is_open:
                    serial_port.close()
            except Exception:
                pass

        usb_port_manager.release_port(
            self.port,
            "gnss",
        )

        self._set_disconnected()

    def _set_disconnected(self):
        with self._lock:
            self._measures["connected"] = False

    def _read_serial_until_disconnect(self):
        valid_packet_received = False
        start_time = time.time()
        last_data_time = start_time

        while not self._stop_event.is_set():
            serial_port = self._serial

            if serial_port is None:
                return valid_packet_received

            if not os.path.exists(self.port):
                return valid_packet_received

            try:
                waiting = serial_port.in_waiting
                data = serial_port.read(
                    waiting or 1
                )

            except Exception as error:
                if self.verbose:
                    print("GNSS serial read error:", error)

                return valid_packet_received

            current_time = time.time()

            if data:
                last_data_time = current_time
                self._buffer += data

                if self._process_buffer():
                    if (
                        not valid_packet_received
                        and self.verbose
                    ):
                        print("Connected to GNSS on:", self.port)

                    valid_packet_received = True

            if (
                current_time - last_data_time
                > self.no_data_reconnect_time
            ):
                if self.verbose:
                    print("No GNSS data received on:", self.port)

                return valid_packet_received

            if (
                not valid_packet_received
                and current_time - start_time
                > self.no_data_reconnect_time
            ):
                if self.verbose:
                    print("No valid UBX NAV-PVT packet on:", self.port)

                return False

        return valid_packet_received

    def _process_buffer(self):
        nav_pvt_received = False

        while True:
            start = self._buffer.find(
                b"\xB5\x62"
            )

            if start < 0:
                self._buffer = self._buffer[-1:]
                break

            if start > 0:
                self._buffer = self._buffer[start:]

            if len(self._buffer) < 6:
                break

            message_class = self._buffer[2]
            message_id = self._buffer[3]

            payload_length = (
                self._buffer[4]
                | (self._buffer[5] << 8)
            )

            packet_length = 6 + payload_length + 2

            if len(self._buffer) < packet_length:
                break

            packet = self._buffer[:packet_length]
            self._buffer = self._buffer[packet_length:]

            calculated_a, calculated_b = ubx_checksum(
                packet[2:-2]
            )

            if (
                calculated_a != packet[-2]
                or calculated_b != packet[-1]
            ):
                continue

            if (
                message_class == 0x01
                and message_id == 0x07
                and payload_length == 92
            ):
                try:
                    measures = self._decode_nav_pvt(
                        packet[6:-2]
                    )

                    self._store_measures(
                        measures
                    )

                    nav_pvt_received = True

                except Exception as error:
                    if self.verbose:
                        print("Could not decode GNSS NAV-PVT packet:", error)

        return nav_pvt_received

    def _local_easting_northing(
        self,
        latitude,
        longitude
    ):
        with self._lock:
            if self.reference_latitude is None:
                self.reference_latitude = latitude
                self.reference_longitude = longitude

            reference_latitude = self.reference_latitude
            reference_longitude = self.reference_longitude

        earth_radius = 6378137.0

        latitude_radians = math.radians(
            latitude
        )

        reference_latitude_radians = math.radians(
            reference_latitude
        )

        latitude_difference = math.radians(
            latitude - reference_latitude
        )

        longitude_difference = math.radians(
            longitude - reference_longitude
        )

        northing = earth_radius * latitude_difference

        easting = (
            earth_radius
            * longitude_difference
            * math.cos(
                (
                    latitude_radians
                    + reference_latitude_radians
                ) / 2.0
            )
        )

        return easting, northing

    def _decode_nav_pvt(self, payload):
        fix_type = payload[20]
        flags = payload[21]
        satellites = payload[23]

        longitude = struct.unpack_from(
            "<i",
            payload,
            24
        )[0] * 0.0000001

        latitude = struct.unpack_from(
            "<i",
            payload,
            28
        )[0] * 0.0000001

        ellipsoid_height = struct.unpack_from(
            "<i",
            payload,
            32
        )[0] / 1000.0

        sea_level_height = struct.unpack_from(
            "<i",
            payload,
            36
        )[0] / 1000.0

        horizontal_accuracy = struct.unpack_from(
            "<I",
            payload,
            40
        )[0] / 1000.0

        vertical_accuracy = struct.unpack_from(
            "<I",
            payload,
            44
        )[0] / 1000.0

        ground_speed = struct.unpack_from(
            "<i",
            payload,
            60
        )[0] / 1000.0

        heading = struct.unpack_from(
            "<i",
            payload,
            64
        )[0] * 0.00001

        speed_accuracy = struct.unpack_from(
            "<I",
            payload,
            68
        )[0] / 1000.0

        heading_accuracy = struct.unpack_from(
            "<I",
            payload,
            72
        )[0] * 0.00001

        pdop = struct.unpack_from(
            "<H",
            payload,
            76
        )[0] * 0.01

        valid_fix = bool(
            flags & 0x01
        )

        differential_corrections = bool(
            flags & 0x02
        )

        rtk_status = (
            flags >> 6
        ) & 0x03

        easting, northing = self._local_easting_northing(
            latitude,
            longitude
        )

        return {
            "latitude": latitude,
            "longitude": longitude,
            "local_easting": easting,
            "local_northing": northing,

            "ellipsoid_height": ellipsoid_height,
            "sea_level_height": sea_level_height,

            "horizontal_accuracy": horizontal_accuracy,
            "vertical_accuracy": vertical_accuracy,
            "speed_accuracy": speed_accuracy,
            "heading_accuracy": heading_accuracy,
            "pdop": pdop,

            "valid_fix": valid_fix,
            "fix_type": fix_type,
            "fix_type_text": fix_type_text(fix_type),
            "rtk_status": rtk_status,
            "rtk_status_text": rtk_status_text(rtk_status),
            "differential_corrections": differential_corrections,
            "satellites": satellites,

            "ground_speed": ground_speed,
            "ground_speed_kmh": ground_speed * 3.6,
            "heading": heading
        }

    def _store_measures(self, measures):
        current_time = time.time()

        with self._lock:
            self._measures.update(measures)
            self._measures["connected"] = True
            self._measures["last_update_time"] = current_time
            self._measures["data_age"] = 0.0

        # Debug each valid GNSS refresh:
        # print("[GNSS REFRESH]", measures["latitude"], measures["longitude"], measures["horizontal_accuracy"], measures["rtk_status_text"], measures["satellites"])


GNSSReader = NMEAReader
NMEA = NMEAReader
