# ============================================================
# USER CONFIGURATION
# ============================================================

FRONT_CONTROLLER_PORT = "/dev/serial/by-id/usb-Basicmicro_Inc._USB_Roboclaw_2x15A_20-if00"
REAR_CONTROLLER_PORT = "/dev/serial/by-id/usb-Basicmicro_Inc._USB_Roboclaw_2x15A_10-if00"

# Replace these two placeholders with the exact paths returned by:
# ls -l /dev/serial/by-id/
ARDUINO_PORT = "/dev/serial/by-id/PUT_ARDUINO_PATH_HERE"
GNSS_PORT = "/dev/serial/by-id/PUT_GNSS_PATH_HERE"

ROBOCLAW_BAUDRATE = 38400
ADDRESS = 0x80

ARDUINO_BAUDRATE = 9600

MAX_MOTOR_DUTY = 32000
DEADZONE = 0.08

CONTROL_PERIOD = 0.05
MOTOR_PERIOD = 0.05
MOTOR_WATCHDOG_TIMEOUT = 0.50

# Exponential smoothing applied to every motor command.
# This also applies when the requested command becomes zero.
MOTOR_SMOOTHING_ALPHA = 0.4

# Device-manager polling period.
DEVICE_CHECK_PERIOD = 0.50

# Arduino normally resets when its serial port is opened.
ARDUINO_RESET_DELAY = 2.0
ARDUINO_IDENTIFY_TIMEOUT = 1.5

BUTTON_USER_DRIVE = 305
BUTTON_PI_RESET = 304
BUTTON_PI_RETURN = 306
BUTTON_RECORDING_START = 307
BUTTON_SPIRAL = 309

BUTTON_VISUAL_RESET = 308
BUTTON_SNAPSHOT = 310
BUTTON_VISUAL_HOME = 311

TICKS_TO_FLOW = 0.001


# ============================================================
# NEW NON-CRITICAL AUXILIARY SYSTEMS
# ============================================================

GNSS_BAUDRATE = 38400
GNSS_SERIAL_TIMEOUT = 0.2
GNSS_RECONNECT_DELAY = 2.0
GNSS_NO_DATA_RECONNECT_TIME = 5.0

CSV_SAMPLE_PERIOD = 0.1
CSV_FLUSH_PERIOD = 1.0
CSV_QUEUE_SIZE = 2000

LCD_UPDATE_PERIOD = 0.5
ARDUINO_OUTPUT_QUEUE_SIZE = 100

RECORDING_ROOT = "robot_recordings"
CSV_FILENAME = "robot_data.csv"
VISUAL_MODEL_FILENAME = "visual_compass.pkl"
VISUAL_MEMORY_DIRECTORY_NAME = "visual_memory"
DEBUG_SNAPSHOT_DIRECTORY_NAME = "debug_snapshots_20x20"


# ============================================================
# MOTOR MAPPING
# ============================================================

MOTOR_MAP = {
    "front_left": {
        "controller": "front",
        "channel": "M2",
        "direction": +1,
    },
    "front_right": {
        "controller": "front",
        "channel": "M1",
        "direction": +1,
    },
    "rear_left": {
        "controller": "rear",
        "channel": "M2",
        "direction": -1,
    },
    "rear_right": {
        "controller": "rear",
        "channel": "M1",
        "direction": +1,
    },
}

CONTROLLER_PORTS = {
    "front": FRONT_CONTROLLER_PORT,
    "rear": REAR_CONTROLLER_PORT,
}
