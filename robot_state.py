import threading
import time


# ============================================================
# GLOBAL STOP EVENT
# ============================================================

stop_event = threading.Event()
program_start_time = time.time()


# ============================================================
# DEVICE STATE
# ============================================================

controllers = {
    "front": None,
    "rear": None,
}

ser = None
joystick = None

camera_capture = None
camera_device = None
camera_connected = False
latest_frame = None

arduino_connected = False

gnss_reader = None


# ============================================================
# LOCKS
# ============================================================

controllers_lock = threading.Lock()

arduino_connection_lock = threading.Lock()
arduino_data_lock = threading.Lock()

joystick_lock = threading.Lock()
motor_command_lock = threading.Lock()

camera_connection_lock = threading.Lock()
cam_lock = threading.Lock()

snapshot_lock = threading.Lock()
recording_lock = threading.Lock()
recording_state_lock = threading.Lock()


# ============================================================
# NAVIGATION / CONTROL STATE
# ============================================================

current_mode = "idle"

cx = None
cx_ring = None
vision_model = None
visual_compass = None

ego_deg = 0.0
lastlin = 0.0

latest_azimuth = 0.0
latest_encoder_flow = 0.0

# Duplicate combined encoder accumulator used only by the LCD.
# It is never consumed by path integration.
encoder_display_flow = 0.0

# Latest encoder deltas received in the most recent Arduino packet.
latest_encoder_deltas = {
    "front_right": 0,
    "front_left": 0,
    "rear_right": 0,
    "rear_left": 0,
}

# Existing cumulative encoder values.
fracc = 0
flacc = 0
rracc = 0
rlacc = 0

latest_arduino_packet_time = None


# ============================================================
# SPIRAL / PIROUETTE STATE
# ============================================================

search_error_acc = 0.0
pirouette_error_acc = 0.0
pirouette_phase = 0

search_error = 0.7
spiral_sign = 1
pirouette_sign = 1

pirouette_start_heading = None
pirouette_has_left_start = False

# Last time an automatic pirouette snapshot was taken.
# Used to limit snapshots during the pirouette to 5 Hz maximum.
last_spiral_snapshot_time = 0.0


# ============================================================
# JOYSTICK STATE
# ============================================================

axis_values = {
    0: 127,
    1: 127,
    2: 127,
    5: 127,
}


# ============================================================
# MOTOR COMMAND STATE
# ============================================================

latest_motor_command = {
    "lin": 0.0,
    "strafe": 0.0,
    "rot": 0.0,
    "timestamp": 0.0,
}

latest_smoothed_motor_command = {
    "lin": 0.0,
    "strafe": 0.0,
    "rot": 0.0,
}

latest_motor_duties = {
    "front_m1": 0,
    "front_m2": 0,
    "rear_m1": 0,
    "rear_m2": 0,
    "front_right": 0,
    "front_left": 0,
    "rear_right": 0,
    "rear_left": 0,
}


# ============================================================
# RECORDING / SNAPSHOT STATE
# ============================================================

current_session_directory = None
current_memory_directory = None
current_debug_snapshot_directory = None

snapshot_id = 0
snapshot_counter = 0
last_snapshot_time = None
last_snapshot_filename = None

# CSV recording is deliberately disabled at startup.
# Button 307 enables it for the current results folder.
recording_enabled = False
recording_start_time = None
recording_generation = 0
