import subprocess
import threading
import cv2
import cv2.aruco as aruco
import numpy as np
import yaml
import time
import signal
import sys
import serial
from pymavlink import mavutil
import pymavlink.dialects.v20.common as mavlink
import os
import RPi.GPIO as GPIO


LOG_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "plnd_roamer.log")
# Clear log on each start to avoid unbounded growth.
log_file = open(LOG_PATH, "w", buffering=1)

class Tee:
    def __init__(self, *streams):
        self.streams = streams

    def write(self, data):
        for stream in self.streams:
            stream.write(data)
            stream.flush()

    def flush(self):
        for stream in self.streams:
            stream.flush()

sys.stdout = Tee(sys.stdout, log_file)
sys.stderr = Tee(sys.stderr, log_file)

try:
    strobe = serial.Serial('/dev/ttyUSB0', 115200, timeout=1) 
    print("Strobe connected on /dev/ttyUSB0")
    time.sleep(1)
except Exception as e:
    print(f"Cannot open /dev/ttyUSB0 → {e}")
    strobe = None
    
def send(mode: int):
    if strobe and 1 <= mode <= 6:
        try:
            strobe.write(f"{mode}\n".encode())
            strobe.flush()
        except:
            pass
    elif mode == 0 and strobe:
        try:
            strobe.write(f"4\n".encode()) # Fallback to GPS_FIXED (Mode 4) for graceful exit
            strobe.flush()
        except:
            pass

current_led_state = "BOOTING"
last_blink_time = time.time()
BLINK_FAST = 0.2
BLINK_SLOW = 0.5
current_altitude = None

current_vz = 0.0      # Vertical velocity (climb rate)
is_armed = False
is_rtl_mode = False
has_rtk_fix = False
has_non_rtk_fix = False
current_mode = 0
last_failsafe_command_time = 0.0
mavlink_status_lock = threading.Lock()
mavlink_recv_lock = threading.Lock()
last_distance_to_target = None
last_distance_time = 0.0
wind_boost_until = 0.0
last_landing_target_send_time = 0.0
prev_angle_x_rad = None
prev_angle_y_rad = None
current_distance_band_state = "medium"
prev_lateral_x_m = None
prev_lateral_y_m = None
prev_lateral_time_s = 0.0
last_marker_seen_time = 0.0
tracking_state_reset_for_loss = False

timesync_offset_ns = None
last_timesync_request_monotonic = 0.0
last_timesync_tx_local_ns = 0
last_timesync_rtt_ms = None

ANGLE_GAIN_X = 1.2
ANGLE_GAIN_Y = 1.2

altitude_lock = threading.Lock()

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

WIDTH = 1280
HEIGHT = 720
HEARTBEAT_TIMEOUT = 2.0
MAX_LANDING_TARGET_HZ = 20.0
LANDING_TARGET_MIN_PERIOD_S = 1.0 / MAX_LANDING_TARGET_HZ
MAX_FRAME_AGE_S = 0.25
TIMESYNC_REQUEST_INTERVAL_S = 1.0
TIMESYNC_OFFSET_ALPHA = 0.2
MAX_TIMESYNC_RTT_NS = 100_000_000  # 100 ms
MAX_CORRECTION_ANGLE_RAD = np.deg2rad(15.0)
ANGLE_LPF_ALPHA = 0.35
ANGLE_LPF_ALPHA_NEAR_GROUND = 0.72
MIN_CORRECTION_GAIN = 0.85
CENTER_DEADBAND_M_FAR = 0.005
CENTER_DEADBAND_M_MID = 0.008
CENTER_DEADBAND_M_NEAR = 0.016
JUMP_REJECT_DIST_M = 0.9
MAX_LATERAL_JUMP_M = 0.06
MAX_LATERAL_JUMP_M_FALLBACK = 0.04
MAX_LATERAL_JUMP_DT_S = 0.25
MAX_LATERAL_JUMP_M_NEAR = 0.025
MAX_LATERAL_JUMP_M_MID = 0.035
MARKER_LOST_RESET_S = 0.20

CAMERA_X_OFFSET_MM = 0
# Positive CAMERA_Y_OFFSET_MM increases rightward correction in current body mapping.
CAMERA_Y_OFFSET_MM = 0
CAMERA_Z_OFFSET_MM = 0

base_marker_positions = {
    5: np.array([0.0, 0.0, 0.0]),
    4: np.array([0.0, 2.5, 0.0]),
    6: np.array([0.0, -2.5, 0.0]),
}

base_marker_sizes = {
    5: 30,
    4: 70,
    6: 70,
}

MARKERS_FOR_ALTITUDE_BAND = {
    "high": {
        "ids": [4, 6],
        "sizes": {4: base_marker_sizes[4], 6: base_marker_sizes[6]},
        "positions": {
            4: base_marker_positions[4],
            6: base_marker_positions[6]
        }
    },
    "medium": {
        "ids": [4, 6],
        "sizes": {4: base_marker_sizes[4], 6: base_marker_sizes[6]},
        "positions": {
            4: base_marker_positions[4],
            6: base_marker_positions[6],
        }
    },
    "low": {
        "ids": [5],
        "sizes": {5: base_marker_sizes[5]},
        "positions": {
            5: base_marker_positions[5]
        }
    },
    "very_low": {
        "ids": [5],
        "sizes": {5: base_marker_sizes[5]},
        "positions": {
            5: base_marker_positions[5]
        }
    }
}

def get_distance_band(distance_m):
    if distance_m is None:
        return "medium", "MEDIUM (Dist Unknown)", 1, 1
    if distance_m > 1.5:
        return "high", "HIGH", 1, 1
    if distance_m > 1.0:
        return "medium", "MEDIUM", 1, 1
    if distance_m > 0.3:
        return "low", "LOW", 0.9, 0.9
    return "very_low", "VERY LOW (Landing)", 0.85, 0.85

def get_distance_band_hysteresis(distance_m, prev_band):
    # Hysteresis avoids rapid medium<->low switching near ~1.0 m.
    if prev_band not in ("high", "medium", "low", "very_low"):
        prev_band = "medium"

    if distance_m is None:
        return get_distance_band(None)

    if prev_band == "high":
        band = "high" if distance_m > 1.35 else "medium"
    elif prev_band == "medium":
        if distance_m > 1.65:
            band = "high"
        elif distance_m < 0.85:
            band = "low"
        else:
            band = "medium"
    elif prev_band == "low":
        if distance_m > 1.15:
            band = "medium"
        elif distance_m < 0.22:
            band = "very_low"
        else:
            band = "low"
    else:  # very_low
        band = "very_low" if distance_m < 0.35 else "low"

    if band == "high":
        return "high", "HIGH", 1.1, 1.1
    if band == "medium":
        return "medium", "MEDIUM", 1.1, 1.1
    if band == "low":
        return "low", "LOW", 1.1, 1.1
    return "very_low", "VERY LOW (Landing)", 0.8, 0.8

def get_dynamic_gain(altitude_m):
    if altitude_m is None:
        return None
    if altitude_m > 2.0:
        return 1.1
    if altitude_m > 1.0:
        return 1.1
    if altitude_m >= 0.5:
        return 1.1
    return 0.8

def get_lpf_alpha(distance_m):
    if distance_m is None:
        return ANGLE_LPF_ALPHA
    if distance_m < 0.8:
        return ANGLE_LPF_ALPHA_NEAR_GROUND
    if distance_m < 1.2:
        return 0.5
    return ANGLE_LPF_ALPHA

def lpf_angle(prev_value, new_value, alpha):
    if prev_value is None:
        return new_value
    return (alpha * new_value) + ((1.0 - alpha) * prev_value)

def get_center_deadband_m(distance_m):
    if distance_m is None:
        return CENTER_DEADBAND_M_MID
    if distance_m < 0.4:
        return CENTER_DEADBAND_M_NEAR
    if distance_m < 0.8:
        return CENTER_DEADBAND_M_MID
    return CENTER_DEADBAND_M_FAR

def apply_center_deadband(x_m, y_m, distance_m):
    deadband_m = get_center_deadband_m(distance_m)
    x_db = 0.0 if abs(x_m) < deadband_m else x_m
    y_db = 0.0 if abs(y_m) < deadband_m else y_m
    return x_db, y_db, deadband_m

def get_max_allowed_lateral_jump_m(distance_m, fallback_used):
    base_limit_m = MAX_LATERAL_JUMP_M_FALLBACK if fallback_used else MAX_LATERAL_JUMP_M
    if distance_m is None:
        return base_limit_m
    if distance_m < 0.5:
        return min(base_limit_m, MAX_LATERAL_JUMP_M_NEAR)
    if distance_m < 0.7:
        return min(base_limit_m, MAX_LATERAL_JUMP_M_MID)
    return base_limit_m

def maybe_reset_tracking_on_target_loss():
    global prev_angle_x_rad, prev_angle_y_rad
    global prev_lateral_x_m, prev_lateral_y_m, prev_lateral_time_s
    global tracking_state_reset_for_loss

    if last_marker_seen_time <= 0:
        return
    if (time.time() - last_marker_seen_time) < MARKER_LOST_RESET_S:
        return
    if tracking_state_reset_for_loss:
        return

    prev_angle_x_rad = None
    prev_angle_y_rad = None
    prev_lateral_x_m = None
    prev_lateral_y_m = None
    prev_lateral_time_s = 0.0
    tracking_state_reset_for_loss = True
    print("⚠️ Target lost >200ms, reset tracking state.")

def maybe_send_timesync_request():
    global last_timesync_request_monotonic, last_timesync_tx_local_ns
    now_mono = time.monotonic()
    if now_mono - last_timesync_request_monotonic < TIMESYNC_REQUEST_INTERVAL_S:
        return
    ts1_local_ns = time.monotonic_ns()
    try:
        master.mav.timesync_send(0, ts1_local_ns)
        last_timesync_tx_local_ns = ts1_local_ns
        last_timesync_request_monotonic = now_mono
    except Exception as e:
        print(f"⚠️ Failed to send TIMESYNC request: {e}")

def handle_timesync_message(msg):
    global timesync_offset_ns, last_timesync_rtt_ms
    rx_local_ns = time.monotonic_ns()

    if msg.tc1 == 0:
        reply_tc1 = rx_local_ns
        if 0 < abs(msg.ts1) < 10_000_000_000_000:
            # Some FC stacks use usec in TIMESYNC; answer in matching units.
            reply_tc1 = rx_local_ns // 1000
        try:
            master.mav.timesync_send(reply_tc1, msg.ts1)
        except Exception as e:
            print(f"⚠️ Failed to reply TIMESYNC: {e}")
        return

    if msg.ts1 <= 0:
        return

    # Track offsets only for replies to local requests.
    if msg.ts1 != last_timesync_tx_local_ns:
        return

    rtt_ns = rx_local_ns - msg.ts1
    if rtt_ns <= 0 or rtt_ns > MAX_TIMESYNC_RTT_NS:
        return

    tc1_in_ns = msg.tc1
    if msg.tc1 != 0:
        unit_ratio = abs(msg.ts1 / msg.tc1)
        if unit_ratio > 100.0:
            # Local ts1 is ns while FC tc1 is usec.
            tc1_in_ns = msg.tc1 * 1000

    measured_offset_ns = tc1_in_ns - ((msg.ts1 + rx_local_ns) // 2)
    if timesync_offset_ns is None:
        timesync_offset_ns = measured_offset_ns
    else:
        timesync_offset_ns = int(
            (1.0 - TIMESYNC_OFFSET_ALPHA) * timesync_offset_ns +
            TIMESYNC_OFFSET_ALPHA * measured_offset_ns
        )
    last_timesync_rtt_ms = rtt_ns / 1e6

def get_fc_corrected_time_usec(local_time_ns):
    if timesync_offset_ns is None:
        return int(time.time() * 1e6)
    fc_time_ns = local_time_ns + timesync_offset_ns
    if fc_time_ns <= 0:
        return int(time.time() * 1e6)
    return int(fc_time_ns // 1000)

def load_camera_params(path):
    with open(path) as f:
        data = yaml.safe_load(f)
    return np.array(data["camera_matrix"]), np.array(data["dist_coeff"])

def build_transform(rvec, tvec):
    R, _ = cv2.Rodrigues(rvec)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = tvec.reshape(3)
    return T

def invert_transform(T):
    R = T[:3, :3]
    t = T[:3, 3]
    T_inv = np.eye(4)
    T_inv[:3, :3] = R.T
    T_inv[:3, 3] = -R.T @ t
    return T_inv


def update_leds(led_state):
    global current_led_state
    
    mode_map = {
        "CAMERA_FAIL": 1,
        "NO_HEARTBEAT": 2,
        "GPS_NO_FIX": 3,
        "GPS_FIXED": 4,
        "RTL": 5,
        "CRITICAL_LANDING": 6
    }

    mode_to_send = mode_map.get(led_state, 3)
    send(mode_to_send)


master = None
drone_is_armed_by_script = False
last_arm_disarm_command_time = 0
last_heartbeat_time = time.time()

def get_mav_result_name(result_code):
    if result_code == 0:
        return "ACCEPTED"
    try:
        if hasattr(master, 'mav') and hasattr(master.mav, 'decode_command_ack_result'):
            return master.mav.decode_command_ack_result(result_code)
        else:
            result_names = {
                1: "TEMPORARILY_REJECTED", 2: "DENIED",
                3: "UNSUPPORTED", 4: "FAILED", 5: "IN_PROGRESS"
            }
            return result_names.get(result_code, f"Unknown Result ({result_code})")
    except Exception:
        return f"Unknown Result ({result_code})"

def disarm_drone_if_close():
    global drone_is_armed_by_script, last_arm_disarm_command_time, master

    if master is None or master.target_system == 0 or master.target_component == 0:
        return

    if (time.time() - last_arm_disarm_command_time > 2):
        print("Sending DISARM command (ID 400) to drone...")
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0, 0, 0, 0, 0, 0, 0
        )
        last_arm_disarm_command_time = time.time()
        print("DISARM command sent. Waiting for ACK...")

        ack = master.recv_match(type='COMMAND_ACK', blocking=False, timeout=1)
        if ack:
            if ack.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                if ack.result == 0:
                    print("✅ Drone disarmed successfully.")
                    drone_is_armed_by_script = False
                else:
                    print(f"❌ Disarm command DENIED. Result: {get_mav_result_name(ack.result)} ({ack.result})")
            else:
                print(f"Received unexpected ACK (ID {ack.command}) for disarm command. Result: {get_mav_result_name(ack.result)}")
        else:
            print("No ACK received for DISARM command within timeout.")

def altitude_monitor():
    global current_altitude, current_vz
    print("📡 Altitude monitor thread started.")
    try:
        # Requesting GLOBAL_POSITION_INT (for altitude) and VFR_HUD (for climb rate)
        master.mav.request_data_stream_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_POSITION, 2, 1
        )
        master.mav.request_data_stream_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 2, 1
        )
        print("Requested GLOBAL_POSITION_INT and VFR_HUD data streams.")
    except Exception as e:
        print("Failed to request data stream:", e)
        return

    while True:
        try:
            # Read GLOBAL_POSITION_INT for altitude
            with mavlink_recv_lock:
                msg_alt = master.recv_match(type='GLOBAL_POSITION_INT', blocking=False, timeout=0.1)
            if msg_alt:
                alt = msg_alt.relative_alt / 1000.0
                with altitude_lock:
                    current_altitude = alt
            
            # Read VFR_HUD for vertical speed
            with mavlink_recv_lock:
                msg_hud = master.recv_match(type='VFR_HUD', blocking=False, timeout=0.1)
            if msg_hud:
                # FIX: Changed incorrect attribute 'vzed' to correct attribute 'climb'
                climb_rate = msg_hud.climb
                with altitude_lock:
                    current_vz = climb_rate
            
            time.sleep(0.01) # Small delay to avoid hammering the CPU
        except Exception as e:
            # Removed the original error flooding print for VFR_HUD
            print(f"⚠️ Altitude monitor error: {e}")
            time.sleep(1)

def signal_handler(sig, frame):
    print("\n🛑 Terminating...")
    if 'pipe' in globals() and pipe:
        pipe.terminate()
    if master and master.target_system != 0 and master.target_component != 0:
        print("Attempting to disarm before script exit...")
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0, 0, 0, 0, 0, 0, 0
        )
        time.sleep(0.5)
    if 'master' in globals() and master:
        master.close()
    if log_file:
        log_file.close()
    
    send(0) 
    if strobe: strobe.close()
    
    GPIO.cleanup()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

cmd = [
    "rpicam-vid",
    "--width", f"{WIDTH}",
    "--height", f"{HEIGHT}",
    "--framerate", "30",
    "--timeout", "0",
    "--codec", "yuv420",
    "--autofocus-mode", "continuous",
    "-o", "-"
]
try:
    pipe = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
except FileNotFoundError:
    print("Error: 'rpicam-vid' command not found. Ensure libcamera is installed and in PATH.")
    signal_handler(signal.SIGINT, None)

def read_frame_yuv(pipe):
    yuv_size = WIDTH * HEIGHT * 3 // 2
    raw = pipe.stdout.read(yuv_size)
    if len(raw) != yuv_size:
        return None, None
    yuv = np.frombuffer(raw, dtype=np.uint8).reshape((HEIGHT * 3 // 2, WIDTH))
    bgr = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_I420)
    frame_timestamp_ns = time.monotonic_ns()
    return bgr, frame_timestamp_ns

K, D = load_camera_params("Camera-Conf/picam64.yml")
dict_aruco = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_MIP_36H12)
params = aruco.DetectorParameters()

try:
    master = mavutil.mavlink_connection('/dev/ttyS0', baud=921600, timeout=0.1)
except serial.SerialException as e:
    print(f"Error: Could not open serial port /dev/ttyS0: {e}") 
    signal_handler(signal.SIGINT, None)

print("\n📡 Headless marker fusion with MAVLink started (Ctrl+C to stop)")
print("Waiting for first heartbeat from drone to set target IDs...")
update_leds("NO_HEARTBEAT") # Set initial LED state to Mode 2
current_led_state = "NO_HEARTBEAT"
timeout_start = time.time()
HEARTBEAT_WAIT_LIMIT = 10 # Wait up to 10 seconds for the first heartbeat

# FIX: Implemented non-blocking heartbeat waiting logic
while time.time() - timeout_start < HEARTBEAT_WAIT_LIMIT:
    msg = master.recv_match(type='HEARTBEAT', blocking=False, timeout=0.1)
    if msg:
        last_heartbeat_time = time.time()
        master.target_system = msg.get_srcSystem()
        master.target_component = msg.get_srcComponent()
        print(f"Heartbeat received! Drone ID: {master.target_system}, Comp ID: {master.target_component}")
        
        # Start monitoring threads and requests only after successful heartbeat
        altitude_thread = threading.Thread(target=altitude_monitor, daemon=True)
        altitude_thread.start()
        
        # Request data stream for VFR_HUD and GLOBAL_POSITION_INT (moved to altitude_monitor)
        # Request raw sensor data stream (originally here)
        master.mav.request_data_stream_send(
            master.target_system, master.target_component, 
            mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS, 2, 1
        )
        maybe_send_timesync_request()
        break # Exit the loop, heartbeat received
    time.sleep(0.01)

if master.target_system == 0:
    print("❌ Failed to receive initial heartbeat within timeout. Setting default IDs.")
    # Set default IDs so the main loop doesn't crash
    master.target_system = 1
    master.target_component = 1
# --- END REVISED HEARTBEAT WAITING LOGIC ---


while True:
    frame, frame_timestamp_ns = read_frame_yuv(pipe)
    
    if frame is None:
        print("⚠️ Camera capture failed. Re-trying...")
        new_led_state = "CAMERA_FAIL" 
        if new_led_state != current_led_state:
            update_leds(new_led_state)
            current_led_state = new_led_state
        time.sleep(0.1)
        continue

    markers_detected = False
    
    try:
        maybe_send_timesync_request()
        
        while master.port.in_waiting:
            with mavlink_recv_lock:
                msg = master.recv_match(blocking=False)
            if msg:
                with mavlink_status_lock:
                    if msg.get_msgId() == mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT:
                        last_heartbeat_time = time.time()
                        master.target_system = msg.get_srcSystem()
                        master.target_component = msg.get_srcComponent()

                        is_armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                        current_mode = msg.custom_mode
                        # ArduPilot RTL is mode 6. Loiter (9) is often used for landing phase in scripts.
                        is_rtl_mode = (msg.custom_mode == 6) or (msg.custom_mode == 9)

                    elif msg.get_type() == 'GPS_RAW_INT':
                        if msg.fix_type == 6:
                            has_rtk_fix = True
                            has_non_rtk_fix = False
                        elif msg.fix_type > 1:
                            has_rtk_fix = False
                            has_non_rtk_fix = True
                        else:
                            has_rtk_fix = False
                            has_non_rtk_fix = False

                    elif msg.get_msgId() == mavutil.mavlink.MAVLINK_MSG_ID_COMMAND_ACK:
                        if msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                            print(f"✅ MAVLink Command ACK received: Result: {get_mav_result_name(msg.result)}")
                    elif msg.get_type() == "TIMESYNC":
                        handle_timesync_message(msg)
    except Exception as e:
        print(f"MAVLink read error: {e}")
        pass
    heartbeat_age = time.time() - last_heartbeat_time
    
    new_led_state = ""
    with altitude_lock:
        alt = current_altitude

    # --- CRITICAL FIX: Modified LED Mode 6 (CRITICAL_LANDING) Logic ---
    # Trigger Mode 6 ONLY if in RTL/Land mode AND below 3.0 meters
    is_critical_landing = alt is not None and is_rtl_mode and alt < 3.0
    
    if is_critical_landing:
        new_led_state = "CRITICAL_LANDING"
        
    else:
        with mavlink_status_lock:
            if heartbeat_age > HEARTBEAT_TIMEOUT:
                new_led_state = "NO_HEARTBEAT"
            elif is_rtl_mode:
                new_led_state = "RTL"
            elif has_rtk_fix or has_non_rtk_fix:
                new_led_state = "GPS_FIXED" 
            else:
                new_led_state = "GPS_NO_FIX" 

    if new_led_state != current_led_state:
        if new_led_state == "CRITICAL_LANDING":
            print(f"🚨 CRITICAL LANDING detected (RTL Mode, Alt: {alt:.2f}m). Setting LED Mode 6 (ALL WHITE static).")
        
        update_leds(new_led_state)
        current_led_state = new_led_state

    with altitude_lock:
        alt = current_altitude
        vz = current_vz

    
    current_band_name_for_dict_access = "N/A"
    display_band_name = "N/A"
    ANGLE_GAIN_X = 1.2
    ANGLE_GAIN_Y = 1.2

    # Distance-based band logic (using last valid target distance).
    current_band_name_for_dict_access, display_band_name, ANGLE_GAIN_X, ANGLE_GAIN_Y = get_distance_band_hysteresis(
        last_distance_to_target, current_distance_band_state
    )
    current_distance_band_state = current_band_name_for_dict_access

    active_marker_ids = MARKERS_FOR_ALTITUDE_BAND[current_band_name_for_dict_access]["ids"]
    active_marker_sizes = MARKERS_FOR_ALTITUDE_BAND[current_band_name_for_dict_access]["sizes"]
    active_marker_positions = MARKERS_FOR_ALTITUDE_BAND[current_band_name_for_dict_access]["positions"]

    corners, ids, rejectedImgPoints = aruco.detectMarkers(frame, dict_aruco, parameters=params)
    
    valid_marker_tvecs = []
    
    if ids is not None:
        marker_filter_fallback_used = False
        detected_ids = ids.flatten().tolist()
        detected_corners = corners
        filtered_ids = []
        filtered_corners = []
        for i, marker_id in enumerate(detected_ids):
            if marker_id in active_marker_ids:
                filtered_ids.append(marker_id)
                filtered_corners.append(detected_corners[i])

        allow_marker_fallback = current_band_name_for_dict_access == "medium"
        if (not filtered_corners) and allow_marker_fallback:
            # Fallback: use any known marker that is visible to avoid estimator dropouts
            # when distance band switches aggressively.
            for i, marker_id in enumerate(detected_ids):
                if marker_id in base_marker_sizes:
                    filtered_ids.append(marker_id)
                    filtered_corners.append(detected_corners[i])
            marker_filter_fallback_used = len(filtered_corners) > 0
            
        ids = np.array(filtered_ids).reshape(-1,1) if filtered_corners else None
        corners = tuple(filtered_corners) if filtered_corners else None

        if ids is not None:
            markers_detected = True
            
            # Debug image saving logic
            if time.time() - getattr(read_frame_yuv, 'last_debug_save_time', 0) > 10:
                debug_filename = "/home/intek/precision_landing/debug.jpg"
                aruco.drawDetectedMarkers(frame, corners, ids)
                cv2.imwrite(debug_filename, frame)
                read_frame_yuv.last_debug_save_time = time.time()
                print(f"Saved debug frame to {debug_filename}")

            marker_size_lookup = dict(active_marker_sizes)
            if marker_filter_fallback_used:
                marker_size_lookup.update(base_marker_sizes)

            for i, marker_id in enumerate(ids.flatten()):
                size = marker_size_lookup[marker_id]
                rvec, tvec, _ = aruco.estimatePoseSingleMarkers([corners[i]], size, K, D)
                valid_marker_tvecs.append(tvec[0][0])
                
            if valid_marker_tvecs:
                fused_tvec = np.mean(valid_marker_tvecs, axis=0)

                x_offset_drone_body_m = (fused_tvec[1] + CAMERA_X_OFFSET_MM) / 1000.0
                y_offset_drone_body_m = (fused_tvec[0] + CAMERA_Y_OFFSET_MM) / 1000.0
                z_offset_drone_body_m = (fused_tvec[2] + CAMERA_Z_OFFSET_MM)/ 1000.0

                if z_offset_drone_body_m < 0.0:
                    print(f"CRITICAL WARNING: Calculated Z-offset is negative ({z_offset_drone_body_m:.2f}m). Skipping MAVLink send.")
                    continue

                if z_offset_drone_body_m * 1000.0 < 100.0:
                    disarm_drone_if_close()
                else:
                    pass

                raw_x_offset_drone_body_m = x_offset_drone_body_m
                raw_y_offset_drone_body_m = y_offset_drone_body_m

                distance_to_target_m = np.sqrt(
                    raw_x_offset_drone_body_m**2 +
                    raw_y_offset_drone_body_m**2 +
                    z_offset_drone_body_m**2
                )

                if distance_to_target_m < 0.05:
                    distance_to_target_m = 0.05

                now = time.time()
                last_marker_seen_time = now
                tracking_state_reset_for_loss = False
                dt_since_prev = now - prev_lateral_time_s if prev_lateral_time_s > 0 else None
                if (
                    prev_lateral_x_m is not None and
                    prev_lateral_y_m is not None and
                    dt_since_prev is not None and
                    dt_since_prev <= MAX_LATERAL_JUMP_DT_S and
                    distance_to_target_m <= JUMP_REJECT_DIST_M
                ):
                    jump_m = np.sqrt(
                        (raw_x_offset_drone_body_m - prev_lateral_x_m) ** 2 +
                        (raw_y_offset_drone_body_m - prev_lateral_y_m) ** 2
                    )
                    max_allowed_jump_m = get_max_allowed_lateral_jump_m(distance_to_target_m, marker_filter_fallback_used)
                    if jump_m > max_allowed_jump_m:
                        print(
                            f"⚠️ Rejecting lateral jump {jump_m*100:.1f}cm "
                            f"(limit {max_allowed_jump_m*100:.1f}cm, band={display_band_name}, "
                            f"fallback={marker_filter_fallback_used})"
                        )
                        continue

                prev_lateral_x_m = raw_x_offset_drone_body_m
                prev_lateral_y_m = raw_y_offset_drone_body_m
                prev_lateral_time_s = now

                x_offset_drone_body_m, y_offset_drone_body_m, center_deadband_m = apply_center_deadband(
                    raw_x_offset_drone_body_m,
                    raw_y_offset_drone_body_m,
                    distance_to_target_m
                )

                frame_age_s = (time.monotonic_ns() - frame_timestamp_ns) / 1e9
                if frame_age_s > MAX_FRAME_AGE_S:
                    print(f"⚠️ Dropping stale frame ({frame_age_s*1000:.1f} ms old), skipping LANDING_TARGET.")
                    continue

                if alt is not None and alt < 1.0 and last_distance_to_target is not None:
                    if now - last_distance_time <= 0.2 and distance_to_target_m - last_distance_to_target > 0.05:
                        wind_boost_until = now + 1.0
                        print(f"💨 Wind spike: dist jump {distance_to_target_m - last_distance_to_target:.2f}m; boost 1.3 for 1s")

                last_distance_to_target = distance_to_target_m
                last_distance_time = now

                gain_x = ANGLE_GAIN_X
                gain_y = ANGLE_GAIN_Y
                dynamic_gain = get_dynamic_gain(alt)
                if dynamic_gain is not None:
                    gain_x = max(gain_x, dynamic_gain)
                    gain_y = max(gain_y, dynamic_gain)
                gain_x = max(gain_x, MIN_CORRECTION_GAIN)
                gain_y = max(gain_y, MIN_CORRECTION_GAIN)
                if now < wind_boost_until:
                    gain_x = max(gain_x, 1.3)
                    gain_y = max(gain_y, 1.3)

                raw_angle_x_rad = np.arctan2(y_offset_drone_body_m, z_offset_drone_body_m) * gain_x
                raw_angle_y_rad = np.arctan2(x_offset_drone_body_m, z_offset_drone_body_m) * gain_y
                raw_angle_x_rad = float(np.clip(raw_angle_x_rad, -MAX_CORRECTION_ANGLE_RAD, MAX_CORRECTION_ANGLE_RAD))
                raw_angle_y_rad = float(np.clip(raw_angle_y_rad, -MAX_CORRECTION_ANGLE_RAD, MAX_CORRECTION_ANGLE_RAD))

                angle_lpf_alpha = get_lpf_alpha(distance_to_target_m)
                angle_x_rad = lpf_angle(prev_angle_x_rad, raw_angle_x_rad, angle_lpf_alpha)
                angle_y_rad = lpf_angle(prev_angle_y_rad, raw_angle_y_rad, angle_lpf_alpha)
                angle_x_rad = float(np.clip(angle_x_rad, -MAX_CORRECTION_ANGLE_RAD, MAX_CORRECTION_ANGLE_RAD))
                angle_y_rad = float(np.clip(angle_y_rad, -MAX_CORRECTION_ANGLE_RAD, MAX_CORRECTION_ANGLE_RAD))
                prev_angle_x_rad = angle_x_rad
                prev_angle_y_rad = angle_y_rad

                if now - last_landing_target_send_time < LANDING_TARGET_MIN_PERIOD_S:
                    continue

                timestamp_us = get_fc_corrected_time_usec(frame_timestamp_ns)

                reported_target_size = 0.5
                if len(filtered_ids) > 0:
                    if filtered_ids[0] in base_marker_sizes:
                        reported_target_size = base_marker_sizes[filtered_ids[0]] / 1000.0

                target_size_x = reported_target_size
                target_size_y = reported_target_size

                msg = mavlink.MAVLink_landing_target_message(
                    time_usec=timestamp_us,
                    target_num=0,
                    frame=mavlink.MAV_FRAME_BODY_NED,
                    angle_x=angle_x_rad,
                    angle_y=angle_y_rad,
                    distance=distance_to_target_m,
                    size_x=target_size_x,
                    size_y=target_size_y
                )
                master.mav.send(msg)
                last_landing_target_send_time = now
                
                raw_x_offset_cm = raw_y_offset_drone_body_m * 100.0
                raw_y_offset_cm = raw_x_offset_drone_body_m * 100.0
                cmd_x_offset_cm = np.tan(angle_x_rad) * z_offset_drone_body_m * 100.0
                cmd_y_offset_cm = np.tan(angle_y_rad) * z_offset_drone_body_m * 100.0
                marker_suffix = "/FALLBACK" if marker_filter_fallback_used else ""
                print(
                    f"⬆️ Sent LANDING_TARGET: rawX({raw_x_offset_cm:.1f}cm), rawY({raw_y_offset_cm:.1f}cm), "
                    f"cmdX({cmd_x_offset_cm:.1f}cm), cmdY({cmd_y_offset_cm:.1f}cm), "
                    f"dist={distance_to_target_m:.2f}m, gain=({gain_x:.2f},{gain_y:.2f}), "
                    f"lpf={angle_lpf_alpha:.2f}, deadband={center_deadband_m*100:.1f}cm. "
                    f"Band: {display_band_name}{marker_suffix}"
                )
            else:
                print("⚠️ Markers detected, but no valid positions estimated (e.g., filtered by altitude band).")
                maybe_reset_tracking_on_target_loss()
        else:
            print("❌ No active markers detected for current altitude band.")
            maybe_reset_tracking_on_target_loss()
    else:
        maybe_reset_tracking_on_target_loss()

    time.sleep(0.005)
# Clean up is handled by signal_handler() on Ctrl+C
# The cleanup block at the end of the script is unreachable if the while True loop runs
