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

LOG_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "plnd_tether.log")
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
    print(f"Cannot open /dev/ttyAMA4 → {e}")
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
            strobe.write(f"4\n".encode())
            strobe.flush()
        except:
            pass

current_led_state = "BOOTING"
last_blink_time = time.time()
BLINK_FAST = 0.2
BLINK_SLOW = 0.5
current_altitude = None

current_vz = 0.0
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

ANGLE_GAIN_X = 1
ANGLE_GAIN_Y = 1

altitude_lock = threading.Lock()

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

WIDTH = 1280
HEIGHT = 720
HEARTBEAT_TIMEOUT = 2.0

CAMERA_X_OFFSET_MM = 0
CAMERA_Y_OFFSET_MM = 0
CAMERA_Z_OFFSET_MM = 0

base_marker_positions = {
    4: np.array([0.0, 0.0, 0.0]),
    5: np.array([0.0, -2.5, 0.0]),
    13: np.array([0.0, 58.0, 73.0])
}

base_marker_sizes = {
    5: 50,
    4: 70,
    13: 30
}

MARKERS_FOR_ALTITUDE_BAND = {
    "high": {
        "ids": [4],
        "sizes": {4: base_marker_sizes[4]},
        "positions": {4: base_marker_positions[4]}
    },
    "medium": {
        "ids": [4, 5],
        "sizes": {4: base_marker_sizes[4], 5: base_marker_sizes[5]},
        "positions": {
            4: base_marker_positions[4],
            5: base_marker_positions[5]
        }
    },
    "low": {
        "ids": [4, 5],
        "sizes": {4: base_marker_sizes[4], 5: base_marker_sizes[5]},
        "positions": {
            4: base_marker_positions[4],
            5: base_marker_positions[5]
        }
    },
    "very_low": {
        "ids": [4,5],
        "sizes": {4: base_marker_sizes[4], 5: base_marker_sizes[5]},
        "positions": {
            4: base_marker_positions[4],
            5: base_marker_positions[5]
        }
    }
}

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
                    print("Drone disarmed successfully.")
                    drone_is_armed_by_script = False
                else:
                    print(f"Disarm command DENIED. Result: {get_mav_result_name(ack.result)} ({ack.result})")
            else:
                print(f"Received unexpected ACK (ID {ack.command}) for disarm command. Result: {get_mav_result_name(ack.result)}")
        else:
            print("No ACK received for DISARM command within timeout.")


def altitude_monitor():
    global current_altitude, current_vz
    print("Altitude monitor thread started.")
    try:
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
            with mavlink_recv_lock:
                msg_alt = master.recv_match(type='GLOBAL_POSITION_INT', blocking=False, timeout=0.1)
            if msg_alt:
                alt = msg_alt.relative_alt / 1000.0
                with altitude_lock:
                    current_altitude = alt

            with mavlink_recv_lock:
                msg_hud = master.recv_match(type='VFR_HUD', blocking=False, timeout=0.1)
            if msg_hud:
                climb_rate = msg_hud.climb
                with altitude_lock:
                    current_vz = climb_rate

            time.sleep(0.01)
        except Exception as e:
            print(f"Altitude monitor error: {e}")
            time.sleep(1)


def signal_handler(sig, frame):
    print("\nTerminating...")
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
    if 'master' in globals() and master and master.port.is_open:
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
        return None
    yuv = np.frombuffer(raw, dtype=np.uint8).reshape((HEIGHT * 3 // 2, WIDTH))
    bgr = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_I420)
    return bgr

K, D = load_camera_params("Camera-Conf/picam64.yml")
dict_aruco = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_MIP_36H12)
params = aruco.DetectorParameters()

STARTUP_DELAY_SEC = 10
print(f"Startup delay: waiting {STARTUP_DELAY_SEC}s before MAVLink connect...")
time.sleep(STARTUP_DELAY_SEC)

try:
    master = mavutil.mavlink_connection('/dev/ttyS0', baud=921600, timeout=0.1)
except serial.SerialException as e:
    print(f"Error: Could not open serial port /dev/ttyS0: {e}")
    signal_handler(signal.SIGINT, None)

print("\nHeadless marker fusion with MAVLink started (Ctrl+C to stop)")
print("Waiting for first heartbeat from drone to set target IDs...")
update_leds("NO_HEARTBEAT")
current_led_state = "NO_HEARTBEAT"
timeout_start = time.time()
HEARTBEAT_WAIT_LIMIT = 10

while time.time() - timeout_start < HEARTBEAT_WAIT_LIMIT:
    with mavlink_recv_lock:
        msg = master.recv_match(type='HEARTBEAT', blocking=False, timeout=0.1)
    if msg:
        last_heartbeat_time = time.time()
        master.target_system = msg.get_srcSystem()
        master.target_component = msg.get_srcComponent()
        print(f"Heartbeat received! Drone ID: {master.target_system}, Comp ID: {master.target_component}")

        altitude_thread = threading.Thread(target=altitude_monitor, daemon=True)
        altitude_thread.start()

        master.mav.request_data_stream_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS, 2, 1
        )
        break
    time.sleep(0.01)

if master.target_system == 0:
    print("Failed to receive initial heartbeat within timeout. Setting default IDs.")
    master.target_system = 1
    master.target_component = 1


while True:
    frame = read_frame_yuv(pipe)

    if frame is None:
        print("Camera capture failed. Re-trying...")
        new_led_state = "CAMERA_FAIL"
        if new_led_state != current_led_state:
            update_leds(new_led_state)
            current_led_state = new_led_state
        time.sleep(0.1)
        continue

    markers_detected = False

    try:
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
                            print(f"MAVLink Command ACK received: Result: {get_mav_result_name(msg.result)}")
    except Exception as e:
        print(f"MAVLink read error: {e}")
        pass

    heartbeat_age = time.time() - last_heartbeat_time

    new_led_state = ""
    with altitude_lock:
        alt = current_altitude

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
            print(f"CRITICAL LANDING detected (RTL Mode, Alt: {alt:.2f}m). Setting LED Mode 6 (ALL WHITE static).")

        update_leds(new_led_state)
        current_led_state = new_led_state

    with altitude_lock:
        alt = current_altitude
        vz = current_vz

    current_band_name_for_dict_access = "N/A"
    display_band_name = "N/A"
    ANGLE_GAIN_X = 0.6
    ANGLE_GAIN_Y = 0.6

    if alt is None:
        current_band_name_for_dict_access = "low"
        display_band_name = "LOW (Alt Unknown)"
    elif alt > 1.5:
        current_band_name_for_dict_access = "high"
        display_band_name = "HIGH"
    elif alt > 1.0:
        current_band_name_for_dict_access = "medium"
        display_band_name = "MEDIUM"
    elif alt > 0.3:
        current_band_name_for_dict_access = "low"
        display_band_name = "LOW"
    else:
        current_band_name_for_dict_access = "very_low"
        display_band_name = "VERY LOW (Landing)"

    active_marker_ids = MARKERS_FOR_ALTITUDE_BAND[current_band_name_for_dict_access]["ids"]
    active_marker_sizes = MARKERS_FOR_ALTITUDE_BAND[current_band_name_for_dict_access]["sizes"]
    active_marker_positions = MARKERS_FOR_ALTITUDE_BAND[current_band_name_for_dict_access]["positions"]

    corners, ids, rejectedImgPoints = aruco.detectMarkers(frame, dict_aruco, parameters=params)

    valid_marker_tvecs = []

    if ids is not None:
        filtered_ids = []
        filtered_corners = []
        for i, marker_id in enumerate(ids.flatten()):
            if marker_id in active_marker_ids:
                filtered_ids.append(marker_id)
                filtered_corners.append(corners[i])

        ids = np.array(filtered_ids).reshape(-1,1) if filtered_corners else None
        corners = tuple(filtered_corners) if filtered_corners else None

        if ids is not None:
            markers_detected = True

            if time.time() - getattr(read_frame_yuv, 'last_debug_save_time', 0) > 10:
                debug_filename = "/home/intek/precision_landing/debug.jpg"
                aruco.drawDetectedMarkers(frame, corners, ids)
                cv2.imwrite(debug_filename, frame)
                read_frame_yuv.last_debug_save_time = time.time()
                print(f"Saved debug frame to {debug_filename}")

            for i, marker_id in enumerate(ids.flatten()):
                size = active_marker_sizes[marker_id]
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

                distance_to_target_m = np.sqrt(x_offset_drone_body_m**2 + y_offset_drone_body_m**2 + z_offset_drone_body_m**2)

                if distance_to_target_m < 0.05:
                    distance_to_target_m = 0.05

                now = time.time()
                if alt is not None and alt < 1.0 and last_distance_to_target is not None:
                    if now - last_distance_time <= 0.2 and distance_to_target_m - last_distance_to_target > 0.05:
                        wind_boost_until = now + 1.0
                        print(f"Wind spike: dist jump {distance_to_target_m - last_distance_to_target:.2f}m; boost 1.3 for 1s")

                last_distance_to_target = distance_to_target_m
                last_distance_time = now

                gain_x = ANGLE_GAIN_X
                gain_y = ANGLE_GAIN_Y
                if now < wind_boost_until:
                    gain_x = 0.7
                    gain_y = 0.7

                angle_x_rad = np.arctan2(y_offset_drone_body_m, z_offset_drone_body_m) * gain_x
                angle_y_rad = np.arctan2(x_offset_drone_body_m, z_offset_drone_body_m) * gain_y

                timestamp_us = int(time.time() * 1e6)

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

                x_offset_cm = np.tan(angle_x_rad) * z_offset_drone_body_m * 100
                y_offset_cm = np.tan(angle_y_rad) * z_offset_drone_body_m * 100
                print(f"Sent LANDING_TARGET: X({x_offset_cm:.1f}cm), Y({y_offset_cm:.1f}cm), dist={distance_to_target_m:.2f}m. Band: {display_band_name}")
            else:
                print("Markers detected, but no valid positions estimated (e.g., filtered by altitude band).")
        else:
            print("No active markers detected for current altitude band.")
    else:
        pass

    time.sleep(0.01)
