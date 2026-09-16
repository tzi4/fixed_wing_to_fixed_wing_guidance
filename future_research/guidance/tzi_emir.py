#!/usr/bin/env python3
"""
tzi_emir.py: combined guidance-controller implementation
Prepared by: Claude Opus 4.6 (Thinking)
Date: 2026-06-22

Architecture: tzi.py, with System, tziGuidance and TestCommander threads.
Heading: initial_camp_emir.py, proportional outer loop and heading-rate PID.
Altitude: initial_camp_emir.py, PD outer loop and altitude-rate PID.
Airspeed: initial_camp_emir.py coverage PID and tzi_final.py slew-rate limit.
Error: tzi_final.py angular errors in degrees, independent of camera scale.
Stabilization: tzi.py virtual gimbal using R_c_b.
Logging: goat_gimbal.py CSV and tzi.py text logs.
Failsafe: initial_camp_emir.py and tzi_final.py coasting and failsafe behavior.
"""

import numpy as np
import math
import redis
import datetime
import os
import atexit
import csv
from mavlinkHandler import MAVLinkHandlerPymavlink as MAVLinkHandler
import json
import ast
import time
import threading
import logging
import shutil
from pymavlink import mavutil

# ═════════════════════════ ══════════════════════════ CONSTANTS ═════════════════════════ ══════════════════════════
RESOLUTION_W = 1280
RESOLUTION_H = 720
MAVLINK_CONNECTION = 'udp:127.0.0.1:14553'  # Same as flight link goat_gimbal
TELEMETRY_HZ = 10                           # Same telemetry speed as goat_gimbal
TELEMETRY_TIMEOUT_S = 2.0
COMMAND_RATE_HZ = 5                         # GUIDED targets persist in ArduPilot
MAV_CMD_GUIDED_CHANGE_SPEED = 43000
LOCK_REQUIRED_S = 4.0
LOCK_SAMPLE_GAP_S = 0.25
# COMMAND_RATE_HZ = 10 # goat_gimbal shipping tempo

# Simulation camera: ~/catkin_ws/src/iq_sim/models/emir-gazebo-plane/model.sdf 1280x720, horizontal_fov=0.27 rad -> fx=fy=4711.9057 px.
CAMERA_HFOV_RAD = 0.27
CAMERA_FX = (RESOLUTION_W / 2.0) / math.tan(CAMERA_HFOV_RAD / 2.0)
CAMERA_FY = CAMERA_FX            # frame pixel assumption

# REAL CAMERA PROFILE
#These values were used in flight with goat_gimbal.py. Uncomment the
#following three assignments for that installation, overriding the
#simulation calibration above. Both profiles use 1280x720 resolution.
#  CAMERA_HFOV_RAD = 0.415
#  CAMERA_FX = 3045.737
#  CAMERA_FY = 3045.565

# Use these flags instead of comment line/override during sequential tuning.  Example, airspeed only: False / False / True.
SEND_HEADING_COMMANDS = True
SEND_ALTITUDE_COMMANDS = True
SEND_AIRSPEED_COMMANDS = True

# MAV_CMD_GUIDED_CHANGE_HEADING param1:
# 0 = course over ground, 1 = raw vehicle heading.
HEADING_TYPE = 1
# HEADING_TYPE = 0 # goat_gimbal approach (course over ground)

# Optional fixed commands for sequential tuning / goat-parity testing.  In the first real hardware iteration, simply turning on airspeed and using 20.0 reproduces test condition goat_gimbal.
FIXED_HEADING_DEG = None
FIXED_ALTITUDE_M = None
FIXED_ALTITUDE_RATE_MPS = None
FIXED_AIRSPEED_MS = None
# FIXED_ALTITUDE_RATE_MPS = 0.8 # goat_gimbal altitude rate FIXED_AIRSPEED_MS = 20.0 # goat_gimbal flight airspeed

# guidance limits
MIN_ALTITUDE = 10
MAX_ALTITUDE = 200

# ─── Camera → Body rotation (camera: z forward, x right, y down) ───
R_c_b = np.array([[0, 0, 1],
                  [1, 0, 0],
                  [0, 1, 0]], dtype=float)
R_c_b_T = R_c_b.T


def compute_R_b_e(roll, pitch, yaw):
    """Body-to-Earth rotation matrix using ZYX Euler angles in radians."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp,   cp*sr,            cp*cr           ]
    ])


class TargetLockTracker:
    """It monitors the competition geometry with uninterrupted duration."""

    def __init__(self, image_w=RESOLUTION_W, image_h=RESOLUTION_H,
                 required_s=LOCK_REQUIRED_S, max_sample_gap_s=LOCK_SAMPLE_GAP_S):
        self.image_w = float(image_w)
        self.image_h = float(image_h)
        self.required_s = float(required_s)
        self.max_sample_gap_s = float(max_sample_gap_s)
        self.started_at = None
        self.last_qualifying_at = None
        self.progress_s = 0.0
        self.acquired = False

    def qualifies(self, bbox):
        if bbox is None:
            return False
        x, y, w, h = (float(value) for value in bbox[:4])
        inside = (
            x >= self.image_w * 0.25 and
            x + w <= self.image_w * 0.75 and
            y >= self.image_h * 0.10 and
            y + h <= self.image_h * 0.90
        )
        large_enough = (
            w >= self.image_w * 0.05 or
            h >= self.image_h * 0.05
        )
        return inside and large_enough

    def reset(self):
        if not self.acquired:
            self.started_at = None
            self.last_qualifying_at = None
            self.progress_s = 0.0

    def update(self, bbox, now=None, active=True):
        now = time.time() if now is None else float(now)
        if self.acquired:
            return True
        if not active:
            self.reset()
            return False

        if bbox is None:
            if (self.last_qualifying_at is not None and
                    now - self.last_qualifying_at > self.max_sample_gap_s):
                self.reset()
            return False

        if not self.qualifies(bbox):
            self.reset()
            return False

        if (self.last_qualifying_at is None or
                now - self.last_qualifying_at > self.max_sample_gap_s):
            self.started_at = now
        self.last_qualifying_at = now
        self.progress_s = max(0.0, now - self.started_at)
        if self.progress_s >= self.required_s:
            self.progress_s = self.required_s
            self.acquired = True
        return self.acquired


# ═══════════════════════════════════════════════════
# CSV FLIGHT LOGGER  (goat_gimbal.py'den)
# ═══════════════════════════════════════════════════
class FlightLogger:
    """Saves flight data to CSV. Indispensable for offline analysis."""

    def __init__(self, file_prefix="tzi_emir_log"):
        ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        self.log_filename = f"{file_prefix}_{ts}.csv"
        self.log_file = open(self.log_filename, mode='w', newline='')
        self.csv_writer = csv.writer(self.log_file)
        self.row_count = 0
        self.flush_interval = 10

        headers = [
            "timestamp", "elapsed_s", "dt", "system_state",
            "flight_mode", "task", "target_found", "command_sent",
            "ack_heading", "ack_altitude", "ack_airspeed",
            # BBox
            "bbox_x", "bbox_y", "bbox_w", "bbox_h",
            "coverage_pct", "filtered_coverage_pct",
            # Stabilized pixel
            "stab_x", "stab_y",
            # Angular errors (degrees)
            "control_error_x_deg", "control_error_y_deg",
            # Attitude
            "roll_deg", "pitch_deg", "yaw_deg", "current_alt_m",
            "yaw_rate_dps", "climb_rate_mps",
            # Heading control
            "p_input_hdg", "p_term_hdg", "cmd_heading_delta_deg",
            "target_heading_deg", "heading_rate_dps",
            # Altitude control
            "p_input_alt", "p_term_alt", "d_term_alt",
            "cmd_alt_delta_m", "target_alt_m", "alt_rate_mps",
            # Speed control
            "speed_error_pct", "p_term_speed", "i_term_speed",
            "cmd_speed_ms", "measured_airspeed_ms",
        ]
        self.csv_writer.writerow(headers)
        self.log_file.flush()

    def log(self, row_data):
        self.csv_writer.writerow(row_data)
        self.row_count += 1
        if self.row_count % self.flush_interval == 0:
            self.log_file.flush()

    def close(self):
        if not self.log_file.closed:
            self.log_file.flush()
            self.log_file.close()


# ═════════════════════════ ══════════════════════════ DIRECT CONTROL COMMANDER THREAD (tzi.py architecture, extended) ═════════════════════════ ══════════════════════════
class TestCommander(threading.Thread):
    """
    Send commands in a periodic thread independent of image processing.
The command frequency is configurable.
  Heading: MAV_CMD_GUIDED_CHANGE_HEADING (43002).
  Altitude: MAV_CMD_GUIDED_CHANGE_ALTITUDE (43001).
  Airspeed: MAV_CMD_GUIDED_CHANGE_SPEED (43000).
    """

    def __init__(self, mav_handler, mavlink_io_lock, rate_hz=5,
                 send_heading=True, send_altitude=True, send_airspeed=True,
                 heading_type=1):
        super().__init__(daemon=True)
        self.mav_handler = mav_handler
        self.master = mav_handler.master
        self.mavlink_io_lock = mavlink_io_lock
        self.rate = rate_hz
        self.running = True
        self.lock = threading.Lock()
        self.send_heading = bool(send_heading)
        self.send_altitude = bool(send_altitude)
        self.send_airspeed = bool(send_airspeed)
        self.heading_type = int(heading_type)
        if self.heading_type not in (0, 1):
            raise ValueError("heading_type can only be 0 (MULTIPLE) or 1 (raw heading)")

        # Here is the message VFR_HUD (to read airspeed)
        with self.mavlink_io_lock:
            self.mav_handler.request_message_interval(
                [mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD], TELEMETRY_HZ
            )

        # Target values ​​(accessed via thread-safe, lock)
        self.target_heading_deg = 0.0
        self.target_heading_rate_dps = 3.0   # ← NEW: dynamic heading rate
        self.target_alt_m = 100.0
        self.target_alt_rate_mps = 0.8       # ← NEW: dynamic altitudinal rate
        self.target_airspeed_ms = 17.0
        self.active = False

        # Latest telemetry in the cache
        self.last_att = None       # (pitch_deg, roll_deg, yaw_deg)
        self.last_loc = None       # (lat, lon, alt_m)
        self.last_airspeed = None  # m/s
        self.last_mode = None      # flight mode string

    def update(self, heading_deg, heading_rate_dps, alt_m, alt_rate_mps, airspeed_ms):
        """Thread-safe target update. 5 parameters."""
        with self.lock:
            self.target_heading_deg = float(heading_deg)
            self.target_heading_rate_dps = float(heading_rate_dps)
            self.target_alt_m = float(alt_m)
            self.target_alt_rate_mps = float(alt_rate_mps)
            self.target_airspeed_ms = float(airspeed_ms)
            self.active = True

    def deactivate(self):
        """Stop command dispatch in a thread-safe manner."""
        with self.lock:
            self.active = False

    def is_active(self):
        with self.lock:
            return self.active

    def _send_heading(self, heading_deg, heading_rate_dps, airspeed_ms):
        """Send MAV_CMD_GUIDED_CHANGE_HEADING (43002).

param1 selects course over ground (0) or raw vehicle heading (1).
param3 is maximum centripetal acceleration in m/s squared.
Guidance produces an interpretable heading rate in deg/s. Convert it
into ArduPlane's required acceleration using a = V * omega.
        """
        heading_accel_mss = max(
            abs(float(airspeed_ms)) * math.radians(abs(float(heading_rate_dps))),
            0.05,  # ArduPlane also applies the same lower limit.
        )
        with self.mavlink_io_lock:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                43002,              # MAV_CMD_GUIDED_CHANGE_HEADING
                0,                  # confirmation
                self.heading_type,  # param1: 0 = COG, 1 = raw vehicle heading
                heading_deg,        # param2: target heading (degree)
                heading_accel_mss,  # param3: maximum centripetal acceleration (m/s²)
                0, 0, 0, 0
            )

    def _send_altitude(self, alt_m, alt_rate_mps):
        """MAV_CMD_GUIDED_CHANGE_ALTITUDE (43001).         param3: climb rate (m/s) — dynamic (initial_camp_emir approach).         param7: desired altitude (relative).
        """
        with self.mavlink_io_lock:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                43001,          # MAV_CMD_GUIDED_CHANGE_ALTITUDE
                0,              # confirmation
                0,              # param1: empty
                0,              # param2: empty
                alt_rate_mps,   # param3: climb rate (m/s)
                0, 0, 0,        # param4-6
                alt_m           # param7: desired altitude (relative)
            )

    def _send_airspeed(self, airspeed_ms):
        """ArduPlane sends the speed command of GUIDED."""
        if self.last_mode != 'GUIDED':
            return
        with self.mavlink_io_lock:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                MAV_CMD_GUIDED_CHANGE_SPEED,
                0,              # confirmation
                0,              # param1: ArduPlane only supports airspeed
                airspeed_ms,    # param2: target airspeed (m/s)
                1.0,            # param3: maximum acceleration (m/s²)
                0, 0, 0, 0
            )

    def _read_telemetry(self):
        """Read telemetry from the Pymavlink cache without blocking."""
        att_msg = self.master.messages.get('ATTITUDE', None)
        if att_msg:
            self.last_att = (
                math.degrees(att_msg.pitch),
                math.degrees(att_msg.roll),
                math.degrees(att_msg.yaw)
            )

        loc_msg = self.master.messages.get('GLOBAL_POSITION_INT', None)
        if loc_msg:
            self.last_loc = (
                loc_msg.lat / 1e7,
                loc_msg.lon / 1e7,
                loc_msg.relative_alt / 1000.0  # ← RELATIVE ALT (order approach)
            )

        vfr = self.master.messages.get('VFR_HUD', None)
        if vfr:
            self.last_airspeed = vfr.airspeed

        hb = self.master.messages.get('HEARTBEAT', None)
        if hb:
            self.last_mode = mavutil.mode_string_v10(hb)

    def run(self):
        period = 1.0 / self.rate
        while self.running:
            with self.lock:
                active = self.active

            if active:
                self._read_telemetry()

                with self.lock:
                    hdg = self.target_heading_deg
                    hdg_rate = self.target_heading_rate_dps
                    alt = self.target_alt_m
                    alt_rate = self.target_alt_rate_mps
                    spd = self.target_airspeed_ms

                # Stop sending old GUIDED targets after mode change.
                if self.last_mode == 'GUIDED':
                    if self.send_heading:
                        self._send_heading(hdg, hdg_rate, spd)
                    if self.send_altitude:
                        self._send_altitude(alt, alt_rate)
                    if self.send_airspeed:
                        self._send_airspeed(spd)

            time.sleep(period)

    def stop(self):
        self.running = False


# ═════════════════════════ ══════════════════════════ SYSTEM BASE CLASS (tzi.py architecture) ═════════════════════════ ══════════════════════════
class System:
    """MAVLink, Redis, Logger infrastructure."""

    def __init__(self):
        self.init_logger()
        self._running = threading.Event()
        self._running.set()
        self._close_lock = threading.Lock()
        self._closed = False

        # Redis
        self.r = redis.Redis(host='localhost', port=6379, db=0)
        self.logger.debug('Redis connection established.')
        self.p = self.r.pubsub()
        self.p.subscribe('tracker_bbox')

        # Camera
        self.W = RESOLUTION_W
        self.H = RESOLUTION_H
        self.center_x = self.W / 2.0
        self.center_y = self.H / 2.0
        self.fx = CAMERA_FX
        self.fy = CAMERA_FY

        self.K = np.array([
            [self.fx, 0,       self.center_x],
            [0,       self.fy, self.center_y],
            [0,       0,       1            ]
        ])
        self.K_inv = np.linalg.inv(self.K)
        self.logger.debug(f"Camera: {self.W}x{self.H}, fx={self.fx:.1f}, fy={self.fy:.1f}")

        # guidance limits
        self.MIN_ALTITUDE = MIN_ALTITUDE
        self.MAX_ALTITUDE = MAX_ALTITUDE

        # MAVLink connection
        self.mavlink_handler = MAVLinkHandler(
            MAVLINK_CONNECTION, message_hz=TELEMETRY_HZ)
        self.logger.debug('Connected to the aircraft.')
        self.r.set('guid', 'False')
        self.r.set('guid_lock', 'False')
        self.r.set('guid_lock_progress', '0.000')

        # Like goat_gimbal, recv/send accesses the same pymavlink object with a shared lock.
        self.mavlink_io_lock = threading.Lock()

        # Last COMMAND_ACK result: command_id -> (MAV_RESULT, receive_time).
        self.command_ack_lock = threading.Lock()
        self.command_acks = {}
        self.telemetry_time_lock = threading.Lock()
        self.telemetry_last_seen = {}

        # MAVLink reader thread (keeps cache updated)
        self._mavlink_reader_thread = threading.Thread(
            target=self._mavlink_reader, daemon=True, name="MAVLinkReader"
        )
        self._mavlink_reader_thread.start()
        self.logger.debug('MAVLink reader thread started.')

        atexit.register(self.exit_handler)

    def _mavlink_reader(self):
        """It keeps the pymavlink cache updated by constantly making recv_match."""
        while self._running.is_set():
            try:
                with self.mavlink_io_lock:
                    msg = self.mavlink_handler.master.recv_match(blocking=False)
                if msg is not None:
                    now = time.time()
                    msg_type = msg.get_type()
                    with self.telemetry_time_lock:
                        self.telemetry_last_seen[msg_type] = now
                    if msg_type == 'COMMAND_ACK':
                        with self.command_ack_lock:
                            self.command_acks[int(msg.command)] = (int(msg.result), now)
                time.sleep(0.01)
            except Exception:
                if not self._running.is_set():
                    break
                self.logger.exception("MAVLink reader error")
                time.sleep(0.1)

    def telemetry_is_fresh(self, message_types, max_age_s=TELEMETRY_TIMEOUT_S):
        """Verify that required telemetry messages have been received recently."""
        now = time.time()
        with self.telemetry_time_lock:
            return all(
                msg_type in self.telemetry_last_seen and
                now - self.telemetry_last_seen[msg_type] <= max_age_s
                for msg_type in message_types
            )

    def get_command_ack(self, command_id):
        """Return the final COMMAND_ACK result with the readable name MAV_RESULT."""
        with self.command_ack_lock:
            ack = self.command_acks.get(int(command_id))
        if ack is None:
            return "NO_ACK"

        result, _ = ack
        result_info = mavutil.mavlink.enums['MAV_RESULT'].get(result)
        return result_info.name if result_info is not None else str(result)

    def exit_handler(self):
        with self._close_lock:
            if self._closed:
                return
            self._closed = True
            self._running.clear()
            try:
                self.r.set('guid', 'False')
            except Exception:
                pass
            cmd_thread = getattr(self, 'cmd_thread', None)
            if cmd_thread is not None:
                cmd_thread.stop()
            try:
                self.p.close()
            except Exception:
                pass
            try:
                self.mavlink_handler.master.close()
            except Exception:
                pass
            csv_logger = getattr(self, 'csv_logger', None)
            if csv_logger is not None:
                csv_logger.close()
            self.logger.debug('Connection closed.')

    def init_logger(self):
        self.logger = logging.Logger('tzi_emir')
        self.logger.setLevel(logging.DEBUG)

        c_handler = logging.StreamHandler()
        log_file_path = 'tzi_emir_guidance.log'
        old_logs_dir = "Logs"

        if not os.path.exists(old_logs_dir):
            os.makedirs(old_logs_dir)

        if os.path.exists(log_file_path):
            ts = datetime.datetime.now()
            new_name = f"tzi_emir_guidance_{ts}.log"
            shutil.move(log_file_path, os.path.join(old_logs_dir, new_name))

        f_handler = logging.FileHandler(log_file_path)
        c_handler.setLevel(logging.DEBUG)
        f_handler.setLevel(logging.DEBUG)

        c_format = logging.Formatter('%(message)s')
        f_format = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
        c_handler.setFormatter(c_format)
        f_handler.setFormatter(f_format)

        self.logger.addHandler(c_handler)
        self.logger.addHandler(f_handler)


# ═════════════════════════ ══════════════════════════ FUSION GUIDANCE CLASS ═════════════════════════ ══════════════════════════
class tziGuidance(System):
    """
    Combined heading, altitude and airspeed controller.

Heading: proportional control and rate PID from initial_camp_emir,
using tzi_final angular errors.
Altitude: PD control and rate PID from initial_camp_emir,
using tzi_final angular errors.
Airspeed: coverage-based PID from initial_camp_emir and slew-rate limiting
from tzi_final.
    """

    def __init__(self):
        super().__init__()

        self.last_message_time = None
        self.last_telemetry_warning_time = 0.0
        self.last_control_debug_time = 0.0

        # Thread-safe bbox storage
        self.latest_bbox = None
        self.latest_bbox_time = None
        self.bbox_lock = threading.Lock()

        # CSV logger  (goat_gimbal'den)
        self.csv_logger = FlightLogger()
        self.start_time = time.time()

        # ═════════════════════ ══════════════════════ HEADING CONTROL (initial_camp_emir + tzi_final) ═════════════════════ ══════════════════════ Main P controller (angular error → heading delta)
        self.Kp_heading = 3.0       # 1° error → 3° heading change
        self.Ki_heading = 0.0
        self.Kd_heading = 0.0

        # Heading Rate PID (angular error → rotation rate)
        self.Kp_rate = 1.2          # 1° error → 1.2 deg/s rotation speed
        self.Ki_rate = 0.0
        self.Kd_rate = 0.12
        # goat_gimbal actual hardware reference: Kp_rate=0.6, Kd_rate=0.12

        self.integral_error_rate = 0.0
        self.prev_error_rate = 0.0
        self.prev_derivative_rate = 0.0

        self.min_heading_rate = 0.85    # Minimum rotation speed (deg/s)
        self.max_heading_rate = 7.5     # Maximum rotation speed (deg/s)
        # goat_gimbal actual hardware reference: max_heading_rate=5.0
        self.max_heading_change_deg = 35.0   # Max heading delta (degrees)

        # ═════════════════════ ══════════════════════ ALTITUDE CONTROL (initial_camp_emir + tzi_final) ═════════════════════ ══════════════════════
        self.Kp_alt = 1.5
        self.Ki_alt = 0.0
        self.Kd_alt = 0.1
        # goat_gimbal actual hardware reference: Kp_alt=0.7, Kd_alt=0.12

        # Altitude Rate PID (angular error → climb/descent rate)
        self.Kp_alt_rate = 1.5
        self.Ki_alt_rate = 0.0
        self.Kd_alt_rate = 0.1

        self.integral_error_alt_rate = 0.0
        self.prev_error_alt_rate = 0.0
        self.prev_derivative_alt_rate = 0.0

        self.min_alt_rate = 0.3       # Minimum climb speed (m/s)
        self.max_alt_rate = 5.0       # Maximum climbing speed (m/s)
        self.max_alt_change_m = 10.0  # Max altitude delta (metre)

        # ═════════════════════ ══════════════════════ AIRSPEED CONTROL (initial_camp_emir coverage PID + tzi_final slew) ═════════════════════ ══════════════════════
        self.Kp_speed = 0.8
        self.Ki_speed = 0.08
        self.Kd_speed = 0.0

        self.integral_error_speed = 0.0
        self.prev_error_speed = 0.0
        self.prev_derivative_speed = 0.0

        self.min_speed = 10.0         # SITL with safety margin for AIRSPEED_MIN=9
        self.max_speed = 22.0
        self.base_speed = 18.0        # Nominal speed of target aircraft
        self.target_coverage_pct = 6.0    # Following distance to be maintained (%)
        self.speed_integral_band = 7.0    # Accumulate i only within this band
        self.speed_slew_rate = 2.0        # Max speed change (m/s/s)
        self.coverage_alpha = 0.35        # EMF coefficient
        self.filtered_coverage = self.target_coverage_pct
        self.last_speed = self.base_speed

        # ═════════════════════ ══════════════════════ COMMON PID CONDITIONS ═════════════════════ ══════════════════════ Deadzone (tzi_final + order common)
        self.deadzone_deg = 0.78

        # Low-pass derivative filter (goat_gimbal'den)
        self.deriv_alpha = 0.05
        self.prev_error_x = 0.0
        self.prev_error_y = 0.0
        self.prev_derivative_x = 0.0
        self.prev_derivative_y = 0.0

        # Heading/Altitude integral
        self.integral_error_x = 0.0
        self.integral_error_y = 0.0

        self.last_time = time.time()
        self.last_pid_time = None

        # Coasting/Failsafe tracking
        self.last_target_time = 0.0
        self.last_target_heading = 0.0
        self.last_target_alt = 100.0
        self.last_heading_rate = self.min_heading_rate
        self.last_alt_rate = self.min_alt_rate
        self.coasting_timeout = 5.0  # seconds
        self.hold_active = False

        # Official lock condition: 4 s within target area and of sufficient size.
        self.lock_tracker = TargetLockTracker(self.W, self.H)
        self._last_lock_log_second = -1

        # GUIDED entry initialization (tzi_final'den)
        self.guided_entry_done = False

        # Initialize TestCommander (tzi architecture)
        self.cmd_thread = TestCommander(
            self.mavlink_handler,
            self.mavlink_io_lock,
            rate_hz=COMMAND_RATE_HZ,
            send_heading=SEND_HEADING_COMMANDS,
            send_altitude=SEND_ALTITUDE_COMMANDS,
            send_airspeed=SEND_AIRSPEED_COMMANDS,
            heading_type=HEADING_TYPE,
        )
        self.cmd_thread.start()
        self.logger.debug('TestCommander thread started (inactive until first bbox).')

    # ─── Auxiliary functions ───

    def clamp(self, value, min_val, max_val):
        return max(min(value, max_val), min_val)

    def _publish_guidance_state(self, active):
        self.r.set('guid', 'True' if active else 'False')

    def _update_lock_state(self, bbox, now, active):
        was_acquired = self.lock_tracker.acquired
        acquired = self.lock_tracker.update(bbox, now=now, active=active)
        self.r.set('guid_lock', 'True' if acquired else 'False')
        self.r.set('guid_lock_progress', f"{self.lock_tracker.progress_s:.3f}")
        progress_second = int(self.lock_tracker.progress_s)
        if progress_second != self._last_lock_log_second and not acquired:
            self._last_lock_log_second = progress_second
            if self.lock_tracker.progress_s > 0.0:
                self.logger.info(
                    f"Lock advance: {self.lock_tracker.progress_s:.1f}/"
                    f"{self.lock_tracker.required_s:.1f} s")
        if acquired and not was_acquired:
            self.logger.warning("🔒 LOCKED: official condition met for 4 seconds without interruption.")
        return acquired

    def stabilize_pixel(self, obj_x, obj_y):
        """Virtual gimbal stabilization from tzi.py."""
        p_raw = np.array([obj_x, obj_y, 1.0])
        r_cam = self.K_inv @ p_raw
        r_body = R_c_b @ r_cam

        att_msg = self.mavlink_handler.master.messages.get('ATTITUDE', None)
        if att_msg:
            roll_rad = att_msg.roll
            pitch_rad = att_msg.pitch
        else:
            roll_rad, pitch_rad = 0.0, 0.0

        R_stab = compute_R_b_e(roll_rad, pitch_rad, 0)
        r_virt_body = R_stab @ r_body
        r_virt_cam = R_c_b_T @ r_virt_body
        p_virt_hom = self.K @ r_virt_cam

        # Guard against rays behind the virtual camera or near a projection
        #singularity. Division there can produce very large or nonfinite pixel
        #values. In that rare case, return the raw pixel to the controller
        #instead of allowing an overflowing command.
        if (np.all(np.isfinite(p_virt_hom)) and
                p_virt_hom[2] > 1e-3):
            stab_x = p_virt_hom[0] / p_virt_hom[2]
            stab_y = p_virt_hom[1] / p_virt_hom[2]
            if math.isfinite(stab_x) and math.isfinite(stab_y):
                return stab_x, stab_y
        return obj_x, obj_y

    def coverage_metric(self, bbox_w, bbox_h):
        """Max percent coverage horizontally or vertically (from initial_camp_emir)."""
        h_cov = (bbox_w / self.W) * 100.0
        v_cov = (bbox_h / self.H) * 100.0
        return max(h_cov, v_cov), h_cov, v_cov

    def get_current_state(self):
        """Read current status from telemetry cache."""
        att_msg = self.mavlink_handler.master.messages.get('ATTITUDE', None)
        if att_msg:
            roll_rad = att_msg.roll
            pitch_rad = att_msg.pitch
            yaw_rad = att_msg.yaw
        else:
            roll_rad, pitch_rad, yaw_rad = 0.0, 0.0, 0.0

        loc_msg = self.mavlink_handler.master.messages.get('GLOBAL_POSITION_INT', None)
        if loc_msg:
            current_alt = loc_msg.relative_alt / 1000.0  # Relative alt (order approach)
        else:
            current_alt = self.last_target_alt

        hb = self.mavlink_handler.master.messages.get('HEARTBEAT', None)
        current_mode = mavutil.mode_string_v10(hb) if hb else "UNKNOWN"

        yaw_deg = math.degrees(yaw_rad) % 360.0

        return roll_rad, pitch_rad, yaw_rad, yaw_deg, current_alt, current_mode

    def command_current_hold(self, airspeed_ms=None):
        """Goat failsafe behavior: make current heading/altitude new target GUIDED."""
        _, _, _, yaw_deg, current_alt, current_mode = self.get_current_state()
        if current_mode != 'GUIDED' or not self.telemetry_is_fresh(
                ('HEARTBEAT', 'ATTITUDE', 'GLOBAL_POSITION_INT')):
            self.hold_active = False
            self.cmd_thread.deactivate()
            return False

        # Sample the hold target only at the moment of transition. Resampling with each bbox message would drag the target of the turning/climbing plane and would not create a true hold.
        if self.hold_active and self.cmd_thread.is_active():
            return True

        hold_heading = yaw_deg
        if HEADING_TYPE == 0:
            loc_msg = self.mavlink_handler.master.messages.get('GLOBAL_POSITION_INT', None)
            if loc_msg is not None and math.hypot(loc_msg.vx, loc_msg.vy) > 50.0:
                hold_heading = math.degrees(math.atan2(loc_msg.vy, loc_msg.vx)) % 360.0

        if airspeed_ms is None:
            airspeed_ms = self.last_speed
        hold_speed = self.clamp(float(airspeed_ms), self.min_speed, self.max_speed)

        self.cmd_thread.update(
            hold_heading, self.min_heading_rate,
            current_alt, self.min_alt_rate,
            hold_speed,
        )
        self.last_target_heading = hold_heading
        self.last_target_alt = current_alt
        self.last_heading_rate = self.min_heading_rate
        self.last_alt_rate = self.min_alt_rate
        self.hold_active = True
        return True

    # ─── MAIN GUIDANCE FUNCTION ───

    def guide_aircraft(self, bbox, current_time):
        """
        BBox receives and calculates control commands in 3 axes (heading, altitude, airspeed).         bbox: (x, y, w, h) — top left corner + width/height
        """
        # Bounding-box center
        obj_x = bbox[0] + (bbox[2] / 2.0)
        obj_y = bbox[1] + (bbox[3] / 2.0)
        bbox_w, bbox_h = float(bbox[2]), float(bbox[3])

        # Stabilization (virtual gimbal)
        stab_x, stab_y = self.stabilize_pixel(obj_x, obj_y)

        # Angular error (degrees) — camera independent (tzi_final approach)
        stab_error_x_deg = math.degrees(math.atan((stab_x - self.center_x) / self.fx))
        stab_error_y_deg = math.degrees(math.atan((stab_y - self.center_y) / self.fy))
        # The lock area is defined in the raw camera frame. Because the virtual gimbal error cancels the pitch/roll movement of the aircraft, it holds the target inertially, but may drag it out of the lock area on the screen. The control therefore uses the raw optical error; The stabilized value is diagnostic only.
        control_error_x_deg = math.degrees(
            math.atan((obj_x - self.center_x) / self.fx))
        control_error_y_deg = math.degrees(
            math.atan((obj_y - self.center_y) / self.fy))

        # Telemetry
        roll_rad, pitch_rad, yaw_rad, yaw_deg, current_alt, current_mode = \
            self.get_current_state()
        att_msg = self.mavlink_handler.master.messages.get('ATTITUDE', None)
        vfr_msg = self.mavlink_handler.master.messages.get('VFR_HUD', None)
        yaw_rate_dps = math.degrees(att_msg.yawspeed) if att_msg else 0.0
        climb_rate_mps = vfr_msg.climb if vfr_msg else 0.0

        required_telemetry = ('HEARTBEAT', 'ATTITUDE', 'GLOBAL_POSITION_INT')
        if not self.telemetry_is_fresh(required_telemetry):
            self._reset_pid_states()
            self.guided_entry_done = False
            self.hold_active = False
            self.cmd_thread.deactivate()
            now = time.time()
            self._publish_guidance_state(False)
            self._update_lock_state(None, now, active=False)
            if now - self.last_telemetry_warning_time > 1.0:
                self.logger.warning("Telemetry stale or missing; GUIDED commands stopped.")
                self.last_telemetry_warning_time = now
            return

        # Coverage (for airspeed PID)
        coverage, h_cov, v_cov = self.coverage_metric(bbox_w, bbox_h)

        debug_this_sample = current_time - self.last_control_debug_time >= 0.2
        if debug_this_sample:
            self.last_control_debug_time = current_time
            self.logger.debug(
                f"Target: Cov={coverage:.1f}% | Raw: ({obj_x:.0f},{obj_y:.0f}) "
                f"CtlErr: ({control_error_x_deg:+.2f}°,{control_error_y_deg:+.2f}°) | "
                f"StabErr: ({stab_error_x_deg:+.2f}°,{stab_error_y_deg:+.2f}°) | "
                f"BBox: {bbox_w:.0f}x{bbox_h:.0f} | Alt: {current_alt:.1f}m | "
                f"Mode: {current_mode}"
            )

        # ─── If GUIDED is not in mode → reset state ───
        if current_mode != 'GUIDED':
            self._reset_pid_states()
            self.guided_entry_done = False
            self.hold_active = False
            self.cmd_thread.deactivate()
            self._publish_guidance_state(False)
            self._update_lock_state(None, current_time, active=False)
            self.logger.debug(f"PID skipped (mode={current_mode}), state reset.")
            return

        self._publish_guidance_state(True)
        self._update_lock_state(bbox, current_time, active=True)

        # ─── GUIDED ENTRY INITIALIZATION (tzi_final'den) ───
        first_guided_sample = not self.guided_entry_done
        if first_guided_sample:
            measured_as = self.cmd_thread.last_airspeed
            if measured_as is not None:
                init_speed = self.clamp(measured_as, self.min_speed, self.max_speed)
                self.last_speed = init_speed
                self.logger.debug(f"GUIDED Entry: Init speed={init_speed:.1f} m/s")
            # Generating an error derivative from scratch in the first frame and making the rate jump.
            self.prev_error_x = control_error_x_deg
            self.prev_error_y = control_error_y_deg
            self.prev_derivative_x = 0.0
            self.prev_derivative_y = 0.0
            self.last_time = current_time
            self.guided_entry_done = True

        # ─── dt account ───
        if first_guided_sample:
            dt = 1.0 / 30.0
        else:
            dt = max(0.001, min(current_time - self.last_time, 0.1))
        self.last_time = current_time

        # ─── Low-pass filtered derivative (goat_gimbal'den) ───
        raw_deriv_x = (control_error_x_deg - self.prev_error_x) / dt
        raw_deriv_y = (control_error_y_deg - self.prev_error_y) / dt
        deriv_x = (self.deriv_alpha * raw_deriv_x) + \
                  ((1.0 - self.deriv_alpha) * self.prev_derivative_x)
        deriv_y = (self.deriv_alpha * raw_deriv_y) + \
                  ((1.0 - self.deriv_alpha) * self.prev_derivative_y)

        self.prev_error_x = control_error_x_deg
        self.prev_error_y = control_error_y_deg
        self.prev_derivative_x = deriv_x
        self.prev_derivative_y = deriv_y

        # ─── Deadzone (tzi_final + emir shared_data) ───
        dz_x = abs(control_error_x_deg) < self.deadzone_deg
        dz_y = abs(control_error_y_deg) < self.deadzone_deg
        p_x = 0.0 if dz_x else control_error_x_deg - math.copysign(
            self.deadzone_deg, control_error_x_deg)
        p_y = 0.0 if dz_y else control_error_y_deg - math.copysign(
            self.deadzone_deg, control_error_y_deg)

        # ─── Integral accumulation (heading/altitude) ───
        if not dz_x:
            self.integral_error_x += control_error_x_deg * dt
        else:
            self.integral_error_x *= 0.99  # Decay in deadzone
        self.integral_error_x = self.clamp(self.integral_error_x, -10.0, 10.0)

        if not dz_y:
            self.integral_error_y += control_error_y_deg * dt
        else:
            self.integral_error_y *= 0.99
        self.integral_error_y = self.clamp(self.integral_error_y, -10.0, 10.0)

        # ═════════════════════ ══════════════════════
        # 1. HEADING CONTROL ═════════════════════ ══════════════════════
        p_term_heading = p_x * self.Kp_heading
        i_term_heading = self.integral_error_x * self.Ki_heading
        d_term_heading = deriv_x * self.Kd_heading

        cmd_head_deg = p_term_heading + i_term_heading + d_term_heading
        cmd_head_deg = self.clamp(cmd_head_deg, -self.max_heading_change_deg,
                                  self.max_heading_change_deg)
        target_heading = (yaw_deg + cmd_head_deg) % 360.0
        if FIXED_HEADING_DEG is not None:
            target_heading = float(FIXED_HEADING_DEG) % 360.0

        # ─── Heading Rate PID (from initial_camp_emir) ───
        rate_error = abs(p_x)

        raw_deriv_rate = 0.0 if first_guided_sample else \
            (rate_error - self.prev_error_rate) / dt
        deriv_rate = (self.deriv_alpha * raw_deriv_rate) + \
                     ((1.0 - self.deriv_alpha) * self.prev_derivative_rate)

        self.integral_error_rate += rate_error * dt
        self.integral_error_rate = self.clamp(self.integral_error_rate, 0, 5.0)

        p_term_rate = rate_error * self.Kp_rate
        i_term_rate = self.integral_error_rate * self.Ki_rate
        d_term_rate = deriv_rate * self.Kd_rate

        heading_rate = p_term_rate + i_term_rate + d_term_rate
        heading_rate = self.clamp(heading_rate, self.min_heading_rate, self.max_heading_rate)
        if dz_x:
            heading_rate = self.min_heading_rate

        self.prev_error_rate = rate_error
        self.prev_derivative_rate = deriv_rate

        # ═════════════════════ ══════════════════════
        # 2. ALTITUDE CONTROL ═════════════════════ ══════════════════════ If the target is below (p_y positive) → altitude should be reduced
        p_term_alt = p_y * self.Kp_alt
        i_term_alt = self.integral_error_y * self.Ki_alt
        d_term_alt = deriv_y * self.Kd_alt

        cmd_alt_m = -1.0 * (p_term_alt + i_term_alt + d_term_alt)
        cmd_alt_m = self.clamp(cmd_alt_m, -self.max_alt_change_m, self.max_alt_change_m)

        target_alt = current_alt + cmd_alt_m
        target_alt = max(self.MIN_ALTITUDE, min(self.MAX_ALTITUDE, target_alt))
        if FIXED_ALTITUDE_M is not None:
            target_alt = self.clamp(
                float(FIXED_ALTITUDE_M), self.MIN_ALTITUDE, self.MAX_ALTITUDE)

        # ─── Altitude Rate PID (from initial_camp_emir) ───
        alt_rate_error = abs(p_y)

        raw_deriv_alt_rate = 0.0 if first_guided_sample else \
            (alt_rate_error - self.prev_error_alt_rate) / dt
        deriv_alt_rate = (self.deriv_alpha * raw_deriv_alt_rate) + \
                         ((1.0 - self.deriv_alpha) * self.prev_derivative_alt_rate)

        self.integral_error_alt_rate += alt_rate_error * dt
        self.integral_error_alt_rate = self.clamp(self.integral_error_alt_rate, 0, 5.0)

        p_term_alt_rate = alt_rate_error * self.Kp_alt_rate
        i_term_alt_rate = self.integral_error_alt_rate * self.Ki_alt_rate
        d_term_alt_rate = deriv_alt_rate * self.Kd_alt_rate

        altitude_rate = p_term_alt_rate + i_term_alt_rate + d_term_alt_rate
        altitude_rate = self.clamp(altitude_rate, self.min_alt_rate, self.max_alt_rate)
        if dz_y:
            altitude_rate = self.min_alt_rate
        if FIXED_ALTITUDE_RATE_MPS is not None:
            altitude_rate = self.clamp(
                float(FIXED_ALTITUDE_RATE_MPS), self.min_alt_rate, self.max_alt_rate)

        self.prev_error_alt_rate = alt_rate_error
        self.prev_derivative_alt_rate = deriv_alt_rate

        # ═════════════════════ ══════════════════════
        # 3. AIRSPEED CONTROL (coverage-based PID + slew rate) ═════════════════════ ══════════════════════ EMA filtered coverage (from initial_camp_emir)
        self.filtered_coverage = (self.coverage_alpha * coverage) + \
                                 ((1.0 - self.coverage_alpha) * self.filtered_coverage)

        speed_error = self.target_coverage_pct - self.filtered_coverage

        # Derivative (low-pass)
        raw_deriv_speed = 0.0 if first_guided_sample else \
            (speed_error - self.prev_error_speed) / dt
        deriv_speed = (self.deriv_alpha * raw_deriv_speed) + \
                      ((1.0 - self.deriv_alpha) * self.prev_derivative_speed)

        # Integral (band-limited + conditional anti-windup)
        proposed_speed_integral = self.integral_error_speed
        if abs(speed_error) < self.speed_integral_band:
            proposed_speed_integral += speed_error * dt
        proposed_speed_integral = self.clamp(proposed_speed_integral, -50.0, 50.0)

        # PID output
        p_term_speed = speed_error * self.Kp_speed
        proposed_i_term = proposed_speed_integral * self.Ki_speed
        d_term_speed = deriv_speed * self.Kd_speed

        unconstrained_speed = self.base_speed + (
            p_term_speed + proposed_i_term + d_term_speed)
        at_upper_limit = unconstrained_speed > self.max_speed and speed_error > 0.0
        at_lower_limit = unconstrained_speed < self.min_speed and speed_error < 0.0
        if not (at_upper_limit or at_lower_limit):
            self.integral_error_speed = proposed_speed_integral
        i_term_speed = self.integral_error_speed * self.Ki_speed
        cmd_speed = self.base_speed + (p_term_speed + i_term_speed + d_term_speed)
        cmd_speed = self.clamp(cmd_speed, self.min_speed, self.max_speed)

        # Slew rate limit (from tzi_final, protects TECS from sudden speed changes)
        max_speed_delta = self.speed_slew_rate * dt
        cmd_speed = self.clamp(cmd_speed,
                               self.last_speed - max_speed_delta,
                               self.last_speed + max_speed_delta)
        if FIXED_AIRSPEED_MS is not None:
            cmd_speed = self.clamp(float(FIXED_AIRSPEED_MS), self.min_speed, self.max_speed)

        self.prev_error_speed = speed_error
        self.prev_derivative_speed = deriv_speed
        self.last_speed = cmd_speed

        # ═══════════════════════════════════════════
        # ANTI-WINDUP  (from initial_camp_emir)
        # ═══════════════════════════════════════════
        if abs(cmd_head_deg) >= self.max_heading_change_deg:
            self.integral_error_x *= 0.9
        if abs(cmd_alt_m) >= self.max_alt_change_m:
            self.integral_error_y *= 0.9

        # ═════════════════════ ══════════════════════ SEND COMMAND (to TestCommander) ═════════════════════ ══════════════════════
        self.hold_active = False
        self.cmd_thread.update(
            target_heading, heading_rate,
            target_alt, altitude_rate,
            cmd_speed
        )

        # Update tracking state (for coasting)
        self.last_target_time = current_time
        self.last_target_heading = target_heading
        self.last_target_alt = target_alt
        self.last_heading_rate = heading_rate
        self.last_alt_rate = altitude_rate

        # ═══════════════════════════════════════════
        # LOGGING
        # ═══════════════════════════════════════════
        measured_as = self.cmd_thread.last_airspeed
        measured_as_str = f"{measured_as:.1f}" if measured_as is not None else "?"
        ack_heading = self.get_command_ack(43002) if SEND_HEADING_COMMANDS else "DISABLED"
        ack_altitude = self.get_command_ack(43001) if SEND_ALTITUDE_COMMANDS else "DISABLED"
        ack_airspeed = self.get_command_ack(MAV_CMD_GUIDED_CHANGE_SPEED) \
            if SEND_AIRSPEED_COMMANDS else "DISABLED"

        if debug_this_sample:
            self.logger.debug(
                f"HDG: target={target_heading:.1f}° rate={heading_rate:.1f}°/s "
                f"delta={cmd_head_deg:+.1f}° | "
                f"ALT: target={target_alt:.1f}m rate={altitude_rate:.1f}m/s "
                f"delta={cmd_alt_m:+.1f}m | "
                f"SPD: cmd={cmd_speed:.1f}m/s meas={measured_as_str}m/s "
                f"cov={self.filtered_coverage:.1f}%"
            )
            self.logger.debug("-" * 60)

        # CSV log
        elapsed = current_time - self.start_time
        self.csv_logger.log([
            f"{current_time:.4f}", f"{elapsed:.3f}", f"{dt:.4f}", "TRACKING",
            current_mode, "Visual", 1, 1,
            ack_heading, ack_altitude, ack_airspeed,
            bbox[0], bbox[1], f"{bbox_w:.1f}", f"{bbox_h:.1f}",
            f"{coverage:.2f}", f"{self.filtered_coverage:.2f}",
            f"{stab_x:.2f}", f"{stab_y:.2f}",
            f"{control_error_x_deg:.4f}", f"{control_error_y_deg:.4f}",
            f"{math.degrees(roll_rad):.2f}", f"{math.degrees(pitch_rad):.2f}",
            f"{yaw_deg:.2f}", f"{current_alt:.2f}",
            f"{yaw_rate_dps:.3f}", f"{climb_rate_mps:.3f}",
            f"{p_x:.4f}", f"{p_term_heading:.4f}", f"{cmd_head_deg:.2f}",
            f"{target_heading:.2f}", f"{heading_rate:.2f}",
            f"{p_y:.4f}", f"{p_term_alt:.4f}", f"{d_term_alt:.4f}",
            f"{cmd_alt_m:.2f}", f"{target_alt:.2f}", f"{altitude_rate:.2f}",
            f"{speed_error:.4f}", f"{p_term_speed:.4f}", f"{i_term_speed:.4f}",
            f"{cmd_speed:.2f}", measured_as_str,
        ])

    # ─── PID State Reset ───

    def _reset_pid_states(self):
        """Reset all PID/PD states."""
        self.integral_error_x = 0.0
        self.integral_error_y = 0.0
        self.prev_error_x = 0.0
        self.prev_error_y = 0.0
        self.prev_derivative_x = 0.0
        self.prev_derivative_y = 0.0

        self.integral_error_rate = 0.0
        self.prev_error_rate = 0.0
        self.prev_derivative_rate = 0.0

        self.integral_error_alt_rate = 0.0
        self.prev_error_alt_rate = 0.0
        self.prev_derivative_alt_rate = 0.0

        self.integral_error_speed = 0.0
        self.prev_error_speed = 0.0
        self.prev_derivative_speed = 0.0

        self.filtered_coverage = self.target_coverage_pct
        self.last_speed = self.base_speed
        self.last_pid_time = None

    # ─── BBox Parsing ───

    def _parse_bbox(self, data):
        """Parses the message Redis. (x, y, w, h) turns into tube or N."""
        if not isinstance(data, (list, tuple)) or len(data) < 4:
            return None

        # If bbox_to_redis.py sends validity in the sixth field, follow that too.
        if len(data) >= 6 and not bool(data[5]):
            return None

        try:
            x, y, w, h = (float(value) for value in data[:4])
        except (TypeError, ValueError):
            return None

        if not all(math.isfinite(value) for value in (x, y, w, h)):
            return None
        if w <= 0.0 or h <= 0.0:
            return None

        # Crop the part outside the image; Completely reject the outside box.
        x1 = self.clamp(x, 0.0, float(self.W))
        y1 = self.clamp(y, 0.0, float(self.H))
        x2 = self.clamp(x + w, 0.0, float(self.W))
        y2 = self.clamp(y + h, 0.0, float(self.H))
        clipped_w = x2 - x1
        clipped_h = y2 - y1
        if clipped_w <= 0.0 or clipped_h <= 0.0:
            return None

        return (int(round(x1)), int(round(y1)),
                int(round(clipped_w)), int(round(clipped_h)))

    # ─── Threads ───

    def _redis_listener(self):
        """Thread 1: Redis listens to pub/sub, writes bbox to thread-safe storage."""
        self.logger.debug("Redis listener thread started.")
        try:
            messages = self.p.listen()
            for message in messages:
                if not self._running.is_set():
                    break
                if message['type'] != 'message':
                    continue

                # mission control
                guid_message = self.r.get('task')
                if guid_message is None or guid_message.decode('utf-8').lower() != 'visual':
                    if self.cmd_thread.is_active():
                        self.command_current_hold()
                    self._publish_guidance_state(False)
                    self._update_lock_state(None, time.time(), active=False)
                    with self.bbox_lock:
                        self.latest_bbox = None
                        self.latest_bbox_time = None
                    continue

                try:
                    raw_data = message['data'].decode('utf-8')
                except UnicodeDecodeError:
                    continue
                try:
                    data = json.loads(raw_data)
                except json.JSONDecodeError:
                    # Python records/tuple format used in flight with goat_gimbal.
                    try:
                        data = ast.literal_eval(raw_data)
                    except (ValueError, SyntaxError):
                        continue

                bbox = self._parse_bbox(data)
                if bbox is not None:
                    now = time.time()
                    with self.bbox_lock:
                        self.latest_bbox = bbox
                        self.latest_bbox_time = now
                        self.last_message_time = now
        except Exception:
            if self._running.is_set():
                self.logger.exception("Redis listener error")

    def _vision_processor(self):
        """Thread 2: Reads the latest bbox on 30 Hz and calls guide_aircraft().         Coasting and Failsafe management is done here."""
        self.logger.debug("Vision processor thread started.")
        period = 1.0 / 30.0
        last_processed_time = None
        last_warning_time = 0.0

        while self._running.is_set():
            time.sleep(period)

            with self.bbox_lock:
                bbox = self.latest_bbox
                bbox_time = self.latest_bbox_time

            # Work if new bbox has arrived
            if bbox is not None and bbox_time != last_processed_time:
                last_processed_time = bbox_time
                self.guide_aircraft(bbox, bbox_time)
            else:
                # ─── NO TARGET: Coasting or Failsafe ───
                now = time.time()
                _, _, _, _, _, current_mode = self.get_current_state()
                lock_active = (
                    current_mode == 'GUIDED' and
                    self.telemetry_is_fresh(
                        ('HEARTBEAT', 'ATTITUDE', 'GLOBAL_POSITION_INT'))
                )
                self._update_lock_state(None, now, active=lock_active)
                if self.last_target_time > 0.0:
                    time_since_last = now - self.last_target_time

                    if time_since_last <= self.coasting_timeout:
                        # COASTING: TestCommander continues sending the last targets.  No additional processing required (inherent coasting — tzi architecture advantage).
                        self._publish_guidance_state(lock_active)
                    else:
                        # FAILSAFE AFTER FIVE SECONDS WITHOUT THE TARGET
                        #ArduPilot retains GUIDED heading and altitude targets until replaced.
                        #Stopping the command thread alone would leave the last turn or climb
                        #active. Send a hold at the current heading and altitude once on loss.
                        if not self.hold_active:
                            hold_speed = self.last_speed
                            self._reset_pid_states()
                            self.guided_entry_done = False
                            self.command_current_hold(airspeed_ms=hold_speed)

                        self._publish_guidance_state(False)

                        holding = self.hold_active

                        if now - last_warning_time > 1.0:
                            if holding:
                                self.logger.warning(
                                    "\n[WARNING] TARGET MISSING FOR MORE THAN 5 SECONDS; "
                                    "CURRENT DIRECTION/altitude IS BEING HELD.")
                            else:
                                self.logger.warning(
                                    "\n[WARNING] TARGET LOST AND HOLD UNABLE TO ESTABLISH; "
                                    "CHANGE MISSION MODE!")
                            last_warning_time = now

                        # CSV log
                        elapsed = now - self.start_time
                        _, _, _, yaw_deg, current_alt, current_mode = \
                            self.get_current_state()
                        self.csv_logger.log([
                            f"{now:.4f}", f"{elapsed:.3f}", f"{time_since_last:.4f}",
                            "TARGET_LOST_HOLD" if holding else "FAILSAFE",
                            current_mode, "Visual", 0, 0,
                            self.get_command_ack(43002) if SEND_HEADING_COMMANDS else "DISABLED",
                            self.get_command_ack(43001) if SEND_ALTITUDE_COMMANDS else "DISABLED",
                            self.get_command_ack(MAV_CMD_GUIDED_CHANGE_SPEED)
                            if SEND_AIRSPEED_COMMANDS else "DISABLED",
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                            0, 0, f"{yaw_deg:.2f}", f"{current_alt:.2f}",
                            0, 0,
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0,
                        ])

    def run(self):
        """Master activator. Starts all threads."""
        listener_thread = threading.Thread(
            target=self._redis_listener, daemon=True, name="RedisListener"
        )
        listener_thread.start()

        processor_thread = threading.Thread(
            target=self._vision_processor, daemon=True, name="VisionProcessor"
        )
        processor_thread.start()

        self.logger.debug("═" * 60)
        self.logger.debug("tzi_emir.py — All threads started.")
        self.logger.debug(
            f"Camera: {self.W}x{self.H}, HFOV={CAMERA_HFOV_RAD:.3f} rad, "
            f"fx={self.fx:.2f}, fy={self.fy:.2f}")
        self.logger.debug(
            "Command axes: "
            f"heading={SEND_HEADING_COMMANDS}, "
            f"altitude={SEND_ALTITUDE_COMMANDS}, "
            f"airspeed={SEND_AIRSPEED_COMMANDS}")
        self.logger.debug(
            f"Heading type: {HEADING_TYPE} "
            f"({'raw heading' if HEADING_TYPE == 1 else 'course over ground'})")
        self.logger.debug(f"Heading: Kp={self.Kp_heading}, Rate PID: Kp={self.Kp_rate}")
        self.logger.debug(f"Altitude: Kp={self.Kp_alt}, Kd={self.Kd_alt}")
        self.logger.debug(f"Speed: coverage-based, base={self.base_speed}m/s, "
                          f"slew={self.speed_slew_rate}m/s/s")
        self.logger.debug("═" * 60)

        try:
            listener_thread.join()
            processor_thread.join()
        except KeyboardInterrupt:
            self.logger.debug("Closing...")
        finally:
            self.exit_handler()


if __name__ == '__main__':
    guidance = tziGuidance()
    guidance.run()
