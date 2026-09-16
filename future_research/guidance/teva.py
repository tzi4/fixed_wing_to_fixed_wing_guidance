#!/usr/bin/env python3
"""
TEVA guidance with target range derived from server telemetry.

The server supplies the target's 3D position at 1 Hz. This version uses that
information for range instead of estimating range from the square root of
bounding-box area, as goat_cam_offset.py does.

Target telemetry supplies range only to the altitude controller. Heading
and the target's position in the image come exclusively from the camera.
The controller therefore still requires visual target detection. This
restriction keeps the evaluation consistent with visual guidance.

At a closing speed of 20 m/s, range changes by about 20 m between samples.
Dead reckoning advances the latest position using the target's last known
velocity and the current own-aircraft position. If the sample becomes stale
(default 3 s), range is invalidated and the controller uses angular control.

Server format on the Redis 'rakip_telemetri' key:
    {"konumBilgileri": [{"iha_enlem": .., "iha_boylam": .., "iha_irtifa": ..}]}
The simulation publisher tools/server_simulator.py uses this format at 1 Hz.
"""
import time
import math
import csv
from datetime import datetime
import redis
import ast
import threading
import queue
import numpy as np
from pymavlink import mavutil
import json
import logging
import argparse

# CONNECTION CONTRACT
#bumblebee_guidance.sh configures hunter SysID 1 MAVProxy outputs on ports
#14550 for QGC, 14551 for formation.py/load_plan.py, and 14553 for guidance.
#There is no output on 14562. Connecting there would wait indefinitely
#for a heartbeat. Use --connect for a different real-flight endpoint.
DEFAULT_CONNECTION = 'udpin:127.0.0.1:14553'
DEFAULT_SYSID = 1
HEARTBEAT_TIMEOUT_S = 30.0
CAMERA_WIDTH = 1920
CAMERA_HEIGHT = 1080
LOCK_REQUIRED_S = 4.0
LOCK_SAMPLE_GAP_S = 0.25
TELEMETRY_TIMEOUT_S = 2.0
CAMERA_PROFILES = {
    # Erenimbus Gazebo sensor (models/emir_aircraft_temp/model.sdf).
    'sim': {'fx': 4543.0, 'fy': 4539.0, 'cx': 1025.0, 'cy': 569.0,
            'mount_phys_deg': 0.0, 'aim_deg': 0.0},
    # Reference shared by user, worked well in real flight.
    'real': {'fx': 4515.0, 'fy': 4510.0, 'cx': 1091.0, 'cy': 633.0,
             'mount_phys_deg': -1.0, 'aim_deg': -5.0},
}

log_filename = f"altitude_difference_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s',
    datefmt='%H:%M:%S',
    handlers=[
        logging.FileHandler(log_filename),
        logging.StreamHandler() # For terminal output
    ]
)
# PARAMETER ORIGINS: FLIGHT REFERENCE AND SIMULATION
#The values in goat_cam_offset.py are the flight-validated reference.
#Simulation measurements must not replace those values without hardware
#validation. The following inventory identifies their provenance.
#
#FLIGHT-VALIDATED VALUES, identical in both controllers:
#  Kp_heading 3.0, Kd_heading 0.06, Kp_alt 2.5, Kd_alt 0.05, Kp_rate 0.6,
#  deadzone_deg 0.4, max_heading_change_deg 35,
#  max_alt_change_m +2.0, min_alt_change_m -5.0.
#
#PLATFORM-INDEPENDENT PROTECTIONS:
#  - Below 16 m AGL, reduce descent authority. Never increase its limit.
#  - Do not generate commands before telemetry arrives.
#  - Suppress the first-frame derivative and bound dt below by half the
#    running median.
#
#SIMULATION-RELATED SETTINGS REQUIRING HARDWARE VALIDATION:
#  1) Kp_h 3.0 / Kd_h 0.06 reparameterizes the range-normalized controller.
#     At R = 48 m it reproduces the command from Kp_alt 2.5 m/degree,
#     preserving the flight tuning at that range. The 48 m reference comes
#     from the 0-100 m operating band. Behavior at other ranges changes
#     intentionally to account for geometry.
#  2) narrow_deadzone_deg 0.15 and the 12 px threshold (P3) depend strongly
#     on simulation. They were derived from the detector noise floor,
#     0.019 degrees below 25 m. Re-measure real detector noise as deviation
#     from a 0.5 s moving average before using these settings. Set
#     quality_deadzone = False to retain the 0.4-degree deadzone.
#  3) Handover-gate thresholds of 6 s, 0.3 m/s and 1.5 degrees detect whether
#     the aircraft has settled. They are not controller gains, but were
#     obtained from simulated handovers. Set HANDOVER_GATE = False if needed.
#
#INACTIVE BY DEFAULT: range_k calibrates bounding-box range and is used only
#   range_source=Used if 'bbox' is selected; teva.py defaults to 'telemetry' so it does not enter the control path.  ================================================================================

# CONFIGURATION FLAGS
#These flags can be set directly in the source.
#
#AUTO-to-GUIDED handover gate:
#  True: hold the entry altitude and heading while the aircraft settles.
#        Keep the PID inactive, then ramp its commands over 2 s.
#  False: apply full guidance commands immediately after handover.
#
#Measured rationale: ArduPlane has its own handover transient even without
#guidance commands. Over the first 15 s, pitch oscillates by 5.8 degrees,
#throttle falls from 88% to 59%, and the aircraft descends by about 3 m.
#Adding guidance during this transient increases the pitch oscillation to
#14.9 degrees and moves the target out of view. Disable on real hardware
#if the platform's handover behavior requires it.
HANDOVER_GATE = True
HANDOVER_WAIT_S = 6.0        # Minimum waiting time for the door
# Measured: a 2.0 s minimum wait was too short. The gate opened at 4.5 s
#while altitude was still falling from 49.96 to 46.88 m. Pitch then
#oscillated by 14.9 degrees over the next 15 s. Starting the same code
#in settled GUIDED flight gave 1.3 degrees. Keep the gate closed until
#the aircraft has actually settled.
SOFT_START_S = 2.0    # Ramp time of the command after the gate is opened
# =====================================================================

# ─── VIRTUAL GIMBAL CONSTANTS ───
R_c_b = np.array([[0, 0, 1],
                  [1, 0, 0],
                  [0, 1, 0]], dtype=float)
R_c_b_T = R_c_b.T

def compute_R_b_e(roll, pitch, yaw):
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp,   cp*sr,            cp*cr           ]
    ])


class TargetLockTracker:
    """Track the official lock geometry over a continuous qualifying interval.

The target box must remain entirely within the central 25%-75% of image
width and 10%-90% of image height. Its width or height must cover at least
5% of the corresponding image dimension. Short frame gaps are tolerated.
Longer gaps reset progress. Once acquired, the lock remains latched until
the run ends.
    """

    def __init__(self, image_w=CAMERA_WIDTH, image_h=CAMERA_HEIGHT,
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
        try:
            x, y, w, h = (float(v) for v in bbox[:4])
        except (TypeError, ValueError):
            return False
        if not all(math.isfinite(v) for v in (x, y, w, h)) or w <= 0 or h <= 0:
            return False
        inside = (
            x >= self.image_w * 0.25 and x + w <= self.image_w * 0.75 and
            y >= self.image_h * 0.10 and y + h <= self.image_h * 0.90
        )
        large_enough = w >= self.image_w * 0.05 or h >= self.image_h * 0.05
        return inside and large_enough

    def reset(self):
        if not self.acquired:
            self.started_at = None
            self.last_qualifying_at = None
            self.progress_s = 0.0

    def update(self, bbox, now=None, active=True):
        now = time.monotonic() if now is None else float(now)
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

class TargetTelemetry(threading.Thread):
    """1 Hz target 3D location (Redis 'rakip_telemetri') from the server.

    Used for range ONLY. The location_data/yon information on the frame CANNOT be taken from here; they come from the camera.
    """

    def __init__(self, stale_s=3.0):
        super().__init__()
        self.daemon = True
        self.running = True
        self.stale_s = stale_s
        self.lock = threading.Lock()
        self.lat = self.lon = self.alt = None
        self.t = 0.0
        self.prev = None          # (t, lat, lon, alt) — for target speed
        self.v_n = self.v_e = self.v_d = 0.0
        try:
            self.r = redis.Redis(host='localhost', port=6379, db=0)
            self.r.ping()
        except Exception as e:
            print(f"[teva] No Redis, target telemetry off: {e}")
            self.running = False

    def run(self):
        while self.running:
            try:
                raw = self.r.get('rakip_telemetri')
                if raw:
                    d = json.loads(raw.decode('utf-8'))
                    k = (d.get('konumBilgileri') or [{}])[0]
                    lat = k.get('iha_enlem'); lon = k.get('iha_boylam'); alt = k.get('iha_irtifa')
                    if lat is not None and lon is not None and alt is not None:
                        now = time.time()
                        # BUG FIX: server writes 1 Hz, we read 5 Hz. If the value had NOT changed, refreshing the timestamp would miscalculate the target speed by up to 5 times. Update only on the actual new instance.
                        with self.lock:
                            identical = (self.lat is not None
                                    and abs(float(lat) - self.lat) < 1e-9
                                    and abs(float(lon) - self.lon) < 1e-9
                                    and abs(float(alt) - self.alt) < 1e-6)
                        if identical:
                            time.sleep(0.05)
                            continue
                        with self.lock:
                            if self.lat is not None and now > self.t:
                                dt = now - self.t
                                if 0.2 < dt < 5.0:
                                    self.v_n = (float(lat) - self.lat) * 111320.0 / dt
                                    self.v_e = ((float(lon) - self.lon) * 111320.0
                                                * math.cos(math.radians(float(lat)))) / dt
                                    self.v_d = (float(alt) - self.alt) / dt
                            self.lat, self.lon, self.alt = float(lat), float(lon), float(alt)
                            self.t = now
            except Exception:
                pass
            time.sleep(0.2)

    def raw_range(self, own_lat, own_lon, own_alt):
        """No fast-forward, no filter range — for diagnosis."""
        with self.lock:
            if self.lat is None or own_lat is None or own_lat != own_lat:
                return None
            lat, lon, alt = self.lat, self.lon, self.alt
        dn = (lat - own_lat) * 111320.0
        de = (lon - own_lon) * 111320.0 * math.cos(math.radians(own_lat))
        du = alt - own_alt
        return math.sqrt(dn * dn + de * de + du * du)

    def range_m(self, own_lat, own_lon, own_alt):
        """3D range (m) relative to our own position. None if stale or no data.

        The last example FAST FORWARDS to the present with the target's last known speed; Our own position is already momentary.
        """
        with self.lock:
            if self.lat is None or own_lat is None or own_lat != own_lat:
                return None
            age_s = time.time() - self.t
            if age_s > self.stale_s:
                return None
            lat = self.lat + (self.v_n * age_s) / 111320.0
            lon = self.lon + (self.v_e * age_s) / (111320.0 * max(0.1, math.cos(math.radians(self.lat))))
            alt = self.alt + self.v_d * age_s
        dn = (lat - own_lat) * 111320.0
        de = (lon - own_lon) * 111320.0 * math.cos(math.radians(own_lat))
        du = alt - own_alt
        return math.sqrt(dn * dn + de * de + du * du)


class FlightLogger:
    def __init__(self, file_prefix="flight_log_guided"):
        self.log_filename = f"{file_prefix}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        self.log_file = open(self.log_filename, mode='w', newline='')
        self.csv_writer = csv.writer(self.log_file)
        self.row_count = 0
        self.flush_interval = 10

        headers = [
            "timestamp", "elapsed_s", "dt", "frame_num",
            "flight_mode", "task", "system_state", "target_found",
            "command_sent", "queue_size", "data_age_ms",
            "bbox_x", "bbox_y", "bbox_w", "bbox_h", "bbox_center_x", "bbox_center_y",
            "target_area_px", "coverage_w_pct", "coverage_h_pct",
            "stab_x", "stab_y", "delta_stab_x", "delta_stab_y",
            "current_alt_m", "roll_deg", "pitch_deg", "yaw_deg",
            "raw_pixel_error_x", "raw_pixel_error_y", "stab_pixel_error_x", "stab_pixel_error_y",
            "raw_error_x_deg", "raw_error_y_deg", "stab_error_x_deg", "stab_error_y_deg",
            "raw_derivative_x", "raw_derivative_y", "filtered_deriv_x", "filtered_deriv_y",
            "deadzone_active_x", "deadzone_active_y", "p_input_x", "p_input_y",
            "integral_accum_x", "p_term_heading", "i_term_heading", "d_term_heading", "cmd_heading_deg", "target_heading_deg",
            "integral_accum_y", "p_term_alt", "i_term_alt", "d_term_alt", "cmd_alt_m", "target_alt_m",
            "anti_windup_x", "anti_windup_y",
            # --- added 2026-07-29: for honest PID diagnosis --- The question "how strongly was the command executed" cannot be answered without the RAW (pre-clamp) command + saturation flag on each axis.
            "sqrt_area_px", "airspeed_ms", "groundspeed_ms", "vz_ms", "throttle_pct",
            "rate_error_deg", "p_term_rate", "i_term_rate", "d_term_rate", "heading_rate_dps",
            "cmd_head_raw_deg", "cmd_alt_raw_m", "sat_head", "sat_alt",
            # These columns identify test settings, not target ground truth.
            #The camera-only guidance process must not read target altitude or position.
            #The separate tools/ground_truth_logger.py records that reference data.
            #These two columns identify the configuration used for each run.
            "aim_pitch_deg", "mount_phys_deg", "lat", "lon",
            "estimated_range_m", "m_per_deg", "range_source",
            "control_coverage_pct", "speed_error_pct", "p_term_speed",
            "i_term_speed", "cmd_speed_ms", "guid_active",
            "lock_qualifying", "lock_progress_s", "lock_acquired"
        ]
        self.csv_writer.writerow(headers)
        self.log_file.flush()
        print(f"Log file created: {self.log_filename}")

    def log(self, row_data):
        self.csv_writer.writerow(row_data)
        self.row_count += 1
        if self.row_count % self.flush_interval == 0:
            self.log_file.flush()

    def close(self):
        if not self.log_file.closed:
            self.log_file.flush()
            self.log_file.close()

class RedisListener(threading.Thread):
    def __init__(self, data_queue, image_w=CAMERA_WIDTH, image_h=CAMERA_HEIGHT):
        super().__init__()
        self.data_queue = data_queue
        self.image_w = float(image_w)
        self.image_h = float(image_h)
        self.daemon = True
        self.running = True
        self.task = "Unknown"

        print("Connecting to server Redis...")
        try:
            self.r = redis.Redis(host='localhost', port=6379, db=0)
            self.pubsub = self.r.pubsub()
            self.pubsub.subscribe('tracker_bbox')
            print("Redis Subscribed to channel 'tracker_bbox'.")
        except Exception as e:
            print(f"Redis Connection Error: {e}")
            self.running = False

    @staticmethod
    def parse_bbox(data, image_w=CAMERA_WIDTH, image_h=CAMERA_HEIGHT):
        """Validate detector message, crop to image and rotate (x,y,w,h)."""
        if not isinstance(data, (list, tuple)) or len(data) < 4:
            return None
        if len(data) >= 6:
            try:
                if float(data[5]) <= 0.0:
                    return None
            except (TypeError, ValueError):
                return None
        try:
            x, y, w, h = (float(v) for v in data[:4])
        except (TypeError, ValueError):
            return None
        if not all(math.isfinite(v) for v in (x, y, w, h)) or w <= 0 or h <= 0:
            return None
        x1 = max(0.0, min(float(image_w), x))
        y1 = max(0.0, min(float(image_h), y))
        x2 = max(0.0, min(float(image_w), x + w))
        y2 = max(0.0, min(float(image_h), y + h))
        if x2 <= x1 or y2 <= y1:
            return None
        return (int(round(x1)), int(round(y1)),
                int(round(x2 - x1)), int(round(y2 - y1)))

    def _put_latest(self, item):
        """Release only the newest frame to the controller; No old bbox backlog."""
        try:
            self.data_queue.put_nowait(item)
            return
        except queue.Full:
            pass
        try:
            self.data_queue.get_nowait()
        except queue.Empty:
            pass
        try:
            self.data_queue.put_nowait(item)
        except queue.Full:
            pass

    def run(self):
        while self.running:
            try:
                task_bytes = self.r.get('task')
                self.task = task_bytes.decode('utf-8') if task_bytes else "Unknown"

                message = self.pubsub.get_message(ignore_subscribe_messages=True, timeout=0.01)
                if message and message['type'] == 'message':
                    data_str = message['data'].decode('utf-8')
                    try:
                        try:
                            bbox_data = json.loads(data_str)
                        except json.JSONDecodeError:
                            bbox_data = ast.literal_eval(data_str)
                        bbox = self.parse_bbox(bbox_data, self.image_w, self.image_h)
                        if bbox is not None:
                            x, y, w, h = bbox
                            obj_x = x + (w / 2.0)
                            obj_y = y + (h / 2.0)
                            target_area = w * h

                            self._put_latest({
                                'type': 'target',
                                'bbox_x': int(x),
                                'bbox_y': int(y),
                                'obj_x': obj_x,
                                'obj_y': obj_y,
                                'bbox_w': float(w),
                                'bbox_h': float(h),
                                'target_area': target_area,
                                'timestamp': time.time(),
                                'task': self.task
                            })
                    except (UnicodeDecodeError, ValueError, SyntaxError, TypeError):
                        continue
            except Exception:
                time.sleep(0.1)

    def get_task(self):
        return self.task

    # get_target_altitude() DEPRECATED (2026-07-29): target altitude should not enter the guidance process. See tools/ground_truth_logger.py


class MavlinkManager(threading.Thread):
    def __init__(self, connection_str=DEFAULT_CONNECTION, target_sysid=DEFAULT_SYSID,
                 heartbeat_timeout=HEARTBEAT_TIMEOUT_S):
        super().__init__()
        self.daemon = True
        self.running = True
        self.lock = threading.Lock()

        self.current_mode = "UNKNOWN"
        self.message_times = {}

        self.current_yaw_rad = 0.0
        self.current_roll_rad = 0.0
        self.current_pitch_rad = 0.0
        self.current_alt_rel = 100.0  # Default starting altitude
        # BOOTSTRAP PROTECTION (2026-07-30)
        #Before telemetry arrives, the default altitude of 100 m could produce
        #a first command near 102 m when the aircraft is actually at 50 m.
        #With param3 = 0, that is an immediate 52 m target step. No recorded run
        #triggered it, but the timing margin fell to 0.87 s. A heartbeat before
        #GLOBAL_POSITION_INT in GUIDED can expose this case. Require both
        #telemetry flags before entering the control path.
        self.attitude_received = False
        self.position_received = False
        # Telemetry for PID and speed plots (VFR_HUD + GLOBAL_POSITION_INT).
        self.current_airspeed = float('nan')
        self.current_groundspeed = float('nan')
        self.current_throttle = float('nan')
        self.current_lat = float('nan')
        self.current_lon = float('nan')
        self.current_vx = float('nan')
        self.current_vy = float('nan')
        self.current_vz = float('nan')

        print(f"Link to MAVLink: {connection_str} (expected SysID {target_sysid})...")
        try:
            self.master = mavutil.mavlink_connection(connection_str)
            # wait_heartbeat() is indefinite: connect to the wrong port and the code will silently hang forever. Waiting with timeout + SysID verification.
            heartbeat = None
            deadline = time.monotonic() + heartbeat_timeout
            while time.monotonic() < deadline:
                candidate = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
                if candidate is None:
                    continue
                if not self.master.probably_vehicle_heartbeat(candidate):
                    continue  # MAVProxy GCS heartbeat
                if candidate.get_srcSystem() == target_sysid:
                    heartbeat = candidate
                    break
            if heartbeat is None:
                raise TimeoutError(
                    f"{connection_str} on {heartbeat_timeout:.0f} in sec "
                    f"SysID {target_sysid} heartbeat did not arrive")
            self.master.target_system = heartbeat.get_srcSystem()
            self.master.target_component = heartbeat.get_srcComponent()
            print(f"Connection Successful. System ID: {self.master.target_system} "
                  f"(comp {self.master.target_component})")
            # Request extra telemetry streams (GLOBAL_POSITION_INT for altitude etc.)
            self.master.mav.request_data_stream_send(
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_POSITION, 10, 1)
            self.master.mav.request_data_stream_send(
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 10, 1)
        except Exception as e:
            print(f"MAVLink Error: {e}")
            self.running = False

    def run(self):
        while self.running:
            try:
                with self.lock:
                    msg = self.master.recv_match(type=['ATTITUDE', 'HEARTBEAT', 'GLOBAL_POSITION_INT', 'VFR_HUD'], blocking=False)

                if msg:
                    self.message_times[msg.get_type()] = time.monotonic()
                    if msg.get_type() == 'ATTITUDE':
                        self.attitude_received = True
                        self.current_yaw_rad = msg.yaw
                        self.current_roll_rad = msg.roll
                        self.current_pitch_rad = msg.pitch
                    elif msg.get_type() == 'HEARTBEAT':
                        if self.master.flightmode:
                            self.current_mode = self.master.flightmode
                    elif msg.get_type() == 'GLOBAL_POSITION_INT':
                        self.position_received = True
                        self.current_alt_rel = msg.relative_alt / 1000.0
                        # For PID plots: ground speed components and attitude.
                        self.current_lat = msg.lat / 1e7
                        self.current_lon = msg.lon / 1e7
                        self.current_vx = msg.vx / 100.0
                        self.current_vy = msg.vy / 100.0
                        self.current_vz = msg.vz / 100.0
                    elif msg.get_type() == 'VFR_HUD':
                        # The baseline is logged even BEFORE the speed control is added.
                        self.current_airspeed = msg.airspeed
                        self.current_groundspeed = msg.groundspeed
                        self.current_throttle = msg.throttle
                time.sleep(0.01)
            except Exception:
                time.sleep(0.1)

    def telemetry_is_fresh(self, names=('HEARTBEAT', 'ATTITUDE', 'GLOBAL_POSITION_INT'),
                           max_age=TELEMETRY_TIMEOUT_S):
        now = time.monotonic()
        return all(now - self.message_times.get(name, -1e9) <= max_age
                   for name in names)

    def commands_allowed(self):
        return (self.running and self.current_mode == 'GUIDED' and
                self.telemetry_is_fresh())

    def send_heading_target(self, heading_deg, heading_rate_dps=3.0):
        if not self.commands_allowed():
            return False
        speed = self.current_airspeed
        if not math.isfinite(speed) or speed < 1.0:
            speed = self.current_groundspeed
        if not math.isfinite(speed) or speed < 1.0:
            speed = 20.0
        # ArduPlane interprets 43002 param3 as centripetal acceleration (m/s^2), not deg/s. Convert the readable rate in Guidance to a=V*omega.
        heading_accel_mss = max(
            abs(float(speed)) * math.radians(abs(float(heading_rate_dps))), 0.05)
        with self.lock:
            try:
                self.master.mav.command_long_send(
                    self.master.target_system,
                    self.master.target_component,
                    mavutil.mavlink.MAV_CMD_GUIDED_CHANGE_HEADING,
                    0, 1, heading_deg, heading_accel_mss, 0, 0, 0, 0
                )
                return True
            except Exception:
                return False

    def send_speed_target(self, speed_ms, accel=2.0):
        """Command 43000: param1 = 0 for airspeed, param2 = speed,
param3 = acceleration.

Used only for test isolation through --fixed-speed. This sender does not
implement a speed controller. Holding speed externally allows vertical
tuning without confusing climb response with acceleration.
        """
        if not self.commands_allowed():
            return False
        with self.lock:
            try:
                self.master.mav.command_long_send(
                    self.master.target_system, self.master.target_component,
                    mavutil.mavlink.MAV_CMD_GUIDED_CHANGE_SPEED,
                    0, 0, speed_ms, accel, 0, 0, 0, 0)
                return True
            except Exception:
                return False

    def send_altitude_target(self, altitude_m):
        """Send MAV_CMD_GUIDED_CHANGE_ALTITUDE (43001) with param3 = 0.

The ArduPlane handler in GCS_MAVLink_Plane.cpp sets target_alt_rate to
1000.0 when param3 is zero, otherwise to fabsf(param3). In mode_guided.cpp:
    delta = (now - target_alt_time_ms) / 1000
    delta_amt = delta * target_alt_rate
    alt = constrain(target, previous - delta_amt, previous + delta_amt)

Each command resets target_alt_time_ms. At a 10 Hz command rate, delta is
about 0.1 s, so param3 = R permits only about 0.1 * R metres per command.
The July 1 setting param3 = 10 allowed about 1.3 m per command in the
recorded run. Requests to climb 25 m were consequently not applied as
intended, and that run did not meaningfully test the vertical controller.
Keep param3 at zero for this 10 Hz command stream. Bound altitude changes
locally with min_alt_change_m and max_alt_change_m.

The handler reads frame, z and param3. It does not use param2. A former
0.8 in param2 did not impose a ramp. Moving it to param3 would severely
restrict the vertical response.

ArduPlane denies z = 0 and z = -1. The caller keeps target_alt at or above
10 m.
        """
        if not self.commands_allowed():
            return False
        with self.lock:
            try:
                self.master.mav.command_long_send(
                    self.master.target_system,
                    self.master.target_component,
                    mavutil.mavlink.MAV_CMD_GUIDED_CHANGE_ALTITUDE,
                    0,          # confirmation
                    0, 0, 0,    # param1, param2, param3(=0: no ramp)
                    0, 0, 0,    # param4, param5, param6
                    altitude_m  # param7 -> COMMAND_INT.z (target altitude)
                )
                return True
            except Exception:
                return False

class AutopilotController:
    def __init__(self, connection_str=DEFAULT_CONNECTION, target_sysid=DEFAULT_SYSID,
                 mount_pitch_deg=None, mount_phys_pitch_deg=None, aim_pitch_deg=None,
                 fixed_heading=None, fixed_speed=None, camera_profile='sim',
                 speed_control_enabled=None):
        if camera_profile not in ('sim', 'real'):
            raise ValueError("camera_profile must be 'sim' or 'real'")
        self.camera_profile = camera_profile
        self.speed_control_enabled = (
            camera_profile == 'sim' if speed_control_enabled is None
            else bool(speed_control_enabled))
        # BBox 30 Hz comes; If the control loop slows down, queuing old frames won't introduce latency and spurious lock time.
        self.data_queue = queue.Queue(maxsize=1)
        self._running = True
        self._closed = False

        self.logger = FlightLogger(file_prefix="teva_log")
        # 1 Hz target 3D position — for range ONLY
        self.target_telem = TargetTelemetry()
        if self.target_telem.running:
            self.target_telem.start()
        self.mavlink = MavlinkManager(connection_str, target_sysid)
        self.redis = RedisListener(self.data_queue)

        if self.mavlink.running and self.redis.running:
            self.mavlink.start()
            self.redis.start()
        else:
            exit(1)

        # --- NEW PID AND CONTROL SETTINGS (HEADING & ALTITUDE) --- Suitable for the "P is quite cumbersome, I and D 0" request We have increased the Kp values ​​significantly so that the system can return to the target
        self.Kp_heading = 3.0    # 1 degree error = 4.5 degrees off target
        self.Ki_heading = 0.0
        self.Kd_heading = 0.06

        self.Kp_alt = 2.5       # 1 degree error = 1.5 meters altitude change
        self.Ki_alt = 0.0
        self.Kd_alt = 0.05

        # HEADING-RATE PID
        #The real-flight reference uses 0.6 and 5 deg/s. In Erenimbus SITL,
        #a 10-degree bank limit loses the target during elliptical turns.
        #The simulation profile permits a bank angle up to 23 degrees,
        #while retaining the real-flight profile's values.
        self.Kp_rate = 1.5 if camera_profile == 'sim' else 0.6
        self.Ki_rate = 0.0
        self.Kd_rate = 0.08     # For reaction to fast moving target

        self.integral_error_rate = 0.0
        self.prev_error_rate = 0.0
        self.prev_derivative_rate = 0.0

        self.min_heading_rate = 0.85    # Minimum rotation speed (deg/s)
        self.max_heading_rate = 12.0 if camera_profile == 'sim' else 5.0

        self.max_heading_change_deg = 35.0
        # ALTITUDE COMMAND LIMIT
        #With param3 = 0, ArduPilot accepts the target with target_alt_rate =
        #1000 m/s. The local clamp therefore provides the effective command limit.
        #A 25 m target jump can make TECS demand maximum climb or descent.
        #The flight-validated old_guidance/goat_gimbal_aircraft.py uses narrower,
        #asymmetric increments, retained here: at most +2 m for climb and -5 m
        #for descent. The asymmetry accounts for the aircraft's slower and more
        #energy-intensive climb response.
        self.max_alt_change_m = 2.0
        self.min_alt_change_m = -5.0

        # RANGE NORMALIZATION (2026-07-29)
        #The controller measures angular error but commands altitude in metres.
        #A fixed Kp_alt of 2.5 m/degree ignores the range dependence: one degree
        #corresponds to 0.87 m at 50 m and 5.24 m at 300 m. It can overcommand
        #near targets and undercommand distant ones.
        #
        #T5 measurements on 2026-07-29: mean |ey| was 1.23 degrees with 23%
        #saturation at 100-300 m, and 0.30 degrees with no saturation at 40-100 m.
        #Below 40 m, peak error reached 6.38 degrees. At 10 m range, 6 degrees is
        #about 1 m of altitude error, while the fixed gain demands 15 m.
        #
        #Convert angular error into a displacement: dh = R * tan(error).
        #This experiment estimates range from camera bounding-box area and does
        #not read target telemetry. The default is the range-based mode.
        #In the 2026-07-29 A/B/C test, the target descended from 50 to 30 m at a
        #mean range of 34 m:
        #  A, angular controller: mean |dz| 1.01 m, peak -8.7 m, saturation 19.4%.
        #  C, P1+P2+P3: mean |dz| 0.73 m, peak -5.4 m, saturation 7.4%.
        self.vertical_mode = 'range_m'       # 'angle' (obsolete) | 'range' (new)
        self.agl_scaled_descent = True    # P1 (ground safety only)
        self.quality_deadzone = True        # P3
        self.narrow_deadzone_deg = 0.15       # P3: noise floor below 25 m is 0.019 deg
        self.range_k = 2119.67         # R = k / sqrt_area_px
        self.range_min_m = 5.0         # calibration valid up to 4 m;
                                        # The 15 m base brought back the old fixed-gain bug at short range
        self.range_max_m = 250.0
        self.range_tau_s = 2.0         # low pass: DC offset changes slowly
        # 'telemetry' = 1 Hz 3D position from server (competition), 'bbox' = sqrt estimate from camera (legacy), 'auto' = telemetry if available, otherwise bbox
        self.range_source = 'telemetry'
        self.last_range_source = '-'
        self._filtered_range = None
        # DIMENSIONLESS GAINS
        #With Kp_h = 1.0, the reference range R = 143 m matches Kp_alt =
        #2.5 m/degree because 143 * tan(1 degree) = 2.50. This preserves the
        #previous tuning at one range and adjusts it geometrically elsewhere.
        #
        #Log analysis on 2026-07-29 showed that 143 m lies outside the 0-100 m
        #operating band. The effective gain in that band fell to a median factor
        #of 0.27, worsening mean OP_A error from 0.92 to 1.13 m.
        #The revised reference R = 48 m preserves the tuning near the middle of
        #the operating band. Only 5.9% of sample commands change by more than
        #0.25 m there, while range-dependent corrections remain outside the band.
        self.Kp_h = 3.0                 # 143/48
        self.Ki_h = 0.0
        self.Kd_h = 0.06                # 0.05/(48*tan1deg), same anchor
        self.deadzone_deg = 0.4

        self.prev_error_x, self.prev_error_y = 0.0, 0.0
        self.integral_error_x, self.integral_error_y = 0.0, 0.0
        self.prev_derivative_x, self.prev_derivative_y = 0.0, 0.0
        self.last_time = time.time()
        self._dt_history = []
        self._pitch_history = []
        self.last_cmd_send_time = 0.0

        self.last_target_time = 0.0
        self.last_target_heading = 0.0
        self.last_target_alt = 0.0
        self.last_heading_rate = self.min_heading_rate
        self.last_speed = 20.0

        self.frame_counter = 0
        self.start_time = time.time()

        # AUTO-TO-GUIDED HANDOVER GATE (2026-07-30)
        #Measured throttle reduction starts before the first guidance command:
        #t + 0.27 s versus t + 0.47 s. Pitch oscillates by about +/-6 degrees
        #for 8-16 s. The camera's vertical half field of view is 6.78 degrees,
        #so the target leaves the frame. Tracking is absent in 17.7-18.5% of
        #frames during the first 10 s.
        #
        #All four AUTO-to-GUIDED runs oscillated. Four runs that started in GUIDED
        #did not. This identifies handover as the source of the transient.
        #Hold the entry altitude and heading after handover until the aircraft
        #settles, keeping the PID inactive to avoid saturation while responding
        #to a transient it did not generate.
        #
        #The --disable-handover-gate option disables the gate on real hardware.
        #The gate addresses amplification of a simulated handover transient.
        #Real aircraft may require different handover behavior.
        self.handover_gate = HANDOVER_GATE
        # Erenimbus Second pitch transient 6-15 on SITL. comes in seconds.  To avoid changing the switching behavior of the real profile, the long wait is applied to the sim profile only.
        self.engage_wait_s = 15.0 if camera_profile == 'sim' else HANDOVER_WAIT_S
        self.engage_max_s = 25.0      # ceiling: don't wait forever
        self.engage_vz_threshold = 0.3       # m/s — ready when vertical speed drops below this
        self.soft_start_s = SOFT_START_S
        self._engage_t = None
        self._engage_alt = None         # Altitude at speed LOCKED
        self._engage_heading = None     # The direction in the revolution is also LOCKED
        self._gate_opened_t = None

        # TEST ISOLATION FOR SEQUENTIAL TUNING
        #Hold heading and speed while measuring vertical response. Simply omitting
        #GUIDED commands would let the aircraft loiter, so resend the held values.
        #  fixed_heading = 'auto': hold the initial yaw.
        #  fixed_speed = m/s: resend command 43000 each command cycle.
        self.fixed_heading = fixed_heading
        self.fixed_speed = fixed_speed      # number or 'auto'
        self._fixed_speed_ms = fixed_speed if isinstance(fixed_speed, (int, float)) else None
        self._fixed_heading_deg = None
        if isinstance(fixed_heading, (int, float)):
            self._fixed_heading_deg = float(fixed_heading)

        # AXIS 3: COVERAGE-BASED SPEED CONTROL
        #Use the cruise speed measured on entry to GUIDED as the baseline.
        #A small target increases speed to close the distance. At the desired
        #5.5% coverage, the command returns toward baseline. An explicit
        #--fixed-speed setting overrides this PID controller.
        self.Kp_speed = 0.30
        self.Ki_speed = 0.02
        self.Kd_speed = 0.0
        self.min_speed = 10.0     # above Erenimbus/ArduPlane AIRSPEED_MIN=9
        self.max_speed = 22.0     # Erenimbus/ArduPlane AIRSPEED_MAX
        self.base_speed = 20.0
        # The official lock threshold is 5%. 5.5% is centered because holding the target at full threshold resets the time in detection noise; 1 Hz telemetry only limits the approach and separation envelope and does not enter into the direction decision.
        self.target_coverage_pct = 5.5
        self.target_range_m = 52.0
        self.range_near_m = 45.0
        self.edge_speed_trim_ms = 1.0
        self.max_approach_delta_near_ms = 1.0
        self.max_approach_delta_far_ms = 2.0
        self.speed_integral_band = 7.0
        # When the TECS energy target changes rapidly, Erenimbus enters pitch/altitude oscillation. The coverage PID is therefore a slow range trimmer.
        self.speed_slew_rate = 0.15
        self.coverage_alpha = 0.35
        self.filtered_coverage = self.target_coverage_pct
        self.integral_error_speed = 0.0
        self.prev_error_speed = 0.0
        self.prev_derivative_speed = 0.0
        self._speed_seeded = False

        # --- CAMERA SETTINGS (Please Update According to Actual System!) ---
        self.camera_width = CAMERA_WIDTH
        self.camera_height = CAMERA_HEIGHT
        self.camera_hfov_rad = 0.42

        profile = CAMERA_PROFILES[self.camera_profile]
        self.center_x, self.center_y = profile['cx'], profile['cy']
        self.fx, self.fy = profile['fx'], profile['fy']
        default_mount_phys = profile['mount_phys_deg']
        default_aim = profile['aim_deg']

        self.lock_tracker = TargetLockTracker(self.camera_width, self.camera_height)
        self._last_lock_log_second = -1
        self._set_redis_state('guid', False)
        self._set_redis_state('guid_lock', False)
        self._set_redis_state('guid_lock_progress', '0.000')

        # CAMERA PITCH MOUNTING ANGLE RELATIVE TO THE AIRFRAME
        #The former single mount_pitch_deg value of -6.0 combined two quantities:
        #  (a) mount_phys_pitch_deg is the physical angle between camera and
        #      autopilot, approximately -1 degree. It is fixed in the body frame,
        #      so the camera rolls with the aircraft. Apply it before derotation.
        #  (b) aim_pitch_deg compensates for angle of attack, approximately
        #      -5 degrees. It specifies the target's desired elevation relative
        #      to the horizon. Apply it after derotation in the horizon-aligned
        #      virtual frame.
        #Applying both offsets in the body frame produces a spurious horizontal
        #error proportional to delta * sin(roll).
        def _ry(deg):
            r = math.radians(deg)
            return np.array([
                [ math.cos(r), 0.0, math.sin(r)],
                [ 0.0,          1.0, 0.0         ],
                [-math.sin(r), 0.0, math.cos(r)]
            ])

        # SIMULATION DEFAULT
        #The camera is parallel to the airframe. Camera pose pitch is zero in
        #models/bumblebee/model.sdf and models/emir_aircraft_temp/model.sdf, so the
        #physical mounting angle is 0.0 degrees.
        #
        #The measured camera-to-autopilot angle on the real aircraft is about
        #-1.0 degree. Use --mount-phys-pitch -1.0 or set the source default for
        #that installation. The angle can also be estimated from flight logs:
        #the arctangent of the regression slope of stab_error_x_deg against
        #roll_deg equals the negative mounting angle.
        self.mount_phys_pitch_deg = default_mount_phys
        # MEANING OF aim_pitch_deg (verified numerically on 2026-07-29)
        #The virtual gimbal removes body pitch mathematically, while the physical
        #camera remains tilted with the airframe. With a vertical half field of
        #view of about 6.8 degrees, the virtual-frame center and physical-image
        #center can differ. aim_pitch_deg supplies the constant offset while the
        #virtual gimbal removes the changing body-rotation component.
        #
        #Measured relationships, with theta denoting body pitch and eps target
        #elevation relative to the horizon:
        #    ey = -(eps + aim), so ey = 0 implies eps = -aim
        #    delta_raw = theta - eps
        #Centering the target in the actual image requires delta_raw = 0:
        #    eps = theta, hence aim = -theta.
        #
        #An earlier Erenimbus estimate used aim = +2.8 degrees. During the
        #2026-08-11 A/B flights, this introduced a persistent vertical error
        #for a target at the same altitude and excited TECS oscillation in run02.
        #The setting aim = 0 acquired locks on straight and elliptical routes in
        #run06 and run10. The simulation profile therefore uses 0 degrees.
        #The real-aircraft profile retains the flight-validated -5 degrees.
        self.aim_pitch_deg = default_aim
        # Fade the aim offset with range: full strength below the lower threshold,
        #then a linear reduction to zero. Telemetry already supplies range.
        #With simulation aim = 0 the contribution is zero. The real profile's
        #-5-degree offset is scaled according to range.
        self.aim_full_range_m = 120.0
        self.aim_zero_range_m = 250.0
        if mount_phys_pitch_deg is not None:
            self.mount_phys_pitch_deg = float(mount_phys_pitch_deg)
        if aim_pitch_deg is not None:
            self.aim_pitch_deg = float(aim_pitch_deg)
        # Legacy --mount-pitch (single parameter) backwards compatibility: given value refers to SUM, resolved as aim = total - mount_phys.
        if mount_pitch_deg is not None:
            self.aim_pitch_deg = float(mount_pitch_deg) - self.mount_phys_pitch_deg

        self.mount_pitch_deg = self.mount_phys_pitch_deg + self.aim_pitch_deg  # for compatibility and logging
        print(f"[camera] profile={self.camera_profile} "
              f"K=({self.fx:.0f},{self.fy:.0f},{self.center_x:.0f},{self.center_y:.0f}) "
              f"mount_phys={self.mount_phys_pitch_deg:+.2f} "
              f"aim={self.aim_pitch_deg:+.2f} (total {self.mount_pitch_deg:+.2f})")

        self.R_mount_phys = _ry(self.mount_phys_pitch_deg)
        self.R_aim = _ry(self.aim_pitch_deg)
        self._last_aim_deg = self.aim_pitch_deg

        self.K = np.array([
            [self.fx, 0,       self.center_x],
            [0,       self.fy, self.center_y],
            [0,       0,       1            ]
        ])
        self.K_inv = np.linalg.inv(self.K)

    def estimated_range(self, target_area, dt):
        """TEVA: 1 Hz 3D location from server first, drops to bbox if necessary."""
        if self.range_source in ('telemetry', 'auto'):
            m = self.target_telem.range_m(getattr(self.mavlink, 'current_lat', None),
                                      getattr(self.mavlink, 'current_lon', None),
                                      self.mavlink.current_alt_rel)
            if m is not None:
                self.last_range_source = 'telemetry'
                m = self.clamp(m, self.range_min_m, self.range_max_m)
                if self._filtered_range is None:
                    self._filtered_range = m
                else:
                    # Telemetry noiseless; filter to smooth only 1 Hz digits and much faster (tau/4).
                    a = dt / (self.range_tau_s / 4.0 + dt) if dt > 0 else 0.0
                    self._filtered_range += a * (m - self._filtered_range)
                return self._filtered_range
            if self.range_source == 'telemetry':
                # It's safer to fall into angular mode than to silently use the wrong range with stale or incomplete data.
                self.last_range_source = 'NONE'
                return None
        self.last_range_source = 'bbox'
        return self._bbox_range(target_area, dt)

    def _bbox_range(self, target_area, dt):
        """Range estimation from camera: R = k / sqrt(area), with low pass filter.

        The target's telemetry is NOT used. Calibration was done offline (tools/range_calibration.py, held test RMSE 17.9 m, valid 4-275 m).         The filter is consciously slow: this is a DC correction, it should not carry noise into the control law.
        """
        if not target_area or target_area <= 0:
            return self._filtered_range
        raw_value = self.range_k / math.sqrt(target_area)
        raw_value = self.clamp(raw_value, self.range_min_m, self.range_max_m)
        if self._filtered_range is None:
            self._filtered_range = raw_value
        else:
            a = dt / (self.range_tau_s + dt) if dt > 0 else 0.0
            self._filtered_range += a * (raw_value - self._filtered_range)
        return self._filtered_range

    def _effective_aim_deg(self, range_est):
        """Apply full aim offset within the range band and fade it outside.

A constant angular offset requires altitude displacement R * tan(aim).
At 120 m the displacement is 5.9 m, but at 300 m it reaches 14.7 m,
which exceeds available command authority. The measured distant case
saturated 35% of the time and failed to settle, motivating the fade.
        """
        if not self.aim_pitch_deg or range_est is None:
            return self.aim_pitch_deg
        r = float(range_est)
        if r <= self.aim_full_range_m:
            k = 1.0
        elif r >= self.aim_zero_range_m:
            k = 0.0
        else:
            k = ((self.aim_zero_range_m - r)
                 / (self.aim_zero_range_m - self.aim_full_range_m))
        return self.aim_pitch_deg * k

    def _seed_speed(self):
        """--fixed-speed auto: capture the actual cruise speed once at handover.

        If an aircraft cruises at 19.6 m/s in AUTO, commanding 18 m/s at
        handover asks TECS to remove 1.6 m/s. The measured handover
        oscillation was proportional to this difference.
        """
        if self._fixed_speed_ms is not None:
            return
        v = getattr(self.mavlink, 'current_airspeed', None)
        if v is not None and v == v and 5.0 < v < 40.0:
            self._fixed_speed_ms = round(float(v), 2)
            print(f"[speed] seeded from cruising speed at constant speed rpm: "
                  f"{self._fixed_speed_ms:.2f} m/s")

    def _set_redis_state(self, key, value):
        try:
            if isinstance(value, bool):
                value = 'True' if value else 'False'
            self.redis.r.set(key, value)
        except Exception:
            pass

    def _update_lock_state(self, bbox, now, active):
        was_acquired = self.lock_tracker.acquired
        acquired = self.lock_tracker.update(bbox, now=now, active=active)
        self._set_redis_state('guid_lock', acquired)
        self._set_redis_state('guid_lock_progress',
                              f"{self.lock_tracker.progress_s:.3f}")
        progress_second = int(self.lock_tracker.progress_s)
        if progress_second != self._last_lock_log_second and not acquired:
            self._last_lock_log_second = progress_second
            if self.lock_tracker.progress_s > 0:
                print(f"[lock] {self.lock_tracker.progress_s:.1f}/"
                      f"{self.lock_tracker.required_s:.1f} s")
        if acquired and not was_acquired:
            print("[lock] LOCKED — official condition met for 4 seconds without interruption")
            logging.warning("LOCKED — official condition met for 4 seconds without interruption")
        return acquired

    def _reset_speed_control(self):
        self.integral_error_speed = 0.0
        self.prev_error_speed = 0.0
        self.prev_derivative_speed = 0.0
        self.filtered_coverage = self.target_coverage_pct
        self._speed_seeded = False

    def _seed_speed_control(self):
        if self._speed_seeded:
            return
        measured = getattr(self.mavlink, 'current_airspeed', float('nan'))
        if math.isfinite(measured):
            self.base_speed = self.clamp(float(measured), self.min_speed, self.max_speed)
        self.last_speed = self.base_speed
        self._speed_seeded = True
        print(f"[speed] coverage PID base {self.base_speed:.2f} m/s")

    def _update_handover_gate(self, current_time, enabled, telemetry_ready,
                           current_alt, pitch_now, yaw_deg):
        """Implement the AUTO-to-GUIDED gate as a latched state machine.

Opening the gate must not let the next vertical-speed excursion close
it again. Likewise, coasting between bounding boxes must not resend an
old PID target. Set handover targets independently of vision updates
and keep an opened gate latched until GUIDED ends.
        """
        if not enabled:
            self._engage_t = None
            self._engage_alt = None
            self._engage_heading = None
            self._gate_opened_t = None
            self._pitch_history.clear()
            return False
        if not telemetry_ready:
            return True
        if self._engage_t is None:
            self._engage_t = current_time
            self._engage_alt = current_alt
            self._engage_heading = yaw_deg
            self.last_target_alt = current_alt
            self.last_target_heading = yaw_deg
            self._gate_opened_t = None
            self._pitch_history.clear()
            if self.speed_control_enabled:
                self._seed_speed_control()
            print(f"[handover] GUIDED detected — until aircraft settles "
                  f"FIXING ({self.engage_wait_s:.1f}-{self.engage_max_s:.1f} s)")
        if not self.handover_gate:
            if self._gate_opened_t is None:
                self._gate_opened_t = current_time
            return False
        # The opened gate GUIDED is wide open fails.
        if self._gate_opened_t is not None:
            return False

        elapsed_s = current_time - self._engage_t
        vz = abs(getattr(self.mavlink, 'current_vz', 0.0) or 0.0)
        if not math.isfinite(vz):
            vz = 0.0
        self._pitch_history.append(math.degrees(pitch_now))
        if len(self._pitch_history) > 80:
            self._pitch_history.pop(0)
        pitch_settled = (len(self._pitch_history) >= 40 and
                        (max(self._pitch_history[-40:]) -
                         min(self._pitch_history[-40:])) < 1.5)
        settling_state = (elapsed_s < self.engage_wait_s or
                      vz > self.engage_vz_threshold or not pitch_settled)
        if elapsed_s > self.engage_max_s:
            settling_state = False
        if not settling_state:
            self._gate_opened_t = current_time
            print("[handover] gate opened — commands 2 s will ramp")
        return settling_state

    def _speed_control(self, bbox_w, bbox_h, dt, first_sample=False,
                       bbox_x=None, bbox_y=None, range_est=None):
        self._seed_speed_control()
        coverage = max(bbox_w / self.camera_width * 100.0,
                       bbox_h / self.camera_height * 100.0)
        self.filtered_coverage = (self.coverage_alpha * coverage +
                                  (1.0 - self.coverage_alpha) * self.filtered_coverage)
        error = self.target_coverage_pct - self.filtered_coverage
        raw_derivative = 0.0 if first_sample else (error - self.prev_error_speed) / dt
        derivative = (0.05 * raw_derivative +
                      0.95 * self.prev_derivative_speed)

        proposed_integral = self.integral_error_speed
        if abs(error) < self.speed_integral_band:
            proposed_integral += error * dt
        proposed_integral = self.clamp(proposed_integral, -50.0, 50.0)

        p_term = error * self.Kp_speed
        proposed_i = proposed_integral * self.Ki_speed
        d_term = derivative * self.Kd_speed
        unconstrained = self.base_speed + p_term + proposed_i + d_term
        at_upper = unconstrained > self.max_speed and error > 0
        at_lower = unconstrained < self.min_speed and error < 0
        if not (at_upper or at_lower):
            self.integral_error_speed = proposed_integral
        i_term = self.integral_error_speed * self.Ki_speed
        command = self.clamp(self.base_speed + p_term + i_term + d_term,
                             self.min_speed, self.max_speed)

        # The target telemetry of the 1 Hz is the advantage of the TEVA: only secure the closing envelope without taking away the horizontal direction from the telemetry. Fragmented or small nearby boxes must not produce a spurious acceleration command.
        if range_est is not None and math.isfinite(float(range_est)):
            range_m = float(range_est)
            approach_cap = (self.max_approach_delta_far_ms
                            if range_m > 100.0
                            else self.max_approach_delta_near_ms)
            command = min(command, self.base_speed + approach_cap)
            if range_m <= self.range_near_m:
                command = min(command, self.base_speed - self.edge_speed_trim_ms)

        # If the target image extends beyond the lock region, recover the geometry first;  Looking at the small image at the edge of the frame and accelerating led to passing by. This door has no effect on the direction command.
        if bbox_x is not None and bbox_y is not None:
            inside = (bbox_x >= self.camera_width * 0.25 and
                      bbox_x + bbox_w <= self.camera_width * 0.75 and
                      bbox_y >= self.camera_height * 0.10 and
                      bbox_y + bbox_h <= self.camera_height * 0.90)
            if not inside:
                command = min(command, self.base_speed - self.edge_speed_trim_ms)
                self.integral_error_speed *= 0.9
                i_term = self.integral_error_speed * self.Ki_speed
        max_delta = self.speed_slew_rate * dt
        command = self.clamp(command, self.last_speed - max_delta,
                             self.last_speed + max_delta)

        self.prev_error_speed = error
        self.prev_derivative_speed = derivative
        self.last_speed = command
        return command, coverage, error, p_term, i_term

    def _send_speed_command(self, command):
        """Keep true reference behavior: sending 43000 if rate PID is off."""
        if self.speed_control_enabled or self.fixed_speed is not None:
            return self.mavlink.send_speed_target(command)
        return False

    def clamp(self, value, min_val, max_val):
        return max(min(value, max_val), min_val)

    def stabilize_pixel(self, obj_x, obj_y):
        p_raw = np.array([obj_x, obj_y, 1.0])
        r_cam = self.K_inv @ p_raw
        r_body = self.R_mount_phys @ (R_c_b @ r_cam)   # physical assembly: BEFORE de-rotation

        roll_rad = self.mavlink.current_roll_rad
        pitch_rad = self.mavlink.current_pitch_rad
        R_stab = compute_R_b_e(roll_rad, pitch_rad, 0)
        # AIM is applied AFTER de-rotation. Because it is range-aware, it is rebuilt at every frame; This is cheap because the range changes slowly.
        aim_e = self._effective_aim_deg(self._filtered_range)
        if aim_e != self._last_aim_deg:
            self._last_aim_deg = aim_e
            r_ = math.radians(aim_e)
            self.R_aim = np.array([[math.cos(r_), 0.0, math.sin(r_)],
                                   [0.0, 1.0, 0.0],
                                   [-math.sin(r_), 0.0, math.cos(r_)]])
        r_virt_body = self.R_aim @ (R_stab @ r_body)
        r_virt_cam = R_c_b_T @ r_virt_body
        p_virt_hom = self.K @ r_virt_cam

        if np.all(np.isfinite(p_virt_hom)) and p_virt_hom[2] > 1e-3:
            sx = p_virt_hom[0] / p_virt_hom[2]
            sy = p_virt_hom[1] / p_virt_hom[2]
            if math.isfinite(sx) and math.isfinite(sy):
                return sx, sy
        return obj_x, obj_y

    def _log_state(self, current_time, dt, task, system_state, target_found,
                   command_sent, queue_size, data_age_ms,
                   bbox_x, bbox_y, bbox_w, bbox_h, obj_x, obj_y, target_area,
                   stab_x, stab_y, delta_stab_x, delta_stab_y,
                   current_alt_m, roll_now, pitch_now, yaw_now,
                   raw_pixel_error_x, raw_pixel_error_y,
                   stab_pixel_error_x, stab_pixel_error_y,
                   raw_error_x_deg, raw_error_y_deg,
                   stab_error_x_deg, stab_error_y_deg,
                   raw_deriv_x, raw_deriv_y, filt_deriv_x, filt_deriv_y,
                   dz_active_x, dz_active_y, p_input_x, p_input_y,
                   integral_x, p_head, i_head, d_head, cmd_head_deg, target_head_deg,
                   integral_y, p_alt, i_alt, d_alt, cmd_alt_m, target_alt_m,
                   aw_x, aw_y,
                   rate_error=0.0, p_rate=0.0, i_rate=0.0, d_rate=0.0, heading_rate_dps=0.0,
                   cmd_head_raw_deg=0.0, cmd_alt_raw_m=0.0, sat_head=0, sat_alt=0,
                   estimated_range_m=None, m_per_deg=None,
                   control_coverage_pct=0.0, speed_error_pct=0.0,
                   p_speed=0.0, i_speed=0.0, cmd_speed_ms=None):
        elapsed = current_time - self.start_time
        coverage_w = (bbox_w / self.camera_width * 100.0) if bbox_w > 0 else 0.0
        coverage_h = (bbox_h / self.camera_height * 100.0) if bbox_h > 0 else 0.0

        dr = lambda v: f"{math.degrees(v):.2f}"

        row = [
            f"{current_time:.4f}", f"{elapsed:.3f}", f"{dt:.4f}", self.frame_counter,
            self.mavlink.current_mode, task, system_state, target_found,
            command_sent, queue_size, f"{data_age_ms:.1f}",
            bbox_x, bbox_y, f"{bbox_w:.1f}", f"{bbox_h:.1f}", f"{obj_x:.2f}", f"{obj_y:.2f}",
            f"{target_area:.1f}", f"{coverage_w:.2f}", f"{coverage_h:.2f}",
            f"{stab_x:.2f}", f"{stab_y:.2f}", f"{delta_stab_x:.2f}", f"{delta_stab_y:.2f}",
            f"{current_alt_m:.2f}", dr(roll_now), dr(pitch_now), dr(yaw_now),
            f"{raw_pixel_error_x:.2f}", f"{raw_pixel_error_y:.2f}", f"{stab_pixel_error_x:.2f}", f"{stab_pixel_error_y:.2f}",
            f"{raw_error_x_deg:.4f}", f"{raw_error_y_deg:.4f}", f"{stab_error_x_deg:.4f}", f"{stab_error_y_deg:.4f}",
            f"{raw_deriv_x:.4f}", f"{raw_deriv_y:.4f}", f"{filt_deriv_x:.4f}", f"{filt_deriv_y:.4f}",
            dz_active_x, dz_active_y, f"{p_input_x:.4f}", f"{p_input_y:.4f}",
            f"{integral_x:.6f}", f"{p_head:.6f}", f"{i_head:.6f}", f"{d_head:.6f}", f"{cmd_head_deg:.2f}", f"{target_head_deg:.2f}",
            f"{integral_y:.6f}", f"{p_alt:.6f}", f"{i_alt:.6f}", f"{d_alt:.6f}", f"{cmd_alt_m:.2f}", f"{target_alt_m:.2f}",
            aw_x, aw_y
        ]
        m = self.mavlink
        nan = float('nan')
        f = lambda v, n=2: (f"{v:.{n}f}" if v is not None and v == v else "")
        row += [
            f"{math.sqrt(target_area):.2f}" if target_area and target_area > 0 else "0.00",
            f(getattr(m, 'current_airspeed', nan)), f(getattr(m, 'current_groundspeed', nan)),
            f(getattr(m, 'current_vz', nan)), f(getattr(m, 'current_throttle', nan), 1),
            f"{rate_error:.4f}", f"{p_rate:.6f}", f"{i_rate:.6f}", f"{d_rate:.6f}", f"{heading_rate_dps:.3f}",
            f"{cmd_head_raw_deg:.4f}", f"{cmd_alt_raw_m:.4f}", sat_head, sat_alt,
            f"{self._last_aim_deg:.3f}", f"{self.mount_phys_pitch_deg:.3f}",
            f(getattr(m, 'current_lat', nan), 7), f(getattr(m, 'current_lon', nan), 7),
            f(estimated_range_m, 1), f(m_per_deg, 4), self.last_range_source,
            f(control_coverage_pct), f(speed_error_pct), f(p_speed), f(i_speed),
            f(cmd_speed_ms),
            int(task == 'Visual' and self.mavlink.current_mode == 'GUIDED'),
            int(self.lock_tracker.qualifies((bbox_x, bbox_y, bbox_w, bbox_h))
                if target_found else False),
            f"{self.lock_tracker.progress_s:.3f}", int(self.lock_tracker.acquired),
        ]
        self.logger.log(row)

    def close(self):
        if self._closed:
            return
        self._closed = True
        self._running = False
        self._set_redis_state('guid', False)
        self.mavlink.running = False
        self.redis.running = False
        self.target_telem.running = False
        try:
            self.redis.pubsub.close()
        except Exception:
            pass
        try:
            self.mavlink.master.close()
        except Exception:
            pass
        self.logger.close()

    def run(self):
        print("System active! Main thread started...")
        try:
            while self._running:
                try:
                    data = self.data_queue.get(timeout=0.05)
                except queue.Empty:
                    data = None

                current_time = time.time()
                task = self.redis.get_task()
                queue_size = self.data_queue.qsize()
                self.frame_counter += 1

                current_alt = self.mavlink.current_alt_rel
                roll_now = self.mavlink.current_roll_rad
                pitch_now = self.mavlink.current_pitch_rad
                yaw_now = self.mavlink.current_yaw_rad
                yaw_deg = math.degrees(yaw_now) % 360.0
                enabled = (task == "Visual" and
                         self.mavlink.current_mode == "GUIDED")
                telemetry_ready = (self.mavlink.attitude_received and
                             self.mavlink.position_received and
                             self.mavlink.telemetry_is_fresh())
                settling_state = self._update_handover_gate(
                    current_time, enabled, telemetry_ready, current_alt, pitch_now, yaw_deg)

                if not data:
                    # COASTING / FAILSAFE
                    self._update_lock_state(None, time.monotonic(),
                                            active=enabled and telemetry_ready)
                    if self.last_target_time > 0.0:
                        td = current_time - self.last_target_time
                        if td <= 5.0:
                            self._set_redis_state('guid', enabled and telemetry_ready)
                            cmd_sent = 0
                            if enabled and telemetry_ready:
                                if current_time - self.last_cmd_send_time >= 0.2: # 5Hz renewal
                                    gate_hold = (self.handover_gate and
                                                 self._gate_opened_t is None)
                                    hold_hdg = (self._engage_heading if gate_hold
                                                else self.last_target_heading)
                                    hold_alt = (self._engage_alt if gate_hold
                                                else self.last_target_alt)
                                    hold_speed = self.base_speed if gate_hold else self.last_speed
                                    self.mavlink.send_heading_target(hold_hdg, self.min_heading_rate if gate_hold else self.last_heading_rate)
                                    self.mavlink.send_altitude_target(hold_alt)
                                    self._send_speed_command(hold_speed)
                                    cmd_sent = 1
                                    self.last_cmd_send_time = current_time

                            self._log_state(
                                current_time, td, task, "COASTING", 0, cmd_sent, queue_size, td*1000,
                                0,0,0,0,0,0,0, 0,0,0,0, current_alt, roll_now, pitch_now, yaw_now,
                                0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0,
                                self.integral_error_x, 0,0,0,0, self.last_target_heading,
                                self.integral_error_y, 0,0,0,0, self.last_target_alt,
                                0,0
                            )
                        else:
                            self._set_redis_state('guid', False)
                            self.integral_error_x, self.integral_error_y = 0.0, 0.0
                            self.prev_derivative_x, self.prev_derivative_y = 0.0, 0.0
                            self.integral_error_rate = 0.0

                            # Failsafe: Continue at current heading and altitude
                            cmd_sent = 0
                            if enabled and telemetry_ready:
                                if current_time - self.last_cmd_send_time >= 0.5:
                                    self.mavlink.send_heading_target(yaw_deg, self.min_heading_rate)
                                    self.mavlink.send_altitude_target(current_alt)
                                    self._send_speed_command(self.last_speed)
                                    cmd_sent = 1
                                    self.last_target_heading = yaw_deg
                                    self.last_target_alt = current_alt
                                    self.last_cmd_send_time = current_time

                            self._log_state(
                                current_time, td, task, "FAILSAFE", 0, cmd_sent, queue_size, td*1000,
                                0,0,0,0,0,0,0, 0,0,0,0, current_alt, roll_now, pitch_now, yaw_now,
                                0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0,
                                0,0,0,0,0,0, 0,0,0,0,0,0, 0,0
                            )
                    else:
                        self._set_redis_state('guid', False)
                    if not enabled:
                        self._reset_speed_control()
                    continue

                # GUIDED COMMAND UPDATE: LOWER BOUND ON dt
                #The former fixed 0.001 s bound inflated the first derivative by 12-15
                #when last_time was initialized in __init__. Measured raw derivatives
                #were 1127-2817 deg/s. Use half the running median as the lower bound.
                first_frame = (self._dt_history == [])
                raw_dt = current_time - self.last_time
                self._dt_history.append(raw_dt)
                if len(self._dt_history) > 60:
                    self._dt_history.pop(0)
                med = sorted(self._dt_history)[len(self._dt_history) // 2]
                dt = max(max(0.001, 0.5 * med), min(raw_dt, 0.1))
                self.last_time = current_time

                obj_x = data['obj_x']
                obj_y = data['obj_y']
                bbox_x = data.get('bbox_x', 0)
                bbox_y = data.get('bbox_y', 0)
                bbox_w, bbox_h = data.get('bbox_w', 0.0), data.get('bbox_h', 0.0)
                target_area = data['target_area']
                data_age_ms = (current_time - data.get('timestamp', current_time)) * 1000.0
                if data_age_ms > 500.0:
                    self._update_lock_state(None, time.monotonic(),
                                            active=enabled and telemetry_ready)
                    continue
                self.last_target_time = current_time
                bbox_tuple = (bbox_x, bbox_y, bbox_w, bbox_h)
                self._set_redis_state('guid', enabled and telemetry_ready)
                self._update_lock_state(bbox_tuple, time.monotonic(),
                                        active=enabled and telemetry_ready)

                stab_x, stab_y = self.stabilize_pixel(obj_x, obj_y)

                stab_error_x_deg = math.degrees(math.atan((stab_x - self.center_x) / self.fx))
                stab_error_y_deg = math.degrees(math.atan((stab_y - self.center_y) / self.fy))

                raw_error_x_deg = math.degrees(math.atan((obj_x - self.center_x) / self.fx))
                raw_error_y_deg = math.degrees(math.atan((obj_y - self.center_y) / self.fy))

                # Derivative account (Low pass)
                alpha = 0.05
                # SUPPRESS THE FIRST-FRAME DERIVATIVE
                #With prev_error initially zero, the first sample appeared to be a step
                #and produced derivatives of 1127-2817 deg/s. The dt floor cannot help
                #before a history exists, so skip derivative calculation on that frame.
                if first_frame:
                    raw_deriv_x = raw_deriv_y = 0.0
                else:
                    raw_deriv_x = (stab_error_x_deg - self.prev_error_x) / dt
                    raw_deriv_y = (stab_error_y_deg - self.prev_error_y) / dt
                deriv_x = (alpha * raw_deriv_x) + ((1.0 - alpha) * self.prev_derivative_x)
                deriv_y = (alpha * raw_deriv_y) + ((1.0 - alpha) * self.prev_derivative_y)

                # Integral
                if task == "Visual" and self.mavlink.current_mode == "GUIDED":
                    if abs(stab_error_x_deg) < self.deadzone_deg: self.integral_error_x *= 0.99  # noqa
                    else: self.integral_error_x += stab_error_x_deg * dt
                    self.integral_error_x = self.clamp(self.integral_error_x, -10.0, 10.0)

                    if abs(stab_error_y_deg) < self.deadzone_deg: self.integral_error_y *= 0.99
                    else: self.integral_error_y += stab_error_y_deg * dt
                    self.integral_error_y = self.clamp(self.integral_error_y, -10.0, 10.0)
                else:
                    self.integral_error_x, self.integral_error_y = 0.0, 0.0

                self.prev_error_x, self.prev_error_y = stab_error_x_deg, stab_error_y_deg
                self.prev_derivative_x, self.prev_derivative_y = deriv_x, deriv_y

                # P3: DETECTION-QUALITY DEADZONE (2026-07-29)
                #The physical width of a fixed angular deadzone grows with range:
                #0.4 degrees is 0.08 m at 12 m and 0.70 m at 100 m. Measured noise was
                #0.019 degrees below 25 m and 0.164 degrees at 50-100 m. Near the target,
                #a 0.4-degree deadzone is about twenty times the noise floor. Narrow it
                #for large, reliably detected boxes and retain it for small boxes.
                dz_now = self.deadzone_deg
                if self.quality_deadzone and target_area and math.sqrt(target_area) >= 12.0:
                    dz_now = self.narrow_deadzone_deg
                dz_x = 1 if abs(stab_error_x_deg) < dz_now else 0
                dz_y = 1 if abs(stab_error_y_deg) < dz_now else 0
                p_x = 0 if dz_x else stab_error_x_deg - math.copysign(dz_now, stab_error_x_deg)
                p_y = 0 if dz_y else stab_error_y_deg - math.copysign(dz_now, stab_error_y_deg)

                # --- 1. HEADING CONTROL ---
                p_term_heading = p_x * self.Kp_heading
                i_term_heading = self.integral_error_x * self.Ki_heading
                d_term_heading = deriv_x * self.Kd_heading

                cmd_head_deg = p_term_heading + i_term_heading + d_term_heading
                cmd_head_raw_deg = cmd_head_deg  # raw PID output before clamp
                cmd_head_deg = self.clamp(cmd_head_deg, -self.max_heading_change_deg, self.max_heading_change_deg)
                sat_head = 1 if abs(cmd_head_raw_deg) > self.max_heading_change_deg else 0
                # Target angle (normalized from 0-360)
                target_heading = (yaw_deg + cmd_head_deg) % 360.0

                # --- HEADING RATE PID (rotation speed control) ---
                rate_error = abs(p_x)  # Magnitude of angular error (applied to deadzone)

                # Derivative (low-pass filtered)
                raw_deriv_rate = (rate_error - self.prev_error_rate) / dt
                deriv_rate = (alpha * raw_deriv_rate) + ((1.0 - alpha) * self.prev_derivative_rate)

                # Integral
                if task == "Visual" and self.mavlink.current_mode == "GUIDED":
                    self.integral_error_rate += rate_error * dt
                    self.integral_error_rate = self.clamp(self.integral_error_rate, 0, 5.0)
                else:
                    self.integral_error_rate = 0.0

                # PID output
                p_term_rate = rate_error * self.Kp_rate
                i_term_rate = self.integral_error_rate * self.Ki_rate
                d_term_rate = deriv_rate * self.Kd_rate

                heading_rate = p_term_rate + i_term_rate + d_term_rate
                heading_rate = self.clamp(heading_rate, self.min_heading_rate, self.max_heading_rate)

                # Minimum rate if in Deadzone
                if dz_x:
                    heading_rate = self.min_heading_rate

                self.prev_error_rate = rate_error
                self.prev_derivative_rate = deriv_rate

                # --- 2. ALTITUDE CHECK --- If the target is lower on the Y axis (p_y positive), altitude MUST BE REDUCED
                range_est = self.estimated_range(target_area, dt)
                if self.vertical_mode == 'range_m' and range_est:
                    # METER equivalent of 1 degree angular error (range dependent)
                    m_per_deg = range_est * math.tan(math.radians(1.0))
                    p_term_alt = p_y * m_per_deg * self.Kp_h
                    i_term_alt = self.integral_error_y * m_per_deg * self.Ki_h
                    d_term_alt = deriv_y * m_per_deg * self.Kd_h
                else:
                    m_per_deg = float('nan')
                    p_term_alt = p_y * self.Kp_alt
                    i_term_alt = self.integral_error_y * self.Ki_alt
                    d_term_alt = deriv_y * self.Kd_alt

                cmd_alt_m = -1 * (p_term_alt + i_term_alt + d_term_alt)
                cmd_alt_raw_m = cmd_alt_m  # raw PID output before clamp
                # SOFT START: It means giving the exact command as soon as the gate is opened and applying the error accumulated in the frame as a STEP throughout the placement. The command ramps from 0 to 1.
                _ramp = 1.0
                if enabled and self.handover_gate and self._gate_opened_t is None:
                    _ramp = 0.0
                elif enabled and not self.handover_gate and self._gate_opened_t is None:
                    self._gate_opened_t = current_time
                if self._gate_opened_t is not None and self.soft_start_s > 0:
                    _ramp = min(1.0, (current_time - self._gate_opened_t)
                                / self.soft_start_s)
                cmd_alt_m *= _ramp

                # P1: GROUND CLEARANCE PROTECTION (2026-07-29)
                #An initial simulation-based change increased the descent-command limit
                #from -5 to -10 m using the measured relation vertical speed =
                #0.28 * command. That ratio depends on the aircraft model, TECS and
                #climb/descent performance. The -5 m limit was validated in real flight,
                #so the controller retains that limit.
                #
                #The platform-independent protection reduces descent authority near the
                #ground, to -1 m at 16 m AGL and zero at 15 m AGL. It never increases
                #the permitted descent command.
                if self.agl_scaled_descent:
                    alt_limit = -min(max(current_alt - 15.0, 1.0),
                                     abs(self.min_alt_change_m))
                else:
                    alt_limit = self.min_alt_change_m
                cmd_alt_m = self.clamp(cmd_alt_m, alt_limit, self.max_alt_change_m)
                sat_alt = 1 if (cmd_alt_raw_m > self.max_alt_change_m or
                                cmd_alt_raw_m < alt_limit) else 0

                target_alt = current_alt + cmd_alt_m
                # Ground safety (prevent from going below 10 meters)
                if target_alt < 10.0:
                    target_alt = 10.0

                # --- 3. AIRSPEED CONTROL ---
                if enabled and self.speed_control_enabled:
                    first_speed_sample = not self._speed_seeded
                    (cmd_speed, control_coverage, speed_error,
                     p_term_speed, i_term_speed) = self._speed_control(
                        bbox_w, bbox_h, dt, first_sample=first_speed_sample,
                        bbox_x=bbox_x, bbox_y=bbox_y, range_est=range_est)
                else:
                    self._reset_speed_control()
                    cmd_speed = self.base_speed
                    control_coverage = max(
                        bbox_w / self.camera_width * 100.0,
                        bbox_h / self.camera_height * 100.0)
                    speed_error = self.target_coverage_pct - control_coverage
                    p_term_speed = i_term_speed = 0.0
                if self.fixed_speed is not None:
                    self._seed_speed()
                    if self._fixed_speed_ms is not None:
                        cmd_speed = self.clamp(float(self._fixed_speed_ms),
                                               self.min_speed, self.max_speed)
                        self.last_speed = cmd_speed

                # Do not read target altitude in this camera-only guidance process.
                #To keep the evaluation independent of target ground truth, collect
                #reference position and altitude through tools/ground_truth_logger.py.
                #Combine the logs only during analysis with tools/pid_plot.py --ground-truth.

                # Anti-windup
                aw_x, aw_y = 0, 0
                if abs(cmd_head_deg) >= self.max_heading_change_deg:
                    self.integral_error_x *= 0.9; aw_x = 1
                if cmd_alt_m >= self.max_alt_change_m or cmd_alt_m <= alt_limit:
                    self.integral_error_y *= 0.9; aw_y = 1

                command_sent = 0
                # --- handover gate + BOOTSTRAP PROTECTION ---
                if enabled and (settling_state or not telemetry_ready):
                    # In aircraft handover transient (or no telemetry yet): RUN PID, command current status. Thus, guidance does not fight against an oscillation that it does not produce.
                    if self.fixed_speed is None:
                        # The coverage PID is also kept at base speed in the handover gate;  The closing command starts with slew after the gate is opened.
                        cmd_speed = self.base_speed
                        self.last_speed = self.base_speed
                        self.integral_error_speed = 0.0
                        self.filtered_coverage = control_coverage
                        self.prev_error_speed = self.target_coverage_pct - control_coverage
                        self.prev_derivative_speed = 0.0
                    if current_time - self.last_cmd_send_time >= 0.1 and telemetry_ready:
                        hdg = (self._fixed_heading_deg if self._fixed_heading_deg is not None
                               else self._engage_heading)
                        self.mavlink.send_heading_target(hdg, self.min_heading_rate)
                        self._send_speed_command(cmd_speed)
                        self.mavlink.send_altitude_target(self._engage_alt or current_alt)
                        self.last_cmd_send_time = current_time
                    # Consciously reset status to start clean when gate opens
                    self.integral_error_x = self.integral_error_y = 0.0
                    self.prev_error_x, self.prev_error_y = stab_error_x_deg, stab_error_y_deg
                    self.prev_derivative_x = self.prev_derivative_y = 0.0
                    self.prev_error_rate = 0.0
                    self.prev_derivative_rate = 0.0
                    self.last_target_alt = self._engage_alt or current_alt
                    self._log_state(
                        current_time, dt, task,
                        "WAITING_FOR_TELEMETRY" if not telemetry_ready else "HANDOVER_SETTLING",
                        1, 0, queue_size, data_age_ms,
                        bbox_x, bbox_y, bbox_w, bbox_h, obj_x, obj_y, target_area,
                        stab_x, stab_y, stab_x - obj_x, stab_y - obj_y,
                        current_alt, roll_now, pitch_now, yaw_now,
                        raw_error_x_deg, raw_error_y_deg, stab_error_x_deg, stab_error_y_deg,
                        raw_error_x_deg, raw_error_y_deg, stab_error_x_deg, stab_error_y_deg,
                        0, 0, 0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, self.last_target_heading,
                        0, 0, 0, 0, 0, current_alt, 0, 0)
                    continue
                if current_time - self.last_cmd_send_time >= 0.1:
                    if enabled:
                        if self.fixed_heading is not None:
                            # HORIZONTAL EXTERNAL FROZEN (test isolation): the target_heading calculated by the homing is NOT sent.
                            if self._fixed_heading_deg is None:
                                self._fixed_heading_deg = math.degrees(yaw_now) % 360
                                print(f"[test] heading frozen: "
                                      f"{self._fixed_heading_deg:.1f} deg")
                            self.mavlink.send_heading_target(
                                self._fixed_heading_deg, self.min_heading_rate)
                        else:
                            self.mavlink.send_heading_target(target_heading, heading_rate)
                        self._send_speed_command(cmd_speed)
                        self.mavlink.send_altitude_target(target_alt)
                        command_sent = 1

                        logging.info(
                            f"[{task}] AUTONOMOUS: Direction: {target_heading:.1f}° | "
                            f"Pitch: {math.degrees(pitch_now):+.2f}° | "
                            f"Raw y: {obj_y:.1f}px | Stab y: {stab_y:.1f}px | "
                            f"Raw Altitude: {cmd_alt_raw_m:+.2f}m | altitude: {target_alt:.1f}m")

                        # print(f"[{task}] AUTONOMOUS: TargetDirection: {target_heading:.1f}° | Rate: {heading_rate:.1f}°/s | altitude: {target_alt:.1f}m")
                    else:
                        print(f"[{task}] STANDBY - Hdf Direction: {target_heading:.1f}° | "
                              f"Pitch: {math.degrees(pitch_now):+.2f}° | Raw y: {obj_y:.1f}px | "
                              f"Stab y: {stab_y:.1f}px | Raw altitude change: {cmd_alt_raw_m:+.2f}m | "
                              f"altitude: {target_alt:.1f}m")

                    self.last_cmd_send_time = current_time

                self.last_target_heading = target_heading
                self.last_target_alt = target_alt
                self.last_heading_rate = heading_rate

                self._log_state(
                    current_time, dt, task, "TRACKING", 1, command_sent, queue_size, data_age_ms,
                    bbox_x, bbox_y, bbox_w, bbox_h, obj_x, obj_y, target_area,
                    stab_x, stab_y, stab_x - obj_x, stab_y - obj_y,
                    current_alt, roll_now, pitch_now, yaw_now,
                    raw_error_x_deg, raw_error_y_deg,
                    stab_error_x_deg, stab_error_y_deg,
                    raw_error_x_deg, raw_error_y_deg,
                    stab_error_x_deg, stab_error_y_deg,
                    raw_deriv_x, raw_deriv_y, deriv_x, deriv_y,
                    dz_x, dz_y, p_x, p_y,
                    self.integral_error_x, p_term_heading, i_term_heading, d_term_heading, cmd_head_deg, target_heading,
                    self.integral_error_y, p_term_alt, i_term_alt, d_term_alt, cmd_alt_m, target_alt,
                    aw_x, aw_y,
                    rate_error=rate_error, p_rate=p_term_rate, i_rate=i_term_rate,
                    d_rate=d_term_rate, heading_rate_dps=heading_rate,
                    cmd_head_raw_deg=cmd_head_raw_deg, cmd_alt_raw_m=cmd_alt_raw_m,
                    sat_head=sat_head, sat_alt=sat_alt,
                    estimated_range_m=range_est, m_per_deg=m_per_deg,
                    control_coverage_pct=control_coverage,
                    speed_error_pct=speed_error,
                    p_speed=p_term_speed, i_speed=i_term_speed,
                    cmd_speed_ms=cmd_speed
                )

        except KeyboardInterrupt:
            print("\nShutting down...")
        finally:
            self.close()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Video guidance (heading + altitude)')
    parser.add_argument('--connect', default=DEFAULT_CONNECTION,
                        help=f'MAVLink connection (default: {DEFAULT_CONNECTION})')
    parser.add_argument('--sysid', type=int, default=DEFAULT_SYSID,
                        help=f'Expected vehicle SysID (default: {DEFAULT_SYSID})')
    parser.add_argument('--camera-profile', choices=('sim', 'real'), default='sim',
                        help="Camera calibration: sim=Erenimbus Gazebo "
                             "(1025/569, 4543/4539, 0/+2.8); real=in flight "
                             "validated profile (1091/633, 4515/4510, -1/-5)")
    parser.add_argument('--mount-pitch', type=float, default=None,
                        help='[Legacy compatibility] Total camera pitch compensation (degrees): '
                             'mount_phys + represents the sum of current, current = value - mount_phys '
                             'It is solved as . If not given, the selected camera profile is used.')
    parser.add_argument('--mount-phys-pitch', type=float, default=None,
                        help='Physical pitch mounting angle between camera and autopilot '
                             '(degrees, default 0.0 means parallel to the body). '
                             'For the reference aircraft use about -1.0. Applied in the body frame '
                             'BEFORE derotation.')
    parser.add_argument('--fixed-heading', default=None,
                        help="TEST ISOLATION: freeze horizontal control. Heading in degrees "
                             "or 'auto' (initial yaw is frozen). "
                             "It is used when tuning the vertical axis.")
    parser.add_argument('--fixed-speed', default=None,
                        help="TEST ISOLATION: keep air speed constant (m/s) or "
                             "'auto' (initialized from the current cruising airspeed). "
                             "'auto' is recommended: commanding 18 m/s while cruising at 19.6 m/s in AUTO "
                             "increases handover oscillation. Envelope: 9-22 m/s.")
    parser.add_argument('--speed-control', dest='speed_control', action='store_true',
                        default=None,
                        help="Turn on coverage based airspeed PID "
                             "(default for the sim profile)")
    parser.add_argument('--disable-speed-control', dest='speed_control', action='store_false',
                        help="Disable airspeed PID and send a constant "
                             "airspeed command as in the reference controller")
    parser.add_argument('--vertical-mode', choices=('angle', 'range_m'), default=None,
                        help="Vertical axis control law: 'angle' (old, fixed "
                             "Kp_alt m/degree) or 'range_m' (dh=R*tan(error); "
                             "range is estimated from the camera)")
    parser.add_argument('--range-source', choices=('telemetry', 'bbox', 'auto'),
                        default=None,
                        help="The range source of the altitude controller. 'telemetry' "
                             "(default) = 1 Hz 3D location from server; 'bbox' = "
                             "sqrt(bbox area) from camera; 'auto' = bbox if no telemetry")
    parser.add_argument('--handover-gate', dest='handover_gate', action='store_true', default=None,
                        help='AUTO->GUIDED handover smoothing gate (DEFAULT ON): '
                             'PID does not work until the aircraft settles, altitude is locked')
    parser.add_argument('--disable-handover-gate', dest='handover_gate', action='store_false',
                        help='disable the handover gate when required on real hardware')
    parser.add_argument('--handover-wait', type=float, default=None,
                        help='handover gate minimum waiting time (s, default 2.0)')
    parser.add_argument('--agl-descent', dest='agl_descent', action='store_true', default=None,
                        help='P1 (DEFAULT ON): ground safety — descent authorization '
                             'decreases near 15 m AGL (maximum permitted descent remains -5 m)')
    parser.add_argument('--disable-agl-descent', dest='agl_descent', action='store_false',
                        help='disable P1 and restore a fixed -5 m descent clamp')
    parser.add_argument('--quality-deadzone', dest='quality_deadzone', action='store_true', default=None,
                        help='P3 (DEFAULT ON): deadzone 0.4 -> 0.15 degrees when bbox is large')
    parser.add_argument('--disable-quality-deadzone', dest='quality_deadzone', action='store_false',
                        help='disable P3')
    parser.add_argument('--aim-pitch', type=float, default=None,
                        help='Angle-of-attack compensation (degrees, default -6.0). Relative to the horizon; '
                             'applied AFTER derotation.')
    args = parser.parse_args()

    sh = args.fixed_heading
    if sh is not None and sh != 'auto':
        sh = float(sh)
    shz = args.fixed_speed
    if shz is not None and shz != 'auto':
        shz = float(shz)
    guidance = AutopilotController(connection_str=args.connect, target_sysid=args.sysid,
                                mount_pitch_deg=args.mount_pitch,
                                mount_phys_pitch_deg=args.mount_phys_pitch,
                                aim_pitch_deg=args.aim_pitch,
                                fixed_heading=sh, fixed_speed=shz,
                                camera_profile=args.camera_profile,
                                speed_control_enabled=args.speed_control)
    if args.vertical_mode:
        guidance.vertical_mode = args.vertical_mode
    if args.range_source:
        guidance.range_source = args.range_source
    print(f"[teva] range source = {guidance.range_source}")
    if args.handover_gate is not None:
        guidance.handover_gate = args.handover_gate
    if args.handover_wait is not None:
        guidance.engage_wait_s = args.handover_wait
    print(f"[handover] gate={'ENABLED' if guidance.handover_gate else 'DISABLED'}"
          + (f"  standby={guidance.engage_wait_s:.1f} s" if guidance.handover_gate else ""))
    if args.agl_descent is not None:
        guidance.agl_scaled_descent = args.agl_descent
    if args.quality_deadzone is not None:
        guidance.quality_deadzone = args.quality_deadzone
    print(f"[vertical] setup_mode={guidance.vertical_mode}" +
          (f"  Kp_h={guidance.Kp_h} Kd_h={guidance.Kd_h} k={guidance.range_k}"
           if guidance.vertical_mode == 'range_m' else f"  Kp_alt={guidance.Kp_alt}"))
    # Write the running setting next to CSV: which setting it was run with will not be a matter of discussion later (autoresearch: experiments cannot be compared if the setting is not documented).
    try:
        import json as _json
        _cfg = {
            'csv': guidance.logger.log_filename,
            'camera_profile': guidance.camera_profile,
            'speed_control_enabled': guidance.speed_control_enabled,
            'aim_pitch_deg': guidance.aim_pitch_deg,
            'mount_phys_pitch_deg': guidance.mount_phys_pitch_deg,
            'fixed_heading': sh, 'fixed_speed': shz,
            'vertical_mode': guidance.vertical_mode, 'Kp_h': guidance.Kp_h, 'Kd_h': guidance.Kd_h,
            'agl_scaled_descent': guidance.agl_scaled_descent,
            'quality_deadzone': guidance.quality_deadzone,
            'narrow_deadzone_deg': guidance.narrow_deadzone_deg,
            'handover_gate': guidance.handover_gate,
            'engage_wait_s': guidance.engage_wait_s,
            'soft_start_s': guidance.soft_start_s,
            'range_k': guidance.range_k, 'range_source': guidance.range_source,
            'Kp_heading': guidance.Kp_heading, 'Ki_heading': guidance.Ki_heading,
            'Kd_heading': guidance.Kd_heading,
            'Kp_alt': guidance.Kp_alt, 'Ki_alt': guidance.Ki_alt, 'Kd_alt': guidance.Kd_alt,
            'Kp_rate': guidance.Kp_rate, 'Kd_rate': guidance.Kd_rate,
            'max_heading_rate': guidance.max_heading_rate,
            'Kp_speed': guidance.Kp_speed, 'Ki_speed': guidance.Ki_speed,
            'speed_slew_rate': guidance.speed_slew_rate,
            'target_coverage_pct': guidance.target_coverage_pct,
            'deadzone_deg': guidance.deadzone_deg,
            'max_alt_change_m': guidance.max_alt_change_m,
            'min_alt_change_m': guidance.min_alt_change_m,
            'max_heading_change_deg': guidance.max_heading_change_deg,
            'camera': {'w': guidance.camera_width, 'h': guidance.camera_height,
                       'fx': guidance.fx, 'fy': guidance.fy,
                       'cx': guidance.center_x, 'cy': guidance.center_y},
        }
        with open(guidance.logger.log_filename + '.config.json', 'w') as _f:
            _json.dump(_cfg, _f, indent=2, ensure_ascii=False)
    except Exception as _e:
        print(f'[warning] failed to write run setting: {_e}')
    guidance.run()
