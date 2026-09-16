import sys
sys.path.insert(0, '/home/tzi4/guidance')
import numpy as np
import math
import redis
import datetime
import os
import atexit
from collections import deque
from statistics import mean
from mavlinkHandler import MAVLinkHandlerPymavlink as MAVLinkHandler
import json
import time 
import threading
import logging
import shutil
from pymavlink import mavutil

# ─── CONSTANTS (config values ​​used in tzi.py) ───
RESOLUTION_W = 1280
RESOLUTION_H = 720
CAMERA_FOCAL_LENGTH = 4711.91  # 467.7 * (1280/640) — same FOV, scaled or 4711.91 mi?
MIN_ALTITUDE = 10
MAX_ALTITUDE = 200
MAX_DELTA_HEADING = 10
MAX_DELTA_ALTITUDE = 5
UAV_PORT = '14553'


# Camera->Body rotation (example assuming camera z forward, x right, y down)
R_c_b = np.array([[0,0,1],
                  [1,0,0],
                  [0,1,0]], dtype=float)
R_c_b_T = R_c_b.T

def compute_R_b_e(roll, pitch, yaw):
    cr,sr = np.cos(roll), np.sin(roll)
    cp,sp = np.cos(pitch), np.sin(pitch)
    cy,sy = np.cos(yaw), np.sin(yaw)
    return np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp   , cp*sr           , cp*cr           ]
    ])

# ----------------- Direct Control Commander Thread -----------------
class TestCommander(threading.Thread):
    """
      - Heading:  MAV_CMD_GUIDED_CHANGE_HEADING  (43002)
      - Altitude: MAV_CMD_GUIDED_CHANGE_ALTITUDE (43001)
      - Airspeed: MAV_CMD_DO_CHANGE_SPEED        (178)
    """
    def __init__(self, mav_handler, rate_hz=5):
        super().__init__(daemon=True)
        self.mav_handler = mav_handler         # MAVLinkHandlerPymavlink instance
        self.master = mav_handler.master       # pymavlink master (to send messages)
        self.rate = rate_hz
        self.running = True
        self.lock = threading.Lock()

        # Here is the message VFR_HUD (for airspeed)
        self.mav_handler.request_message_interval(
            [mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD], 10  # 10 Hz is enough
        )

        # Null values ​​to avoid getting AttributeError
        self.target_heading_deg = 0.0    # degrees (0-360)
        self.target_alt_m = 100.0        # meters (AMSL)
        self.target_airspeed_ms = 15.0   # m/s (target airspeed)
        self.active = False

        # Latest telemetry in the cache
        self.last_att = None       # (pitch_deg, roll_deg, yaw_deg)
        self.last_loc = None       # (lat, lon, alt_m)
        self.last_airspeed = None  # m/s
        self.last_mode = None      # flight mode string (ex: 'GUIDED', 'AUTO')

    def update(self, heading_deg, alt_m, airspeed_ms):
        """Thread-safe target update."""
        with self.lock:
            self.target_heading_deg = float(heading_deg)
            self.target_alt_m = float(alt_m)
            self.target_airspeed_ms = float(airspeed_ms)
            self.active = True

    def _send_heading(self, heading_deg):
        """
        param1: 0 = course over ground, 1 = raw magnetic heading param2: heading (values, 0-360) param3: heading rate (deg/s, must be greater than 0!)
        """
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            43002,  # MAV_CMD_GUIDED_CHANGE_HEADING
            0,      # confirmation
            1,      # param1: 1 = raw magnetic heading (nose direction)
            heading_deg,  # param2: target heading (degree)
            40,     # param3: heading rate (deg/s) - If 0, heading will not change!
            0, 0, 0, 0  # param4-7 (unused)
        )

    def _send_altitude(self, alt_m):
        """
        param3: Climb rate (m/s)
        param7: Desired altitude (AMSL)
        """
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            43001,  # MAV_CMD_GUIDED_CHANGE_ALTITUDE
            0,      # confirmation
            0,      # param1
            0,      # param2
            0,      # param3: climb rate (m/s), 0 = default
            0, 0, 0,  # param4-6 (unused)
            alt_m   # param7: desired altitude (AMSL)
        )

    def _send_airspeed(self, airspeed_ms):
        """MAV_CMD_DO_CHANGE_SPEED (178) — Standard MAVLink speed command.         Only allow airspeed command when in GUIDED mode.         param1: SPEED_TYPE (0 = airspeed, 1 = groundspeed) param2: target speed (m/s) (-1 = no change, -2 = default) param3: throttle (%) (-1 = no change, -2 = default)
        """
        if self.last_mode != 'GUIDED':
            return
            
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            178,            # MAV_CMD_DO_CHANGE_SPEED
            0,              # confirmation
            0,              # param1: 0 = airspeed (SPEED_TYPE)
            airspeed_ms,    # param2: target airspeed (m/s)
            -1,             # param3: throttle (-1 = no change)
            0, 0, 0, 0     # param4-7 (unused)
        )

    def _read_telemetry(self):
        """Read all telemetry from pymavlink internal cache (non-blocking).
                 NOTE: recv_match IS NOT USED! All messages are cached by the MAVLink reader thread, we only read from the cache.
        """
        # Read attitude from cache
        att_msg = self.mav_handler.master.messages.get('ATTITUDE', None)
        if att_msg:
            self.last_att = (math.degrees(att_msg.pitch), math.degrees(att_msg.roll), math.degrees(att_msg.yaw))
        
        # Read location from cache
        loc_msg = self.mav_handler.master.messages.get('GLOBAL_POSITION_INT', None)
        if loc_msg:
            self.last_loc = (loc_msg.lat/1e7, loc_msg.lon/1e7, loc_msg.alt/1000.0)
        
        # Airspeed — read from pymavlink internal cache (no race condition!)
        vfr = self.master.messages.get('VFR_HUD', None)
        if vfr:
            self.last_airspeed = vfr.airspeed
            
        # Flight Mode
        hb = self.master.messages.get('HEARTBEAT', None)
        if hb:
            self.last_mode = mavutil.mode_string_v10(hb)

    def run(self):
        period = 1.0 / self.rate
        while self.running:
            if self.active:
                # 1) Read telemetry
                self._read_telemetry()
                
                # 2) Get targets
                with self.lock:
                    hdg = self.target_heading_deg
                    alt = self.target_alt_m
                    spd = self.target_airspeed_ms
                
                # SENDER
                hdg = 0.0
                alt = 40.0
                
                self._send_heading(hdg)
                self._send_altitude(alt)
                self._send_airspeed(spd)
                
                # 4) Head to the terminal
                if self.last_att and self.last_loc:
                    curr_pitch, curr_roll, curr_yaw = self.last_att
                    _, _, curr_alt = self.last_loc
                    curr_as = self.last_airspeed if self.last_airspeed else 0
                
            time.sleep(period)

    def stop(self):
        self.running = False


class System():
    def __init__(self):
        self.init_logger()
        self.r = redis.Redis(host='localhost', port=6379, db=0)
        self.logger.debug('Redis connection established.')
        self.p = self.r.pubsub()
        self.p.subscribe('tracker_bbox')


        # VISUAL
        self.W = RESOLUTION_W
        self.H = RESOLUTION_H
        self.center_x = self.W / 2
        self.center_y = self.H / 2

        # INTRINSICS
        f_oc = CAMERA_FOCAL_LENGTH
        self.fx = f_oc  # Horizontal focal length in pixels
        self.fy = f_oc  # Vertical frame length (pixels) — Uses Gazebo frame pixels
        self.K = np.array([
            [f_oc, 0, self.center_x],
            [0, f_oc, self.center_y],
            [0, 0, 1]
        ])
        self.logger.debug(f"Camera Intrinsic Matrix K:\n{self.K}")
        self.logger.debug(f"fx={self.fx:.1f}, fy={self.fy:.1f}")
        self.K_inv = np.linalg.inv(self.K)
        self.logger.debug(f"Inverse K Matrix:\n{self.K_inv}")

        # GUIDANCE
        self.MIN_ALTITUDE = MIN_ALTITUDE
        self.MAX_ALTITUDE = MAX_ALTITUDE

        self.mavlink_handler = MAVLinkHandler(f'127.0.0.1:{UAV_PORT}')

        self.logger.debug('Connected to the aircraft.')
        self.r.set('guid','False')

        # MAVLink reader thread — keeps the cache updated by constantly making recv_match
        self._mavlink_reader_thread = threading.Thread(
            target=self._mavlink_reader, daemon=True, name="MAVLinkReader")
        self._mavlink_reader_thread.start()
        self.logger.debug('MAVLink reader thread started.')

        # Exit handler
        atexit.register(self.exit_handler)

    def _mavlink_reader(self):
        """By constantly making recv_match, pymavlink keeps its internal cache updated.         In this way, other threads can read the non-blocking cache."""
        while True:
            try:
                self.mavlink_handler.master.recv_match(blocking=True, timeout=0.1)
            except Exception:
                time.sleep(0.01)

    def exit_handler(self):
        self.mavlink_handler.master.close()
        self.logger.debug('Connection closed.')


    def init_logger(self):
        # Customcustom logger in order to log to both console and file
        self.logger = logging.Logger('tzi')
        # Set the log level
        self.logger.setLevel(logging.DEBUG)
        # Create handlers
        c_handler = logging.StreamHandler()
        
        log_file_path = 'tzi_final_guidance.log'
        old_logs_dir = "Logs"
        
        if not os.path.exists(old_logs_dir):
            os.makedirs(old_logs_dir)
            
        if os.path.exists(log_file_path):
            # Generate a unique name for the log file in the old logs directory
            timestamp = datetime.datetime.now()
            new_log_file_name = f"tzi_guidance_{timestamp}.log"
            new_log_file_path = os.path.join(old_logs_dir, new_log_file_name)
            # Move the log file to the old logs directory
            shutil.move(log_file_path, new_log_file_path)
            
        f_handler = logging.FileHandler(log_file_path)
        # Set levels for handlers
        c_handler.setLevel(logging.DEBUG)
        f_handler.setLevel(logging.DEBUG)

        # Create formatters and add it to handlers
        c_format = logging.Formatter('%(message)s')
        f_format = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
        c_handler.setFormatter(c_format)
        f_handler.setFormatter(f_format)

        # Add handlers to the self.logger
        self.logger.addHandler(c_handler)
        self.logger.addHandler(f_handler)

class tziGuidance(System):
    def __init__(self):
        super().__init__()

        self.last_message_time = None  # To track the last message time for mode switching

        # --- Thread-safe repository for the latest bbox ---
        self.latest_bbox = None
        self.latest_bbox_time = None
        self.bbox_lock = threading.Lock()
        
        # Initial Values
        self.test_heading_deg = 0     # degrees (0-360, magnetic heading)
        self.test_alt_m = 50          # metre (AMSL)
        self.test_airspeed_ms = 15.0  # m/s (target airspeed)

        
        # ANGULAR ERROR CONVERSION FOR CAMERA-INDEPENDENT PID/PD
        #Compute angle errors from atan(pixel_error / fx), expressed in degrees.
        #Gains are derived from tzi.py's reference tuning at 640x480 and f = 467.7:
        #  factor = f_ref * pi/180 = 467.7 * 0.01745 = 8.163.
        #  Kp_angular = Kp_pixel * 8.163.
        #  Kd_angular = Kd_pixel * 8.163.
        #This avoids retuning solely because resolution or focal length changes.

        # AIRSPEED PID — angular size (degrees) Proportional scaling: throttle range [50,127]=77 → airspeed range [10,22.8]=12.8 (scale=0.1662) Original angular K values: Kp=38.37, Ki=3.67, Kd=4.90
        self.airspeed_kp = 6.377   # 38.37 × 0.1662
        self.airspeed_ki = 0.610   # 3.67  × 0.1662
        self.airspeed_kd = 0.814   # 4.90  × 0.1662
        self.airspeed_integral = 0.0
        self.airspeed_prev_error = 0.0
        # Target: Middle of 52m-73m range (~62.5m)
        self.airspeed_target_angular_deg = 0.505   # (0.590+0.420)/2 = midpoint
        self.airspeed_deadzone_deg = 0.085         # (0.590-0.420)/2 = radius
        self.airspeed_integral_min = -1.66  # -10.0 × 0.1662
        self.airspeed_integral_max = 9.14   # 55.0 × 0.1662
        self.airspeed_base = 15.0  # Initial airspeed value (m/s)
        self.airspeed_min = 10.0
        self.airspeed_max = 22.8
        self.airspeed_max_accel = 2.0  # Accelerate/decelerate at most 2.0 m/s per second (Slew Rate)
        self.last_commanded_airspeed = self.airspeed_base
        
        # Clamp limits
        self.max_delta_heading = MAX_DELTA_HEADING
        self.max_delta_altitude = MAX_DELTA_ALTITUDE

        # Soft deadzone — angular (degrees), camera independent
        self.deadzone_deg = 0.78  # Same as gimbal_roll_pitch.py

        # HEADING PD — tzi.py in angular (degrees): Kp = 10/320 = 0.03125 deg/px, Kd = 0.02 deg·s/px Conversion: ×8.163
        self.heading_kp = 0.255   # 0.03125 × 8.163
        self.heading_kd = 0.163   # 0.02   × 8.163
        self.heading_prev_error = 0.0

        # ALTITUDE PD — tzi.py in angular (degrees): Kp = 5/240 = 0.02083 m/px, Kd = 0.01 m·s/px Conversion: ×8.163
        self.altitude_kp = 0.170   # 0.02083 × 8.163
        self.altitude_kd = 0.0816  # 0.01   × 8.163
        self.altitude_prev_error = 0.0

        self.last_pid_time = None
        
        # Initialize TestCommander (Direct Control)
        self.cmd_thread = TestCommander(self.mavlink_handler, rate_hz=5)
        # NOTE: update() is not called here — commander passive fails until the first bbox arrives. This way the plane will not climb to 50m before the bbox arrives
        self.cmd_thread.start()



    def guide_aircraft(self, bbox, current_time): #bbox: x1,y1,w,h
        object_x,object_y = bbox[0] + (bbox[2] / 2), bbox[1] + (bbox[3] / 2)
        
        # Calculate r_cam using Back-Projection
        if self.K_inv is not None:
            # p_raw = [u, v, 1]
            p_raw = np.array([object_x, object_y, 1.0])
            r_cam = self.K_inv @ p_raw
            
            # Camera -> Body
            r_body = R_c_b @ r_cam
            
            # Stabilization (Body -> Virtual Body) Read ATTITUDE from Cache (non-blocking). It comes in radians.
            att_msg = self.mavlink_handler.master.messages.get('ATTITUDE', None)
            if att_msg:
                roll_rad = att_msg.roll
                pitch_rad = att_msg.pitch
                yaw_rad = att_msg.yaw
            else:
                roll_rad, pitch_rad, yaw_rad = 0, 0, 0

            
            # Calculate R_stab (with Yaw = 0, compute_R_b_e is called)
            R_stab = compute_R_b_e(roll_rad, pitch_rad, 0)
            
            # r_virt_body = R_stab * r_body
            r_virt_body = R_stab @ r_body
            
            # Projection to Virtual Pixel (Re-Projection)
            r_virt_cam = R_c_b_T @ r_virt_body
            
            # p_virt_hom = K * r_virt_cam
            p_virt_hom = self.K @ r_virt_cam
            
            # Normalize Homogeneous -> Cartesian (u_virt, v_virt)
            if p_virt_hom[2] != 0:
                u_virt = p_virt_hom[0] / p_virt_hom[2]
                v_virt = p_virt_hom[1] / p_virt_hom[2]
            else:
                u_virt, v_virt = 0, 0
        
            # BBox area calculation — convert to angular size (camera independent)
            bbox_w, bbox_h = bbox[2], bbox[3]
            bbox_sqrt_area = math.sqrt(bbox_w * bbox_h)
            bbox_angular_deg = math.degrees(math.atan(bbox_sqrt_area / self.fx))
            
            # Coverage percentages (Horizontal and Total Area)
            cov_horiz_pct = (bbox_w / self.W) * 100.0
            cov_area_pct = ((bbox_w * bbox_h) / (self.W * self.H)) * 100.0

            # Get current flight mode
            current_mode = "UNKNOWN"
            hb = self.mavlink_handler.master.messages.get('HEARTBEAT', None)
            if hb:
                current_mode = mavutil.mode_string_v10(hb)

            # Log stabilized virtual pixel with flight mode
            self.logger.debug(f"Target: Horiz={cov_horiz_pct:.2f}% Area={cov_area_pct:.2f}% | Virtual: u_virt={u_virt:.2f}, v_virt={v_virt:.2f} | BBox: {bbox} (ang={bbox_angular_deg:.3f}°) | Mode: {current_mode}")

            # --- PID/PD calculation, reset state if GUIDED is not in mode ---
            if current_mode != 'GUIDED':
                self.airspeed_integral = 0.0
                self.airspeed_prev_error = 0.0
                self.heading_prev_error = 0.0
                self.altitude_prev_error = 0.0
                self.last_pid_time = None
                self.logger.debug(f"PID/PD skipped (mode={current_mode}), state reset.")
                return

            # --- PID Controller (Airspeed) ---
            is_first_pid = getattr(self, 'last_pid_time', None) is None
            if is_first_pid:
                dt = 0.033  # Assumption of approximately 30Hz (Only in the first run)
                # GUIDED Accept initial speed (airspeed) when first entering mode
                if getattr(self.cmd_thread, 'last_airspeed', None) is not None:
                    initial_spd = max(self.airspeed_min, min(self.airspeed_max, self.cmd_thread.last_airspeed))
                    self.airspeed_base = initial_spd
                    self.last_commanded_airspeed = initial_spd
                    self.logger.debug(f"GUIDED Entry: Initialized Airspeed Base to {initial_spd:.1f} m/s")
            else:
                dt = current_time - self.last_pid_time
                if dt <= 0.001:
                    dt = 0.033

            self.last_pid_time = current_time

            # Error calculation: angular size (degrees) — camera independent
            spd_error = self.airspeed_target_angular_deg - bbox_angular_deg

            # Proportional — soft deadzone
            spd_p_input = 0.0 if abs(spd_error) < self.airspeed_deadzone_deg else spd_error - math.copysign(self.airspeed_deadzone_deg, spd_error)
            spd_p_term = self.airspeed_kp * spd_p_input

            # Integral step (integral increment amount in this cycle)
            spd_integral_step = self.airspeed_ki * spd_error * dt

            # Derivative
            if is_first_pid:
                self.airspeed_prev_error = spd_error
            spd_derivative = (spd_error - self.airspeed_prev_error) / dt
            spd_d_term = self.airspeed_kd * spd_derivative
            self.airspeed_prev_error = spd_error

            # Raw output = P + (Old I + New I) + D
            spd_pid_output = spd_p_term + self.airspeed_integral + spd_integral_step + spd_d_term
            raw_airspeed = self.airspeed_base + spd_pid_output

            # Update the integral at each step, then apply asymmetric clamp
            self.airspeed_integral += spd_integral_step
            self.airspeed_integral = max(self.airspeed_integral_min, min(self.airspeed_integral_max, self.airspeed_integral))

            # Limit output (raw output of PID)
            target_airspeed = max(self.airspeed_min, min(self.airspeed_max, raw_airspeed))
            
            # SLEW RATE CONTROL (Acceleration Limiting)
            max_delta_v = self.airspeed_max_accel * dt
            if target_airspeed > self.last_commanded_airspeed + max_delta_v:
                airspeed_out = self.last_commanded_airspeed + max_delta_v
            elif target_airspeed < self.last_commanded_airspeed - max_delta_v:
                airspeed_out = self.last_commanded_airspeed - max_delta_v
            else:
                airspeed_out = target_airspeed
                
            self.last_commanded_airspeed = airspeed_out
            self.test_airspeed_ms = airspeed_out

            # --- HEADING PD Controller (angular, degree) --- Convert pixel error to angular error
            hdg_error = math.degrees(math.atan((u_virt - self.center_x) / self.fx))
            if is_first_pid:
                self.heading_prev_error = hdg_error
            # Soft Deadzone (in degrees)
            hdg_p_input = 0.0 if abs(hdg_error) < self.deadzone_deg else hdg_error - math.copysign(self.deadzone_deg, hdg_error)
            hdg_p_term = self.heading_kp * hdg_p_input
            hdg_derivative = (hdg_error - self.heading_prev_error) / dt
            hdg_d_term = self.heading_kd * hdg_derivative
            self.heading_prev_error = hdg_error
            delta_heading_raw = hdg_p_term + hdg_d_term

            # Heading delta clamp (±max_delta_heading degrees)
            delta_heading = max(-self.max_delta_heading, min(self.max_delta_heading, delta_heading_raw))

            # Normalize current yaw to 0-360 degrees
            current_heading_deg = np.degrees(yaw_rad)
            if current_heading_deg < 0:
                current_heading_deg += 360.0
                
            new_heading = (current_heading_deg + delta_heading) % 360.0
            self.test_heading_deg = new_heading

            # --- ALTITUDE PD controller (angular, degrees) --- Convert pixel error to angular error — up positive
            alt_error = -math.degrees(math.atan((v_virt - self.center_y) / self.fy))
            if is_first_pid:
                self.altitude_prev_error = alt_error
            # Soft Deadzone (in degrees)
            alt_p_input = 0.0 if abs(alt_error) < self.deadzone_deg else alt_error - math.copysign(self.deadzone_deg, alt_error)
            alt_p_term = self.altitude_kp * alt_p_input
            alt_derivative = (alt_error - self.altitude_prev_error) / dt
            alt_d_term = self.altitude_kd * alt_derivative
            self.altitude_prev_error = alt_error
            delta_altitude_raw = alt_p_term + alt_d_term

            # Altitudinal delta clamp (narrowed for a calmer response)
            delta_altitude = max(-2.0, min(2.0, delta_altitude_raw))

            # Fetch current physical altitude
            loc_msg = self.mavlink_handler.master.messages.get('GLOBAL_POSITION_INT', None)
            if loc_msg:
                current_alt = loc_msg.alt / 1000.0  # AMSL (mm -> m)
            else:
                current_alt = self.test_alt_m

            new_altitude = current_alt + delta_altitude
            
            # Clamp altitude between the configured minimum and maximum
            new_altitude = max(self.MIN_ALTITUDE, min(self.MAX_ALTITUDE, new_altitude))
            self.test_alt_m = new_altitude

            # OVERRIDE FOR SEQUENTIAL TUNING
            # new_altitude = 48.0
            # self.test_alt_m = new_altitude

            # Update target to Commander thread
            self.cmd_thread.update(self.test_heading_deg, self.test_alt_m, self.test_airspeed_ms)
            # Get actual measured airspeed
            measured_as = self.cmd_thread.last_airspeed
            measured_as_str = f"{measured_as:.1f}" if measured_as is not None else "?"
            
            self.logger.debug(f"Airspeed PID: Cmd={airspeed_out:.1f}m/s (Raw={target_airspeed:.1f}) Meas={measured_as_str}m/s | BBoxAng={bbox_angular_deg:.3f}° TgtAng={self.airspeed_target_angular_deg:.3f}° | Err={spd_error:.3f}° | P={spd_p_term:.2f} I={self.airspeed_integral:.2f} D={spd_d_term:.2f} Sum={spd_pid_output:.2f} | dt={dt:.3f}")
            self.logger.debug(f"Heading PD: Hdg={current_heading_deg:.1f} | Err={hdg_error:.3f}° | P={hdg_p_term:.2f} D={hdg_d_term:.2f} | dHdg_raw={delta_heading_raw:.2f} dHdg={delta_heading:.2f} | NewHdg={new_heading:.1f}")
            self.logger.debug(f"Altitude PD: Alt={current_alt:.1f} | Err={alt_error:.3f}° | P={alt_p_term:.2f} D={alt_d_term:.2f} | dAlt_raw={delta_altitude_raw:.2f} dAlt={delta_altitude:.2f} | NewAlt={new_altitude:.1f}")
            self.logger.debug("-" * 50)

    def _parse_bbox(self, data):
        """Parse a Redis message and return an (x, y, w, h) tuple or None."""
        x, y, w, h = None, None, None, None
        
        # List format: [x, y, w, h, coverage(opt)]
        if isinstance(data, (list, tuple)) and len(data) >= 4:
            x, y, w, h = data[:4]
            self.horizontal_coverage = data[4] if len(data) >= 5 else None

        if w is not None:
            if getattr(self, 'horizontal_coverage', None) is None:
                self.horizontal_coverage = (w / self.W) * 100 if self.W > 0 else 0
            return (int(x), int(y), int(w), int(h))
            
        return None

    def _redis_listener(self):
        """Thread 1: Redis listens to pub/sub, quickly parses every incoming message and writes it to self.latest_bbox. It does not perform any heavy operations."""
        self.logger.debug("Redis listener thread started.")
        for message in self.p.listen():
            if message['type'] != 'message':
                continue

            # mission control
            guid_message = self.r.get('task')
            if guid_message is None:
                guid_message = b'visual'
            if guid_message.decode('utf-8').lower() != 'visual':
                continue

            # Parse
            try:
                data = json.loads(message['data'].decode('utf-8'))
            except (json.JSONDecodeError, UnicodeDecodeError):
                continue

            bbox = self._parse_bbox(data)
            if bbox is not None:
                now = time.time()
                with self.bbox_lock:
                    self.latest_bbox = bbox
                    self.latest_bbox_time = now
                    self.last_message_time = now

    def _vision_processor(self):
        """Thread 2: Reads the latest bbox at a fixed frequency (30 Hz) and calls guide_aircraft(). It does not reprocess the same bbox.         When the target disappears, coasting for 5 seconds, then applies failsafe."""
        self.logger.debug("Vision processor thread started.")
        period = 1.0 / 30.0
        last_processed_time = None
        last_target_time = 0.0
        last_warning_time = 0.0

        while True:
            time.sleep(period)
            with self.bbox_lock:
                bbox = self.latest_bbox
                bbox_time = self.latest_bbox_time

            # Process if a new bbox arrives
            if bbox is not None and bbox_time != last_processed_time:
                last_processed_time = bbox_time
                last_target_time = time.time()
                self.guide_aircraft(bbox, bbox_time)
            else:
                # NO TARGET: Coasting or Failsafe
                now = time.time()
                if last_target_time > 0.0:
                    time_since_last = now - last_target_time
                    if time_since_last <= 5.0:
                        # ── COASTING (5s): TestCommander continues sending the last heading/alt/throttle values ​​— no additional action required ──
                        pass
                    else:
                        # ── FAILSAFE: Hanged for 5 seconds! ── Reset PID/PD states
                        self.airspeed_integral = 0.0
                        self.airspeed_prev_error = 0.0
                        self.heading_prev_error = 0.0
                        self.altitude_prev_error = 0.0
                        self.last_pid_time = None

                        # Deactivate Commander (airplane surrenders to autopilot)
                        self.cmd_thread.active = False

                        # An alert is sent to the terminal every second.
                        if now - last_warning_time > 1.0:
                            self.logger.warning(
                                "\n[WARNING] TARGET MISSING FOR MORE THAN 5 SECONDS! CHANGE MISSION MODE!!!")
                            last_warning_time = now

    def run(self):
        # Thread 1: Redis listener (only read and parse)
        listener_thread = threading.Thread(
            target=self._redis_listener, daemon=True, name="RedisListener")
        listener_thread.start()

        # Thread 2: Vision processor (call to guide_aircraft)
        processor_thread = threading.Thread(
            target=self._vision_processor, daemon=True, name="VisionProcessor")
        processor_thread.start()

        self.logger.debug("All threads are started. Main thread is waiting...")
        listener_thread.join()
        processor_thread.join()


if __name__ == '__main__':
    goat = tziGuidance()
    goat.run()
