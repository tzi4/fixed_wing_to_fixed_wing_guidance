#!/usr/bin/env python3

import time
import math

from datetime import datetime
import redis
import ast
import threading
import queue
import numpy as np
from pymavlink import mavutil
import json
import logging

import os
import shutil

# ─── LOG SETUP ─── Active logs: guidance_logs/ Old logs: old_guidance_logs/ (automatically moved at every startup)
LOG_DIR = "guidance_logs"
OLD_LOG_DIR = "old_guidance_logs"

def setup_logging():
    os.makedirs(LOG_DIR, exist_ok=True)
    os.makedirs(OLD_LOG_DIR, exist_ok=True)

    # Move all logs from previous work to the archive
    for fname in os.listdir(LOG_DIR):
        src = os.path.join(LOG_DIR, fname)
        if os.path.isfile(src):
            dst = os.path.join(OLD_LOG_DIR, fname)
            # If there is a file with the same name, add a counter at the end to avoid overwriting it.
            if os.path.exists(dst):
                base, ext = os.path.splitext(fname)
                i = 1
                while os.path.exists(os.path.join(OLD_LOG_DIR, f"{base}_{i}{ext}")):
                    i += 1
                dst = os.path.join(OLD_LOG_DIR, f"{base}_{i}{ext}")
            shutil.move(src, dst)

    log_path = os.path.join(LOG_DIR, f"guidance_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log")

    root = logging.getLogger()
    root.setLevel(logging.DEBUG)  # Detail to file, summary to terminal

    fmt_file = logging.Formatter('%(asctime)s.%(msecs)03d [%(levelname)s] %(message)s', datefmt='%H:%M:%S')
    fmt_term = logging.Formatter('%(asctime)s [%(levelname)s] %(message)s', datefmt='%H:%M:%S')

    fh = logging.FileHandler(log_path)
    fh.setLevel(logging.DEBUG)      # Detailed telemetry (TLM lines) only to file
    fh.setFormatter(fmt_file)

    sh = logging.StreamHandler()
    sh.setLevel(logging.INFO)       # Terminal only summary/event lines
    sh.setFormatter(fmt_term)

    root.addHandler(fh)
    root.addHandler(sh)
    logging.info(f"Log file: {log_path}")

setup_logging()
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

class RedisListener(threading.Thread):
    def __init__(self, data_queue):
        super().__init__()
        self.data_queue = data_queue
        self.daemon = True
        self.running = True
        self.task = "Unknown"
        
        logging.info("Connecting to server Redis...")
        try:
            self.r = redis.Redis(host='localhost', port=6379, db=0)
            self.pubsub = self.r.pubsub()
            self.pubsub.subscribe('tracker_bbox')
            logging.info("Redis Subscribed to channel 'tracker_bbox'.")
        except Exception as e:
            logging.error(f"Redis Connection Error: {e}")
            self.running = False
            
    def run(self):
        while self.running:
            try:
                task_bytes = self.r.get('task')
                self.task = task_bytes.decode('utf-8') if task_bytes else "Unknown"
                
                message = self.pubsub.get_message(ignore_subscribe_messages=True, timeout=0.01)
                if message and message['type'] == 'message':
                    data_str = message['data'].decode('utf-8')
                    try:
                        bbox_data = ast.literal_eval(data_str)
                        if len(bbox_data) >= 4:
                            x, y, w, h = bbox_data[0:4]
                            obj_x = x + (w / 2.0)
                            obj_y = y + (h / 2.0)
                            target_area = w * h
                            
                            self.data_queue.put({
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
                    except:
                        pass
            except:
                time.sleep(0.1)
                
    def get_task(self):
        return self.task

    def get_target_altitude(self):
        try:
            # Get data as bytes from Redist
            data_bytes = self.r.get('rakip_telemetri')
            if data_bytes:
                # Convert byte to string, then to JSON object (dictionary)
                data = json.loads(data_bytes.decode('utf-8'))

                # Pull the altitude from the first element of the array konumBilgileri according to the structure JSON
                target_altitude = data.get("konumBilgileri", [{}])[0].get("iha_irtifa", None)
                return float(target_altitude) if target_altitude is not None else None
        except Exception as e:
            # Pass to avoid crashing the system if there is a parse error or no data.
            pass
        return None

class MavlinkManager(threading.Thread):
    def __init__(self, connection_str='udp:127.0.0.1:14562'):
        super().__init__()
        self.daemon = True
        self.running = True
        self.lock = threading.Lock()
        
        self.current_mode = "UNKNOWN"
        self.current_yaw_rad = 0.0
        self.current_roll_rad = 0.0
        self.current_pitch_rad = 0.0
        self.current_alt_rel = 100.0  # Default starting altitude
        self.current_airspeed = 20.0
        self.current_climb = 0.0      # Actual vertical speed (VFR_HUD.climb)
        
        logging.info(f"MAVLink connection: {connection_str}...")
        try:
            self.master = mavutil.mavlink_connection(connection_str)
            self.master.wait_heartbeat()
            logging.info(f"Connection Successful. System ID: {self.master.target_system}")
            # Request extra telemetry streams (GLOBAL_POSITION_INT for altitude etc.)
            self.master.mav.request_data_stream_send(
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_POSITION, 10, 1)
            self.master.mav.request_data_stream_send(
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 10, 1)
            # Transmission VFR_HUD is usually in the EXTRA2 stream
            self.master.mav.request_data_stream_send(
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_EXTRA2, 10, 1)
        except Exception as e:
            logging.error(f"MAVLink Error: {e}")
            self.running = False

    def run(self):
        while self.running:
            try:
                with self.lock:
                    msg = self.master.recv_match(type=['ATTITUDE', 'HEARTBEAT', 'GLOBAL_POSITION_INT', 'VFR_HUD'], blocking=False)
                
                if msg:
                    if msg.get_type() == 'ATTITUDE':
                        self.current_yaw_rad = msg.yaw
                        self.current_roll_rad = msg.roll
                        self.current_pitch_rad = msg.pitch
                    elif msg.get_type() == 'HEARTBEAT':
                        if self.master.flightmode:
                            self.current_mode = self.master.flightmode
                    elif msg.get_type() == 'GLOBAL_POSITION_INT':
                        self.current_alt_rel = msg.relative_alt / 1000.0
                    elif msg.get_type() == 'VFR_HUD':         
                        self.current_airspeed = msg.airspeed
                        self.current_climb = msg.climb
                time.sleep(0.01)
            except:
                time.sleep(0.1)

    def send_guided_targets(self, heading_deg, target_alt_m, target_speed_m_s):
        with self.lock:
            try:
                # 1. SPEED COMMAND (DO_CHANGE_SPEED)
                self.master.mav.command_int_send(
                    self.master.target_system, self.master.target_component,
                    mavutil.mavlink.MAV_FRAME_GLOBAL,
                    mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
                    0, 0,
                    0, target_speed_m_s, -1, 0, # param2=speed, param3=throttle change (-1)
                    0, 0, 0
                )
                
                # 2. DIRECTION COMMAND (GUIDED_CHANGE_HEADING)
                self.master.mav.command_int_send(
                    self.master.target_system, self.master.target_component,
                    mavutil.mavlink.MAV_FRAME_GLOBAL,
                    mavutil.mavlink.MAV_CMD_GUIDED_CHANGE_HEADING,
                    0, 0,
                    1, heading_deg, 3.0, 0,  # param1=Absolute angle, param2=angle, param3=Rotation Speed
                    0, 0, 0
                )
                
                # 3. altitude COMMAND (GUIDED_CHANGE_ALTITUDE)
                self.master.mav.command_int_send(
                    self.master.target_system, self.master.target_component,
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, # Frame 3
                    mavutil.mavlink.MAV_CMD_GUIDED_CHANGE_ALTITUDE,
                    0, 0,
                    0, 0, 0, 0,
                    0, 0, target_alt_m # z axis value
                )
            except Exception as e:
                pass


            
class AutopilotController:
    def __init__(self, connection_str='udp:127.0.0.1:14554'):
        self.data_queue = queue.Queue()
        
        self.mavlink = MavlinkManager(connection_str)
        self.redis = RedisListener(self.data_queue)
        
        if self.mavlink.running and self.redis.running:
            self.mavlink.start()
            self.redis.start()
        else:
            exit(1)

        # --- NEW PID AND CONTROL SETTINGS (HEADING & ALTITUDE) --- Suitable for the "P is quite cumbersome, I and D 0" request We have increased the Kp values ​​significantly so that the system can return to the target
        self.Kp_heading = 2.5   # 1 degree error = 4.5 degrees off target
        self.Ki_heading = 0.0
        self.Kd_heading = 0.5

        self.Kp_alt = 1      # 1 degree error = 1.5 meters altitude change
        self.Ki_alt = 0.0
        self.Kd_alt = 0.3
        
        self.filtered_target_speed = 20.0





        # --- VERTICAL SPEED (CLIMB RATE) PID SETTINGS --- Oscillation analysis (172133): The slow oscillation with ~6s period was caused by integral windup (not P/D). Ki reduced + integral limit narrowed to 4.  Kp is medium: pulls the target to the center but does not overdrive. Kd soft cushioning.
        self.Kp_vz = 0.5     # Gathering to the center; If it is too high it will exceed the center
        self.Ki_vz = 0.02    # Small: only leaks permanent offset, no oscillation
        self.Kd_vz = 0.15    # Absorption; err_y quick change brakes




        # --- HEADING RATE PID (rotation speed control) ---
        self.Kp_rate = 0.6      # 1° angular error → 5 deg/s rotation speed
        self.Ki_rate = 0.0
        self.Kd_rate = 0.08     # For reaction to fast moving target

        self.integral_error_rate = 0.0
        self.prev_error_rate = 0.0
        self.prev_derivative_rate = 0.0

        self.min_heading_rate = 0.85    # Minimum rotation speed (deg/s)
        self.max_heading_rate = 5.0   # Maximum rotation speed (deg/s)

        self.max_heading_change_deg = 35.0     
        self.max_alt_change_m = 25.0    
        self.deadzone_deg = 0.4  

        self.prev_error_x, self.prev_error_y = 0.0, 0.0
        self.integral_error_x, self.integral_error_y = 0.0, 0.0
        self.prev_derivative_x, self.prev_derivative_y = 0.0, 0.0
        self.last_time = time.time()
        self.last_cmd_send_time = 0.0

        self.last_target_time = 0.0
        self.last_target_heading = 0.0
        self.last_target_alt = 0.0
        self.last_heading_rate = self.min_heading_rate
        self.alt_target = None   # Integrated altitude target (starts with current_alt in the first tracking frame)
        
        self.frame_counter = 0
        self.start_time = time.time()

        # --- CAMERA SETTINGS (Please Update According to Actual System!) ---
        self.camera_width = 1920  
        self.camera_height = 1080 
        self.camera_hfov_rad = 0.42
        
        # GAZEBO CAMERA
        self.center_x = 960    
        self.center_y =  540   

        # REAL CAMERA self.center_x = 1091 self.center_y = 633

        
        self.fx = 4515
        self.fy = 4510

        self.K = np.array([
            [self.fx, 0,       self.center_x],
            [0,       self.fy, self.center_y],
            [0,       0,       1            ]
        ])
        self.K_inv = np.linalg.inv(self.K)

    def clamp(self, value, min_val, max_val):
        return max(min(value, max_val), min_val)

    def stabilize_pixel(self, obj_x, obj_y):
        p_raw = np.array([obj_x, obj_y, 1.0])
        r_cam = self.K_inv @ p_raw
        r_body = R_c_b @ r_cam
        
        roll_rad = self.mavlink.current_roll_rad
        pitch_rad = self.mavlink.current_pitch_rad
        R_stab = compute_R_b_e(roll_rad, pitch_rad, 0)
        r_virt_body = R_stab @ r_body
        r_virt_cam = R_c_b_T @ r_virt_body
        p_virt_hom = self.K @ r_virt_cam
        
        if p_virt_hom[2] != 0:
            return p_virt_hom[0] / p_virt_hom[2], p_virt_hom[1] / p_virt_hom[2]
        return obj_x, obj_y  

    def _log_telemetry(self, state, **kv):
        """Writes detailed telemetry to the file (DEBUG) in a single line key=value format.         It does not drop into the terminal; Easily separated after flight with grep/pandas.         Example: grep 'TLM state=TRACKING' guidance_logs/*.log
        """
        parts = [f"TLM state={state}",
                 f"mode={self.mavlink.current_mode}",
                 f"elapsed={time.time() - self.start_time:.3f}"]
        for k, v in kv.items():
            if isinstance(v, float):
                parts.append(f"{k}={v:.4f}")
            else:
                parts.append(f"{k}={v}")
        logging.debug(" ".join(parts))

    def run(self):
        logging.info("System active! Main thread started...")
        try:
            while True:
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

                if not data:
                    # COASTING / FAILSAFE
                    if self.last_target_time > 0.0:
                        td = current_time - self.last_target_time
                        if td <= 5.0:
                            cmd_sent = 0
                            if task == "Visual" and self.mavlink.current_mode == "GUIDED":
                                if current_time - self.last_cmd_send_time >= 0.2: # 5Hz renewal
                                    # TARGET LOST: Fix altitude at current value, fly straight while maintaining last known speed and heading
                                    self.alt_target = None  # Start fresh in recapture
                                    last_spd = getattr(self, 'last_target_speed', 20.0)
                                    self.mavlink.send_guided_targets(self.last_target_heading, current_alt, last_spd)
                                    cmd_sent = 1
                                    self.last_cmd_send_time = current_time
    
                            
                            self._log_telemetry(
                                "COASTING", task=task, cmd_sent=cmd_sent,
                                target_lost_s=td, alt=current_alt,
                                hdg=self.last_target_heading,
                                spd_cmd=getattr(self, 'last_target_speed', 20.0),
                                airspeed=self.mavlink.current_airspeed,
                                climb_act=self.mavlink.current_climb
                            )
                        else:
                            self.integral_error_x, self.integral_error_y = 0.0, 0.0
                            self.prev_derivative_x, self.prev_derivative_y = 0.0, 0.0
                            self.integral_error_rate = 0.0
                            self.alt_target = None
                            
                            # Failsafe: Continue at current heading and altitude
                            cmd_sent = 0
                            if task == "Visual" and self.mavlink.current_mode == "GUIDED":
                                if current_time - self.last_cmd_send_time >= 0.5:
                                    # COMPLETE FAILURE: Fly in the direction pointed by the nose at standard speed (20.0) and current altitude.
                                    self.mavlink.send_guided_targets(yaw_deg, current_alt, 20.0)
                                    cmd_sent = 1
                                    self.last_target_heading = yaw_deg
                                    self.last_target_speed = 20.0
                                    self.last_cmd_send_time = current_time
                                    
                            if cmd_sent:
                                logging.warning(f"FAILSAFE: Target is {td:.1f}s lost. Level flight: Direction={yaw_deg:.1f}° altitude={current_alt:.1f}m Speed=20.0m/s")
                            self._log_telemetry(
                                "FAILSAFE", task=task, cmd_sent=cmd_sent,
                                target_lost_s=td, alt=current_alt, hdg=yaw_deg,
                                airspeed=self.mavlink.current_airspeed,
                                climb_act=self.mavlink.current_climb
                            )
                    continue

                # --- MAV_CMD GUIDED UPDATE ---
                dt = max(0.001, min(current_time - self.last_time, 0.1))
                self.last_time = current_time
                
                obj_x = data['obj_x']
                obj_y = data['obj_y']
                bbox_x = data.get('bbox_x', 0)
                bbox_y = data.get('bbox_y', 0)
                bbox_w, bbox_h = data.get('bbox_w', 0.0), data.get('bbox_h', 0.0)
                target_area = data['target_area']
                data_age_ms = (current_time - data.get('timestamp', current_time)) * 1000.0
                self.last_target_time = current_time
                
                stab_x, stab_y = self.stabilize_pixel(obj_x, obj_y)
                
                stab_error_x_deg = math.degrees(math.atan((stab_x - self.center_x) / self.fx))
                stab_error_y_deg = math.degrees(math.atan((stab_y - self.center_y) / self.fy))

                raw_error_x_deg = math.degrees(math.atan((obj_x - self.center_x) / self.fx))
                raw_error_y_deg = math.degrees(math.atan((obj_y - self.center_y) / self.fy))

                # Derivative account (Low pass)
                alpha = 0.05 
                raw_deriv_x = (stab_error_x_deg - self.prev_error_x) / dt
                raw_deriv_y = (stab_error_y_deg - self.prev_error_y) / dt
                deriv_x = (alpha * raw_deriv_x) + ((1.0 - alpha) * self.prev_derivative_x)
                deriv_y = (alpha * raw_deriv_y) + ((1.0 - alpha) * self.prev_derivative_y)

                # Integral
                if task == "Visual" and self.mavlink.current_mode == "GUIDED":
                    if abs(stab_error_x_deg) < self.deadzone_deg: self.integral_error_x *= 0.99
                    else: self.integral_error_x += stab_error_x_deg * dt
                    self.integral_error_x = self.clamp(self.integral_error_x, -10.0, 10.0)

                    if abs(stab_error_y_deg) < self.deadzone_deg: self.integral_error_y *= 0.99
                    else: self.integral_error_y += stab_error_y_deg * dt
                    # Narrowed the Y integral limit (10→4): the wide limit would swell to +9 when the target was on top and overcorrect once past the center, causing slow oscillations of ~6s. The narrow limit breaks the dominance of the integral.
                    self.integral_error_y = self.clamp(self.integral_error_y, -4.0, 4.0)
                else:
                    self.integral_error_x, self.integral_error_y = 0.0, 0.0

                self.prev_error_x, self.prev_error_y = stab_error_x_deg, stab_error_y_deg
                self.prev_derivative_x, self.prev_derivative_y = deriv_x, deriv_y

                # Deadzone
                dz_x = 1 if abs(stab_error_x_deg) < self.deadzone_deg else 0
                dz_y = 1 if abs(stab_error_y_deg) < self.deadzone_deg else 0
                p_x = 0 if dz_x else stab_error_x_deg - math.copysign(self.deadzone_deg, stab_error_x_deg)
                p_y = 0 if dz_y else stab_error_y_deg - math.copysign(self.deadzone_deg, stab_error_y_deg)

                # --- 1. HEADING CONTROL ---
                p_term_heading = p_x * self.Kp_heading
                i_term_heading = self.integral_error_x * self.Ki_heading
                d_term_heading = deriv_x * self.Kd_heading
                
                cmd_head_deg = p_term_heading + i_term_heading + d_term_heading
                cmd_head_deg = self.clamp(cmd_head_deg, -self.max_heading_change_deg, self.max_heading_change_deg)
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

                # --- 2. CLIMB RATE (VZ) CHECK --- If the target is low on the camera, p_y is positive, it should crash (-Vz). (We fixed the bug with a multiplier of -1.0)
                p_term_vz = p_y * self.Kp_vz
                i_term_vz = self.integral_error_y * self.Ki_vz
                d_term_vz = deriv_y * self.Kd_vz
                
                # Z axis target speed (m/s)
                climb_rate_raw = -1.0 * (p_term_vz + i_term_vz + d_term_vz)
                
                # Apply climb/collapse limits based on aircraft structural limits (CAUTION: These values ​​MUST BE THE SAME as TECS_CLMB_MAX and TECS_SINK_MAX in ArduPilot!)
                climb_rate_cmd = self.clamp(climb_rate_raw, -5.0, 2.0)

                # Ground safety (Prevent collapse if current altitude is below 10 meters)
                if current_alt < 10.0 and climb_rate_cmd < 0:
                    climb_rate_cmd = 0.0

                current_airspeed = self.mavlink.current_airspeed
                
                # ------------------------------------------------------------- PHYSICAL FLIGHT PROTECTIONS -------------------------------------------------------------
                V_cruise = 20.0
                V_max = 24.0     
                V_min = 16.0  
                
                # 1. Turning Load Factor Correction Reduces the climbing demand to avoid losing wing carry in sharp turns.  As roll increases with cos(roll)^2, Vz reduces the demand.
                load_factor_scale = math.cos(roll_now) ** 2
                climb_rate_cmd *= load_factor_scale

                # 2. Stall Protection — REMOVED This protection was multiplying positive Vz (climb) by zero when airspeed < 17.5 m/s. Since the airspeed in SITL was always ~15 m/s, even if the target was at the top of the camera, the generated positive Vz was instantly reset and the plane did not climb at all. Stall safety is already provided on the ArduPilot side with AIRSPEED_MIN/ ARSPD_FBW_MIN; It is unnecessary to restrict again here.
                stall_factor = 1.0   # neutral (kept for logging compatibility)

                # 3. Specific Energy Control
                g = 9.81
                V_safe = max(current_airspeed, 12.0) # Lower bound to avoid division by zero error
                K_energy = 0.8 # Energy gain multiplier (0.5 - 1.0 is ideal, you can adjust it in tests)
                
                # Specific Energy formula: dV = (g * Vz / V) * K_energy Increases speed demand in climbing (motor turns on), decreases speed in diving (motor throttles).
                dV_energy = (g * climb_rate_cmd / V_safe) * K_energy
                
                # To avoid positive feedback (runaway) error, we base on fixed V_cruise, not current speed!
                target_speed_raw = V_cruise + dV_energy

                if not hasattr(self, 'filtered_target_speed'):
                    self.filtered_target_speed = V_cruise

                # We loosen the filter (0.05 -> 0.20) so that the engines can provide energy instantly when needed.
                alpha_speed = 0.20  
                self.filtered_target_speed = (alpha_speed * target_speed_raw) + ((1.0 - alpha_speed) * self.filtered_target_speed)
                
                # Confine speed to mechanical limits
                target_speed = self.clamp(self.filtered_target_speed, V_min, V_max)

                # ---------------------------------------------------------------- SENDING COMMANDS SECTION ------------------------------------------------------------- Anti-windup
                aw_x, aw_y = 0, 0
                if abs(cmd_head_deg) >= self.max_heading_change_deg:
                    self.integral_error_x *= 0.9; aw_x = 1
                # If the Vz command has reached the limits OR if the aircraft cannot actually follow the command, extinguish the Y integral (otherwise, in the unreachable sink, the integral would swell to +10 and suppress the P term, reacting late when the target was caught above).
                cmd_saturated = climb_rate_raw >= 6.0 or climb_rate_raw <= -5.0
                vz_untracked = abs(climb_rate_cmd - self.mavlink.current_climb) > 3.0
                if cmd_saturated or vz_untracked:
                    self.integral_error_y *= 0.9; aw_y = 1

                command_sent = 0
                if current_time - self.last_cmd_send_time >= 0.1:
                    if task == "Visual" and self.mavlink.current_mode == "GUIDED":
                        
                        # ── LOOK-AHEAD altitude TARGET ── T_LOOK: The horizon that converts Vz to altitude offset. 3.0s was too long; When climb_cmd=5, it commanded the plane beyond 15 m, TECS saw this as an unattainable jump and approached slowly/delayed (~0.8s was measured).  With the 1.5s, the target stays within range where the aircraft can actually chase it, the command is transmitted more like "current Vz".
                        T_LOOK = 2
                        moving_target_alt = current_alt + (climb_rate_cmd * T_LOOK)
                        # ground safety
                        moving_target_alt = max(moving_target_alt, 10.0)
                        
                        # Tracking vulnerability diagnosis: if Vz performed by the command is constantly diverging, the problem is not in the outer loop but in the performance of TECS/aircraft (see TECS_OPTIONS).
                        vz_gap = climb_rate_cmd - self.mavlink.current_climb
                        if abs(vz_gap) > 3.0 and abs(climb_rate_cmd) > 2.0:
                            logging.warning(f"Vz TRACKING OPEN: command={climb_rate_cmd:.1f} actual={self.mavlink.current_climb:.1f} "
                                            f"(difference={vz_gap:.1f}) airspeed={self.mavlink.current_airspeed:.1f} → TECS sink/climb may be limited")
                        
                        # Transmit the 3-command package we prepared to the autopilot with COMMAND_INT
                        self.mavlink.send_guided_targets(target_heading, moving_target_alt, target_speed)
                        command_sent = 1
                        self.alt_target = moving_target_alt  # for logging

                        logging.info(f"[{task}] AUTONOMOUS: Direction={target_heading:.1f}° | err_y={stab_error_y_deg:+.2f}° (raw={raw_error_y_deg:+.2f}°) | Vz={climb_rate_cmd:+.2f} | Speed={target_speed:.1f} | Target altitude={moving_target_alt:.1f}m | Real Vz={self.mavlink.current_climb:+.2f}")
                    else :
                        logging.info(f"[{task}] WAIT: Direction={target_heading:.1f}° | err_y={stab_error_y_deg:+.2f}° (raw={raw_error_y_deg:+.2f}°) | Vz={climb_rate_cmd:+.2f} | Speed={target_speed:.1f} | altitude={current_alt:.1f}m")
                    self.last_cmd_send_time = current_time

                self.last_target_heading = target_heading
                self.last_target_speed = target_speed

                self._log_telemetry(
                    "TRACKING", task=task, cmd_sent=command_sent,
                    dt=dt, data_age_ms=data_age_ms, queue=queue_size,
                    # Image / error
                    obj_x=obj_x, obj_y=obj_y, bbox_w=bbox_w, bbox_h=bbox_h,
                    err_x_deg=stab_error_x_deg, err_y_deg=stab_error_y_deg,
                    deriv_x=deriv_x, deriv_y=deriv_y, dz_x=dz_x, dz_y=dz_y,
                    # aircraft status
                    alt=current_alt,
                    roll_deg=math.degrees(roll_now), pitch_deg=math.degrees(pitch_now),
                    yaw_deg=yaw_deg,
                    airspeed=self.mavlink.current_airspeed,
                    climb_act=self.mavlink.current_climb,
                    # Heading channel
                    cmd_head_deg=cmd_head_deg, target_heading=target_heading,
                    heading_rate=heading_rate,
                    # Vz channel and energy chain (climb_cmd = final value after all protections)
                    climb_raw=climb_rate_raw, load_factor_scale=load_factor_scale,
                    stall_factor=stall_factor, climb_cmd=climb_rate_cmd,
                    integral_y=self.integral_error_y, aw_y=aw_y,
                    p_term_vz=p_term_vz, i_term_vz=i_term_vz, d_term_vz=d_term_vz,
                    alt_target=self.alt_target if self.alt_target is not None else current_alt,
                    dV_energy=dV_energy, target_speed=target_speed
                )

        except KeyboardInterrupt:
            logging.info("Closing...")
            self.mavlink.running = False
            self.redis.running = False
            logging.shutdown()

if __name__ == '__main__':
    guidance = AutopilotController(connection_str='udp:127.0.0.1:14554')
    guidance.run()