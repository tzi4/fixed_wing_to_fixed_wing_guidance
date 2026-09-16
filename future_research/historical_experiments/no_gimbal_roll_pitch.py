#!/usr/bin/env python3

import time
import math
import csv
from datetime import datetime
import redis
import ast
from pymavlink import mavutil

class GoatGuidTest:
    def __init__(self, connection_str='udp:127.0.0.1:14552'):
        # --- REJECTION LINK ---
        print("Connecting to server Redis...")
        try:
            self.r = redis.Redis(host='localhost', port=6379, db=0)
            self.pubsub = self.r.pubsub()
            self.pubsub.subscribe('tracker_bbox')
            print("Redis Subscribed to channel 'tracker_bbox'.")
        except Exception as e:
            print(f"Redis Connection Error: {e}")
            exit(1)

        # --- LOGGING (CSV) SETTINGS ---
        self.log_filename = f"pd_tracker_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        self.log_file = open(self.log_filename, mode='w', newline='')
        self.csv_writer = csv.writer(self.log_file)
        
        # 18 Column Headings in just the right order
        headers = [
            "timestamp", "dt", "flight_mode", "target_found",
            "error_x_deg", "error_y_deg", "heading_diff_deg",
            "p_term_roll", "i_term_roll", "d_term_roll", "cmd_roll_rad", "cmd_roll_deg_clamped",
            "p_term_pitch", "i_term_pitch", "d_term_pitch", "cmd_pitch_rad", "cmd_pitch_deg_clamped",
            "target_area_px"
        ]
        self.csv_writer.writerow(headers)
        print(f"Log file created: {self.log_filename}")

        # --- MAVLINK LINK ---
        print(f"Connecting to the plane with MAVLink: {connection_str}...")
        try:
            self.master = mavutil.mavlink_connection(connection_str)
            self.master.wait_heartbeat()
            print(f"MAVLink Connection Successful! System ID: {self.master.target_system}")
        except Exception as e:
            print(f"MAVLink Connection Error: {e}")
            exit(1)

        self.current_mode = "UNKNOWN"
        self.current_yaw_rad = 0.0

        # --- PID AND CONTROL SETTINGS ---
        self.multiply_factor = 1.0

        self.Kp_roll = 0.02875 * self.multiply_factor # roll pid increased by 15%
        self.Ki_roll = 0.00115 * self.multiply_factor
        self.Kd_roll = 0.0090 * self.multiply_factor  # D term increased to reduce oscillation (Old 0.00575, Very high 0.0150)

        self.Kp_pitch = 0.01575 * self.multiply_factor  # pitch pid increased by 5%
        self.Ki_pitch = 0.0021 * self.multiply_factor
        self.Kd_pitch = 0.0060 * self.multiply_factor   # D term increased (Old: 0.00315, Very high 0.0100)

        self.max_roll_deg = 35.0     
        self.max_pitch_deg = 20.0    
        self.deadzone_deg = 0.78  

        # Memory
        self.prev_error_x, self.prev_error_y = 0.0, 0.0
        self.integral_error_x, self.integral_error_y = 0.0, 0.0
        self.prev_derivative_x, self.prev_derivative_y = 0.0, 0.0
        self.last_time = time.time()

        #self.was_target_found = False

        # --- CAMERA AND OPTICAL SETTINGS --- MUST BE EXACTLY THE SAME AS self.W and self.H in detection.py
        self.camera_width = 1280  
        self.camera_height = 720 
        self.camera_hfov = 14.12 
        self.camera_vfov = 7.96 

        self.center_x = self.camera_width / 2.0
        self.center_y = self.camera_height / 2.0

        hfov_rad = math.radians(self.camera_hfov)
        vfov_rad = math.radians(self.camera_vfov)
        self.fx = self.center_x / math.tan(hfov_rad / 2.0)
        self.fy = self.center_y / math.tan(vfov_rad / 2.0)

    def clamp(self, value, min_val, max_val):
        return max(min(value, max_val), min_val)

    def update_mavlink_state(self):
        msg = self.master.recv_match(type=['ATTITUDE', 'HEARTBEAT'], blocking=False)
        if msg:
            if msg.get_type() == 'ATTITUDE':
                self.current_yaw_rad = msg.yaw
            elif msg.get_type() == 'HEARTBEAT':
                if self.master.flightmode:
                    self.current_mode = self.master.flightmode

    def set_mode(self, mode_name):
        if mode_name not in self.master.mode_mapping():
            return
        mode_id = self.master.mode_mapping()[mode_name]
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )

    def send_attitude_target(self, roll_rad, pitch_rad, yaw_rad):
        # Quaternion account
        cr, sr = math.cos(roll_rad * 0.5), math.sin(roll_rad * 0.5)
        cp, sp = math.cos(pitch_rad * 0.5), math.sin(pitch_rad * 0.5)
        cy, sy = math.cos(yaw_rad * 0.5), math.sin(yaw_rad * 0.5)
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        q = [w, x, y, z]
        
        type_mask = 7 
        self.master.mav.set_attitude_target_send(
            0, self.master.target_system, self.master.target_component,
            type_mask, q, 0, 0, 0, 0.80
        )

    def run(self):
        print("System active! Waiting for YOLO data via Redis...")
        
        last_target_time = 0.0
        last_final_roll_rad = 0.0
        last_final_pitch_rad = 0.0
        
        try:
            while True:
                # 1. Update MAVLink Data
                self.update_mavlink_state()

                # 2. Get TASK (Mode) Information from Redis
                try:
                    task_bytes = self.r.get('task')
                    task = task_bytes.decode('utf-8') if task_bytes else "Unknown"
                except:
                    task = "Unknown"

                # 3. NEW STRUCTURE: Wait until data comes from Redis (Synchronization) timeout = 0.05 means: Wait for 50ms to get a new frame.
                message = self.pubsub.get_message(ignore_subscribe_messages=True, timeout=0.05)
                
                current_time = time.time()
                
                if not message or message['type'] != 'message':
                    # NO MESSAGE: Either the target is actually lost or FPS is low. (to avoid breaking dt and PID continue!)
                    if last_target_time > 0.0:
                        time_since_last = current_time - last_target_time
                        if time_since_last <= 5.0:
                            # SEND LAST CALCULATED COMMAND to avoid MAVLink timeout for 5 seconds
                            if task == "Visual" and self.current_mode == "GUIDED":
                                self.send_attitude_target(last_final_roll_rad, last_final_pitch_rad, self.current_yaw_rad)
                        else:
                            # Hanged for 5 seconds! Failsafe: Straight flight
                            self.integral_error_x = 0.0
                            self.integral_error_y = 0.0
                            self.prev_derivative_x = 0.0
                            self.prev_derivative_y = 0.0
                            if task == "Visual" and self.current_mode == "GUIDED":
                                self.send_attitude_target(0.0, 0.0, self.current_yaw_rad)
                                last_final_roll_rad = 0.0
                                last_final_pitch_rad = 0.0
                                last_target_time = 0.0 # Don't come in again
                    continue
                
                # CALCULATE dt ONLY WHEN A NEW FRAME COMES (To avoid damaging the Derivative/D term!)
                dt = current_time - self.last_time
                if dt < 0.001: dt = 0.001 # Prevent division by zero error
                
                if dt > 0.1:
                    dt = 0.1
                
                self.last_time = current_time

                # Log defaults
                target_found = False
                target_area, error_x_deg, error_y_deg, heading_diff_deg = 0.0, 0.0, 0.0, 0.0
                p_term_roll, i_term_roll, d_term_roll, cmd_roll_rad, final_roll_deg = 0.0, 0.0, 0.0, 0.0, 0.0
                p_term_pitch, i_term_pitch, d_term_pitch, cmd_pitch_rad, final_pitch_deg = 0.0, 0.0, 0.0, 0.0, 0.0

                if message and message['type'] == 'message':
                    data_str = message['data'].decode('utf-8')
                    try:
                        bbox_data = ast.literal_eval(data_str)
                        if len(bbox_data) >= 4:
                            x, y, w, h = bbox_data[0:4]
                            
                            # Bounding box center (Gives 99% the same result as the old cv2.moments)
                            obj_x = x + (w / 2.0)
                            obj_y = y + (h / 2.0)
                            target_area = w * h
                            target_found = True

                            last_target_time = current_time
                    except Exception as e:
                        pass
                else:
                    target_found = False

                # 4. PID CALCULATIONS
                if target_found:
                    pixel_error_x = obj_x - self.center_x
                    pixel_error_y = obj_y - self.center_y

                    # Conversion to Degrees with Arctangent
                    error_x_deg = math.degrees(math.atan(pixel_error_x / self.fx))
                    error_y_deg = math.degrees(math.atan(pixel_error_y / self.fy))
                    heading_diff_deg = error_x_deg 

                    # Derivative and Low-Pass Filter
                    raw_derivative_x = (error_x_deg - self.prev_error_x) / dt
                    raw_derivative_y = (error_y_deg - self.prev_error_y) / dt
                    alpha = 0.02 # D term was reduced from 0.15 to 0.02 because it was too noisy (oscillatory) (Much softer brake)
                    derivative_x = (alpha * raw_derivative_x) + ((1.0 - alpha) * self.prev_derivative_x)
                    derivative_y = (alpha * raw_derivative_y) + ((1.0 - alpha) * self.prev_derivative_y)

                    if task == "Visual" and self.current_mode == "GUIDED":
                        self.integral_error_x = self.clamp(self.integral_error_x + error_x_deg * dt, -30.0, 30.0)
                        self.integral_error_y = self.clamp(self.integral_error_y + error_y_deg * dt, -30.0, 30.0)
                    else:
                        self.integral_error_x = 0.0
                        self.integral_error_y = 0.0

                    self.prev_error_x, self.prev_error_y = error_x_deg, error_y_deg
                    self.prev_derivative_x, self.prev_derivative_y = derivative_x, derivative_y

                    # Soft Deadzone
                    p_x = 0 if abs(error_x_deg) < self.deadzone_deg else error_x_deg - math.copysign(self.deadzone_deg, error_x_deg)
                    p_y = 0 if abs(error_y_deg) < self.deadzone_deg else error_y_deg - math.copysign(self.deadzone_deg, error_y_deg)

                    # PID Command Generation
                    p_term_roll = p_x * self.Kp_roll
                    i_term_roll = self.integral_error_x * self.Ki_roll
                    
                    p_term_pitch = p_y * self.Kp_pitch
                    i_term_pitch = self.integral_error_y * self.Ki_pitch

                    # --- NEW: D TERM CLAMPING --- We calculate raw D terms
                    raw_d_roll = derivative_x * self.Kd_roll
                    raw_d_pitch = derivative_y * self.Kd_pitch

                    # We clamp the D term to +/- 5 degrees
                    d_term_roll = self.clamp(raw_d_roll, -0.087, 0.087)
                    d_term_pitch = self.clamp(raw_d_pitch, -0.052, 0.052)
                    # -----------------------------------------------
                    
                    
                    cmd_roll_rad = p_term_roll + i_term_roll + d_term_roll
                    cmd_pitch_rad = -1 * (p_term_pitch + i_term_pitch + d_term_pitch) # Should the correct direction be determined before the flight?

                    cmd_roll_deg = math.degrees(cmd_roll_rad)
                    cmd_pitch_deg = math.degrees(cmd_pitch_rad)

                    final_roll_deg = self.clamp(cmd_roll_deg, -self.max_roll_deg, self.max_roll_deg)
                    final_pitch_deg = self.clamp(cmd_pitch_deg, -self.max_pitch_deg, self.max_pitch_deg)

                    final_roll_rad = math.radians(final_roll_deg)
                    final_pitch_rad = math.radians(final_pitch_deg)

                    # --- SENDING CONTROL TO THE AIRCRAFT ---
                    if task == "Visual" and self.current_mode == "GUIDED":
                        self.send_attitude_target(final_roll_rad, final_pitch_rad, self.current_yaw_rad)
                        print(f"[{task}] AUTONOMOUS TRACKING: Roll: {final_roll_deg:.1f} | Pitch: {final_pitch_deg:.1f}")
                    else:
                        print(f"[{task}] DEBUG - Target Found. Predictive Command -> Roll: {final_roll_deg:.1f} | Pitch: {final_pitch_deg:.1f}")
                        
                    last_final_roll_rad = final_roll_rad
                    last_final_pitch_rad = final_pitch_rad

                # 5. SAVE LOGS TO CSV
                log_row = [
                    f"{current_time:.4f}", f"{dt:.4f}", self.current_mode, int(target_found),
                    f"{error_x_deg:.2f}", f"{error_y_deg:.2f}", f"{heading_diff_deg:.2f}",
                    f"{p_term_roll:.6f}", f"{i_term_roll:.6f}", f"{d_term_roll:.6f}", f"{cmd_roll_rad:.6f}", f"{final_roll_deg:.2f}",
                    f"{p_term_pitch:.6f}", f"{i_term_pitch:.6f}", f"{d_term_pitch:.6f}", f"{cmd_pitch_rad:.6f}", f"{final_pitch_deg:.2f}",
                    f"{target_area:.1f}"
                ]
                self.csv_writer.writerow(log_row)

        except KeyboardInterrupt:
            print("\nShutting down...")
            self.log_file.close()

if __name__ == '__main__':
    # SITL or update IP and Port according to your hardware
    guidance = GoatGuidTest(connection_str='udp:127.0.0.1:14552')
    guidance.run()