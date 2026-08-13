#!/usr/bin/env python3

# ==============================================================================
# 3.1 PRO - IHA GUDUM SISTEMI FUZYON KODU
# ==============================================================================
# Bu kod, "TZI" mimarisi (Moduler Threading, Angular Error, Failsafe) ile 
# "Emir" ucus dinamiklerinin (Rate-Based PID, Coverage Speed, EMA Filter)
# 3.1 Pro konsepti altinda birlestirildigi nihai ucus sistemidir.
# ==============================================================================

import sys
sys.path.insert(0, '/home/tzi4/gudum')
import numpy as np
import math
import redis
import datetime
import os
import atexit
import csv
from mavlinkHandler import MAVLinkHandlerPymavlink as MAVLinkHandler
import json
import time 
import threading
import logging
import shutil
from pymavlink import mavutil

# ─── SABİTLER ───
RESOLUTION_W = 1280
RESOLUTION_H = 720
CAMERA_FOCAL_LENGTH = 3045.737  # Emir'in goat_gimbal.py fx degeri
MIN_ALTITUDE = 10.0
MAX_ALTITUDE = 200.0
MAX_DELTA_HEADING = 35.0  # Emir'in limiti
MAX_DELTA_ALTITUDE = 10.0 # Emir'in limiti
UAV_PORT = '14552' # Emir'in kodunda 14552, tzi'de 14553 kullanilmis.

# Kamera->Body rotasyonu (kamera z ileri, x sag, y asagi kabulu)
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

# ----------------- FLIGHT LOGGER (3.1 PRO) -----------------
class FlightLogger_3_1_Pro:
    def __init__(self, file_prefix="flight_log_3_1_pro"):
        log_dir = "loglar_3_1_pro"
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)
            
        self.log_filename = os.path.join(log_dir, f"{file_prefix}_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
        self.log_file = open(self.log_filename, mode='w', newline='')
        self.csv_writer = csv.writer(self.log_file)
        self.row_count = 0
        self.flush_interval = 10
        
        headers = [
            "timestamp", "dt", "flight_mode", "system_state", "target_found",
            "bbox_x", "bbox_y", "bbox_w", "bbox_h", "target_area", "coverage_pct",
            "current_alt_m", "roll_deg", "pitch_deg", "yaw_deg",
            "raw_error_x_deg", "raw_error_y_deg", "stab_error_x_deg", "stab_error_y_deg",
            "deriv_x", "deriv_y", "dz_active_x", "dz_active_y",
            "p_term_head", "i_term_head", "d_term_head", "target_heading_deg", "heading_rate",
            "p_term_alt", "i_term_alt", "d_term_alt", "target_alt_m", "alt_rate",
            "speed_error", "target_speed"
        ]
        self.csv_writer.writerow(headers)
        self.log_file.flush()
        
    def log(self, row_data):
        self.csv_writer.writerow(row_data)
        self.row_count += 1
        if self.row_count % self.flush_interval == 0:
            self.log_file.flush()

    def close(self):
        self.log_file.flush()
        self.log_file.close()

# ----------------- DIRECT CONTROL COMMANDER THREAD (3.1 PRO) -----------------
class TestCommander_3_1_Pro(threading.Thread):
    def __init__(self, mav_handler, rate_hz=10):
        super().__init__(daemon=True, name="TestCommander_3_1_Pro")
        self.mav_handler = mav_handler
        self.master = mav_handler.master
        self.rate = rate_hz
        self.running = True
        self.lock = threading.Lock()

        # Telemetri talepleri
        self.mav_handler.request_message_interval([mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD], 10)

        # Hedef Degerler
        self.target_heading_deg = 0.0
        self.heading_rate_dps = 0.85
        
        self.target_alt_m = 100.0
        self.alt_rate_ms = 0.3
        
        self.target_airspeed_ms = 17.0
        self.active = False

        self.last_att = None
        self.last_loc = None
        self.last_airspeed = None
        self.last_mode = None

    def update(self, heading_deg, heading_rate, alt_m, alt_rate, airspeed_ms):
        with self.lock:
            self.target_heading_deg = float(heading_deg)
            self.heading_rate_dps = float(heading_rate)
            self.target_alt_m = float(alt_m)
            self.alt_rate_ms = float(alt_rate)
            self.target_airspeed_ms = float(airspeed_ms)
            self.active = True

    def _send_heading(self, heading_deg, rate_dps):
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_GUIDED_CHANGE_HEADING, 
            0, 1, heading_deg, rate_dps, 0, 0, 0, 0
        )

    def _send_altitude(self, alt_m, rate_ms):
        # 3 = MAV_FRAME_GLOBAL_RELATIVE_ALT
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_GUIDED_CHANGE_ALTITUDE, 
            0, 0, rate_ms, 0, 0, 0, 0, alt_m
        )

    def _send_airspeed(self, airspeed_ms):
        if self.last_mode != 'GUIDED':
            return
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 
            0, 0, airspeed_ms, -1, 0, 0, 0, 0
        )

    def _read_telemetry(self):
        att_msg = self.mav_handler.master.messages.get('ATTITUDE', None)
        if att_msg:
            self.last_att = (math.degrees(att_msg.pitch), math.degrees(att_msg.roll), math.degrees(att_msg.yaw))
        
        loc_msg = self.mav_handler.master.messages.get('GLOBAL_POSITION_INT', None)
        if loc_msg:
            self.last_loc = (loc_msg.lat/1e7, loc_msg.lon/1e7, loc_msg.relative_alt/1000.0) 
        
        vfr = self.master.messages.get('VFR_HUD', None)
        if vfr:
            self.last_airspeed = vfr.airspeed
            
        hb = self.master.messages.get('HEARTBEAT', None)
        if hb:
            self.last_mode = mavutil.mode_string_v10(hb)

    def run(self):
        period = 1.0 / self.rate
        while self.running:
            if self.active:
                self._read_telemetry()
                with self.lock:
                    hdg = self.target_heading_deg
                    hdg_rate = self.heading_rate_dps
                    alt = self.target_alt_m
                    alt_rate = self.alt_rate_ms
                    spd = self.target_airspeed_ms
                
                self._send_heading(hdg, hdg_rate)
                self._send_altitude(alt, alt_rate)
                self._send_airspeed(spd)
            time.sleep(period)

    def stop(self):
        self.running = False

# ----------------- SYSTEM BASE CLASS (3.1 PRO) -----------------
class System_3_1_Pro():
    def __init__(self):
        self.init_logger()
        self.r = redis.Redis(host='localhost', port=6379, db=0)
        self.logger.debug('Redis connection established. (3.1 Pro)')
        self.p = self.r.pubsub()
        self.p.subscribe('tracker_bbox')

        self.W = RESOLUTION_W
        self.H = RESOLUTION_H
        self.center_x = self.W / 2.0
        self.center_y = self.H / 2.0
        self.fx = CAMERA_FOCAL_LENGTH
        self.fy = CAMERA_FOCAL_LENGTH

        self.K = np.array([
            [self.fx, 0, self.center_x],
            [0, self.fy, self.center_y],
            [0, 0, 1]
        ])
        self.K_inv = np.linalg.inv(self.K)

        self.MIN_ALTITUDE = MIN_ALTITUDE
        self.MAX_ALTITUDE = MAX_ALTITUDE

        self.mavlink_handler = MAVLinkHandler(f'127.0.0.1:{UAV_PORT}')
        
        self.mavlink_handler.master.mav.request_data_stream_send(
            self.mavlink_handler.master.target_system, self.mavlink_handler.master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_POSITION, 10, 1)
        self.mavlink_handler.master.mav.request_data_stream_send(
            self.mavlink_handler.master.target_system, self.mavlink_handler.master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 10, 1)

        self.logger.debug('Connected to the aircraft. (3.1 Pro)')
        self.r.set('guid','False')

        self._mavlink_reader_thread = threading.Thread(target=self._mavlink_reader, daemon=True, name="MAVLinkReader_3_1_Pro")
        self._mavlink_reader_thread.start()

        atexit.register(self.exit_handler)

    def _mavlink_reader(self):
        while True:
            try:
                self.mavlink_handler.master.recv_match(blocking=True, timeout=0.1)
            except Exception:
                time.sleep(0.01)

    def exit_handler(self):
        self.mavlink_handler.master.close()
        self.logger.debug('Connection closed.')

    def init_logger(self):
        self.logger = logging.Logger('3_1_pro_logger')
        self.logger.setLevel(logging.DEBUG)
        c_handler = logging.StreamHandler()
        
        log_file_path = 'tzi_3_1_pro_guidance.log'
        old_logs_dir = "loglar_3_1_pro"
        if not os.path.exists(old_logs_dir):
            os.makedirs(old_logs_dir)
            
        if os.path.exists(log_file_path):
            timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
            new_log_file_path = os.path.join(old_logs_dir, f"tzi_3_1_pro_{timestamp}.log")
            shutil.move(log_file_path, new_log_file_path)
            
        f_handler = logging.FileHandler(log_file_path)
        c_handler.setLevel(logging.DEBUG)
        f_handler.setLevel(logging.DEBUG)

        c_format = logging.Formatter('%(message)s')
        f_format = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
        c_handler.setFormatter(c_format)
        f_handler.setFormatter(f_format)

        self.logger.addHandler(c_handler)
        self.logger.addHandler(f_handler)


# ----------------- FUSION GUIDANCE LOGIC (3.1 PRO) -----------------
class FusionGuidance_3_1_Pro(System_3_1_Pro):
    def __init__(self):
        super().__init__()

        self.flight_logger = FlightLogger_3_1_Pro()

        self.latest_bbox = None
        self.latest_bbox_time = None
        self.bbox_lock = threading.Lock()

        # PID Parametreleri (Emir'in kodundan uyarlanmis)
        self.Kp_heading = 3.0
        self.Ki_heading = 0.0
        self.Kd_heading = 0.0
        self.Kp_rate = 1.2
        self.Ki_rate = 0.0
        self.Kd_rate = 0.12

        self.Kp_alt = 1.5
        self.Ki_alt = 0.0
        self.Kd_alt = 0.1
        self.Kp_alt_rate = 1.5
        self.Ki_alt_rate = 0.0
        self.Kd_alt_rate = 0.1

        self.Kp_speed = 0.2
        self.Ki_speed = 0.03
        self.Kd_speed = 0.0

        # State Variables
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

        # Limitler ve Config
        self.min_heading_rate = 0.85
        self.max_heading_rate = 7.5
        self.min_alt_rate = 0.3
        self.max_alt_rate = 5.0
        self.min_speed = 14.0
        self.max_speed = 22.0
        self.base_speed = 17.0

        self.target_coverage_pct = 6.0
        self.speed_integral_band = 5.0
        self.speed_slew_rate = 1.0
        self.coverage_alpha = 0.35
        self.filtered_coverage = self.target_coverage_pct

        self.deadzone_deg = 0.78
        self.max_heading_change_deg = MAX_DELTA_HEADING
        self.max_alt_change_m = MAX_DELTA_ALTITUDE

        self.last_pid_time = None
        self.last_speed = self.base_speed

        self.cmd_thread = TestCommander_3_1_Pro(self.mavlink_handler, rate_hz=10) # 10Hz komut yenileme
        self.cmd_thread.start()

    def clamp(self, value, min_val, max_val):
        return max(min(value, max_val), min_val)

    def guide_aircraft(self, bbox, current_time):
        object_x, object_y = bbox[0] + (bbox[2] / 2), bbox[1] + (bbox[3] / 2)
        bbox_w, bbox_h = bbox[2], bbox[3]

        # Stabilizasyon (Virtual Gimbal)
        if self.K_inv is not None:
            p_raw = np.array([object_x, object_y, 1.0])
            r_cam = self.K_inv @ p_raw
            r_body = R_c_b @ r_cam
            
            att_msg = self.mavlink_handler.master.messages.get('ATTITUDE', None)
            roll_rad = att_msg.roll if att_msg else 0.0
            pitch_rad = att_msg.pitch if att_msg else 0.0
            yaw_rad = att_msg.yaw if att_msg else 0.0

            R_stab = compute_R_b_e(roll_rad, pitch_rad, 0)
            r_virt_body = R_stab @ r_body
            r_virt_cam = R_c_b_T @ r_virt_body
            p_virt_hom = self.K @ r_virt_cam
            
            if p_virt_hom[2] != 0:
                u_virt = p_virt_hom[0] / p_virt_hom[2]
                v_virt = p_virt_hom[1] / p_virt_hom[2]
            else:
                u_virt, v_virt = 0, 0

            # Hata Acilari (Derece - TZI Okulu)
            stab_error_x_deg = math.degrees(math.atan((u_virt - self.center_x) / self.fx))
            stab_error_y_deg = math.degrees(math.atan((v_virt - self.center_y) / self.fy))

            raw_error_x_deg = math.degrees(math.atan((object_x - self.center_x) / self.fx))
            raw_error_y_deg = math.degrees(math.atan((object_y - self.center_y) / self.fy))

            current_mode = "UNKNOWN"
            hb = self.mavlink_handler.master.messages.get('HEARTBEAT', None)
            if hb:
                current_mode = mavutil.mode_string_v10(hb)

            if current_mode != 'GUIDED':
                self._reset_pid_states()
                return

            # Zaman Deltasi (dt)
            if getattr(self, 'last_pid_time', None) is None:
                dt = 0.033
                self.prev_error_x = stab_error_x_deg
                self.prev_error_y = stab_error_y_deg
            else:
                dt = current_time - self.last_pid_time
                if dt <= 0.001: dt = 0.033
            self.last_pid_time = current_time

            # Turev (Low Pass Filter - EMA - Emir Okulu)
            alpha = 0.05
            raw_deriv_x = (stab_error_x_deg - self.prev_error_x) / dt
            raw_deriv_y = (stab_error_y_deg - self.prev_error_y) / dt
            deriv_x = (alpha * raw_deriv_x) + ((1.0 - alpha) * self.prev_derivative_x)
            deriv_y = (alpha * raw_deriv_y) + ((1.0 - alpha) * self.prev_derivative_y)

            # --- 1. HEADING & HEADING RATE KONTROLU (3.1 PRO) ---
            dz_x = 1 if abs(stab_error_x_deg) < self.deadzone_deg else 0
            p_x = 0.0 if dz_x else stab_error_x_deg - math.copysign(self.deadzone_deg, stab_error_x_deg)

            if not dz_x: self.integral_error_x += stab_error_x_deg * dt
            else: self.integral_error_x *= 0.99
            self.integral_error_x = self.clamp(self.integral_error_x, -10.0, 10.0)

            p_term_heading = p_x * self.Kp_heading
            i_term_heading = self.integral_error_x * self.Ki_heading
            d_term_heading = deriv_x * self.Kd_heading
            cmd_head_deg = self.clamp(p_term_heading + i_term_heading + d_term_heading, -self.max_heading_change_deg, self.max_heading_change_deg)
            
            yaw_deg = math.degrees(yaw_rad) % 360.0
            target_heading = (yaw_deg + cmd_head_deg) % 360.0

            # Heading Rate (Donus Hizi)
            rate_error = abs(p_x)
            raw_deriv_rate = (rate_error - self.prev_error_rate) / dt
            deriv_rate = (alpha * raw_deriv_rate) + ((1.0 - alpha) * self.prev_derivative_rate)
            self.integral_error_rate = self.clamp(self.integral_error_rate + rate_error * dt, 0, 5.0)

            p_term_rate = rate_error * self.Kp_rate
            i_term_rate = self.integral_error_rate * self.Ki_rate
            d_term_rate = deriv_rate * self.Kd_rate
            heading_rate = self.clamp(p_term_rate + i_term_rate + d_term_rate, self.min_heading_rate, self.max_heading_rate)
            if dz_x: heading_rate = self.min_heading_rate

            # --- 2. ALTITUDE & ALTITUDE RATE KONTROLU (3.1 PRO) ---
            dz_y = 1 if abs(stab_error_y_deg) < self.deadzone_deg else 0
            p_y = 0.0 if dz_y else stab_error_y_deg - math.copysign(self.deadzone_deg, stab_error_y_deg)

            if not dz_y: self.integral_error_y += stab_error_y_deg * dt
            else: self.integral_error_y *= 0.99
            self.integral_error_y = self.clamp(self.integral_error_y, -10.0, 10.0)

            p_term_alt = p_y * self.Kp_alt
            i_term_alt = self.integral_error_y * self.Ki_alt
            d_term_alt = deriv_y * self.Kd_alt
            
            cmd_alt_m = self.clamp(-1 * (p_term_alt + i_term_alt + d_term_alt), -self.max_alt_change_m, self.max_alt_change_m)
            
            loc_msg = self.mavlink_handler.master.messages.get('GLOBAL_POSITION_INT', None)
            current_alt = (loc_msg.relative_alt / 1000.0) if loc_msg else 100.0
            target_alt = max(self.MIN_ALTITUDE, min(self.MAX_ALTITUDE, current_alt + cmd_alt_m))

            # Altitude Rate
            alt_rate_error = abs(p_y)
            raw_deriv_alt_rate = (alt_rate_error - self.prev_error_alt_rate) / dt
            deriv_alt_rate = (alpha * raw_deriv_alt_rate) + ((1.0 - alpha) * self.prev_derivative_alt_rate)
            self.integral_error_alt_rate = self.clamp(self.integral_error_alt_rate + alt_rate_error * dt, 0, 5.0)

            p_term_alt_rate = alt_rate_error * self.Kp_alt_rate
            i_term_alt_rate = self.integral_error_alt_rate * self.Ki_alt_rate
            d_term_alt_rate = deriv_alt_rate * self.Kd_alt_rate
            altitude_rate = self.clamp(p_term_alt_rate + i_term_alt_rate + d_term_alt_rate, self.min_alt_rate, self.max_alt_rate)
            if dz_y: altitude_rate = self.min_alt_rate

            # --- 3. AIRSPEED KONTROLU (Coverage Based) ---
            h_cov = (bbox_w / self.W) * 100.0
            v_cov = (bbox_h / self.H) * 100.0
            coverage = max(h_cov, v_cov)
            
            self.filtered_coverage = (self.coverage_alpha * coverage) + ((1.0 - self.coverage_alpha) * self.filtered_coverage)
            speed_error = self.target_coverage_pct - self.filtered_coverage

            p_input_speed = speed_error
            raw_deriv_speed = (speed_error - self.prev_error_speed) / dt
            deriv_speed = (alpha * raw_deriv_speed) + ((1.0 - alpha) * self.prev_derivative_speed)

            if abs(speed_error) < self.speed_integral_band:
                self.integral_error_speed += speed_error * dt
            self.integral_error_speed = self.clamp(self.integral_error_speed, -100.0, 100.0)

            p_term_speed = p_input_speed * self.Kp_speed
            i_term_speed = self.integral_error_speed * self.Ki_speed
            d_term_speed = deriv_speed * self.Kd_speed

            cmd_speed_raw = self.base_speed + (p_term_speed + i_term_speed + d_term_speed)
            cmd_speed = self.clamp(cmd_speed_raw, self.min_speed, self.max_speed)

            # Slew Rate
            max_speed_delta = self.speed_slew_rate * dt
            target_speed = self.clamp(cmd_speed, self.last_speed - max_speed_delta, self.last_speed + max_speed_delta)

            # Komutlari Gonder
            self.cmd_thread.update(target_heading, heading_rate, target_alt, altitude_rate, target_speed)

            # State Guncelleme
            self.prev_error_x, self.prev_error_y = stab_error_x_deg, stab_error_y_deg
            self.prev_derivative_x, self.prev_derivative_y = deriv_x, deriv_y
            self.prev_error_rate = rate_error
            self.prev_derivative_rate = deriv_rate
            self.prev_error_alt_rate = alt_rate_error
            self.prev_derivative_alt_rate = deriv_alt_rate
            self.prev_error_speed = speed_error
            self.prev_derivative_speed = deriv_speed
            self.last_speed = target_speed

            # Anti-windup
            if abs(cmd_head_deg) >= self.max_heading_change_deg: self.integral_error_x *= 0.9
            if abs(cmd_alt_m) >= self.max_alt_change_m: self.integral_error_y *= 0.9

            # CSV Loglama (3.1 Pro)
            target_area = bbox_w * bbox_h
            self.flight_logger.log([
                f"{current_time:.4f}", f"{dt:.4f}", current_mode, "TRACKING_3_1_PRO", 1,
                int(bbox[0]), int(bbox[1]), bbox_w, bbox_h, target_area, f"{self.filtered_coverage:.2f}",
                f"{current_alt:.2f}", math.degrees(roll_rad), math.degrees(pitch_rad), yaw_deg,
                f"{raw_error_x_deg:.4f}", f"{raw_error_y_deg:.4f}", f"{stab_error_x_deg:.4f}", f"{stab_error_y_deg:.4f}",
                f"{deriv_x:.4f}", f"{deriv_y:.4f}", dz_x, dz_y,
                f"{p_term_heading:.4f}", f"{i_term_heading:.4f}", f"{d_term_heading:.4f}", f"{target_heading:.2f}", f"{heading_rate:.2f}",
                f"{p_term_alt:.4f}", f"{i_term_alt:.4f}", f"{d_term_alt:.4f}", f"{target_alt:.2f}", f"{altitude_rate:.2f}",
                f"{speed_error:.2f}", f"{target_speed:.2f}"
            ])

    def _reset_pid_states(self):
        self.integral_error_x = 0.0
        self.integral_error_y = 0.0
        self.integral_error_rate = 0.0
        self.integral_error_alt_rate = 0.0
        self.integral_error_speed = 0.0
        self.last_pid_time = None

    def _parse_bbox(self, data):
        if isinstance(data, (list, tuple)) and len(data) >= 4:
            x, y, w, h = data[:4]
            if w is not None:
                return (int(x), int(y), int(w), int(h))
        return None

    def _redis_listener(self):
        self.logger.debug("Redis listener thread başladi (3.1 Pro).")
        for message in self.p.listen():
            if message['type'] != 'message': continue
            
            guid_message = self.r.get('gorev')
            if guid_message is None or guid_message.decode('utf-8').lower() != 'goruntulu':
                continue

            try:
                data = json.loads(message['data'].decode('utf-8'))
                bbox = self._parse_bbox(data)
                if bbox is not None:
                    now = time.time()
                    with self.bbox_lock:
                        self.latest_bbox = bbox
                        self.latest_bbox_time = now
            except:
                continue

    def _vision_processor(self):
        self.logger.debug("Vision processor thread başladı (3.1 Pro).")
        period = 1.0 / 30.0
        last_processed_time = None
        last_target_time = 0.0

        while True:
            time.sleep(period)
            with self.bbox_lock:
                bbox = self.latest_bbox
                bbox_time = self.latest_bbox_time

            if bbox is not None and bbox_time != last_processed_time:
                last_processed_time = bbox_time
                last_target_time = time.time()
                self.guide_aircraft(bbox, bbox_time)
            else:
                now = time.time()
                if last_target_time > 0.0:
                    time_since_last = now - last_target_time
                    if time_since_last > 5.0:
                        # Failsafe 3.1 Pro
                        self._reset_pid_states()
                        self.cmd_thread.active = False
                        self.logger.warning("[FAILSAFE - 3.1 PRO] HEDEF 5 SANİYEDEN UZUN SÜREDİR KAYIP!")
                        
                        self.flight_logger.log([
                            f"{now:.4f}", "0", "FAILSAFE", "LOST_3_1_PRO", 0,
                            0, 0, 0, 0, 0, "0.0",
                            "0.0", 0, 0, 0,
                            "0", "0", "0", "0",
                            "0", "0", 0, 0,
                            "0", "0", "0", "0", "0",
                            "0", "0", "0", "0", "0",
                            "0", "0"
                        ])

    def run(self):
        listener_thread = threading.Thread(target=self._redis_listener, daemon=True, name="RedisListener_3_1_Pro")
        listener_thread.start()

        processor_thread = threading.Thread(target=self._vision_processor, daemon=True, name="VisionProcessor_3_1_Pro")
        processor_thread.start()

        self.logger.debug("Tüm thread'ler başlatıldı (3.1 Pro). Ana thread bekliyor...")
        listener_thread.join()
        processor_thread.join()

if __name__ == '__main__':
    fuse = FusionGuidance_3_1_Pro()
    fuse.run()
