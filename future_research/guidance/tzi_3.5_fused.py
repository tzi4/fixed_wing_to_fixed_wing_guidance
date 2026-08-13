#!/usr/bin/env python3
import sys
sys.path.insert(0, '/home/tzi4/gudum')

import numpy as np
import math
import redis
import datetime
import os
import atexit
from collections import deque
import json
import time 
import threading
import logging
import shutil
import csv
from pymavlink import mavutil
from mavlinkHandler import MAVLinkHandlerPymavlink as MAVLinkHandler

# ─── SABİTLER ───
RESOLUTION_W = 1280
RESOLUTION_H = 720
CAMERA_HFOV_RAD = 0.415  # Emir'in kodundaki HFOV (radyan)
MIN_ALTITUDE = 10
MAX_ALTITUDE = 200
UAV_PORT = '14553'

# Kamera->Body rotasyonu (kamera z ileri, x sağ, y aşağı)
R_c_b = np.array([[0,0,1],
                  [1,0,0],
                  [0,1,0]], dtype=float)
R_c_b_T = R_c_b.T

def compute_R_b_e(roll, pitch, yaw):
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp   , cp*sr           , cp*cr           ]
    ])

# ─── DETAYLI UÇUŞ LOGLAYICI (FlightLogger) ───
class FlightLogger:
    def __init__(self, file_prefix="flight_log_3_5_flash"):
        self.log_filename = f"{file_prefix}_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        self.log_file = open(self.log_filename, mode='w', newline='')
        self.csv_writer = csv.writer(self.log_file)
        self.row_count = 0
        self.flush_interval = 10
        
        headers = [
            "timestamp", "elapsed_s", "dt", "frame_num",
            "flight_mode", "gorev", "system_state", "target_found",
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
            "anti_windup_x", "anti_windup_y"
        ]
        self.csv_writer.writerow(headers)
        self.log_file.flush()
        print(f"Detailed CSV Log created: {self.log_filename}")
        
    def log(self, row_data):
        self.csv_writer.writerow(row_data)
        self.row_count += 1
        if self.row_count % self.flush_interval == 0:
            self.log_file.flush()

    def close(self):
        self.log_file.flush()
        self.log_file.close()

# ─── OTOPİLOT COMMANDER THREAD (TestCommander) ───
class TestCommander(threading.Thread):
    """
    - Heading:  MAV_CMD_GUIDED_CHANGE_HEADING  (43002) ile değişken rate kullanarak yönlendirme
    - Altitude: MAV_CMD_GUIDED_CHANGE_ALTITUDE (43001) ile değişken climb/descent rate kullanarak yönlendirme
    - Airspeed: MAV_CMD_DO_CHANGE_SPEED        (178) ile airspeed hızlandırma kontrolü
    """
    def __init__(self, mav_handler, rate_hz=5):
        super().__init__(daemon=True)
        self.mav_handler = mav_handler         # MAVLinkHandlerPymavlink objesi
        self.master = mav_handler.master       # pymavlink master
        self.rate = rate_hz
        self.running = True
        self.lock = threading.Lock()

        # VFR_HUD mesajını iste (airspeed ölçümü için)
        self.mav_handler.request_message_interval(
            [mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD], 10  # 10 Hz
        )

        self.target_heading_deg = 0.0
        self.target_alt_m = 100.0
        self.target_airspeed_ms = 17.0
        self.target_heading_rate = 1.0
        self.target_altitude_rate = 0.5
        self.active = False

        # Telemetri cache
        self.last_att = None       # (pitch_deg, roll_deg, yaw_deg)
        self.last_loc = None       # (lat, lon, alt_m)
        self.last_airspeed = None  # m/s
        self.last_mode = None      # Flight Mode string (e.g. 'GUIDED')

    def update(self, heading_deg, alt_m, airspeed_ms, heading_rate, altitude_rate):
        """Thread-safe hedef güncelleme."""
        with self.lock:
            self.target_heading_deg = float(heading_deg)
            self.target_alt_m = float(alt_m)
            self.target_airspeed_ms = float(airspeed_ms)
            self.target_heading_rate = float(heading_rate)
            self.target_altitude_rate = float(altitude_rate)
            self.active = True

    def _send_heading(self, heading_deg, heading_rate):
        """
        param1: 1 = raw magnetic heading (burun yönü)
        param2: heading (degrees, 0-360)
        param3: heading rate (deg/s) - değişken rate
        """
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            43002,  # MAV_CMD_GUIDED_CHANGE_HEADING
            0,      # confirmation
            1,      # param1: 1 = raw magnetic heading
            heading_deg,  # param2: hedef heading
            heading_rate,  # param3: dönüş hızı (deg/s)
            0, 0, 0, 0  # param4-7 (unused)
        )

    def _send_altitude(self, alt_m, altitude_rate):
        """
        param2: Climb/Descent rate (m/s)
        param7: Desired altitude (AMSL / Relative depending on frame)
        """
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            43001,  # MAV_CMD_GUIDED_CHANGE_ALTITUDE
            0,      # confirmation
            0,      # param1: 0 = default frame
            altitude_rate,  # param2: tırmanma/alçalma hızı (m/s)
            0, 0, 0, 0,  # param3-6
            alt_m   # param7: hedef irtifa
        )

    def _send_airspeed(self, airspeed_ms):
        """
        param1: SPEED_TYPE (0 = airspeed)
        param2: hedef hız (m/s)
        param3: throttle (-1 = değişiklik yok)
        """
        if self.last_mode != 'GUIDED':
            return

        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            178,            # MAV_CMD_DO_CHANGE_SPEED
            0,              # confirmation
            0,              # param1: 0 = airspeed
            airspeed_ms,    # param2: hedef hız (m/s)
            -1,             # param3: throttle
            0, 0, 0, 0      # param4-7
        )

    def _read_telemetry(self):
        """Pymavlink dahili cache'inden telemetri verilerini okur (non-blocking)."""
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
                    alt = self.target_alt_m
                    spd = self.target_airspeed_ms
                    hdg_rate = self.target_heading_rate
                    alt_rate = self.target_altitude_rate
                
                # Uçuş testi öncesinde hiçbir hardcoded override barındırmaz
                self._send_heading(hdg, hdg_rate)
                self._send_altitude(alt, alt_rate)
                self._send_airspeed(spd)
                
            time.sleep(period)

    def stop(self):
        self.running = False


# ─── TEMEL SİSTEM SINIFI (System) ───
class System():
    def __init__(self):
        self.init_logger()
        self.r = redis.Redis(host='localhost', port=6379, db=0)
        self.logger.debug('Redis connection established.')
        self.p = self.r.pubsub()
        self.p.subscribe('tracker_bbox')

        # GÖRSEL ALAN VE KAMERA KALİBRASYON MATRİSİ
        self.W = RESOLUTION_W
        self.H = RESOLUTION_H
        self.center_x = self.W / 2.0
        self.center_y = self.H / 2.0

        # Odak Uzaklığı Hesabı (HFOV üzerinden)
        self.fx = self.center_x / math.tan(CAMERA_HFOV_RAD / 2.0)
        self.fy = self.fx
        self.K = np.array([
            [self.fx, 0, self.center_x],
            [0, self.fy, self.center_y],
            [0, 0, 1]
        ])
        self.logger.debug(f"Camera Intrinsic Matrix K:\n{self.K}")
        self.K_inv = np.linalg.inv(self.K)
        self.logger.debug(f"Inverse K Matrix:\n{self.K_inv}")

        self.MIN_ALTITUDE = MIN_ALTITUDE
        self.MAX_ALTITUDE = MAX_ALTITUDE

        # Otopilot Bağlantısı
        self.mavlink_handler = MAVLinkHandler(f'127.0.0.1:{UAV_PORT}')
        self.logger.debug(f'Connected to the aircraft on port {UAV_PORT}.')
        self.r.set('guid', 'False')

        # MAVLink veri okuyucu thread (cache güncelleme)
        self._mavlink_reader_thread = threading.Thread(
            target=self._mavlink_reader, daemon=True, name="MAVLinkReader")
        self._mavlink_reader_thread.start()
        self.logger.debug('MAVLink reader thread started.')

        # Çıkış yöneticisi
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
        self.logger = logging.Logger('tzi_3_5_flash')
        self.logger.setLevel(logging.DEBUG)
        
        c_handler = logging.StreamHandler()
        log_file_path = 'tzi_3_5_flash_guidance.log'
        old_logs_dir = "Logs"
        
        if not os.path.exists(old_logs_dir):
            os.makedirs(old_logs_dir)
            
        if os.path.exists(log_file_path):
            timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
            new_log_file_name = f"tzi_3_5_flash_guidance_{timestamp}.log"
            new_log_file_path = os.path.join(old_logs_dir, new_log_file_name)
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


# ─── BİRLEŞTİRİLMİŞ KILAVUZLUK SINIFI (tziGuidance) ───
class tziGuidance(System):
    def __init__(self):
        super().__init__()

        self.last_message_time = None

        # BBox cache
        self.latest_bbox = None
        self.latest_bbox_time = None
        self.bbox_lock = threading.Lock()
        
        # Başlangıç Değerleri
        self.test_heading_deg = 0.0
        self.test_alt_m = 100.0
        self.test_airspeed_ms = 17.0
        self.test_heading_rate = 1.0
        self.test_altitude_rate = 0.5

        # ─── CONTROL PARAMS & GAINS (Emir'in Uçuşta Test Ettiği Değerler) ───
        self.Kp_heading = 3.0
        self.Ki_heading = 0.0
        self.Kd_heading = 0.0

        self.Kp_alt = 1.5
        self.Ki_alt = 0.0
        self.Kd_alt = 0.1

        # HEADING RATE PID (değişken dönüş hızı)
        self.Kp_rate = 1.2
        self.Ki_rate = 0.0
        self.Kd_rate = 0.12
        self.min_heading_rate = 0.85
        self.max_heading_rate = 7.5

        # ALTITUDE RATE PID (değişken tırmanma/alçalma hızı)
        self.Kp_alt_rate = 1.5
        self.Ki_alt_rate = 0.0
        self.Kd_alt_rate = 0.1
        self.min_alt_rate = 0.3
        self.max_alt_rate = 5.0

        # SPEED PID (Menzil Takip Kontrolü)
        self.Kp_speed = 0.2
        self.Ki_speed = 0.03
        self.Kd_speed = 0.0
        self.min_speed = 14.0
        self.max_speed = 22.0
        self.base_speed = 17.0
        self.target_coverage_pct = 6.0
        self.speed_integral_band = 5.0
        self.speed_slew_rate = 1.0
        self.coverage_alpha = 0.35

        # Eşik ve Sınırlar
        self.max_heading_change_deg = 35.0
        self.max_alt_change_m = 10.0
        self.deadzone_deg = 0.78

        # PID Durum Değişkenleri
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
        self.last_cmd_send_time = 0.0
        self.frame_counter = 0
        self.start_time = time.time()

        # Detaylı CSV Kaydedici
        self.flight_logger = FlightLogger()

        # Commander Başlat
        self.cmd_thread = TestCommander(self.mavlink_handler, rate_hz=5)
        self.cmd_thread.start()

    def guide_aircraft(self, bbox, current_time):
        self.frame_counter += 1
        bbox_x, bbox_y, bbox_w, bbox_h = bbox[0], bbox[1], bbox[2], bbox[3]
        obj_x, obj_y = bbox_x + (bbox_w / 2.0), bbox_y + (bbox_h / 2.0)
        target_area = bbox_w * bbox_h
        
        # 1) Sanal Gimbal Piksel Stabilizasyonu (Back-Projection & Re-Projection)
        p_raw = np.array([obj_x, obj_y, 1.0])
        r_cam = self.K_inv @ p_raw
        r_body = R_c_b @ r_cam
        
        att_msg = self.mavlink_handler.master.messages.get('ATTITUDE', None)
        if att_msg:
            roll_rad = att_msg.roll
            pitch_rad = att_msg.pitch
            yaw_rad = att_msg.yaw
        else:
            roll_rad, pitch_rad, yaw_rad = 0.0, 0.0, 0.0

        yaw_deg = math.degrees(yaw_rad) % 360.0
        
        R_stab = compute_R_b_e(roll_rad, pitch_rad, 0.0)
        r_virt_body = R_stab @ r_body
        r_virt_cam = R_c_b_T @ r_virt_body
        p_virt_hom = self.K @ r_virt_cam
        
        if p_virt_hom[2] != 0:
            stab_x = p_virt_hom[0] / p_virt_hom[2]
            stab_y = p_virt_hom[1] / p_virt_hom[2]
        else:
            stab_x, stab_y = obj_x, obj_y

        # 2) Çözünürlük Bağımsız Açısal Hata Hesabı
        stab_error_x_deg = math.degrees(math.atan((stab_x - self.center_x) / self.fx))
        stab_error_y_deg = math.degrees(math.atan((stab_y - self.center_y) / self.fy))

        raw_error_x_deg = math.degrees(math.atan((obj_x - self.center_x) / self.fx))
        raw_error_y_deg = math.degrees(math.atan((obj_y - self.center_y) / self.fy))

        # Uçuş Modu Alımı
        current_mode = "UNKNOWN"
        hb = self.mavlink_handler.master.messages.get('HEARTBEAT', None)
        if hb:
            current_mode = mavutil.mode_string_v10(hb)

        # Görev Durumu Alımı
        gorev_bytes = self.r.get('gorev')
        gorev = gorev_bytes.decode('utf-8') if gorev_bytes else "Bilinmiyor"

        # GUIDED modda değilsek PID durumlarını sıfırla
        if current_mode != 'GUIDED':
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
            
            # CSV Log Yaz (Güdümsüz)
            self._log_state(
                current_time, 0.033, gorev, "INACTIVE", 1, 0, 0, 0.0,
                bbox_x, bbox_y, bbox_w, bbox_h, obj_x, obj_y, target_area,
                stab_x, stab_y, stab_x - obj_x, stab_y - obj_y,
                self.test_alt_m, roll_rad, pitch_rad, yaw_rad,
                raw_error_x_deg, raw_error_y_deg, stab_error_x_deg, stab_error_y_deg,
                raw_error_x_deg, raw_error_y_deg, stab_error_x_deg, stab_error_y_deg,
                0,0,0,0, 0,0, 0,0,
                0, 0,0,0, yaw_deg, yaw_deg,
                0, 0,0,0, self.test_alt_m, self.test_alt_m,
                0, 0
            )
            return

        # PID Zaman Adımı (dt)
        if self.last_pid_time is None:
            dt = 0.033
        else:
            dt = current_time - self.last_pid_time
            if dt <= 0.001:
                dt = 0.033
        self.last_pid_time = current_time

        # 3) Alçak Geçiren Filtre (Low-Pass Filter) Türev Hesabı
        alpha = 0.05
        raw_deriv_x = (stab_error_x_deg - self.prev_error_x) / dt
        raw_deriv_y = (stab_error_y_deg - self.prev_error_y) / dt
        deriv_x = (alpha * raw_deriv_x) + ((1.0 - alpha) * self.prev_derivative_x)
        deriv_y = (alpha * raw_deriv_y) + ((1.0 - alpha) * self.prev_derivative_y)

        # 4) İntegral Hesabı ve Deadzone
        dz_x = 1 if abs(stab_error_x_deg) < self.deadzone_deg else 0
        dz_y = 1 if abs(stab_error_y_deg) < self.deadzone_deg else 0

        if dz_x:
            self.integral_error_x *= 0.99
            p_x = 0.0
        else:
            self.integral_error_x += stab_error_x_deg * dt
            p_x = stab_error_x_deg - math.copysign(self.deadzone_deg, stab_error_x_deg)
        self.integral_error_x = max(-10.0, min(10.0, self.integral_error_x))

        if dz_y:
            self.integral_error_y *= 0.99
            p_y = 0.0
        else:
            self.integral_error_y += stab_error_y_deg * dt
            p_y = stab_error_y_deg - math.copysign(self.deadzone_deg, stab_error_y_deg)
        self.integral_error_y = max(-10.0, min(10.0, self.integral_error_y))

        self.prev_error_x = stab_error_x_deg
        self.prev_error_y = stab_error_y_deg
        self.prev_derivative_x = deriv_x
        self.prev_derivative_y = deriv_y

        # 5) HEADING KONTROLÜ
        p_term_heading = p_x * self.Kp_heading
        i_term_heading = self.integral_error_x * self.Ki_heading
        d_term_heading = deriv_x * self.Kd_heading
        
        cmd_head_deg = p_term_heading + i_term_heading + d_term_heading
        cmd_head_deg = max(-self.max_heading_change_deg, min(self.max_heading_change_deg, cmd_head_deg))
        target_heading = (yaw_deg + cmd_head_deg) % 360.0

        # Değişken Dönüş Hızı (Variable Heading Rate)
        rate_error = abs(p_x)
        raw_deriv_rate = (rate_error - self.prev_error_rate) / dt
        deriv_rate = (alpha * raw_deriv_rate) + ((1.0 - alpha) * self.prev_derivative_rate)
        self.integral_error_rate = max(0.0, min(5.0, self.integral_error_rate + rate_error * dt))
        
        heading_rate = (rate_error * self.Kp_rate) + (self.integral_error_rate * self.Ki_rate) + (deriv_rate * self.Kd_rate)
        heading_rate = max(self.min_heading_rate, min(self.max_heading_rate, heading_rate))
        if dz_x:
            heading_rate = self.min_heading_rate
            
        self.prev_error_rate = rate_error
        self.prev_derivative_rate = deriv_rate

        # 6) ALTITUDE KONTROLÜ
        p_term_alt = p_y * self.Kp_alt
        i_term_alt = self.integral_error_y * self.Ki_alt
        d_term_alt = deriv_y * self.Kd_alt
        
        # Hedef aşağıdaysa (p_y pozitif), irtifa DÜŞÜRÜLMELİ
        cmd_alt_m = -1.0 * (p_term_alt + i_term_alt + d_term_alt)
        cmd_alt_m = max(-self.max_alt_change_m, min(self.max_alt_change_m, cmd_alt_m))

        loc_msg = self.mavlink_handler.master.messages.get('GLOBAL_POSITION_INT', None)
        current_alt = loc_msg.relative_alt / 1000.0 if loc_msg else self.test_alt_m
        target_alt = max(float(self.MIN_ALTITUDE), min(float(self.MAX_ALTITUDE), current_alt + cmd_alt_m))

        # Değişken Tırmanma/Alçalma Hızı (Variable Altitude Rate)
        alt_rate_error = abs(p_y)
        raw_deriv_alt_rate = (alt_rate_error - self.prev_error_alt_rate) / dt
        deriv_alt_rate = (alpha * raw_deriv_alt_rate) + ((1.0 - alpha) * self.prev_derivative_alt_rate)
        self.integral_error_alt_rate = max(0.0, min(5.0, self.integral_error_alt_rate + alt_rate_error * dt))

        altitude_rate = (alt_rate_error * self.Kp_alt_rate) + (self.integral_error_alt_rate * self.Ki_alt_rate) + (deriv_alt_rate * self.Kd_alt_rate)
        altitude_rate = max(self.min_alt_rate, min(self.max_alt_rate, altitude_rate))
        if dz_y:
            altitude_rate = self.min_alt_rate

        self.prev_error_alt_rate = alt_rate_error
        self.prev_derivative_alt_rate = deriv_alt_rate

        # 7) SPEED KONTROLÜ (BBox Coverage % Üzerinden)
        h_cov = (bbox_w / self.W) * 100.0
        v_cov = (bbox_h / self.H) * 100.0
        coverage = max(h_cov, v_cov)
        self.filtered_coverage = (self.coverage_alpha * coverage) + ((1.0 - self.coverage_alpha) * self.filtered_coverage)
        
        speed_error = self.target_coverage_pct - self.filtered_coverage
        raw_deriv_speed = (speed_error - self.prev_error_speed) / dt
        deriv_speed = (alpha * raw_deriv_speed) + ((1.0 - alpha) * self.prev_derivative_speed)

        # Hız integrali sadece setpoint bandında birikir
        if abs(speed_error) < self.speed_integral_band:
            self.integral_error_speed = max(-100.0, min(100.0, self.integral_error_speed + speed_error * dt))
        else:
            self.integral_error_speed *= 0.95  # Yavaş sönüm

        p_term_speed = speed_error * self.Kp_speed
        i_term_speed = self.integral_error_speed * self.Ki_speed
        d_term_speed = deriv_speed * self.Kd_speed
        
        raw_cmd_speed = self.base_speed + (p_term_speed + i_term_speed + d_term_speed)
        target_cmd_speed = max(self.min_speed, min(self.max_speed, raw_cmd_speed))

        # Slew Rate İvme Sınırlayıcı (1.0 m/s / s)
        max_speed_delta = self.speed_slew_rate * dt
        cmd_speed = max(self.last_speed - max_speed_delta, min(self.last_speed + max_speed_delta, target_cmd_speed))
        
        self.prev_error_speed = speed_error
        self.prev_derivative_speed = deriv_speed
        self.last_speed = cmd_speed

        # 8) Anti-Windup Mantığı
        aw_x, aw_y = 0, 0
        if abs(cmd_head_deg) >= self.max_heading_change_deg:
            self.integral_error_x *= 0.9
            aw_x = 1
        if abs(cmd_alt_m) >= self.max_alt_change_m:
            self.integral_error_y *= 0.9
            aw_y = 1

        # Değişkenleri güncelle ve Commander'a gönder
        self.test_heading_deg = target_heading
        self.test_alt_m = target_alt
        self.test_airspeed_ms = cmd_speed
        self.test_heading_rate = heading_rate
        self.test_altitude_rate = altitude_rate

        self.cmd_thread.update(
            self.test_heading_deg, self.test_alt_m, self.test_airspeed_ms,
            self.test_heading_rate, self.test_altitude_rate
        )

        # Commander'ın veri gönderip göndermediğini kontrol et
        command_sent = 1 if self.cmd_thread.active else 0
        data_age_ms = 0.0

        # Detaylı CSV Dosyasına Kaydet
        self._log_state(
            current_time, dt, gorev, "TRACKING", 1, command_sent, 0, data_age_ms,
            bbox_x, bbox_y, bbox_w, bbox_h, obj_x, obj_y, target_area,
            stab_x, stab_y, stab_x - obj_x, stab_y - obj_y,
            current_alt, roll_rad, pitch_rad, yaw_rad,
            raw_error_x_deg, raw_error_y_deg, stab_error_x_deg, stab_error_y_deg,
            raw_error_x_deg, raw_error_y_deg, stab_error_x_deg, stab_error_y_deg,
            raw_deriv_x, raw_deriv_y, deriv_x, deriv_y,
            dz_x, dz_y, p_x, p_y,
            self.integral_error_x, p_term_heading, i_term_heading, d_term_heading, cmd_head_deg, target_heading,
            self.integral_error_y, p_term_alt, i_term_alt, d_term_alt, cmd_alt_m, target_alt,
            aw_x, aw_y
        )

    def _log_state(self, current_time, dt, gorev, system_state, target_found,
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
                   aw_x, aw_y):
        elapsed = current_time - self.start_time
        coverage_w = (bbox_w / self.W * 100.0) if bbox_w > 0 else 0.0
        coverage_h = (bbox_h / self.H * 100.0) if bbox_h > 0 else 0.0

        row = [
            f"{current_time:.4f}", f"{elapsed:.3f}", f"{dt:.4f}", self.frame_counter,
            self.cmd_thread.last_mode if self.cmd_thread.last_mode else "UNKNOWN", gorev, system_state, target_found,
            command_sent, queue_size, f"{data_age_ms:.1f}",
            bbox_x, bbox_y, f"{bbox_w:.1f}", f"{bbox_h:.1f}", f"{obj_x:.2f}", f"{obj_y:.2f}",
            f"{target_area:.1f}", f"{coverage_w:.2f}", f"{coverage_h:.2f}",
            f"{stab_x:.2f}", f"{stab_y:.2f}", f"{delta_stab_x:.2f}", f"{delta_stab_y:.2f}",
            f"{current_alt_m:.2f}", f"{math.degrees(roll_now):.2f}", f"{math.degrees(pitch_now):.2f}", f"{math.degrees(yaw_now):.2f}",
            f"{raw_pixel_error_x:.2f}", f"{raw_pixel_error_y:.2f}", f"{stab_pixel_error_x:.2f}", f"{stab_pixel_error_y:.2f}",
            f"{raw_error_x_deg:.4f}", f"{raw_error_y_deg:.4f}", f"{stab_error_x_deg:.4f}", f"{stab_error_y_deg:.4f}",
            f"{raw_deriv_x:.4f}", f"{raw_deriv_y:.4f}", f"{filt_deriv_x:.4f}", f"{filt_deriv_y:.4f}",
            dz_active_x, dz_active_y, f"{p_input_x:.4f}", f"{p_input_y:.4f}",
            f"{integral_x:.6f}", f"{p_head:.6f}", f"{i_head:.6f}", f"{d_head:.6f}", f"{cmd_head_deg:.2f}", f"{target_head_deg:.2f}",
            f"{integral_y:.6f}", f"{p_alt:.6f}", f"{i_alt:.6f}", f"{d_alt:.6f}", f"{cmd_alt_m:.2f}", f"{target_alt_m:.2f}",
            aw_x, aw_y
        ]
        self.flight_logger.log(row)

    def _redis_listener(self):
        self.logger.debug("Redis listener thread started.")
        for message in self.p.listen():
            if message['type'] != 'message':
                continue

            # Görev kontrolü
            guid_message = self.r.get('gorev')
            if guid_message is None:
                guid_message = b'goruntulu'
            if guid_message.decode('utf-8').lower() != 'goruntulu':
                continue

            try:
                data = json.loads(message['data'].decode('utf-8'))
            except (json.JSONDecodeError, UnicodeDecodeError):
                continue

            # Parse [x, y, w, h]
            if isinstance(data, (list, tuple)) and len(data) >= 4:
                x, y, w, h = data[:4]
                now = time.time()
                with self.bbox_lock:
                    self.latest_bbox = (int(x), int(y), int(w), int(h))
                    self.latest_bbox_time = now
                    self.last_message_time = now

    def _vision_processor(self):
        self.logger.debug("Vision processor thread started.")
        period = 1.0 / 30.0
        last_processed_time = None
        last_target_time = 0.0
        last_warning_time = 0.0

        while True:
            time.sleep(period)
            current_time = time.time()
            
            with self.bbox_lock:
                bbox = self.latest_bbox
                bbox_time = self.latest_bbox_time

            # Yeni bbox geldiyse işle
            if bbox is not None and bbox_time != last_processed_time:
                last_processed_time = bbox_time
                last_target_time = current_time
                self.guide_aircraft(bbox, bbox_time)
            else:
                # HEDEF KAYIP: Coasting veya Failsafe
                if last_target_time > 0.0:
                    time_since_last = current_time - last_target_time
                    
                    if time_since_last <= 5.0:
                        # ── COASTING (5s) ──
                        # TestCommander son targets değerlerini göndermeye devam eder
                        pass
                    else:
                        # ── FAILSAFE (5s Aşıldı) ──
                        # PID Integral ve Prev durumlarını sıfırla
                        self.integral_error_x = 0.0
                        self.integral_error_y = 0.0
                        self.integral_error_rate = 0.0
                        self.integral_error_alt_rate = 0.0
                        self.integral_error_speed = 0.0
                        self.prev_error_speed = 0.0
                        
                        # Otopilottan güncel konum/yön al
                        att_msg = self.mavlink_handler.master.messages.get('ATTITUDE', None)
                        loc_msg = self.mavlink_handler.master.messages.get('GLOBAL_POSITION_INT', None)
                        
                        current_yaw = math.degrees(att_msg.yaw) % 360.0 if att_msg else self.test_heading_deg
                        current_alt = loc_msg.relative_alt / 1000.0 if loc_msg else self.test_alt_m
                        
                        # Güvenli hold komutlarını set et
                        self.test_heading_deg = current_yaw
                        self.test_alt_m = current_alt
                        self.test_airspeed_ms = self.base_speed
                        self.test_heading_rate = self.min_heading_rate
                        self.test_altitude_rate = self.min_alt_rate
                        
                        self.cmd_thread.update(
                            self.test_heading_deg, self.test_alt_m, self.test_airspeed_ms,
                            self.test_heading_rate, self.test_altitude_rate
                        )

                        # CSV Log Yaz (Failsafe durumunda)
                        gorev_bytes = self.r.get('gorev')
                        gorev = gorev_bytes.decode('utf-8') if gorev_bytes else "Bilinmiyor"
                        
                        self._log_state(
                            current_time, time_since_last, gorev, "FAILSAFE", 0, 1, 0, time_since_last*1000,
                            0,0,0,0,0,0,0, 0,0,0,0, current_alt, 
                            att_msg.roll if att_msg else 0, att_msg.pitch if att_msg else 0, att_msg.yaw if att_msg else 0,
                            0,0,0,0, 0,0,0,0,
                            0,0,0,0, 0,0, 0,0,
                            0, 0,0,0, current_yaw, current_yaw,
                            0, 0,0,0, current_alt, current_alt,
                            0, 0
                        )

                        if current_time - last_warning_time > 1.0:
                            self.logger.warning(
                                f"\n[UYARI] HEDEF {time_since_last:.1f} SANİYEDİR KAYIP! FAILSAFE AKTİF: UÇAK STABİL HEDEFLERDE TUTULUYOR!!!")
                            last_warning_time = current_time

    def run(self):
        listener_thread = threading.Thread(
            target=self._redis_listener, daemon=True, name="RedisListener")
        listener_thread.start()

        processor_thread = threading.Thread(
            target=self._vision_processor, daemon=True, name="VisionProcessor")
        processor_thread.start()

        self.logger.debug("All threads started. Main thread waiting...")
        listener_thread.join()
        processor_thread.join()

if __name__ == '__main__':
    try:
        guidance = tziGuidance()
        guidance.run()
    except KeyboardInterrupt:
        print("\nShutdown requested by user. Closing log files.")
        if hasattr(guidance, 'flight_logger'):
            guidance.flight_logger.close()
