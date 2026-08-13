#!/usr/bin/env python3

import time
import math
import json
from datetime import datetime
import redis
import threading
import numpy as np
from pymavlink import mavutil
import logging
import os
import shutil

'''
12 mm lense geçilirse satır 248 0.36 olmalı.
'''

# ─── SANAL GİMBAL SABİTLERİ ───
# Kamera->Body rotasyonu: Burnuna takılı, ileri bakan kamera
# Kamera eksenleri (OpenCV): x=sağ, y=aşağı, z=ileri(görüntü içine)
# Body eksenleri (ArduPilot): x=ileri(burun), y=sağ(kanat), z=aşağı
#
# R_CB_ID ile seçim yap, uçuş testinde hangisi doğru çalışıyorsa onu bırak:
#   1: Normal    — kamera düz takılı (USB konnektör üstte veya altta)
#   2: 180° dönük — kamera ters (baş aşağı) takılı
#   3: 90° CW    — kamera saat yönünde 90° döndürülmüş (USB sağda)
#   4: 90° CCW   — kamera saat yönü tersine 90° döndürülmüş (USB solda)
#
# Debug: Uçağı elinizle sağa yatırın (roll+). Doğru matristeyse:
#   - "Gimbal Sonra" açı değerleri SABİT kalır
#   - "Gimbal Önce" açı değerleri değişir
# Yanlış matristeyse: Gimbal Sonra daha fazla sallanır veya ters yöne gider.

R_CB_ID = 1  # ← BUNU DEĞİŞTİR (1, 2, 3 veya 4)

R_CB_OPTIONS = {
    1: np.array([[ 0, 0, 1],    # Normal: cam_z→body_x, cam_x→body_y, cam_y→body_z
                 [ 1, 0, 0],
                 [ 0, 1, 0]], dtype=float),

    2: np.array([[ 0, 0, 1],    # 180° dönük: cam_z→body_x, cam_x→-body_y, cam_y→-body_z
                 [-1, 0, 0],
                 [ 0,-1, 0]], dtype=float),

    3: np.array([[ 0, 0, 1],    # 90° CW: cam_z→body_x, cam_x→body_z, cam_y→-body_y
                 [ 0,-1, 0],
                 [ 1, 0, 0]], dtype=float),

    4: np.array([[ 0, 0, 1],    # 90° CCW: cam_z→body_x, cam_x→-body_z, cam_y→body_y
                 [ 0, 1, 0],
                 [-1, 0, 0]], dtype=float),
}

R_c_b = R_CB_OPTIONS[R_CB_ID]
R_c_b_T = R_c_b.T
print(f"Sanal Gimbal R_c_b matrisi: ID={R_CB_ID}\n{R_c_b}")

def compute_R_b_e(roll, pitch, yaw):
    """Body->Earth rotasyon matrisi (ZYX Euler convention)."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp,   cp*sr,            cp*cr           ]
    ])


class System:
    """Base system class — Redis, MAVLink, kamera, logging altyapısı."""

    def __init__(self, connection_str='udp:127.0.0.1:14552'):
        self.init_logger()

        # Redis
        self.r = redis.Redis(host='localhost', port=6379, db=0)
        self.logger.debug('Redis connection established.')
        self.p = self.r.pubsub()
        self.p.subscribe('tracker_bbox')
        self.logger.debug("Redis 'tracker_bbox' kanalına abone olundu.")

        # --- KAMERA VE OPTİK AYARLARI ---
        # SVPRO 2MP Global Shutter, 5-50mm Zoom Lens, 1/2.8" sensör
        # Sensör genişliği ≈ 5.64mm (1/2.8" 16:9)
        #
        # ZOOM TABLOSU (SVPRO 5-50mm, 1280x720):
        # ┌──────────┬────────────┬────────────┬────────────┐
        # │ Lens mm  │ HFOV (rad) │ HFOV (°)   │ fx (piksel)│
        # ├──────────┼────────────┼────────────┼────────────┤
        # │  5 mm    │   1.028    │   58.9°    │    778     │
        # │  8 mm    │   0.678    │   38.8°    │   1208     │
        # │ 12 mm    │   0.462    │   26.5°    │   1820     │
        # │ 16 mm    │   0.349    │   20.0°    │   2431     │
        # │ 20 mm    │   0.280    │   16.0°    │   3041     │
        # │ 25 mm    │   0.225    │   12.9°    │   3797     │
        # │ 35 mm    │   0.161    │    9.2°    │   5320     │
        # │ 50 mm    │   0.113    │    6.5°    │   7587     │
        # └──────────┴────────────┴────────────┴────────────┘
        # Lens üzerindeki mm işaretine bak, tablodaki HFOV değerini gir.
        # İnterpolasyon: 23mm → 20mm(0.280) ile 25mm(0.225) arası
        #   (23-20)/(25-20) = 0.6 → 0.280 + 0.6*(0.225-0.280) = 0.247 rad ≈ 14.2°

        self.camera_width = 1280
        self.camera_height = 720
        self.camera_hfov_rad = 0.247  # ← 23mm lens (interpolasyon: 20-25mm arası)

        self.center_x = self.camera_width / 2.0
        self.center_y = self.camera_height / 2.0

        # fx, fy hesabı (HFOV'dan)
        self.fx = self.center_x / math.tan(self.camera_hfov_rad / 2.0)
        self.fy = self.fx  # Kare piksel varsayımı

        # --- SANAL GİMBAL: Intrinsic Matris ---
        self.K = np.array([
            [self.fx, 0,       self.center_x],
            [0,       self.fy, self.center_y],
            [0,       0,       1            ]
        ])
        self.K_inv = np.linalg.inv(self.K)
        self.logger.debug(f"Sanal Gimbal aktif. fx={self.fx:.1f}, fy={self.fy:.1f}")

        # MAVLink
        self.logger.debug(f"MAVLink ile uçağa bağlanılıyor: {connection_str}...")
        self.master = mavutil.mavlink_connection(connection_str)
        self.master.wait_heartbeat()
        self.logger.debug(f"MAVLink Bağlantısı Başarılı! Sistem ID: {self.master.target_system}")

        # MAVLink state
        self.current_mode = "UNKNOWN"
        self.current_yaw_rad = 0.0
        self.current_roll_rad = 0.0
        self.current_pitch_rad = 0.0

        # MAVLink reader thread
        self._mavlink_reader_thread = threading.Thread(
            target=self._mavlink_reader, daemon=True, name="MAVLinkReader")
        self._mavlink_reader_thread.start()
        self.logger.debug('MAVLink reader thread started.')

    def _mavlink_reader(self):
        """Sürekli recv_match yaparak pymavlink dahili cache'ini güncel tutar."""
        while True:
            try:
                self.master.recv_match(blocking=True, timeout=0.1)
                att_msg = self.master.messages.get('ATTITUDE', None)
                if att_msg:
                    self.current_roll_rad = att_msg.roll
                    self.current_pitch_rad = att_msg.pitch
                    self.current_yaw_rad = att_msg.yaw
                hb_msg = self.master.messages.get('HEARTBEAT', None)
                if hb_msg:
                    flightmode = mavutil.mode_string_v10(hb_msg)
                    if flightmode:
                        self.current_mode = flightmode
            except Exception:
                time.sleep(0.01)

    def init_logger(self):
        self.logger = logging.Logger('23n')
        self.logger.setLevel(logging.DEBUG)
        c_handler = logging.StreamHandler()
        log_file_path = '23n_guidance.log'
        old_logs_dir = "Logs"
        if not os.path.exists(old_logs_dir):
            os.makedirs(old_logs_dir)
        if os.path.exists(log_file_path):
            timestamp = datetime.now()
            new_log_file_name = f"23n_guidance_{timestamp}.log"
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
class AutopilotGuidance(System):
    # ─── LETTERBOX DÜZELTME AYARLARI ───
    DELETTERBOX_ENABLED = True
    ORIG_W = 1280
    ORIG_H = 720
    NET_W  = 640
    NET_H  = 640

    def __init__(self, connection_str='udp:127.0.0.1:14552'):
        super().__init__(connection_str)

        # MAVLink send lock
        self.mav_lock = threading.Lock()

        # Letterbox parametreleri
        self.lb_gain = min(self.NET_W / self.ORIG_W, self.NET_H / self.ORIG_H)
        self.lb_pad_x = (self.NET_W - self.ORIG_W * self.lb_gain) / 2
        self.lb_pad_y = (self.NET_H - self.ORIG_H * self.lb_gain) / 2
        if self.DELETTERBOX_ENABLED:
            self.logger.debug(f"De-letterbox aktif: gain={self.lb_gain}, pad_x={self.lb_pad_x}, pad_y={self.lb_pad_y}")

        # --- Bbox thread-safe depo (tzi.py pattern) ---
        self.latest_bbox = None
        self.latest_bbox_time = None
        self.bbox_lock = threading.Lock()
        self.gorev = "Bilinmiyor"

        # --- PID VE KONTROL AYARLARI ---
        self.Kp_roll = 0.02875
        self.Ki_roll = 0.00115
        self.Kd_roll = 0.0090

        self.Kp_pitch = 0.01575
        self.Ki_pitch = 0.0021
        self.Kd_pitch = 0.0060

        self.max_roll_deg = 35.0
        self.max_pitch_deg = 20.0
        self.deadzone_deg = 0.78

        # Hafıza değişkenleri (Matematiksel Stabilite)
        self.prev_error_x, self.prev_error_y = 0.0, 0.0
        self.integral_error_x, self.integral_error_y = 0.0, 0.0
        self.prev_derivative_x, self.prev_derivative_y = 0.0, 0.0
        self.last_time = time.time()

        # Failsafe ve Coasting (5 saniye) Değişkenleri
        self.last_target_time = 0.0
        self.last_final_roll_rad = 0.0
        self.last_final_pitch_rad = 0.0

    def clamp(self, value, min_val, max_val):
        return max(min(value, max_val), min_val)

    def _deletterbox(self, x, y, w, h):
        """640x640 letterbox uzayından 1280x720 orijinal uzaya dönüştür.

        1280x720 → 640x640 letterbox'ta:
          - x: padding yok (pad_x=0), sadece ölçek (gain=0.5)
          - y: 140px üst + 140px alt padding var

        Düzeltme yapılmazsa y'de sabit 280px ofset oluşur.
        """
        x = (x - self.lb_pad_x) / self.lb_gain
        y = (y - self.lb_pad_y) / self.lb_gain
        w = w / self.lb_gain
        h = h / self.lb_gain
        x = max(0, min(x, self.ORIG_W - 1))
        y = max(0, min(y, self.ORIG_H - 1))
        w = max(1, min(w, self.ORIG_W))
        h = max(1, min(h, self.ORIG_H))
        return x, y, w, h

    def _parse_bbox(self, data):
        """Gelen Redis mesajını parse eder, (x, y, w, h) tuple döner veya None."""
        x, y, w, h = None, None, None, None
        if isinstance(data, (list, tuple)) and len(data) >= 4:
            x, y, w, h = data[:4]
        if w is not None:
            return (float(x), float(y), float(w), float(h))
        return None

    def stabilize_pixel(self, obj_x, obj_y):
        """Sanal gimbal: UAV roll/pitch kompanzasyonu ile stabilize piksel döner.

        Pipeline: Piksel → K⁻¹ (back-project) → Kamera ışını → Body frame
                  → R_stab ile stabilize → Tekrar kamera frame → K (re-project)

        Yaw=0 ile R_stab hesaplanır: sadece roll ve pitch titreşimleri kompanze edilir.
        """
        p_raw = np.array([obj_x, obj_y, 1.0])
        r_cam = self.K_inv @ p_raw
        r_body = R_c_b @ r_cam
        roll_rad = self.current_roll_rad
        pitch_rad = self.current_pitch_rad
        R_stab = compute_R_b_e(roll_rad, pitch_rad, 0)
        r_virt_body = R_stab @ r_body
        r_virt_cam = R_c_b_T @ r_virt_body
        p_virt_hom = self.K @ r_virt_cam
        if p_virt_hom[2] != 0:
            return p_virt_hom[0] / p_virt_hom[2], p_virt_hom[1] / p_virt_hom[2]
        return obj_x, obj_y

    def set_mode(self, mode_name):
        with self.mav_lock:
            if mode_name not in self.master.mode_mapping():
                return
            mode_id = self.master.mode_mapping()[mode_name]
            self.master.mav.set_mode_send(
                self.master.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id
            )

    def send_attitude_target(self, roll_rad, pitch_rad, yaw_rad):
        cr, sr = math.cos(roll_rad * 0.5), math.sin(roll_rad * 0.5)
        cp, sp = math.cos(pitch_rad * 0.5), math.sin(pitch_rad * 0.5)
        cy, sy = math.cos(yaw_rad * 0.5), math.sin(yaw_rad * 0.5)
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        q = [w, x, y, z]
        type_mask = 7
        with self.mav_lock:
            try:
                self.master.mav.set_attitude_target_send(
                    0, self.master.target_system, self.master.target_component,
                    type_mask, q, 0, 0, 0, 0.80
                )
            except Exception:
                pass

    def _redis_listener(self):
        """Thread: Redis pub/sub'i dinler, parse edip latest_bbox'a yazar."""
        self.logger.debug("Redis listener thread başladı.")
        for message in self.p.listen():
            if message['type'] != 'message':
                continue
            # Görev kontrolü
            gorev_bytes = self.r.get('gorev')
            self.gorev = gorev_bytes.decode('utf-8') if gorev_bytes else "Bilinmiyor"
            # Parse
            try:
                data = json.loads(message['data'].decode('utf-8'))
            except (json.JSONDecodeError, UnicodeDecodeError):
                continue
            bbox = self._parse_bbox(data)
            if bbox is not None:
                x, y, w, h = bbox
                if self.DELETTERBOX_ENABLED:
                    x, y, w, h = self._deletterbox(x, y, w, h)
                obj_x = x + (w / 2.0)
                obj_y = y + (h / 2.0)
                target_area = w * h
                now = time.time()
                with self.bbox_lock:
                    self.latest_bbox = {
                        'obj_x': obj_x, 'obj_y': obj_y,
                        'bbox_w': float(w), 'bbox_h': float(h),
                        'target_area': target_area,
                    }
                    self.latest_bbox_time = now

    def _vision_processor(self):
        """Thread: 30 Hz'de en güncel bbox'ı okuyup guidance logic çalıştırır."""
        self.logger.debug("Vision processor thread başladı.")
        period = 1.0 / 30.0
        last_processed_time = None

        while True:
            time.sleep(period)
            with self.bbox_lock:
                data = self.latest_bbox
                data_time = self.latest_bbox_time

            current_time = time.time()
            gorev = self.gorev
            has_new_data = (data is not None and data_time != last_processed_time)

            if not has_new_data:
                # MESAJ YOK: Coasting / Failsafe
                if self.last_target_time > 0.0:
                    time_since_last = current_time - self.last_target_time
                    if time_since_last <= 5.0:
                        if gorev == "Goruntulu" and self.current_mode == "GUIDED":
                            self.send_attitude_target(
                                self.last_final_roll_rad,
                                self.last_final_pitch_rad,
                                self.current_yaw_rad
                            )
                    else:
                        self.integral_error_x = 0.0
                        self.integral_error_y = 0.0
                        self.prev_derivative_x = 0.0
                        self.prev_derivative_y = 0.0
                        if gorev == "Goruntulu" and self.current_mode == "GUIDED":
                            self.send_attitude_target(0.0, 0.0, self.current_yaw_rad)
                            self.last_final_roll_rad = 0.0
                            self.last_final_pitch_rad = 0.0
                            if not hasattr(self, 'last_warning_time') or current_time - self.last_warning_time > 1.0:
                                self.logger.debug("\n[UYARI] HEDEF 5 SANİYEDEN UZUN SÜREDİR KAYIP! GÖREV MODUNU DEĞİŞTİRİN!!!")
                                self.last_warning_time = current_time
                continue

            # SADECE YENİ FRAME GELDİĞİNDE:
            last_processed_time = data_time
            dt = current_time - self.last_time
            if dt < 0.001: dt = 0.001
            if dt > 0.1: dt = 0.1
            self.last_time = current_time

            obj_x = data['obj_x']
            obj_y = data['obj_y']
            bbox_w = data.get('bbox_w', 0.0)
            bbox_h = data.get('bbox_h', 0.0)
            target_area = data['target_area']
            self.last_target_time = current_time

            # --- SANAL GİMBAL STABİLİZASYONU ---
            stab_x, stab_y = self.stabilize_pixel(obj_x, obj_y)

            # --- HATA (ERROR) HESAPLAMALARI ---
            pixel_error_x = stab_x - self.center_x
            pixel_error_y = stab_y - self.center_y
            stab_error_x_deg = math.degrees(math.atan(pixel_error_x / self.fx))
            stab_error_y_deg = math.degrees(math.atan(pixel_error_y / self.fy))

            raw_pixel_error_x = obj_x - self.center_x
            raw_pixel_error_y = obj_y - self.center_y
            raw_error_x_deg = math.degrees(math.atan(raw_pixel_error_x / self.fx))
            raw_error_y_deg = math.degrees(math.atan(raw_pixel_error_y / self.fy))

            error_x_deg = stab_error_x_deg
            error_y_deg = stab_error_y_deg
            heading_diff_deg = error_x_deg

            raw_derivative_x = (error_x_deg - self.prev_error_x) / dt
            raw_derivative_y = (error_y_deg - self.prev_error_y) / dt

            alpha = 0.02
            derivative_x = (alpha * raw_derivative_x) + ((1.0 - alpha) * self.prev_derivative_x)
            derivative_y = (alpha * raw_derivative_y) + ((1.0 - alpha) * self.prev_derivative_y)

            if gorev == "Goruntulu" and self.current_mode == "GUIDED":
                self.integral_error_x = self.clamp(self.integral_error_x + error_x_deg * dt, -30.0, 30.0)
                self.integral_error_y = self.clamp(self.integral_error_y + error_y_deg * dt, -30.0, 30.0)
            else:
                self.integral_error_x = 0.0
                self.integral_error_y = 0.0

            self.prev_error_x, self.prev_error_y = error_x_deg, error_y_deg
            self.prev_derivative_x, self.prev_derivative_y = derivative_x, derivative_y

            # Yumuşak Deadzone
            p_x = 0 if abs(error_x_deg) < self.deadzone_deg else error_x_deg - math.copysign(self.deadzone_deg, error_x_deg)
            p_y = 0 if abs(error_y_deg) < self.deadzone_deg else error_y_deg - math.copysign(self.deadzone_deg, error_y_deg)

            p_term_roll = p_x * self.Kp_roll
            i_term_roll = self.integral_error_x * self.Ki_roll
            p_term_pitch = p_y * self.Kp_pitch
            i_term_pitch = self.integral_error_y * self.Ki_pitch

            raw_d_roll = derivative_x * self.Kd_roll
            raw_d_pitch = derivative_y * self.Kd_pitch

            d_term_roll = self.clamp(raw_d_roll, -0.087, 0.087)
            d_term_pitch = self.clamp(raw_d_pitch, -0.052, 0.052)

            cmd_roll_rad = p_term_roll + i_term_roll + d_term_roll
            cmd_pitch_rad = -1 * (p_term_pitch + i_term_pitch + d_term_pitch)

            cmd_roll_deg = math.degrees(cmd_roll_rad)
            cmd_pitch_deg = math.degrees(cmd_pitch_rad)

            final_roll_deg = self.clamp(cmd_roll_deg, -self.max_roll_deg, self.max_roll_deg)
            final_pitch_deg = self.clamp(cmd_pitch_deg, -self.max_pitch_deg, self.max_pitch_deg)

            final_roll_rad = math.radians(final_roll_deg)
            final_pitch_rad = math.radians(final_pitch_deg)

            # --- KONTROLÜN UÇAĞA GÖNDERİLMESİ ---
            ac_roll_deg = math.degrees(self.current_roll_rad)
            ac_pitch_deg = math.degrees(self.current_pitch_rad)
            ac_yaw_deg = math.degrees(self.current_yaw_rad)

            if gorev == "Goruntulu" and self.current_mode == "GUIDED":
                self.send_attitude_target(final_roll_rad, final_pitch_rad, self.current_yaw_rad)
                self.logger.debug(f"[{gorev}] OTONOM TAKİP: CmdRoll={final_roll_deg:.1f}° CmdPitch={final_pitch_deg:.1f}° | "
                      f"AC: R={ac_roll_deg:.1f}° P={ac_pitch_deg:.1f}° Y={ac_yaw_deg:.1f}° | "
                      f"Önce: px=({obj_x:.1f},{obj_y:.1f}) açı=({raw_error_x_deg:.2f}°,{raw_error_y_deg:.2f}°) | "
                      f"Sonra: px=({stab_x:.1f},{stab_y:.1f}) açı=({stab_error_x_deg:.2f}°,{stab_error_y_deg:.2f}°) | "
                      f"BBox: {bbox_w:.0f}x{bbox_h:.0f}")
            else:
                self.logger.debug(f"[{gorev}] DEBUG: CmdRoll={final_roll_deg:.1f}° CmdPitch={final_pitch_deg:.1f}° | "
                      f"AC: R={ac_roll_deg:.1f}° P={ac_pitch_deg:.1f}° Y={ac_yaw_deg:.1f}° | "
                      f"Önce: px=({obj_x:.1f},{obj_y:.1f}) açı=({raw_error_x_deg:.2f}°,{raw_error_y_deg:.2f}°) | "
                      f"Sonra: px=({stab_x:.1f},{stab_y:.1f}) açı=({stab_error_x_deg:.2f}°,{stab_error_y_deg:.2f}°) | "
                      f"BBox: {bbox_w:.0f}x{bbox_h:.0f}")

            self.last_final_roll_rad = final_roll_rad
            self.last_final_pitch_rad = final_pitch_rad

            # --- LOG YAZDIRMA ---
            roll_now = self.current_roll_rad
            pitch_now = self.current_pitch_rad
            delta_stab_x = stab_x - obj_x
            delta_stab_y = stab_y - obj_y

            self.logger.debug(
                f"LOG: dt={dt:.4f} mode={self.current_mode} | "
                f"raw=({obj_x:.2f},{obj_y:.2f}) stab=({stab_x:.2f},{stab_y:.2f}) "
                f"delta=({delta_stab_x:.2f},{delta_stab_y:.2f}) | "
                f"rpy_rad=({roll_now:.4f},{pitch_now:.4f}) rpy_deg=({math.degrees(roll_now):.2f},{math.degrees(pitch_now):.2f}) | "
                f"err_raw=({raw_error_x_deg:.2f},{raw_error_y_deg:.2f}) err_stab=({stab_error_x_deg:.2f},{stab_error_y_deg:.2f}) hdg_diff={heading_diff_deg:.2f} | "
                f"PID_roll: P={p_term_roll:.6f} I={i_term_roll:.6f} D={d_term_roll:.6f} cmd={cmd_roll_rad:.6f} final={final_roll_deg:.2f} | "
                f"PID_pitch: P={p_term_pitch:.6f} I={i_term_pitch:.6f} D={d_term_pitch:.6f} cmd={cmd_pitch_rad:.6f} final={final_pitch_deg:.2f} | "
                f"bbox=({bbox_w:.1f},{bbox_h:.1f},{target_area:.1f})"
            )

    def run(self):
        self.logger.debug("Sistem aktif! Tüm thread'ler başlatılıyor...")

        # Thread 1: Redis listener
        listener_thread = threading.Thread(
            target=self._redis_listener, daemon=True, name="RedisListener")
        listener_thread.start()

        # Thread 2: Vision processor (30 Hz guidance)
        processor_thread = threading.Thread(
            target=self._vision_processor, daemon=True, name="VisionProcessor")
        processor_thread.start()

        self.logger.debug("Tüm thread'ler başlatıldı. Ana thread bekliyor...")
        try:
            listener_thread.join()
            processor_thread.join()
        except KeyboardInterrupt:
            self.logger.debug("\nKapatılıyor...")


if __name__ == '__main__':
    gudum = AutopilotGuidance(connection_str='udp:127.0.0.1:14552')
    gudum.run()
