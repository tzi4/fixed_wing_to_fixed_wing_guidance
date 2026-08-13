#!/usr/bin/env python3
"""
tzi_emir.py — Güdüm Kodları Füzyon Versiyonu
═══════════════════════════════════════════════
Hazırlayan:  Claude Opus 4.6 (Thinking)
Tarih:       2026-06-22

Mimari:     tzi.py  (System → tziGuidance, TestCommander thread ayrımı)
Heading:    kamp_basi_emir.py  (P + Heading Rate PID, dinamik dönüş hızı)
Altitude:   kamp_basi_emir.py  (PD + Altitude Rate PID, dinamik tırmanma)
Airspeed:   kamp_basi_emir.py  (coverage-based PID) + tzi_final.py (slew rate)
Hata:       tzi_final.py  (açısal — derece cinsinden, kamera bağımsız)
Stabilize:  tzi.py  (virtual gimbal, R_c_b tabanlı)
Logging:    goat_gimbal.py  (CSV) + tzi.py (text log)
Failsafe:   kamp_basi_emir.py + tzi_final.py (coasting + failsafe)
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

# ═══════════════════════════════════════════════════
# SABİTLER
# ═══════════════════════════════════════════════════
RESOLUTION_W = 1920
RESOLUTION_H = 1080
MAVLINK_CONNECTION = 'udp:127.0.0.1:14553'  # goat_gimbal uçuş bağlantısıyla aynı
TELEMETRY_HZ = 10                           # goat_gimbal ile aynı telemetri hızı
TELEMETRY_TIMEOUT_S = 2.0
COMMAND_RATE_HZ = 5                         # GUIDED hedefleri ArduPilot'ta kalıcıdır
# COMMAND_RATE_HZ = 10                      # goat_gimbal gönderim temposu

# Simülasyon kamerası (2026-07-26'dan beri GERÇEK DONANIMLA BİREBİR):
# models/bumblebee/model.sdf -> 1920x1080, horizontal_fov=0.42 rad
# -> fx = fy = 960/tan(0.21) = 4504.03 px, cx=960, cy=540.
# Dikey FOV = 2*atan(540/4504.03) = 0.2386 rad (±6.837°).
CAMERA_HFOV_RAD = 0.42
CAMERA_FX = (RESOLUTION_W / 2.0) / math.tan(CAMERA_HFOV_RAD / 2.0)
CAMERA_FY = CAMERA_FX            # Kare piksel varsayımı

# GERÇEK KAMERA PROFİLİ artık ayrı bir profil DEĞİL: sim sensörü gerçek spec'e
# (1920x1080, 0.42 rad) çekildiği için yukarıdaki değerler uçuşta da geçerli.
# Kamera montaj eğimi de model.sdf'te parite için uygulandı (2.678° burun yukarı),
# dolayısıyla sanal gimbal matematiği iki tarafta da aynı sayılarla çalışır.
# Eski (1280x720) sim profili: HFOV 0.27, fx 4711.9057 — artık kullanılmıyor.

# Sequential tuning sırasında yorum satırı/override yerine bu bayrakları kullan.
# Örnek, yalnız airspeed: False / False / True.
SEND_HEADING_COMMANDS = True
SEND_ALTITUDE_COMMANDS = True
SEND_AIRSPEED_COMMANDS = True

# MAV_CMD_GUIDED_CHANGE_HEADING param1:
# 0 = course over ground, 1 = raw vehicle heading.
HEADING_TYPE = 1
# HEADING_TYPE = 0  # goat_gimbal yaklaşımı (course over ground)

# Sequential tuning / goat-parite testi için opsiyonel sabit komutlar.
# İlk gerçek donanım tekrarında yalnız airspeed'i açıp 20.0 kullanmak,
# goat_gimbal test koşulunu yeniden üretir.
FIXED_HEADING_DEG = None
FIXED_ALTITUDE_M = None
FIXED_ALTITUDE_RATE_MPS = None
FIXED_AIRSPEED_MS = None
# FIXED_ALTITUDE_RATE_MPS = 0.8   # goat_gimbal irtifa rate'i
# FIXED_AIRSPEED_MS = 20.0        # goat_gimbal uçuş airspeed'i

# Güdüm limitleri
MIN_ALTITUDE = 10
MAX_ALTITUDE = 200

# ─── Kamera → Body rotasyonu (kamera: z ileri, x sağ, y aşağı) ───
R_c_b = np.array([[0, 0, 1],
                  [1, 0, 0],
                  [0, 1, 0]], dtype=float)
R_c_b_T = R_c_b.T


def compute_R_b_e(roll, pitch, yaw):
    """Body → Earth rotasyon matrisi (ZYX Euler, radyan)."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp,   cp*sr,            cp*cr           ]
    ])


# ═══════════════════════════════════════════════════
# CSV FLIGHT LOGGER  (goat_gimbal.py'den)
# ═══════════════════════════════════════════════════
class FlightLogger:
    """Uçuş verilerini CSV'ye kaydeder. Offline analiz için vazgeçilmez."""

    def __init__(self, file_prefix="tzi_emir_log"):
        ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        self.log_filename = f"{file_prefix}_{ts}.csv"
        self.log_file = open(self.log_filename, mode='w', newline='')
        self.csv_writer = csv.writer(self.log_file)
        self.row_count = 0
        self.flush_interval = 10

        headers = [
            "timestamp", "elapsed_s", "dt", "system_state",
            "flight_mode", "gorev", "target_found", "command_sent",
            "ack_heading", "ack_altitude", "ack_airspeed",
            # BBox
            "bbox_x", "bbox_y", "bbox_w", "bbox_h",
            "coverage_pct", "filtered_coverage_pct",
            # Stabilized pixel
            "stab_x", "stab_y",
            # Angular errors (degrees)
            "stab_error_x_deg", "stab_error_y_deg",
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
        self.log_file.flush()
        self.log_file.close()


# ═══════════════════════════════════════════════════
# DIRECT CONTROL COMMANDER THREAD  (tzi.py mimarisi, genişletilmiş)
# ═══════════════════════════════════════════════════
class TestCommander(threading.Thread):
    """
    Bağımsız, ayarlanabilir frekansta komut gönderici thread.
    Heading, Altitude ve Airspeed komutlarını periyodik olarak gönderir.
    Görüntü işlemeden tamamen ayrışmıştır.

    Komutlar:
      - Heading:   MAV_CMD_GUIDED_CHANGE_HEADING  (43002)
      - Altitude:  MAV_CMD_GUIDED_CHANGE_ALTITUDE (43001)
      - Airspeed:  MAV_CMD_DO_CHANGE_SPEED        (178)
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
            raise ValueError("heading_type yalnız 0 (COG) veya 1 (raw heading) olabilir")

        # VFR_HUD mesajını iste (airspeed okumak için)
        with self.mavlink_io_lock:
            self.mav_handler.request_message_interval(
                [mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD], TELEMETRY_HZ
            )

        # Hedef değerler (thread-safe, lock ile erişilir)
        self.target_heading_deg = 0.0
        self.target_heading_rate_dps = 3.0   # ← YENİ: dinamik heading rate
        self.target_alt_m = 100.0
        self.target_alt_rate_mps = 0.8       # ← YENİ: dinamik altitude rate
        self.target_airspeed_ms = 17.0
        self.active = False

        # Son okunan telemetri (cache)
        self.last_att = None       # (pitch_deg, roll_deg, yaw_deg)
        self.last_loc = None       # (lat, lon, alt_m)
        self.last_airspeed = None  # m/s
        self.last_mode = None      # uçuş modu string

    def update(self, heading_deg, heading_rate_dps, alt_m, alt_rate_mps, airspeed_ms):
        """Thread-safe hedef güncelleme. 5 parametre."""
        with self.lock:
            self.target_heading_deg = float(heading_deg)
            self.target_heading_rate_dps = float(heading_rate_dps)
            self.target_alt_m = float(alt_m)
            self.target_alt_rate_mps = float(alt_rate_mps)
            self.target_airspeed_ms = float(airspeed_ms)
            self.active = True

    def deactivate(self):
        """Komut gönderimini thread-safe biçimde durdur."""
        with self.lock:
            self.active = False

    def is_active(self):
        with self.lock:
            return self.active

    def _send_heading(self, heading_deg, heading_rate_dps, airspeed_ms):
        """MAV_CMD_GUIDED_CHANGE_HEADING (43002).
        param1: 0=course over ground, 1=raw vehicle heading.
        param3: maksimum merkezcil ivme (m/s²).

        Guidance katmanı okunabilir/tune edilebilir bir dönüş hızı (deg/s)
        üretir. ArduPlane'in beklediği ivmeye a = V * omega ile burada çevrilir.
        """
        heading_accel_mss = max(
            abs(float(airspeed_ms)) * math.radians(abs(float(heading_rate_dps))),
            0.05,  # ArduPlane de aynı alt sınırı uygular.
        )
        with self.mavlink_io_lock:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                43002,              # MAV_CMD_GUIDED_CHANGE_HEADING
                0,                  # confirmation
                self.heading_type,  # param1: 0 = COG, 1 = raw vehicle heading
                heading_deg,        # param2: hedef heading (derece)
                heading_accel_mss,  # param3: max merkezcil ivme (m/s²)
                0, 0, 0, 0
            )

    def _send_altitude(self, alt_m, alt_rate_mps):
        """MAV_CMD_GUIDED_CHANGE_ALTITUDE (43001).
        param3: climb rate (m/s) — dinamik (kamp_basi_emir yaklaşımı).
        param7: desired altitude (relative).
        """
        with self.mavlink_io_lock:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                43001,          # MAV_CMD_GUIDED_CHANGE_ALTITUDE
                0,              # confirmation
                0,              # param1: boş
                0,              # param2: boş
                alt_rate_mps,   # param3: climb rate (m/s)
                0, 0, 0,        # param4-6
                alt_m           # param7: desired altitude (relative)
            )

    def _send_airspeed(self, airspeed_ms):
        """MAV_CMD_DO_CHANGE_SPEED (178). Sadece GUIDED modda gönderilir."""
        if self.last_mode != 'GUIDED':
            return
        with self.mavlink_io_lock:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                178,            # MAV_CMD_DO_CHANGE_SPEED
                0,              # confirmation
                0,              # param1: 0 = airspeed
                airspeed_ms,    # param2: hedef airspeed (m/s)
                -1,             # param3: throttle (-1 = değişiklik yok)
                0, 0, 0, 0
            )

    def _read_telemetry(self):
        """Pymavlink cache'inden non-blocking telemetri oku."""
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
                loc_msg.relative_alt / 1000.0  # ← RELATIVE ALT (emir yaklaşımı)
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

                # Mod değişiminden sonra eski GUIDED hedeflerini göndermeyi kes.
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


# ═══════════════════════════════════════════════════
# SYSTEM BASE CLASS  (tzi.py mimarisi)
# ═══════════════════════════════════════════════════
class System:
    """MAVLink, Redis, Logger altyapısı."""

    def __init__(self):
        self.init_logger()

        # Redis
        self.r = redis.Redis(host='localhost', port=6379, db=0)
        self.logger.debug('Redis connection established.')
        self.p = self.r.pubsub()
        self.p.subscribe('tracker_bbox')

        # Kamera
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

        # Güdüm limitleri
        self.MIN_ALTITUDE = MIN_ALTITUDE
        self.MAX_ALTITUDE = MAX_ALTITUDE

        # MAVLink bağlantısı
        self.mavlink_handler = MAVLinkHandler(
            MAVLINK_CONNECTION, message_hz=TELEMETRY_HZ)
        self.logger.debug('Connected to the aircraft.')
        self.r.set('guid', 'False')

        # goat_gimbal'daki gibi recv/send aynı pymavlink nesnesine ortak kilitle erişir.
        self.mavlink_io_lock = threading.Lock()

        # Son COMMAND_ACK sonucu: command_id -> (MAV_RESULT, receive_time).
        self.command_ack_lock = threading.Lock()
        self.command_acks = {}
        self.telemetry_time_lock = threading.Lock()
        self.telemetry_last_seen = {}

        # MAVLink reader thread (cache güncel tutar)
        self._mavlink_reader_thread = threading.Thread(
            target=self._mavlink_reader, daemon=True, name="MAVLinkReader"
        )
        self._mavlink_reader_thread.start()
        self.logger.debug('MAVLink reader thread started.')

        atexit.register(self.exit_handler)

    def _mavlink_reader(self):
        """Sürekli recv_match yaparak pymavlink cache'ini güncel tutar."""
        while True:
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
                self.logger.exception("MAVLink reader hatası")
                time.sleep(0.1)

    def telemetry_is_fresh(self, message_types, max_age_s=TELEMETRY_TIMEOUT_S):
        """Gerekli telemetri mesajlarının yakın zamanda alındığını doğrula."""
        now = time.time()
        with self.telemetry_time_lock:
            return all(
                msg_type in self.telemetry_last_seen and
                now - self.telemetry_last_seen[msg_type] <= max_age_s
                for msg_type in message_types
            )

    def get_command_ack(self, command_id):
        """Son COMMAND_ACK sonucunu okunabilir MAV_RESULT adıyla döndür."""
        with self.command_ack_lock:
            ack = self.command_acks.get(int(command_id))
        if ack is None:
            return "NO_ACK"

        result, _ = ack
        result_info = mavutil.mavlink.enums['MAV_RESULT'].get(result)
        return result_info.name if result_info is not None else str(result)

    def exit_handler(self):
        self.mavlink_handler.master.close()
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


# ═══════════════════════════════════════════════════
# FÜZYON GUIDANCE CLASS
# ═══════════════════════════════════════════════════
class tziGuidance(System):
    """
    Heading + Altitude + Airspeed füzyon kontrolcüsü.

    Heading:  P + Rate PID (kamp_basi_emir) — açısal hata (tzi_final)
    Altitude: PD + Rate PID (kamp_basi_emir) — açısal hata (tzi_final)
    Airspeed: Coverage-based PID (kamp_basi_emir) + Slew rate (tzi_final)
    """

    def __init__(self):
        super().__init__()

        self.last_message_time = None
        self.last_telemetry_warning_time = 0.0

        # Thread-safe bbox deposu
        self.latest_bbox = None
        self.latest_bbox_time = None
        self.bbox_lock = threading.Lock()

        # CSV logger  (goat_gimbal'den)
        self.csv_logger = FlightLogger()
        self.start_time = time.time()

        # ═══════════════════════════════════════════
        # HEADING KONTROL  (kamp_basi_emir + tzi_final)
        # ═══════════════════════════════════════════
        # Ana P kontrolcü (açısal hata → heading delta)
        self.Kp_heading = 3.0       # 1° hata → 3° heading değişimi
        self.Ki_heading = 0.0
        self.Kd_heading = 0.0

        # Heading Rate PID (açısal hata → dönüş hızı)
        self.Kp_rate = 1.2          # 1° hata → 1.2 deg/s dönüş hızı
        self.Ki_rate = 0.0
        self.Kd_rate = 0.12
        # goat_gimbal gerçek donanım referansı: Kp_rate=0.6, Kd_rate=0.12

        self.integral_error_rate = 0.0
        self.prev_error_rate = 0.0
        self.prev_derivative_rate = 0.0

        self.min_heading_rate = 0.85    # Minimum dönüş hızı (deg/s)
        self.max_heading_rate = 7.5     # Maksimum dönüş hızı (deg/s)
        # goat_gimbal gerçek donanım referansı: max_heading_rate=5.0
        self.max_heading_change_deg = 35.0   # Max heading delta (derece)

        # ═══════════════════════════════════════════
        # ALTITUDE KONTROL  (kamp_basi_emir + tzi_final)
        # ═══════════════════════════════════════════
        self.Kp_alt = 1.5
        self.Ki_alt = 0.0
        self.Kd_alt = 0.1
        # goat_gimbal gerçek donanım referansı: Kp_alt=0.7, Kd_alt=0.12

        # Altitude Rate PID (açısal hata → tırmanma/alçalma hızı)
        self.Kp_alt_rate = 1.5
        self.Ki_alt_rate = 0.0
        self.Kd_alt_rate = 0.1

        self.integral_error_alt_rate = 0.0
        self.prev_error_alt_rate = 0.0
        self.prev_derivative_alt_rate = 0.0

        self.min_alt_rate = 0.3       # Minimum tırmanma hızı (m/s)
        self.max_alt_rate = 5.0       # Maksimum tırmanma hızı (m/s)
        self.max_alt_change_m = 10.0  # Max altitude delta (metre)

        # ═══════════════════════════════════════════
        # AIRSPEED KONTROL  (kamp_basi_emir coverage PID + tzi_final slew)
        # ═══════════════════════════════════════════
        self.Kp_speed = 0.2
        self.Ki_speed = 0.03
        self.Kd_speed = 0.0

        self.integral_error_speed = 0.0
        self.prev_error_speed = 0.0
        self.prev_derivative_speed = 0.0

        self.min_speed = 14.0         # AIRSPEED_MIN üstü
        self.max_speed = 22.0
        self.base_speed = 17.0        # Nötr hız (cruise)
        self.target_coverage_pct = 6.0    # Korunmak istenen takip mesafesi (%)
        self.speed_integral_band = 5.0    # I sadece bu band içinde biriksin
        self.speed_slew_rate = 1.0        # Max hız değişimi (m/s/s)  (tzi_final'den)
        self.coverage_alpha = 0.35        # EMA katsayısı
        self.filtered_coverage = self.target_coverage_pct
        self.last_speed = self.base_speed

        # ═══════════════════════════════════════════
        # ORTAK PID DURUMLARI
        # ═══════════════════════════════════════════
        # Deadzone (tzi_final + emir ortak)
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
        self.coasting_timeout = 5.0  # saniye
        self.hold_active = False

        # GUIDED entry initialization (tzi_final'den)
        self.guided_entry_done = False

        # Initialize TestCommander (tzi mimarisi)
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

    # ─── Yardımcı fonksiyonlar ───

    def clamp(self, value, min_val, max_val):
        return max(min(value, max_val), min_val)

    def stabilize_pixel(self, obj_x, obj_y):
        """Virtual gimbal stabilizasyonu (tzi.py'den)."""
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

        # Işın sanal kameranın arkasına düştüğünde veya ufukta tekilleştiğinde
        # bölme çok büyük/sayısal olmayan piksel üretmesin. Bu nadir durumda ham
        # piksel, kontrolcüye taşmış bir komuttan daha güvenli bir geri dönüşüdür.
        if (np.all(np.isfinite(p_virt_hom)) and
                p_virt_hom[2] > 1e-3):
            stab_x = p_virt_hom[0] / p_virt_hom[2]
            stab_y = p_virt_hom[1] / p_virt_hom[2]
            if math.isfinite(stab_x) and math.isfinite(stab_y):
                return stab_x, stab_y
        return obj_x, obj_y

    def coverage_metric(self, bbox_w, bbox_h):
        """Yatay veya dikeyde max kaplama yüzdesi (kamp_basi_emir'den)."""
        h_cov = (bbox_w / self.W) * 100.0
        v_cov = (bbox_h / self.H) * 100.0
        return max(h_cov, v_cov), h_cov, v_cov

    def get_current_state(self):
        """Telemetri cache'inden güncel durumu oku."""
        att_msg = self.mavlink_handler.master.messages.get('ATTITUDE', None)
        if att_msg:
            roll_rad = att_msg.roll
            pitch_rad = att_msg.pitch
            yaw_rad = att_msg.yaw
        else:
            roll_rad, pitch_rad, yaw_rad = 0.0, 0.0, 0.0

        loc_msg = self.mavlink_handler.master.messages.get('GLOBAL_POSITION_INT', None)
        if loc_msg:
            current_alt = loc_msg.relative_alt / 1000.0  # Relative alt (emir yaklaşımı)
        else:
            current_alt = self.last_target_alt

        hb = self.mavlink_handler.master.messages.get('HEARTBEAT', None)
        current_mode = mavutil.mode_string_v10(hb) if hb else "UNKNOWN"

        yaw_deg = math.degrees(yaw_rad) % 360.0

        return roll_rad, pitch_rad, yaw_rad, yaw_deg, current_alt, current_mode

    def command_current_hold(self, airspeed_ms=None):
        """Goat failsafe davranışı: mevcut yön/irtifayı yeni GUIDED hedefi yap."""
        _, _, _, yaw_deg, current_alt, current_mode = self.get_current_state()
        if current_mode != 'GUIDED' or not self.telemetry_is_fresh(
                ('HEARTBEAT', 'ATTITUDE', 'GLOBAL_POSITION_INT')):
            self.hold_active = False
            self.cmd_thread.deactivate()
            return False

        # Hold hedefini yalnız geçiş anında örnekle. Her bbox mesajında yeniden
        # örneklemek, dönen/tırmanan uçağın hedefini sürükler ve gerçek bir hold
        # oluşturmazdı.
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

    # ─── ANA GUIDANCE FONKSİYONU ───

    def guide_aircraft(self, bbox, current_time):
        """
        BBox alıp 3 eksende (heading, altitude, airspeed) kontrol komutu hesaplar.
        bbox: (x, y, w, h) — sol üst köşe + genişlik/yükseklik
        """
        # BBox merkezi
        obj_x = bbox[0] + (bbox[2] / 2.0)
        obj_y = bbox[1] + (bbox[3] / 2.0)
        bbox_w, bbox_h = float(bbox[2]), float(bbox[3])

        # Stabilizasyon (virtual gimbal)
        stab_x, stab_y = self.stabilize_pixel(obj_x, obj_y)

        # Açısal hata (derece) — kamera bağımsız (tzi_final yaklaşımı)
        stab_error_x_deg = math.degrees(math.atan((stab_x - self.center_x) / self.fx))
        stab_error_y_deg = math.degrees(math.atan((stab_y - self.center_y) / self.fy))

        # Telemetri
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
            if now - self.last_telemetry_warning_time > 1.0:
                self.logger.warning("Telemetri eski/eksik; GUIDED komutları durduruldu.")
                self.last_telemetry_warning_time = now
            return

        # Coverage (airspeed PID için)
        coverage, h_cov, v_cov = self.coverage_metric(bbox_w, bbox_h)

        # Debug log
        self.logger.debug(
            f"Target: Cov={coverage:.1f}% | Stab: ({stab_x:.0f},{stab_y:.0f}) "
            f"Err: ({stab_error_x_deg:+.2f}°,{stab_error_y_deg:+.2f}°) | "
            f"BBox: {bbox_w:.0f}x{bbox_h:.0f} | Alt: {current_alt:.1f}m | "
            f"Mode: {current_mode}"
        )

        # ─── GUIDED modda değilse → state sıfırla ───
        if current_mode != 'GUIDED':
            self._reset_pid_states()
            self.guided_entry_done = False
            self.hold_active = False
            self.cmd_thread.deactivate()
            self.logger.debug(f"PID skipped (mode={current_mode}), state reset.")
            return

        # ─── GUIDED ENTRY INITIALIZATION (tzi_final'den) ───
        first_guided_sample = not self.guided_entry_done
        if first_guided_sample:
            measured_as = self.cmd_thread.last_airspeed
            if measured_as is not None:
                init_speed = self.clamp(measured_as, self.min_speed, self.max_speed)
                self.last_speed = init_speed
                self.logger.debug(f"GUIDED Entry: Init speed={init_speed:.1f} m/s")
            # İlk karede sıfırdan hata türevi üretip rate'i zıplatma.
            self.prev_error_x = stab_error_x_deg
            self.prev_error_y = stab_error_y_deg
            self.prev_derivative_x = 0.0
            self.prev_derivative_y = 0.0
            self.last_time = current_time
            self.guided_entry_done = True

        # ─── dt hesabı ───
        if first_guided_sample:
            dt = 1.0 / 30.0
        else:
            dt = max(0.001, min(current_time - self.last_time, 0.1))
        self.last_time = current_time

        # ─── Low-pass filtered derivative (goat_gimbal'den) ───
        raw_deriv_x = (stab_error_x_deg - self.prev_error_x) / dt
        raw_deriv_y = (stab_error_y_deg - self.prev_error_y) / dt
        deriv_x = (self.deriv_alpha * raw_deriv_x) + \
                  ((1.0 - self.deriv_alpha) * self.prev_derivative_x)
        deriv_y = (self.deriv_alpha * raw_deriv_y) + \
                  ((1.0 - self.deriv_alpha) * self.prev_derivative_y)

        self.prev_error_x = stab_error_x_deg
        self.prev_error_y = stab_error_y_deg
        self.prev_derivative_x = deriv_x
        self.prev_derivative_y = deriv_y

        # ─── Deadzone (tzi_final + emir ortak) ───
        dz_x = abs(stab_error_x_deg) < self.deadzone_deg
        dz_y = abs(stab_error_y_deg) < self.deadzone_deg
        p_x = 0.0 if dz_x else stab_error_x_deg - math.copysign(self.deadzone_deg, stab_error_x_deg)
        p_y = 0.0 if dz_y else stab_error_y_deg - math.copysign(self.deadzone_deg, stab_error_y_deg)

        # ─── İntegral birikimi (heading/altitude) ───
        if not dz_x:
            self.integral_error_x += stab_error_x_deg * dt
        else:
            self.integral_error_x *= 0.99  # Decay in deadzone
        self.integral_error_x = self.clamp(self.integral_error_x, -10.0, 10.0)

        if not dz_y:
            self.integral_error_y += stab_error_y_deg * dt
        else:
            self.integral_error_y *= 0.99
        self.integral_error_y = self.clamp(self.integral_error_y, -10.0, 10.0)

        # ═══════════════════════════════════════════
        # 1. HEADING KONTROLÜ
        # ═══════════════════════════════════════════
        p_term_heading = p_x * self.Kp_heading
        i_term_heading = self.integral_error_x * self.Ki_heading
        d_term_heading = deriv_x * self.Kd_heading

        cmd_head_deg = p_term_heading + i_term_heading + d_term_heading
        cmd_head_deg = self.clamp(cmd_head_deg, -self.max_heading_change_deg,
                                  self.max_heading_change_deg)
        target_heading = (yaw_deg + cmd_head_deg) % 360.0
        if FIXED_HEADING_DEG is not None:
            target_heading = float(FIXED_HEADING_DEG) % 360.0

        # ─── Heading Rate PID (kamp_basi_emir'den) ───
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

        # ═══════════════════════════════════════════
        # 2. ALTITUDE KONTROLÜ
        # ═══════════════════════════════════════════
        # Hedef aşağıdaysa (p_y pozitif) → irtifa düşürülmeli
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

        # ─── Altitude Rate PID (kamp_basi_emir'den) ───
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

        # ═══════════════════════════════════════════
        # 3. AIRSPEED KONTROLÜ  (coverage-based PID + slew rate)
        # ═══════════════════════════════════════════
        # EMA filtered coverage (kamp_basi_emir'den)
        self.filtered_coverage = (self.coverage_alpha * coverage) + \
                                 ((1.0 - self.coverage_alpha) * self.filtered_coverage)

        speed_error = self.target_coverage_pct - self.filtered_coverage

        # Derivative (low-pass)
        raw_deriv_speed = 0.0 if first_guided_sample else \
            (speed_error - self.prev_error_speed) / dt
        deriv_speed = (self.deriv_alpha * raw_deriv_speed) + \
                      ((1.0 - self.deriv_alpha) * self.prev_derivative_speed)

        # Integral (band-limited, kamp_basi_emir'den)
        if abs(speed_error) < self.speed_integral_band:
            self.integral_error_speed += speed_error * dt
        self.integral_error_speed = self.clamp(self.integral_error_speed, -100.0, 100.0)

        # PID çıkışı
        p_term_speed = speed_error * self.Kp_speed
        i_term_speed = self.integral_error_speed * self.Ki_speed
        d_term_speed = deriv_speed * self.Kd_speed

        cmd_speed = self.base_speed + (p_term_speed + i_term_speed + d_term_speed)
        cmd_speed = self.clamp(cmd_speed, self.min_speed, self.max_speed)

        # Slew rate limiti (tzi_final'den, TECS'i ani hız değişimlerinden korur)
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
        # ANTI-WINDUP  (kamp_basi_emir'den)
        # ═══════════════════════════════════════════
        if abs(cmd_head_deg) >= self.max_heading_change_deg:
            self.integral_error_x *= 0.9
        if abs(cmd_alt_m) >= self.max_alt_change_m:
            self.integral_error_y *= 0.9

        # ═══════════════════════════════════════════
        # KOMUT GÖNDER  (TestCommander'a)
        # ═══════════════════════════════════════════
        self.hold_active = False
        self.cmd_thread.update(
            target_heading, heading_rate,
            target_alt, altitude_rate,
            cmd_speed
        )

        # Tracking state güncelle (coasting için)
        self.last_target_time = current_time
        self.last_target_heading = target_heading
        self.last_target_alt = target_alt
        self.last_heading_rate = heading_rate
        self.last_alt_rate = altitude_rate

        # ═══════════════════════════════════════════
        # LOGLAMA
        # ═══════════════════════════════════════════
        measured_as = self.cmd_thread.last_airspeed
        measured_as_str = f"{measured_as:.1f}" if measured_as is not None else "?"
        ack_heading = self.get_command_ack(43002) if SEND_HEADING_COMMANDS else "DISABLED"
        ack_altitude = self.get_command_ack(43001) if SEND_ALTITUDE_COMMANDS else "DISABLED"
        ack_airspeed = self.get_command_ack(178) if SEND_AIRSPEED_COMMANDS else "DISABLED"

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
            current_mode, "Goruntulu", 1, 1,
            ack_heading, ack_altitude, ack_airspeed,
            bbox[0], bbox[1], f"{bbox_w:.1f}", f"{bbox_h:.1f}",
            f"{coverage:.2f}", f"{self.filtered_coverage:.2f}",
            f"{stab_x:.2f}", f"{stab_y:.2f}",
            f"{stab_error_x_deg:.4f}", f"{stab_error_y_deg:.4f}",
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
        """Tüm PID/PD state'lerini sıfırla."""
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
        """Redis mesajını parse eder. (x, y, w, h) tuple veya None döner."""
        if not isinstance(data, (list, tuple)) or len(data) < 4:
            return None

        # bbox_to_redis.py altıncı alanda validity gönderirse ona da uy.
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

        # Görüntü dışında kalan kısmı kırp; tamamen dışarıdaki kutuyu reddet.
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

    # ─── Thread'ler ───

    def _redis_listener(self):
        """Thread 1: Redis pub/sub dinler, bbox'ı thread-safe depoya yazar."""
        self.logger.debug("Redis listener thread başladı.")
        for message in self.p.listen():
            if message['type'] != 'message':
                continue

            # Görev kontrolü
            guid_message = self.r.get('gorev')
            if guid_message is None or guid_message.decode('utf-8').lower() != 'goruntulu':
                if self.cmd_thread.is_active():
                    self.command_current_hold()
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
                # goat_gimbal ile uçuşta kullanılan Python liste/tuple formatı.
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

    def _vision_processor(self):
        """Thread 2: 30 Hz'de en güncel bbox'ı okuyup guide_aircraft() çağırır.
        Coasting ve Failsafe yönetimi burada yapılır."""
        self.logger.debug("Vision processor thread başladı.")
        period = 1.0 / 30.0
        last_processed_time = None
        last_warning_time = 0.0

        while True:
            time.sleep(period)

            with self.bbox_lock:
                bbox = self.latest_bbox
                bbox_time = self.latest_bbox_time

            # Yeni bbox geldiyse işle
            if bbox is not None and bbox_time != last_processed_time:
                last_processed_time = bbox_time
                self.guide_aircraft(bbox, bbox_time)
            else:
                # ─── HEDEF YOK: Coasting veya Failsafe ───
                now = time.time()
                if self.last_target_time > 0.0:
                    time_since_last = now - self.last_target_time

                    if time_since_last <= self.coasting_timeout:
                        # COASTING: TestCommander son hedefleri göndermeye devam eder.
                        # Ek işlem gerekmez (inherent coasting — tzi mimarisi avantajı).
                        pass
                    else:
                        # FAILSAFE: 5 saniye aşıldı!
                        # ArduPilot GUIDED heading/altitude hedeflerini yeni bir
                        # komut gelene kadar korur. Sadece commander'ı durdurmak
                        # son dönüş/tırmanış hedefini uçakta bırakacağından,
                        # kayıp anındaki mevcut yön ve irtifayı bir kez hold et.
                        if not self.hold_active:
                            hold_speed = self.last_speed
                            self._reset_pid_states()
                            self.guided_entry_done = False
                            self.command_current_hold(airspeed_ms=hold_speed)

                        holding = self.hold_active

                        if now - last_warning_time > 1.0:
                            if holding:
                                self.logger.warning(
                                    "\n[UYARI] HEDEF 5 SANİYEDEN UZUN SÜREDİR KAYIP; "
                                    "MEVCUT YÖN/İRTİFA HOLD EDİLİYOR.")
                            else:
                                self.logger.warning(
                                    "\n[UYARI] HEDEF KAYIP VE HOLD KURULAMADI; "
                                    "GÖREV MODUNU DEĞİŞTİRİN!")
                            last_warning_time = now

                        # CSV log
                        elapsed = now - self.start_time
                        _, _, _, yaw_deg, current_alt, current_mode = \
                            self.get_current_state()
                        self.csv_logger.log([
                            f"{now:.4f}", f"{elapsed:.3f}", f"{time_since_last:.4f}",
                            "TARGET_LOST_HOLD" if holding else "FAILSAFE",
                            current_mode, "Goruntulu", 0, 0,
                            self.get_command_ack(43002) if SEND_HEADING_COMMANDS else "DISABLED",
                            self.get_command_ack(43001) if SEND_ALTITUDE_COMMANDS else "DISABLED",
                            self.get_command_ack(178) if SEND_AIRSPEED_COMMANDS else "DISABLED",
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                            0, 0, f"{yaw_deg:.2f}", f"{current_alt:.2f}",
                            0, 0,
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0,
                        ])

    def run(self):
        """Ana çalıştırıcı. Tüm thread'leri başlatır."""
        listener_thread = threading.Thread(
            target=self._redis_listener, daemon=True, name="RedisListener"
        )
        listener_thread.start()

        processor_thread = threading.Thread(
            target=self._vision_processor, daemon=True, name="VisionProcessor"
        )
        processor_thread.start()

        self.logger.debug("═" * 60)
        self.logger.debug("tzi_emir.py — Tüm threadler başlatıldı.")
        self.logger.debug(
            f"Kamera: {self.W}x{self.H}, HFOV={CAMERA_HFOV_RAD:.3f} rad, "
            f"fx={self.fx:.2f}, fy={self.fy:.2f}")
        self.logger.debug(
            "Komut eksenleri: "
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
            self.logger.debug("Kapatılıyor...")
            self.csv_logger.close()
            self.cmd_thread.stop()


if __name__ == '__main__':
    guidance = tziGuidance()
    guidance.run()
