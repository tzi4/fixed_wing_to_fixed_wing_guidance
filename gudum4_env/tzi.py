import numpy as np
import math
import random
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

# ─── SABİTLER (tzi.py'de kullanılan config değerleri) ───
RESOLUTION_W = 640
RESOLUTION_H = 480
CAMERA_FOCAL_LENGTH = 467.7
MIN_ALTITUDE = 10
MAX_ALTITUDE = 200
MAX_DELTA_HEADING = 15
MAX_DELTA_ALTITUDE = 6
UAV_PORT = '14553'


# Kamera->Body rotasyonu (kamera z ileri, x sağ, y aşağı kabulü ile örnek)
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
      - Airspeed: TRIM_THROTTLE parametresi ile kontrol ediliyor
    """
    def __init__(self, mav_handler, rate_hz=5):
        super().__init__(daemon=True)
        self.mav_handler = mav_handler         # MAVLinkHandlerPymavlink objesi
        self.master = mav_handler.master       # pymavlink master (mesaj göndermek için)
        self.rate = rate_hz
        self.running = True
        self.lock = threading.Lock()

        # VFR_HUD mesajını da iste (airspeed için)
        self.mav_handler.request_message_interval(
            [mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD], 10  # 10 Hz yeterli
        )

        # AttributeError Almamak için boş değerler 
        self.target_heading_deg = 0.0    # degrees (0-360)
        self.target_alt_m = 100.0        # meters (AMSL)
        self.target_throttle_pct = 50.0  # TRIM_THROTTLE (0-127)
        self.active = False

        # Son okunan telemetri (cache)
        self.last_att = None       # (pitch_deg, roll_deg, yaw_deg)
        self.last_loc = None       # (lat, lon, alt_m)
        self.last_airspeed = None  # m/s
        self.last_mode = None      # ucus modu string (örn: 'GUIDED', 'AUTO')

    def update(self, heading_deg, alt_m, throttle_pct):
        """Thread-safe hedef güncelleme."""
        with self.lock:
            self.target_heading_deg = float(heading_deg)
            self.target_alt_m = float(alt_m)
            self.target_throttle_pct = float(throttle_pct)
            self.active = True

    def _send_heading(self, heading_deg):
        """
        param1: 0 = course over ground, 1 = raw magnetic heading
        param2: heading (degrees, 0-360)
        param3: heading rate (deg/s, 0'dan büyük olmali!)
        """
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            43002,  # MAV_CMD_GUIDED_CHANGE_HEADING
            0,      # confirmation
            1,      # param1: 1 = raw magnetic heading (burun yönü)
            heading_deg,  # param2: hedef heading (derece)
            40,     # param3: heading rate (deg/s) - 0 olursa heading değişmez!
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

    def _send_throttle(self, throttle_pct):
        """TRIM_THROTTLE parametresini param_set ile güncelle.
        Sadece GUIDED modundayken throttle degisikligine izin ver.
        TRIM_THROTTLE: TECS cruise throttle (50-127).
        Bu parametre runtime'da değiştirildiğinde TECS hedef hizi
        otomatik olarak günceller.
        """
        if self.last_mode != 'GUIDED':
            return
            
        throttle_pct = max(0, min(127, throttle_pct))
        self.master.mav.param_set_send(
            self.master.target_system,
            self.master.target_component,
            b'TRIM_THROTTLE',
            float(throttle_pct),
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        )

    def _read_telemetry(self):
        """Tüm telemetriyi pymavlink dahili cache'inden oku (non-blocking).
        
        NOT: recv_match KULLANILMAZ! Tüm mesajlar MAVLink reader thread
        tarafından cache'lenir, biz sadece cache'ten okuruz.
        """
        # Attitude — cache'ten oku
        att_msg = self.mav_handler.master.messages.get('ATTITUDE', None)
        if att_msg:
            self.last_att = (math.degrees(att_msg.pitch), math.degrees(att_msg.roll), math.degrees(att_msg.yaw))
        
        # Location — cache'ten oku
        loc_msg = self.mav_handler.master.messages.get('GLOBAL_POSITION_INT', None)
        if loc_msg:
            self.last_loc = (loc_msg.lat/1e7, loc_msg.lon/1e7, loc_msg.alt/1000.0)
        
        # Airspeed — pymavlink dahili cache'inden oku (race condition yok!)
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
                # 1) Telemetriyi oku
                self._read_telemetry()
                
                # 2) Hedefleri al
                with self.lock:
                    hdg = self.target_heading_deg
                    alt = self.target_alt_m
                    thr = self.target_throttle_pct
                
                # SENDER
                hdg = 0.0
                alt = 40.0
                
                self._send_heading(hdg)
                self._send_altitude(alt)
                self._send_throttle(thr)
                
                # 4) Terminale bas
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
        self.K = np.array([
            [f_oc, 0, self.center_x],
            [0, f_oc, self.center_y],
            [0, 0, 1]
        ])
        self.logger.debug(f"Camera Intrinsic Matrix K:\n{self.K}")
        self.K_inv = np.linalg.inv(self.K)
        self.logger.debug(f"Inverse K Matrix:\n{self.K_inv}")

        # GUIDANCE
        self.MIN_ALTITUDE = MIN_ALTITUDE
        self.MAX_ALTITUDE = MAX_ALTITUDE

        self.mavlink_handler = MAVLinkHandler(f'127.0.0.1:{UAV_PORT}')

        self.logger.debug('Connected to the aircraft.')
        self.r.set('guid','False')

        # MAVLink reader thread — sürekli recv_match yaparak cache'i güncel tutar
        self._mavlink_reader_thread = threading.Thread(
            target=self._mavlink_reader, daemon=True, name="MAVLinkReader")
        self._mavlink_reader_thread.start()
        self.logger.debug('MAVLink reader thread started.')

        # Exit handler
        atexit.register(self.exit_handler)

    def _mavlink_reader(self):
        """Sürekli recv_match yaparak pymavlink dahili cache'ini güncel tutar.
        Bu sayede diğer thread'ler non-blocking cache okuması yapabilir."""
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
        
        log_file_path = 'tzi_guidance.log'
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

        # --- En güncel bbox için thread-safe depo ---
        self.latest_bbox = None
        self.latest_bbox_time = None
        self.bbox_lock = threading.Lock()
        
        # Initial Values
        self.test_heading_deg = 0     # derece (0-360, magnetic heading)
        self.test_alt_m = 50          # metre (AMSL)
        self.test_throttle_pct = 69   # % (TRIM_THROTTLE)

        
        # THROTTLE PID Variables for Airspeed
        self.throttle_kp = 4.7   # Tuning P
        self.throttle_ki = 0.45  # Tuning I
        self.throttle_kd = 0.6   # Tuning D
        self.throttle_integral = 0.0
        self.throttle_prev_error = 0.0
        self.throttle_target_sqrt_area = 24.0  
        self.throttle_integral_min = -10.0  # Asimetrik integral alt limiti 
        self.throttle_integral_max = 55.0   # Asimetrik integral üst limiti
        self.throttle_base = 50.0  # Baslangic TRIM_THROTTLE degeri (uzerine PID eklenecek)
        
        # Clamp limitleri
        self.max_delta_heading = MAX_DELTA_HEADING
        self.max_delta_altitude = MAX_DELTA_ALTITUDE

        # HEADING PD Variables — P = max_delta_heading / (W/2)
        self.heading_target_u = self.W / 2.0   # 320 for 640px
        self.heading_kp = self.max_delta_heading / self.heading_target_u
        self.heading_kd = 0.02
        self.heading_prev_error = 0.0

        # ALTITUDE PD Variables — P = max_delta_altitude / (H/2)
        self.altitude_target_v = self.H / 2.0  # 240 for 480px
        self.altitude_kp = self.max_delta_altitude / self.altitude_target_v
        self.altitude_kd = 0.01
        self.altitude_prev_error = 0.0

        self.last_pid_time = None
        
        # Initialize TestCommander (Direct Control)
        self.cmd_thread = TestCommander(self.mavlink_handler, rate_hz=5)
        self.cmd_thread.update(self.test_heading_deg, self.test_alt_m, self.test_throttle_pct)
        self.cmd_thread.start()



    def guide_aircraft(self, bbox, current_time): #bbox: x1,y1,w,h
        object_x,object_y = bbox[0] + (bbox[2] / 2), bbox[1] + (bbox[3] / 2)
        
        # Calculate r_cam using Back-Projection
        if self.K_inv is not None:
            # p_raw = [u, v, 1]
            p_raw = np.array([object_x, object_y, 1.0])
            r_cam = self.K_inv @ p_raw
            
            # Kamera -> Body
            r_body = R_c_b @ r_cam
            
            # Stabilization (Body -> Virtual Body)
            # Cache'ten ATTITUDE oku (non-blocking). Radyan olarak gelir.
            att_msg = self.mavlink_handler.master.messages.get('ATTITUDE', None)
            if att_msg:
                roll_rad = att_msg.roll
                pitch_rad = att_msg.pitch
                yaw_rad = att_msg.yaw
            else:
                roll_rad, pitch_rad, yaw_rad = 0, 0, 0

            
            # R_stab hesapla (Yaw = 0 ile compute_R_b_e çağırılır)
            R_stab = compute_R_b_e(roll_rad, pitch_rad, 0)
            
            # r_virt_body = R_stab * r_body
            r_virt_body = R_stab @ r_body
            
            # Sanal Piksele İzdüşüm (Re-Projection)
            r_virt_cam = R_c_b_T @ r_virt_body
            
            # p_virt_hom = K * r_virt_cam
            p_virt_hom = self.K @ r_virt_cam
            
            # Normalize Homogeneous -> Cartesian (u_virt, v_virt)
            if p_virt_hom[2] != 0:
                u_virt = p_virt_hom[0] / p_virt_hom[2]
                v_virt = p_virt_hom[1] / p_virt_hom[2]
            else:
                u_virt, v_virt = 0, 0
        
            # BBox alan hesabı
            bbox_w, bbox_h = bbox[2], bbox[3]
            bbox_area = bbox_w * bbox_h
            bbox_sqrt_area = math.sqrt(bbox_area)

            # Get current flight mode
            current_mode = "UNKNOWN"
            hb = self.mavlink_handler.master.messages.get('HEARTBEAT', None)
            if hb:
                current_mode = mavutil.mode_string_v10(hb)

            # Log stabilized virtual pixel with flight mode
            self.logger.debug(f"Virtual Gimbal: u_virt={u_virt:.2f}, v_virt={v_virt:.2f} | BBox: {bbox} ({bbox_w}x{bbox_h}, sqrt={bbox_sqrt_area:.1f}) | RPY(rad): {roll_rad:.3f},{pitch_rad:.3f},{yaw_rad:.3f} | Mode: {current_mode}")



            # --- GUIDED modda değilse PID/PD hesaplama, state sıfırla ---
            if current_mode != 'GUIDED':
                self.throttle_integral = 0.0
                self.throttle_prev_error = 0.0
                self.heading_prev_error = 0.0
                self.altitude_prev_error = 0.0
                self.last_pid_time = None
                self.logger.debug(f"PID/PD skipped (mode={current_mode}), state reset.")
                return

            # --- PID Kontrolcusu (Throttle) ---
            is_first_pid = getattr(self, 'last_pid_time', None) is None
            if is_first_pid:
                dt = 0.033  # Yaklasik 30Hz varsayimi (Sadece ilk calismada)
            else:
                dt = current_time - self.last_pid_time
                if dt <= 0.001:
                    dt = 0.033

            self.last_pid_time = current_time

            # Error hesabi: hedef - mevcut (hedef buyukluge ulasilmaya calisiliyor)
            thr_error = self.throttle_target_sqrt_area - bbox_sqrt_area

            # Proportional
            thr_p_term = self.throttle_kp * thr_error

            # Integral step (bu dongudeki integral artis miktari)
            thr_integral_step = self.throttle_ki * thr_error * dt

            # Derivative
            if is_first_pid:
                self.throttle_prev_error = thr_error
            thr_derivative = (thr_error - self.throttle_prev_error) / dt
            thr_d_term = self.throttle_kd * thr_derivative
            self.throttle_prev_error = thr_error

            # Ham cikis = P + (Eski I + Yeni I) + D
            thr_pid_output = thr_p_term + self.throttle_integral + thr_integral_step + thr_d_term
            raw_throttle = self.throttle_base + thr_pid_output

            # Integrali her adımda güncelle, ardından asimetrik clamp uygula
            self.throttle_integral += thr_integral_step
            self.throttle_integral = max(self.throttle_integral_min, min(self.throttle_integral_max, self.throttle_integral))

            # Çıkışı sınırla (Min 50, Max 127)
            throttle_out = max(50.0, min(127.0, raw_throttle))
            self.test_throttle_pct = throttle_out

            # --- HEADING PD Kontrolcusu ---
            # If target is on the right (u_virt > 320), increase heading (turn right).
            hdg_error = u_virt - self.heading_target_u
            if is_first_pid:
                self.heading_prev_error = hdg_error
            hdg_p_term = self.heading_kp * hdg_error
            hdg_derivative = (hdg_error - self.heading_prev_error) / dt
            hdg_d_term = self.heading_kd * hdg_derivative
            self.heading_prev_error = hdg_error
            delta_heading_raw = hdg_p_term + hdg_d_term

            # Heading delta clamp (±max_delta_heading derece)
            delta_heading = max(-self.max_delta_heading, min(self.max_delta_heading, delta_heading_raw))

            # Normalize current yaw to 0-360 degrees
            current_heading_deg = np.degrees(yaw_rad)
            if current_heading_deg < 0:
                current_heading_deg += 360.0
                
            new_heading = (current_heading_deg + delta_heading) % 360.0
            self.test_heading_deg = new_heading

            # --- ALTITUDE PD Kontrolcusu ---
            # If target is below center (v_virt > 240), decrease altitude.
            alt_error = self.altitude_target_v - v_virt
            if is_first_pid:
                self.altitude_prev_error = alt_error
            alt_p_term = self.altitude_kp * alt_error
            alt_derivative = (alt_error - self.altitude_prev_error) / dt
            alt_d_term = self.altitude_kd * alt_derivative
            self.altitude_prev_error = alt_error
            delta_altitude_raw = alt_p_term + alt_d_term

            # Altitude delta clamp (±max_delta_altitude metre)
            delta_altitude = max(-self.max_delta_altitude, min(self.max_delta_altitude, delta_altitude_raw))

            # Fetch current physical altitude
            loc_msg = self.mavlink_handler.master.messages.get('GLOBAL_POSITION_INT', None)
            if loc_msg:
                current_alt = loc_msg.alt / 1000.0  # AMSL (mm -> m)
            else:
                current_alt = self.test_alt_m

            new_altitude = current_alt + delta_altitude
            
            # Cap altitude (min ve max — config'den okunan değerler)
            new_altitude = max(self.MIN_ALTITUDE, min(self.MAX_ALTITUDE, new_altitude))
            self.test_alt_m = new_altitude

            # OVERRIDE FOR SEQUENTIAL TUNING
            # new_altitude = 48.0
            # self.test_alt_m = new_altitude

            # Commander thread'e hedefi guncelle
            self.cmd_thread.update(self.test_heading_deg, self.test_alt_m, self.test_throttle_pct)
            
            self.logger.debug(f"Throttle PID: Thr={throttle_out:.1f} | SqrtBBox={bbox_sqrt_area:.1f} | Err={thr_error:.1f} | P={thr_p_term:.2f} I={self.throttle_integral:.2f} D={thr_d_term:.2f} | dt={dt:.3f}")
            self.logger.debug(f"Heading PD: Hdg={current_heading_deg:.1f} | Err={hdg_error:.1f} | P={hdg_p_term:.2f} D={hdg_d_term:.2f} | dHdg_raw={delta_heading_raw:.2f} dHdg={delta_heading:.2f} | NewHdg={new_heading:.1f}")
            self.logger.debug(f"Altitude PD: Alt={current_alt:.1f} | Err={alt_error:.1f} | P={alt_p_term:.2f} D={alt_d_term:.2f} | dAlt_raw={delta_altitude_raw:.2f} dAlt={delta_altitude:.2f} | NewAlt={new_altitude:.1f}")
            self.logger.debug("-" * 50)

    def _parse_bbox(self, data):
        """Gelen Redis mesajini parse eder, (x, y, w, h) tuple döner veya None."""
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
        """Thread 1: Redis pub/sub'i dinler, gelen her mesaji hizla parse edip
        self.latest_bbox'a yazar. Hiçbir ağir islem yapmaz."""
        self.logger.debug("Redis listener thread başladi.")
        for message in self.p.listen():
            if message['type'] != 'message':
                continue

            # Görev kontrolü
            guid_message = self.r.get('gorev')
            if guid_message is None:
                guid_message = b'goruntulu'
            if guid_message.decode('utf-8').lower() != 'goruntulu':
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
        """Thread 2: Sabit frekansta (30 Hz) en güncel bbox'ı okuyup
        guide_aircraft() çağırır. Aynı bbox'ı tekrar işlemez."""
        self.logger.debug("Vision processor thread başladı.")
        period = 1.0 / 30.0
        last_processed_time = None

        while True:
            time.sleep(period)
            with self.bbox_lock:
                bbox = self.latest_bbox
                bbox_time = self.latest_bbox_time

            # Yeni bir bbox geldiyse işle
            if bbox is not None and bbox_time != last_processed_time:
                last_processed_time = bbox_time
                self.guide_aircraft(bbox, bbox_time)

    def run(self):
        # Thread 1: Redis listener (sadece okuma ve parse)
        listener_thread = threading.Thread(
            target=self._redis_listener, daemon=True, name="RedisListener")
        listener_thread.start()

        # Thread 2: Vision processor (guide_aircraft çağrısı)
        processor_thread = threading.Thread(
            target=self._vision_processor, daemon=True, name="VisionProcessor")
        processor_thread.start()

        self.logger.debug("Tüm thread'ler başlatıldı. Ana thread bekliyor...")
        listener_thread.join()
        processor_thread.join()


if __name__ == '__main__':
    goat = tziGuidance()
    goat.run()
