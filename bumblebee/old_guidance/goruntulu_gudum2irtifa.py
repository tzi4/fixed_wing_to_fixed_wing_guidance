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

# ─── LOG KURULUMU ───
# Aktif loglar:  guidance_logs/
# Eski loglar:   old_guidance_logs/  (her başlangıçta otomatik taşınır)
LOG_DIR = "guidance_logs"
OLD_LOG_DIR = "old_guidance_logs"

def setup_logging():
    os.makedirs(LOG_DIR, exist_ok=True)
    os.makedirs(OLD_LOG_DIR, exist_ok=True)

    # Önceki çalışmadan kalan tüm logları arşive taşı
    for fname in os.listdir(LOG_DIR):
        src = os.path.join(LOG_DIR, fname)
        if os.path.isfile(src):
            dst = os.path.join(OLD_LOG_DIR, fname)
            # Aynı isimde dosya varsa üzerine yazmamak için sonuna sayaç ekle
            if os.path.exists(dst):
                base, ext = os.path.splitext(fname)
                i = 1
                while os.path.exists(os.path.join(OLD_LOG_DIR, f"{base}_{i}{ext}")):
                    i += 1
                dst = os.path.join(OLD_LOG_DIR, f"{base}_{i}{ext}")
            shutil.move(src, dst)

    log_path = os.path.join(LOG_DIR, f"guidance_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log")

    root = logging.getLogger()
    root.setLevel(logging.DEBUG)  # Dosyaya detay, terminale özet

    fmt_file = logging.Formatter('%(asctime)s.%(msecs)03d [%(levelname)s] %(message)s', datefmt='%H:%M:%S')
    fmt_term = logging.Formatter('%(asctime)s [%(levelname)s] %(message)s', datefmt='%H:%M:%S')

    fh = logging.FileHandler(log_path)
    fh.setLevel(logging.DEBUG)      # Detaylı telemetri (TLM satırları) sadece dosyaya
    fh.setFormatter(fmt_file)

    sh = logging.StreamHandler()
    sh.setLevel(logging.INFO)       # Terminal sadece özet/olay satırları
    sh.setFormatter(fmt_term)

    root.addHandler(fh)
    root.addHandler(sh)
    logging.info(f"Log dosyası: {log_path}")

setup_logging()
# ─── SANAL GİMBAL SABİTLERİ ───
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
        self.gorev = "Bilinmiyor"
        
        logging.info("Redis sunucusuna bağlanılıyor...")
        try:
            self.r = redis.Redis(host='localhost', port=6379, db=0)
            self.pubsub = self.r.pubsub()
            self.pubsub.subscribe('tracker_bbox')
            logging.info("Redis 'tracker_bbox' kanalına abone olundu.")
        except Exception as e:
            logging.error(f"Redis Bağlantı Hatası: {e}")
            self.running = False
            
    def run(self):
        while self.running:
            try:
                gorev_bytes = self.r.get('gorev')
                self.gorev = gorev_bytes.decode('utf-8') if gorev_bytes else "Bilinmiyor"
                
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
                                'gorev': self.gorev
                            })
                    except:
                        pass
            except:
                time.sleep(0.1)
                
    def get_gorev(self):
        return self.gorev

    def get_rakip_irtifa(self):
        try:
            # Redisten veriyi byte olarak al
            data_bytes = self.r.get('rakip_telemetri')
            if data_bytes:
                # Byte'ı stringe, ardından JSON objesine (dictionary) çevir
                data = json.loads(data_bytes.decode('utf-8'))

                # JSON yapısına göre konumBilgileri dizisinin ilk elemanından irtifayı çek
                rakip_irtifa = data.get("konumBilgileri", [{}])[0].get("iha_irtifa", None)
                return float(rakip_irtifa) if rakip_irtifa is not None else None
        except Exception as e:
            # Parse hatası veya veri yoksa sistemi çökertmemek için pass geç
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
        self.current_alt_rel = 100.0  # Varsayılan başlangıç irtifası
        self.current_airspeed = 20.0
        self.current_climb = 0.0      # Gerçekleşen dikey hız (VFR_HUD.climb)
        
        logging.info(f"MAVLink bağlantısı: {connection_str}...")
        try:
            self.master = mavutil.mavlink_connection(connection_str)
            self.master.wait_heartbeat()
            logging.info(f"Bağlantı Başarılı. Sistem ID: {self.master.target_system}")
            # Ekstra telemetri akışlarını talep et (Altitude için GLOBAL_POSITION_INT vb.)
            self.master.mav.request_data_stream_send(
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_POSITION, 10, 1)
            self.master.mav.request_data_stream_send(
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 10, 1)
            # VFR_HUD verisi genellikle EXTRA2 akışındadır
            self.master.mav.request_data_stream_send(
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_EXTRA2, 10, 1)
        except Exception as e:
            logging.error(f"MAVLink Hatası: {e}")
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
                # 1. HIZ KOMUTU (DO_CHANGE_SPEED)
                self.master.mav.command_int_send(
                    self.master.target_system, self.master.target_component,
                    mavutil.mavlink.MAV_FRAME_GLOBAL,
                    mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
                    0, 0,
                    0, target_speed_m_s, -1, 0, # param2=hız, param3=throttle değişimi (-1)
                    0, 0, 0
                )
                
                # 2. YÖN KOMUTU (GUIDED_CHANGE_HEADING)
                self.master.mav.command_int_send(
                    self.master.target_system, self.master.target_component,
                    mavutil.mavlink.MAV_FRAME_GLOBAL,
                    mavutil.mavlink.MAV_CMD_GUIDED_CHANGE_HEADING,
                    0, 0,
                    1, heading_deg, 3.0, 0,  # param1=Mutlak Açı, param2=Açı, param3=Dönüş Hızı
                    0, 0, 0
                )
                
                # 3. İRTİFA KOMUTU (GUIDED_CHANGE_ALTITUDE)
                self.master.mav.command_int_send(
                    self.master.target_system, self.master.target_component,
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, # Frame 3
                    mavutil.mavlink.MAV_CMD_GUIDED_CHANGE_ALTITUDE,
                    0, 0,
                    0, 0, 0, 0,
                    0, 0, target_alt_m # z ekseni değeri
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

        # --- YENİ PID VE KONTROL AYARLARI (HEADING & ALTITUDE) ---
        # "P oldukça hantal, I ve D 0" talebine uygun
        # Sistemin hedefe dönebilmesi için Kp değerlerini ciddi oranda artırdık
        self.Kp_heading = 2.5   # 1 derece hata = 4.5 derece hedefe sapma
        self.Ki_heading = 0.0
        self.Kd_heading = 0.5

        self.Kp_alt = 1      # 1 derece hata = 1.5 metre irtifa değişimi
        self.Ki_alt = 0.0
        self.Kd_alt = 0.3
        
        self.filtered_target_speed = 20.0





        # --- DİKEY HIZ (CLIMB RATE) PID AYARLARI ---
        # Osilasyon analizi (172133): ~6s periyotlu yavaş salınım integral windup'tan
        # kaynaklanıyordu (P/D'den değil). Ki düşürüldü + integral sınırı 4'e daraltıldı.
        # Kp orta seviyede: hedefi merkeze çeker ama aşırı sürmez. Kd yumuşak sönümleme.
        self.Kp_vz = 0.5     # Merkeze toparlama; çok yüksek olursa merkezi aşar
        self.Ki_vz = 0.02    # Küçük: yalnızca kalıcı ofseti sızdırır, salınım yapmaz
        self.Kd_vz = 0.15    # Sönümleme; err_y hızlı değişince frenler




        # --- HEADING RATE PID (dönüş hızı kontrolü) ---
        self.Kp_rate = 0.6      # 1° açısal hata → 5 deg/s dönüş hızı
        self.Ki_rate = 0.0
        self.Kd_rate = 0.08     # Hızlı hareket eden hedefe tepki için

        self.integral_error_rate = 0.0
        self.prev_error_rate = 0.0
        self.prev_derivative_rate = 0.0

        self.min_heading_rate = 0.85    # Minimum dönüş hızı (deg/s)
        self.max_heading_rate = 5.0   # Maksimum dönüş hızı (deg/s)

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
        self.alt_target = None   # Entegre irtifa hedefi (ilk takip karesinde current_alt ile başlar)
        
        self.frame_counter = 0
        self.start_time = time.time()

        # --- KAMERA AYARLARI (Lütfen Gerçek Sisteme Göre Güncelleyin!) ---
        self.camera_width = 1920  
        self.camera_height = 1080 
        self.camera_hfov_rad = 0.42
        
        # GAZEBO KAMERASI
        self.center_x = 960    
        self.center_y =  540   

        # GERÇEK KAMERA
        # self.center_x = 1091
        # self.center_y =  633

        
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
        """Detaylı telemetriyi tek satır key=value formatında dosyaya (DEBUG) yazar.
        Terminale düşmez; uçuş sonrası grep/pandas ile kolayca ayrıştırılır.
        Örn: grep 'TLM state=TRACKING' guidance_logs/*.log
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
        logging.info("Sistem aktif! Ana döngü (Main Thread) başlatıldı...")
        try:
            while True:
                try:
                    data = self.data_queue.get(timeout=0.05)
                except queue.Empty:
                    data = None

                current_time = time.time()
                gorev = self.redis.get_gorev()
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
                            if gorev == "Goruntulu" and self.mavlink.current_mode == "GUIDED":
                                if current_time - self.last_cmd_send_time >= 0.2: # 5Hz yenileme
                                    # HEDEF KAYBOLDU: İrtifayı mevcut değerde sabitle, son bilinen hızı ve yönü koruyarak düz uç
                                    self.alt_target = None  # Yeniden yakalamada taze başla
                                    last_spd = getattr(self, 'last_target_speed', 20.0)
                                    self.mavlink.send_guided_targets(self.last_target_heading, current_alt, last_spd)
                                    cmd_sent = 1
                                    self.last_cmd_send_time = current_time
    
                            
                            self._log_telemetry(
                                "COASTING", gorev=gorev, cmd_sent=cmd_sent,
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
                            if gorev == "Goruntulu" and self.mavlink.current_mode == "GUIDED":
                                if current_time - self.last_cmd_send_time >= 0.5:
                                    # TAM FAILSAFE: Burnun gösterdiği yöne doğru standart hızda (20.0) ve mevcut irtifada uç
                                    self.mavlink.send_guided_targets(yaw_deg, current_alt, 20.0)
                                    cmd_sent = 1
                                    self.last_target_heading = yaw_deg
                                    self.last_target_speed = 20.0
                                    self.last_cmd_send_time = current_time
                                    
                            if cmd_sent:
                                logging.warning(f"FAILSAFE: Hedef {td:.1f}s'dir kayıp. Düz uçuş: Yön={yaw_deg:.1f}° İrtifa={current_alt:.1f}m Hız=20.0m/s")
                            self._log_telemetry(
                                "FAILSAFE", gorev=gorev, cmd_sent=cmd_sent,
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

                # Türev hesabı (Low pass)
                alpha = 0.05 
                raw_deriv_x = (stab_error_x_deg - self.prev_error_x) / dt
                raw_deriv_y = (stab_error_y_deg - self.prev_error_y) / dt
                deriv_x = (alpha * raw_deriv_x) + ((1.0 - alpha) * self.prev_derivative_x)
                deriv_y = (alpha * raw_deriv_y) + ((1.0 - alpha) * self.prev_derivative_y)

                # İntegral 
                if gorev == "Goruntulu" and self.mavlink.current_mode == "GUIDED":
                    if abs(stab_error_x_deg) < self.deadzone_deg: self.integral_error_x *= 0.99
                    else: self.integral_error_x += stab_error_x_deg * dt
                    self.integral_error_x = self.clamp(self.integral_error_x, -10.0, 10.0)

                    if abs(stab_error_y_deg) < self.deadzone_deg: self.integral_error_y *= 0.99
                    else: self.integral_error_y += stab_error_y_deg * dt
                    # Y integral sınırı daraltıldı (10→4): geniş sınır, hedef üstteyken
                    # +9'a şişip merkezi geçtikten sonra aşırı düzeltme yapıp ~6s'lik
                    # yavaş osilasyona yol açıyordu. Dar sınır integralin baskınlığını kırar.
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

                # --- 1. HEADING KONTROLÜ ---
                p_term_heading = p_x * self.Kp_heading
                i_term_heading = self.integral_error_x * self.Ki_heading
                d_term_heading = deriv_x * self.Kd_heading
                
                cmd_head_deg = p_term_heading + i_term_heading + d_term_heading
                cmd_head_deg = self.clamp(cmd_head_deg, -self.max_heading_change_deg, self.max_heading_change_deg)
                # Hedef açı (0-360 arası normalize)
                target_heading = (yaw_deg + cmd_head_deg) % 360.0

                # --- HEADING RATE PID (dönüş hızı kontrolü) ---
                rate_error = abs(p_x)  # Açısal hatanın büyüklüğü (deadzone uygulanmış)

                # Türev (low-pass filtered)
                raw_deriv_rate = (rate_error - self.prev_error_rate) / dt
                deriv_rate = (alpha * raw_deriv_rate) + ((1.0 - alpha) * self.prev_derivative_rate)

                # İntegral
                if gorev == "Goruntulu" and self.mavlink.current_mode == "GUIDED":
                    self.integral_error_rate += rate_error * dt
                    self.integral_error_rate = self.clamp(self.integral_error_rate, 0, 5.0)
                else:
                    self.integral_error_rate = 0.0

                # PID çıkışı
                p_term_rate = rate_error * self.Kp_rate
                i_term_rate = self.integral_error_rate * self.Ki_rate
                d_term_rate = deriv_rate * self.Kd_rate

                heading_rate = p_term_rate + i_term_rate + d_term_rate
                heading_rate = self.clamp(heading_rate, self.min_heading_rate, self.max_heading_rate)

                # Deadzone içindeyse minimum rate
                if dz_x:
                    heading_rate = self.min_heading_rate

                self.prev_error_rate = rate_error
                self.prev_derivative_rate = deriv_rate

                # --- 2. DİKEY HIZ (CLIMB RATE - VZ) KONTROLÜ ---
                # Hedef kamerada aşağıdaysa p_y pozitiftir, çökmesi (-Vz) gerekir. (-1.0 çarpanı ile tersliği düzelttik)
                p_term_vz = p_y * self.Kp_vz
                i_term_vz = self.integral_error_y * self.Ki_vz
                d_term_vz = deriv_y * self.Kd_vz
                
                # Z ekseni hedef hızı (m/s)
                climb_rate_raw = -1.0 * (p_term_vz + i_term_vz + d_term_vz)
                
                # Uçağın yapısal sınırlarına göre tırmanma/çökme limitlerini uygula
                # (DİKKAT: Bu değerler ArduPilot'taki TECS_CLMB_MAX ve TECS_SINK_MAX ile AYNI OLMALIDIR!)
                climb_rate_cmd = self.clamp(climb_rate_raw, -5.0, 2.0)

                # Zemin güvenliği (Mevcut irtifa 10 metrenin altındaysa çökmeyi engelle)
                if current_alt < 10.0 and climb_rate_cmd < 0:
                    climb_rate_cmd = 0.0

                current_airspeed = self.mavlink.current_airspeed
                
                # ----------------------------------------------------------
                # FİZİKSEL UÇUŞ KORUMALARI
                # ----------------------------------------------------------
                V_cruise = 20.0
                V_max = 24.0     
                V_min = 16.0  
                
                # 1. Dönüş Yükü (Load Factor) Düzeltmesi
                # Keskin dönüşlerde kanat taşımasını kaybetmemek için tırmanma talebini azaltır.
                # cos(roll)^2 ile yatış arttıkça Vz talebini sönümler.
                load_factor_scale = math.cos(roll_now) ** 2
                climb_rate_cmd *= load_factor_scale

                # 2. Stall Koruması — KALDIRILDI
                # Bu koruma airspeed < 17.5 m/s iken pozitif Vz'yi (tırmanışı) sıfırla
                # çarpıyordu. SITL'de airspeed sürekli ~15 m/s olduğu için hedef kameranın
                # en üstünde olsa bile üretilen pozitif Vz anında sıfırlanıyor, uçak hiç
                # tırmanmıyordu. Stall güvenliği ArduPilot tarafında AIRSPEED_MIN/
                # ARSPD_FBW_MIN ile zaten sağlanıyor; burada tekrar kısıtlamak gereksiz.
                stall_factor = 1.0   # nötr (loglama uyumluluğu için tutuluyor)

                # 3. Özgül Enerji Kontrolü (Specific Energy Control)
                g = 9.81
                V_safe = max(current_airspeed, 12.0) # Sıfıra bölme hatasını engellemek için alt sınır
                K_energy = 0.8 # Enerji kazanç çarpanı (0.5 - 1.0 arası idealdir, testlerde ayarlayabilirsin)
                
                # Özgül Enerji formülü: dV = (g * Vz / V) * K_energy
                # Tırmanışta hız talebini artırır (motor açar), dalışta hızı düşürür (motor kısar).
                dV_energy = (g * climb_rate_cmd / V_safe) * K_energy
                
                # Pozitif geri besleme (runaway) hatasını önlemek için mevcut hızı değil, sabit V_cruise'u baz alıyoruz!
                target_speed_raw = V_cruise + dV_energy

                if not hasattr(self, 'filtered_target_speed'):
                    self.filtered_target_speed = V_cruise

                # Filtreyi gevşetiyoruz (0.05 -> 0.20) ki motorlar ihtiyaç anında enerjiyi anında verebilsin
                alpha_speed = 0.20  
                self.filtered_target_speed = (alpha_speed * target_speed_raw) + ((1.0 - alpha_speed) * self.filtered_target_speed)
                
                # Hızı mekanik limitlere hapset
                target_speed = self.clamp(self.filtered_target_speed, V_min, V_max)

                # ----------------------------------------------------------
                # KOMUTLARI GÖNDERME BÖLÜMÜ
                # ----------------------------------------------------------
                # Anti-windup
                aw_x, aw_y = 0, 0
                if abs(cmd_head_deg) >= self.max_heading_change_deg:
                    self.integral_error_x *= 0.9; aw_x = 1
                # Vz komutu limitlere dayandıysa VEYA uçak komutu fiilen takip edemiyorsa
                # Y integralini söndür (aksi halde ulaşılamayan sink'te integral +10'a şişip
                # P terimini bastırıyor, hedef üste yakalanınca geç tepki veriyordu).
                cmd_saturated = climb_rate_raw >= 6.0 or climb_rate_raw <= -5.0
                vz_untracked = abs(climb_rate_cmd - self.mavlink.current_climb) > 3.0
                if cmd_saturated or vz_untracked:
                    self.integral_error_y *= 0.9; aw_y = 1

                command_sent = 0
                if current_time - self.last_cmd_send_time >= 0.1:
                    if gorev == "Goruntulu" and self.mavlink.current_mode == "GUIDED":
                        
                        # ── LOOK-AHEAD İRTİFA HEDEFİ ──
                        # T_LOOK: Vz'yi irtifa ofsetine çeviren ufuk. 3.0s çok uzundu; climb_cmd=5
                        # olduğunda uçağa 15 m ötesini komutluyor, TECS bunu ulaşılamaz bir
                        # sıçrama gibi görüp yavaş/gecikmeli yaklaşıyordu (~0.8s lag ölçüldü).
                        # 1.5s ile hedef uçağın gerçekten kovalayabileceği mesafede kalıyor,
                        # komut daha çok "şimdiki Vz" gibi iletiliyor.
                        T_LOOK = 2
                        moving_target_alt = current_alt + (climb_rate_cmd * T_LOOK)
                        # Zemin güvenliği
                        moving_target_alt = max(moving_target_alt, 10.0)
                        
                        # Takip açığı teşhisi: komut ile gerçekleşen Vz sürekli ayrışıyorsa
                        # sorun dış döngüde değil TECS/uçak performansındadır (bkz. TECS_OPTIONS).
                        vz_gap = climb_rate_cmd - self.mavlink.current_climb
                        if abs(vz_gap) > 3.0 and abs(climb_rate_cmd) > 2.0:
                            logging.warning(f"Vz TAKİP AÇIĞI: komut={climb_rate_cmd:.1f} gerçek={self.mavlink.current_climb:.1f} "
                                            f"(fark={vz_gap:.1f}) airspeed={self.mavlink.current_airspeed:.1f} → TECS sink/climb sınırlı olabilir")
                        
                        # Hazırladığımız 3'lü komut paketini COMMAND_INT ile otopilota ilet
                        self.mavlink.send_guided_targets(target_heading, moving_target_alt, target_speed)
                        command_sent = 1
                        self.alt_target = moving_target_alt  # loglama için

                        logging.info(f"[{gorev}] OTONOM: Yön={target_heading:.1f}° | err_y={stab_error_y_deg:+.2f}° (ham={raw_error_y_deg:+.2f}°) | Vz={climb_rate_cmd:+.2f} | Hız={target_speed:.1f} | Hedef İrtifa={moving_target_alt:.1f}m | Gerçek Vz={self.mavlink.current_climb:+.2f}")
                    else :
                        logging.info(f"[{gorev}] BEKLEME: Yön={target_heading:.1f}° | err_y={stab_error_y_deg:+.2f}° (ham={raw_error_y_deg:+.2f}°) | Vz={climb_rate_cmd:+.2f} | Hız={target_speed:.1f} | İrtifa={current_alt:.1f}m")
                    self.last_cmd_send_time = current_time

                self.last_target_heading = target_heading
                self.last_target_speed = target_speed

                self._log_telemetry(
                    "TRACKING", gorev=gorev, cmd_sent=command_sent,
                    dt=dt, data_age_ms=data_age_ms, queue=queue_size,
                    # Görüntü / hata
                    obj_x=obj_x, obj_y=obj_y, bbox_w=bbox_w, bbox_h=bbox_h,
                    err_x_deg=stab_error_x_deg, err_y_deg=stab_error_y_deg,
                    deriv_x=deriv_x, deriv_y=deriv_y, dz_x=dz_x, dz_y=dz_y,
                    # Uçak durumu
                    alt=current_alt,
                    roll_deg=math.degrees(roll_now), pitch_deg=math.degrees(pitch_now),
                    yaw_deg=yaw_deg,
                    airspeed=self.mavlink.current_airspeed,
                    climb_act=self.mavlink.current_climb,
                    # Heading kanalı
                    cmd_head_deg=cmd_head_deg, target_heading=target_heading,
                    heading_rate=heading_rate,
                    # Vz kanalı ve enerji zinciri (climb_cmd = tüm korumalar sonrası nihai değer)
                    climb_raw=climb_rate_raw, load_factor_scale=load_factor_scale,
                    stall_factor=stall_factor, climb_cmd=climb_rate_cmd,
                    integral_y=self.integral_error_y, aw_y=aw_y,
                    p_term_vz=p_term_vz, i_term_vz=i_term_vz, d_term_vz=d_term_vz,
                    alt_target=self.alt_target if self.alt_target is not None else current_alt,
                    dV_energy=dV_energy, target_speed=target_speed
                )

        except KeyboardInterrupt:
            logging.info("Kapatılıyor...")
            self.mavlink.running = False
            self.redis.running = False
            logging.shutdown()

if __name__ == '__main__':
    gudum = AutopilotController(connection_str='udp:127.0.0.1:14554')
    gudum.run()