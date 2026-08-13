#!/usr/bin/env python3

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

'''
MAV_CMD_GUIDED_CHANGE_ALTITUDE ve MAV_CMD_GUIDED_CHANGE_HEADING tabanlı yeni mimari.
İrtifa ve Heading için ayrı PID mantığı kullanılmıştır.
P değerleri oldukça yavaş (hantal), I ve D sıfırdır.
''' 

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

class FlightLogger:
    def __init__(self, file_prefix="flight_log_guided"):
        self.log_filename = f"{file_prefix}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
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
        print(f"Log dosyası oluşturuldu: {self.log_filename}")
        
    def log(self, row_data):
        self.csv_writer.writerow(row_data)
        self.row_count += 1
        if self.row_count % self.flush_interval == 0:
            self.log_file.flush()

    def close(self):
        self.log_file.flush()
        self.log_file.close()

class RedisListener(threading.Thread):
    def __init__(self, data_queue):
        super().__init__()
        self.data_queue = data_queue
        self.daemon = True
        self.running = True
        self.gorev = "Bilinmiyor"
        
        print("Redis sunucusuna bağlanılıyor...")
        try:
            self.r = redis.Redis(host='localhost', port=6379, db=0)
            self.pubsub = self.r.pubsub()
            self.pubsub.subscribe('tracker_bbox')
            print("Redis 'tracker_bbox' kanalına abone olundu.")
        except Exception as e:
            print(f"Redis Bağlantı Hatası: {e}")
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

class MavlinkManager(threading.Thread):
    def __init__(self, connection_str='udp:127.0.0.1:14552'):
        super().__init__()
        self.daemon = True
        self.running = True
        self.lock = threading.Lock()
        
        self.current_mode = "UNKNOWN"
        self.current_yaw_rad = 0.0
        self.current_roll_rad = 0.0
        self.current_pitch_rad = 0.0
        self.current_alt_rel = 100.0  # Varsayılan başlangıç irtifası
        
        print(f"MAVLink bağlantısı: {connection_str}...")
        try:
            self.master = mavutil.mavlink_connection(connection_str)
            self.master.wait_heartbeat()
            print(f"Bağlantı Başarılı. Sistem ID: {self.master.target_system}")
            # Ekstra telemetri akışlarını talep et (Altitude için GLOBAL_POSITION_INT vb.)
            self.master.mav.request_data_stream_send(
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_POSITION, 10, 1)
            self.master.mav.request_data_stream_send(
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 10, 1)
        except Exception as e:
            print(f"MAVLink Hatası: {e}")
            self.running = False

    def run(self):
        while self.running:
            try:
                with self.lock:
                    msg = self.master.recv_match(type=['ATTITUDE', 'HEARTBEAT', 'GLOBAL_POSITION_INT'], blocking=False)
                
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
                time.sleep(0.01)
            except:
                time.sleep(0.1)

    def send_heading_target(self, heading_deg):
        with self.lock:
            try:
                self.master.mav.command_long_send(
                    self.master.target_system,
                    self.master.target_component,
                    mavutil.mavlink.MAV_CMD_GUIDED_CHANGE_HEADING,
                    0, 0, heading_deg, 3, 0, 0, 0, 0
                )
            except:
                pass

    def send_altitude_target(self, altitude_m):
        with self.lock:
            try:
                # 3 = MAV_FRAME_GLOBAL_RELATIVE_ALT
                self.master.mav.command_long_send(
                    self.master.target_system,
                    self.master.target_component,
                    mavutil.mavlink.MAV_CMD_GUIDED_CHANGE_ALTITUDE,
                    0, 0, 2, 0, 0, 0, 0, altitude_m
                )
            except:
                pass

class AutopilotController:
    def __init__(self, connection_str='udp:127.0.0.1:14552'):
        self.data_queue = queue.Queue()
        
        self.logger = FlightLogger()
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
        self.Kp_heading = 3    # 1 derece hata = 4.5 derece hedefe sapma
        self.Ki_heading = 0.0
        self.Kd_heading = 0.0

        self.Kp_alt = 2       # 1 derece hata = 1.5 metre irtifa değişimi
        self.Ki_alt = 0.0
        self.Kd_alt = 0.0

        self.max_heading_change_deg = 35.0     
        self.max_alt_change_m = 10.0    
        self.deadzone_deg = 0.78  

        self.prev_error_x, self.prev_error_y = 0.0, 0.0
        self.integral_error_x, self.integral_error_y = 0.0, 0.0
        self.prev_derivative_x, self.prev_derivative_y = 0.0, 0.0
        self.last_time = time.time()
        self.last_cmd_send_time = 0.0

        self.last_target_time = 0.0
        self.last_target_heading = 0.0
        self.last_target_alt = 0.0
        
        self.frame_counter = 0
        self.start_time = time.time()

        # --- KAMERA AYARLARI (Lütfen Gerçek Sisteme Göre Güncelleyin!) ---
        self.camera_width = 1280  
        self.camera_height = 720 
        self.camera_hfov_rad = 0.27 

        self.center_x = self.camera_width / 2.0
        self.center_y = self.camera_height / 2.0
        self.fx = self.center_x / math.tan(self.camera_hfov_rad / 2.0)  
        self.fy = self.fx  

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
        coverage_w = (bbox_w / self.camera_width * 100.0) if bbox_w > 0 else 0.0
        coverage_h = (bbox_h / self.camera_height * 100.0) if bbox_h > 0 else 0.0

        dr = lambda v: f"{math.degrees(v):.2f}"
        
        row = [
            f"{current_time:.4f}", f"{elapsed:.3f}", f"{dt:.4f}", self.frame_counter,
            self.mavlink.current_mode, gorev, system_state, target_found,
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
        self.logger.log(row)

    def run(self):
        print("Sistem aktif! Ana döngü (Main Thread) başlatıldı...")
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
                                    self.mavlink.send_heading_target(self.last_target_heading)
                                    self.mavlink.send_altitude_target(self.last_target_alt)
                                    cmd_sent = 1
                                    self.last_cmd_send_time = current_time
                            
                            self._log_state(
                                current_time, td, gorev, "COASTING", 0, cmd_sent, queue_size, td*1000,
                                0,0,0,0,0,0,0, 0,0,0,0, current_alt, roll_now, pitch_now, yaw_now,
                                0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0,
                                self.integral_error_x, 0,0,0,0, self.last_target_heading,
                                self.integral_error_y, 0,0,0,0, self.last_target_alt,
                                0,0
                            )
                        else:
                            self.integral_error_x, self.integral_error_y = 0.0, 0.0
                            self.prev_derivative_x, self.prev_derivative_y = 0.0, 0.0
                            
                            # Failsafe: Continue at current heading and altitude
                            cmd_sent = 0
                            if gorev == "Goruntulu" and self.mavlink.current_mode == "GUIDED":
                                if current_time - self.last_cmd_send_time >= 0.5:
                                    self.mavlink.send_heading_target(yaw_deg)
                                    self.mavlink.send_altitude_target(current_alt)
                                    cmd_sent = 1
                                    self.last_target_heading = yaw_deg
                                    self.last_target_alt = current_alt
                                    self.last_cmd_send_time = current_time
                                    
                            self._log_state(
                                current_time, td, gorev, "FAILSAFE", 0, cmd_sent, queue_size, td*1000,
                                0,0,0,0,0,0,0, 0,0,0,0, current_alt, roll_now, pitch_now, yaw_now,
                                0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0,
                                0,0,0,0,0,0, 0,0,0,0,0,0, 0,0
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
                    self.integral_error_y = self.clamp(self.integral_error_y, -10.0, 10.0)
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

                # --- 2. ALTITUDE KONTROLÜ ---
                # Hedef Y ekseninde aşağıdaysa (p_y pozitif), irtifa DÜŞÜRÜLMELİ
                p_term_alt = p_y * self.Kp_alt
                i_term_alt = self.integral_error_y * self.Ki_alt
                d_term_alt = deriv_y * self.Kd_alt
                
                cmd_alt_m = -1 * (p_term_alt + i_term_alt + d_term_alt)
                cmd_alt_m = self.clamp(cmd_alt_m, -self.max_alt_change_m, self.max_alt_change_m)
                
                target_alt = current_alt + cmd_alt_m
                # Zemin güvenliği (10 metre altına inmesini engelle)
                if target_alt < 10.0:
                    target_alt = 10.0

                # Anti-windup
                aw_x, aw_y = 0, 0
                if abs(cmd_head_deg) >= self.max_heading_change_deg:
                    self.integral_error_x *= 0.9; aw_x = 1
                if abs(cmd_alt_m) >= self.max_alt_change_m:
                    self.integral_error_y *= 0.9; aw_y = 1

                command_sent = 0
                # GUIDED modunda ArduPilot komut yoğunluğunu ezmemek için 5-10Hz (0.1sn) limiti
                if current_time - self.last_cmd_send_time >= 0.1:
                    if gorev == "Goruntulu" and self.mavlink.current_mode == "GUIDED":
                        self.mavlink.send_heading_target(target_heading)
                        self.mavlink.send_altitude_target(target_alt)
                        command_sent = 1
                        print(f"[{gorev}] OTONOM: HedefYön: {target_heading:.1f}° | İrtifa: {target_alt:.1f}m")
                    else:
                        print(f"[{gorev}] BEKLEME - Hdf Yön: {target_heading:.1f}° | İrtifa: {target_alt:.1f}m")
                    
                    self.last_cmd_send_time = current_time

                self.last_target_heading = target_heading
                self.last_target_alt = target_alt

                self._log_state(
                    current_time, dt, gorev, "TRACKING", 1, command_sent, queue_size, data_age_ms,
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
                    aw_x, aw_y
                )

        except KeyboardInterrupt:
            print("\nKapatılıyor...")
            self.logger.close()
            self.mavlink.running = False
            self.redis.running = False

if __name__ == '__main__':
    gudum = AutopilotController(connection_str='udp:127.0.0.1:14552')
    gudum.run()
