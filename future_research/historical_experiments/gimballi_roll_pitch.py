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
Gaz kontrolu yok
irtifa sınırı yok
kalman yok
yaw kontrolu yok
Sanal gimbal stabilizasyonu aktif
''' 

# ─── SANAL GİMBAL SABİTLERİ ───
# Kamera->Body rotasyonu (kamera z ileri, x sağ, y aşağı)
R_c_b = np.array([[0, 0, 1],
                  [1, 0, 0],
                  [0, 1, 0]], dtype=float)
R_c_b_T = R_c_b.T

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

class FlightLogger:
    def __init__(self, file_prefix="pd_tracker_log"):
        self.log_filename = f"{file_prefix}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        self.log_file = open(self.log_filename, mode='w', newline='')
        self.csv_writer = csv.writer(self.log_file)
        
        headers = [
            "timestamp", "dt", "flight_mode", "target_found",
            # Sanal Gimbal verileri
            "raw_x", "raw_y", "stab_x", "stab_y",
            "delta_stab_x", "delta_stab_y",
            "roll_rad", "pitch_rad", "roll_deg", "pitch_deg",
            # Sapma (Error) Verileri
            "raw_error_x_deg", "raw_error_y_deg", 
            "stab_error_x_deg", "stab_error_y_deg",
            "heading_diff_deg",
            # PID verileri
            "p_term_roll", "i_term_roll", "d_term_roll", "cmd_roll_rad", "final_roll_deg",
            "p_term_pitch", "i_term_pitch", "d_term_pitch", "cmd_pitch_rad", "final_pitch_deg",
            # Hedef Verileri
            "bbox_w", "bbox_h", "target_area_px"
        ]
        self.csv_writer.writerow(headers)
        print(f"Log dosyası oluşturuldu: {self.log_filename}")
        
    def log(self, row_data):
        self.csv_writer.writerow(row_data)

    def close(self):
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
                # Görev verisi güncelleme
                gorev_bytes = self.r.get('gorev')
                self.gorev = gorev_bytes.decode('utf-8') if gorev_bytes else "Bilinmiyor"
                
                # Sadece blocking dinleme, yeni frame geldiği an kuyruğa yaz
                # Timeout süresi çok kısa tutularak thread'in esnekliği sağlanır
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
                            
                            # Çözümlenmiş veriyi ana döngüye ilet
                            self.data_queue.put({
                                'type': 'target',
                                'obj_x': obj_x,
                                'obj_y': obj_y,
                                'bbox_w': float(w),
                                'bbox_h': float(h),
                                'target_area': target_area,
                                'timestamp': time.time(),
                                'gorev': self.gorev
                            })
                    except Exception as e:
                        pass
            except Exception as e:
                time.sleep(0.1)
                
    def get_gorev(self):
        return self.gorev

class MavlinkManager(threading.Thread):
    def __init__(self, connection_str='udp:127.0.0.1:14552'):
        super().__init__()
        self.daemon = True
        self.running = True
        
        # MAVLink Thread safety için kilit (Lock)
        self.lock = threading.Lock()
        
        # Durum verileri
        self.current_mode = "UNKNOWN"
        self.current_yaw_rad = 0.0
        self.current_roll_rad = 0.0
        self.current_pitch_rad = 0.0
        
        print(f"MAVLink ile uçağa bağlanılıyor: {connection_str}...")
        try:
            self.master = mavutil.mavlink_connection(connection_str)
            self.master.wait_heartbeat()
            print(f"MAVLink Bağlantısı Başarılı! Sistem ID: {self.master.target_system}")
        except Exception as e:
            print(f"MAVLink Bağlantı Hatası: {e}")
            self.running = False

    def run(self):
        while self.running:
            try:
                # Sadece okuma işlemini yapıp state'i güncelle - Lock Korumalı
                with self.lock:
                    msg = self.master.recv_match(type=['ATTITUDE', 'HEARTBEAT'], blocking=False)
                
                if msg:
                    if msg.get_type() == 'ATTITUDE':
                        self.current_yaw_rad = msg.yaw
                        self.current_roll_rad = msg.roll
                        self.current_pitch_rad = msg.pitch
                    elif msg.get_type() == 'HEARTBEAT':
                        if self.master.flightmode:
                            self.current_mode = self.master.flightmode
                            
                # CPU tüketimini izole etmek için uyku payı
                time.sleep(0.01)
            except Exception as e:
                time.sleep(0.1)

    def set_mode(self, mode_name):
        with self.lock:
            if mode_name not in self.master.mode_mapping():
                return
            mode_id = self.master.mode_mapping()[mode_name]
            self.master.mav.set_mode_send(
                self.master.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id
            )

    def send_attitude_target(self, roll_rad, pitch_rad, yaw_rad):
        # Quaternion hesabı
        cr, sr = math.cos(roll_rad * 0.5), math.sin(roll_rad * 0.5)
        cp, sp = math.cos(pitch_rad * 0.5), math.sin(pitch_rad * 0.5)
        cy, sy = math.cos(yaw_rad * 0.5), math.sin(yaw_rad * 0.5)
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        q = [w, x, y, z]
        
        type_mask = 7
        
        # Gönderme işlemini korumalı (lock) yap
        with self.lock:
            try:
                self.master.mav.set_attitude_target_send(
                    0, self.master.target_system, self.master.target_component,
                    type_mask, q, 0, 0, 0, 0.80
                )
            except Exception as e:
                pass


class AutopilotController:
    def __init__(self, connection_str='udp:127.0.0.1:14552'):
        self.data_queue = queue.Queue()
        
        # Altsistem Thread'lerini başlat
        self.logger = FlightLogger()
        self.mavlink = MavlinkManager(connection_str)
        self.redis = RedisListener(self.data_queue)
        
        if self.mavlink.running and self.redis.running:
            self.mavlink.start()
            self.redis.start()
        else:
            print("Sistem bileşenleri başlatılamadı. Kapatılıyor...")
            exit(1)

        # --- PID VE KONTROL AYARLARI ---
        self.multiply_factor = 1.15

        self.Kp_roll = 0.02875 * self.multiply_factor
        self.Ki_roll = 0.00115 * self.multiply_factor
        self.Kd_roll = 0.0090 * self.multiply_factor

        self.Kp_pitch = 0.016 * self.multiply_factor
        self.Ki_pitch = 0.0021 * self.multiply_factor
        self.Kd_pitch = 0.0060 * self.multiply_factor

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
        
        # --- KAMERA VE OPTİK AYARLARI ---
        # Gazebo SDF'den: horizontal_fov = 0.51 rad, image: 1280x720
        self.camera_width = 1280  
        self.camera_height = 720 
        self.camera_hfov_rad = 0.27 # SDF'den direkt (radyan) #.51(eski)0.36 

        self.center_x = self.camera_width / 2.0
        self.center_y = self.camera_height / 2.0

        # fx, fy hesabı (SDF hfov'dan)
        self.fx = self.center_x / math.tan(self.camera_hfov_rad / 2.0)  # ≈ 2459.6
        self.fy = self.fx  # Gazebo kare piksel kullanır → fx = fy

        # --- SANAL GİMBAL: Intrinsic Matris ---
        # FOV'dan hesaplanan fx/fy kullanılıyor (kamera kalibrasyonuyla uyumlu)
        self.K = np.array([
            [self.fx, 0,       self.center_x],
            [0,       self.fy, self.center_y],
            [0,       0,       1            ]
        ])
        self.K_inv = np.linalg.inv(self.K)
        print(f"Sanal Gimbal aktif. fx={self.fx:.1f}, fy={self.fy:.1f}")

    def clamp(self, value, min_val, max_val):
        return max(min(value, max_val), min_val)

    def stabilize_pixel(self, obj_x, obj_y):
        """Sanal gimbal: UAV roll/pitch kompanzasyonu ile stabilize piksel döner.
        
        Pipeline: Piksel → K⁻¹ (back-project) → Kamera ışını → Body frame
                  → R_stab ile stabilize → Tekrar kamera frame → K (re-project)
        
        Yaw=0 ile R_stab hesaplanır: sadece roll ve pitch titreşimleri kompanze edilir.
        """
        # 1. Piksel → Kamera ışını (back-projection)
        p_raw = np.array([obj_x, obj_y, 1.0])
        r_cam = self.K_inv @ p_raw
        
        # 2. Kamera frame → Body frame
        r_body = R_c_b @ r_cam
        
        # 3. Stabilizasyon: Body → Virtual Body (roll/pitch kompanzasyonu)
        roll_rad = self.mavlink.current_roll_rad
        pitch_rad = self.mavlink.current_pitch_rad
        R_stab = compute_R_b_e(roll_rad, pitch_rad, 0)  # yaw=0: sadece roll/pitch
        r_virt_body = R_stab @ r_body
        
        # 4. Virtual Body → Kamera frame
        r_virt_cam = R_c_b_T @ r_virt_body
        
        # 5. Re-projection: Kamera ışını → Stabilize piksel
        p_virt_hom = self.K @ r_virt_cam
        
        if p_virt_hom[2] != 0:
            return p_virt_hom[0] / p_virt_hom[2], p_virt_hom[1] / p_virt_hom[2]
        return obj_x, obj_y  # fallback: stabilizasyon başarısızsa ham piksel

    def run(self):
        print("Sistem aktif! Ana döngü (Main Thread) başlatıldı...")
        try:
            while True:
                # Redis'ten (Queue) veri gelmesini en fazla 50ms bekle. Orijinal "get_message(timeout=0.05)" mantığıyla aynı.
                try:
                    data = self.data_queue.get(timeout=0.05)
                except queue.Empty:
                    data = None

                current_time = time.time()
                gorev = self.redis.get_gorev()

                if not data:
                    # MESAJ YOK: Ya hedef gerçekten kayıp ya da FPS düşük. (Coasting Koruması)
                    if self.last_target_time > 0.0:
                        time_since_last = current_time - self.last_target_time
                        if time_since_last <= 5.0:
                            # 5 Saniye boyunca Coasting Modu
                            if gorev == "Goruntulu" and self.mavlink.current_mode == "GUIDED":
                                self.mavlink.send_attitude_target(
                                    self.last_final_roll_rad, 
                                    self.last_final_pitch_rad, 
                                    self.mavlink.current_yaw_rad
                                )
                        else:
                            # 5 Saniye aşıldı! Failsafe: Düz uçuş (Devamlı yollanmalı ki Otopilot Loiter'a geçmesin)
                            self.integral_error_x = 0.0
                            self.integral_error_y = 0.0
                            self.prev_derivative_x = 0.0
                            self.prev_derivative_y = 0.0
                            if gorev == "Goruntulu" and self.mavlink.current_mode == "GUIDED":
                                self.mavlink.send_attitude_target(0.0, 0.0, self.mavlink.current_yaw_rad)
                                self.last_final_roll_rad = 0.0
                                self.last_final_pitch_rad = 0.0
                                
                                # Saniyede bir kez terminale uyarı bas (Terminali kilitlenmeden uyarmak için)
                                if not hasattr(self, 'last_warning_time') or current_time - self.last_warning_time > 1.0:
                                    print("\n[UYARI] HEDEF 5 SANİYEDEN UZUN SÜREDİR KAYIP! GÖREV MODUNU DEĞİŞTİRİN!!!")
                                    self.last_warning_time = current_time
                                    
                                # self.last_target_time = 0.0 silindi. Böylece sürekli bu bloğa girip uçağ 0,0 göndermeye devam edecek.
                    
                    # PID'i bozmamak için döngüyü atla
                    continue

                # SADECE YENİ FRAME GELDİĞİNDE:
                dt = current_time - self.last_time
                if dt < 0.001: dt = 0.001
                if dt > 0.1: dt = 0.1
                self.last_time = current_time
                
                # Kuyruktan gelen tespit verilerini çıkart
                obj_x = data['obj_x']
                obj_y = data['obj_y']
                bbox_w = data.get('bbox_w', 0.0)
                bbox_h = data.get('bbox_h', 0.0)
                target_area = data['target_area']
                self.last_target_time = current_time
                
                # --- SANAL GİMBAL STABİLİZASYONU ---
                # Roll/pitch sabit kanatta viraj/tırmanma başlatır, kamerayı doğrudan
                # hedefe çevirmez. Manevra sırasındaki piksel kayması geçici gürültüdür.
                # Gimbal bu gürültüyü temizler, PID'in erken tepki vermesini engeller.
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

                # PID stabil hata ile çalışıyor
                error_x_deg = stab_error_x_deg  
                error_y_deg = stab_error_y_deg
                heading_diff_deg = error_x_deg 

                raw_derivative_x = (error_x_deg - self.prev_error_x) / dt
                raw_derivative_y = (error_y_deg - self.prev_error_y) / dt
                
                # Low Pass Filtre (α=0.10: eski 0.02 osilasyona neden oluyordu)
                alpha = 0.03 
                derivative_x = (alpha * raw_derivative_x) + ((1.0 - alpha) * self.prev_derivative_x)
                derivative_y = (alpha * raw_derivative_y) + ((1.0 - alpha) * self.prev_derivative_y)

                if gorev == "Goruntulu" and self.mavlink.current_mode == "GUIDED":
                    # Deadzone-aware integral: deadzone içinde leaky integrator
                    if abs(error_x_deg) < self.deadzone_deg:
                        self.integral_error_x *= 0.995  # Yavaşça sıfıra çek
                    else:
                        self.integral_error_x += error_x_deg * dt
                    self.integral_error_x = self.clamp(self.integral_error_x, -8.0, 8.0)

                    if abs(error_y_deg) < self.deadzone_deg:
                        self.integral_error_y *= 0.995
                    else:
                        self.integral_error_y += error_y_deg * dt
                    self.integral_error_y = self.clamp(self.integral_error_y, -8.0, 8.0)
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

                # D-Term Clamping
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

                # Anti-windup: PID çıktısı doyuma girdiğinde integral'i azalt
                if abs(cmd_roll_deg) > self.max_roll_deg:
                    self.integral_error_x *= 0.9
                if abs(cmd_pitch_deg) > self.max_pitch_deg:
                    self.integral_error_y *= 0.9

                final_roll_rad = math.radians(final_roll_deg)
                final_pitch_rad = math.radians(final_pitch_deg)

                # --- KONTROLÜN UÇAĞA GÖNDERİLMESİ ---
                if gorev == "Goruntulu" and self.mavlink.current_mode == "GUIDED":
                    self.mavlink.send_attitude_target(final_roll_rad, final_pitch_rad, self.mavlink.current_yaw_rad)
                    print(f"[{gorev}] OTONOM TAKİP: Roll: {final_roll_deg:.1f} | Pitch: {final_pitch_deg:.1f}")
                else:
                    print(f"[{gorev}] DEBUG - Hedef Bulundu. Tahmini Komut -> Roll: {final_roll_deg:.1f} | Pitch: {final_pitch_deg:.1f}")

                self.last_final_roll_rad = final_roll_rad
                self.last_final_pitch_rad = final_pitch_rad

                # --- LOG YAZDIRMA ---
                roll_now = self.mavlink.current_roll_rad
                pitch_now = self.mavlink.current_pitch_rad
                delta_stab_x = stab_x - obj_x  # Stabilizasyonun pikseli ne kadar kaydırdığı
                delta_stab_y = stab_y - obj_y
                
                log_row = [
                    f"{current_time:.4f}", f"{dt:.4f}", self.mavlink.current_mode, 1,
                    # Sanal Gimbal verileri
                    f"{obj_x:.2f}", f"{obj_y:.2f}", f"{stab_x:.2f}", f"{stab_y:.2f}",
                    f"{delta_stab_x:.2f}", f"{delta_stab_y:.2f}",
                    f"{roll_now:.4f}", f"{pitch_now:.4f}",
                    f"{math.degrees(roll_now):.2f}", f"{math.degrees(pitch_now):.2f}",
                    # Sapma (Error) Verileri
                    f"{raw_error_x_deg:.2f}", f"{raw_error_y_deg:.2f}",
                    f"{stab_error_x_deg:.2f}", f"{stab_error_y_deg:.2f}",
                    f"{heading_diff_deg:.2f}",
                    # PID verileri
                    f"{p_term_roll:.6f}", f"{i_term_roll:.6f}", f"{d_term_roll:.6f}", f"{cmd_roll_rad:.6f}", f"{final_roll_deg:.2f}",
                    f"{p_term_pitch:.6f}", f"{i_term_pitch:.6f}", f"{d_term_pitch:.6f}", f"{cmd_pitch_rad:.6f}", f"{final_pitch_deg:.2f}",
                    # Hedef Verileri
                    f"{bbox_w:.1f}", f"{bbox_h:.1f}", f"{target_area:.1f}"
                ]
                self.logger.log(log_row)

        except KeyboardInterrupt:
            print("\nKapatılıyor...")
            self.logger.close()
            self.mavlink.running = False
            self.redis.running = False


if __name__ == '__main__':
    gudum = AutopilotController(connection_str='udp:127.0.0.1:14552')
    gudum.run()