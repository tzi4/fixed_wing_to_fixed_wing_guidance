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
import json
import logging
import argparse

# ─── BAĞLANTI SÖZLEŞMESİ (TEK KAYNAK) ───
# bumblebee_gudum.sh hunter (SysID 1) MAVProxy çıkışları: 14550 (QGC), 14551
# (formation.py/load_plan.py), 14553 (güdüm). 14562 diye bir çıkış YOKTUR;
# oraya bağlanan kod heartbeat beklerken sonsuza kadar takılır. Gerçek uçuşta
# farklı bir port kullanılıyorsa --connect ile verilir.
DEFAULT_CONNECTION = 'udpin:127.0.0.1:14553'
DEFAULT_SYSID = 1
HEARTBEAT_TIMEOUT_S = 30.0

log_filename = f"irtifa_farkı_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s',
    datefmt='%H:%M:%S',
    handlers=[
        logging.FileHandler(log_filename),
        logging.StreamHandler() # Terminal çıktısı için
    ]
)
# =====================================================================
# SAYILARIN KAYNAGI — gercek ucus mu, sim mi?
# =====================================================================
# Kural: goat_cam_offset.py'nin sayilari GERCEK UCUSTA dogrulanmis referanstir.
# Simde olculen bir katsayiyla gercek ucus degeri DEGISTIRILMEZ. Asagidaki
# liste, gercek donanima gecerken neye guvenilecegini tek bakista gosterir.
#
# GERCEK UCUSTAN GELEN (dokunulmadi, iki dosyada birebir ayni):
#   Kp_heading 3.0, Kd_heading 0.06, Kp_alt 2.5, Kd_alt 0.05, Kp_rate 0.6,
#   deadzone_deg 0.4, max_heading_change_deg 35,
#   max_alt_change_m +2.0, min_alt_change_m -5.0
#
# UCAKTAN BAGIMSIZ (guvenli, her platformda gecerli):
#   - AGL zemin emniyeti: 16 m AGL altinda alcalma yetkisi daralir. Tavani
#     ASLA artirmaz, yalnizca azaltir.
#   - Bootstrap korumasi: telemetri gelmeden komut uretilmez.
#   - Ilk kare turev korumasi ve dt alt siniri (kosan medyanin yarisi).
#
# SIMDEN TURETILDI — GERCEK DONANIMDA YENIDEN OLCULMELI:
#   1) Kp_h 3.0 / Kd_h 0.06 (menzil normalizasyonu ankraji). Bu bir YENIDEN
#      PARAMETRELESTIRME: R = 48 m'de eski Kp_alt 2.5 m/derece ile BIREBIR
#      ayni komutu uretir, yani gercek tune o menzilde korunur. 48 m degeri
#      gorev bilgisinden (0-100 m calisma bandi) geliyor, sim'den degil.
#      Diger menzillerde davranis bilincli olarak farklidir — duzeltmenin
#      kendisi budur.
#   2) deadzone_dar_deg 0.15 ve 12 px esigi (P3). EN COK SIME BAGLI OLAN.
#      Simdeki dedektor gurultu tabanindan turetildi (25 m alti 0.019 derece).
#      Gercek dedektorun gurultusu farkli olacaktir; ayni yontemle (0.5 s
#      hareketli ortalamadan sapma) yeniden olculmeden guvenilmemeli.
#      Guvenmiyorsan: kalite_deadzone = False yap, 0.4 dereceye doner.
#   3) Devir kapisi esikleri (6 s, 0.3 m/s, 1.5 derece). Bunlar bir KAZANC
#      degil, "ucak oturdu mu" DEDEKTORU. Yine de sim'in devir davranisindan
#      turetildi; gercekte gerekirse DEVIR_KAPISI = False.
#
# AKTIF OLMAYAN: menzil_k (bbox'tan menzil kalibrasyonu) yalnizca
#   menzil_kaynak='bbox' secilirse kullanilir; teva.py varsayilani
#   'telemetri' oldugu icin kontrol yoluna girmez.
# =====================================================================

# =====================================================================
# AYAR BAYRAKLARI — kodun icinden ON/OFF yapilir (CLI'ya gerek yok)
# =====================================================================
# AUTO->GUIDED devir yumusatma kapisi.
#   True  : GUIDED algilaninca ucak oturana kadar PID CALISMAZ; irtifa devir
#           anindaki degere kilitlenir, yon sabit tutulur, sonra komut 2 s
#           icinde rampalanarak devreye girer.
#   False : eski davranis (devirden hemen sonra tam komut).
# NEDEN VAR: olculdu — GUIDED'a gecerken ArduPlane'in kendi gecicisi var
# (gudum HIC calismazken bile ilk 15 s'de pitch 5.8 derece salaniyor, gaz
# %88'den %59'a dusuyor, ucak ~3 m cokuyor). Gudum bu gecicinin uzerine
# komut binerse salinim 14.9 dereceye cikiyor ve hedef kadrajdan cikiyor.
# GERCEK DONANIMDA gerekirse False yapin.
DEVIR_KAPISI = True
DEVIR_BEKLEME_S = 6.0        # kapinin asgari bekleme suresi
# NOT (olculdu): 2.0 s COK ERKENDI — kapi 4.5 s'de aciliyordu ama ucak hala
# cokuyordu (49.96 -> 46.88 m) ve sonraki 15 s'de pitch 14.9 derece salaniyordu.
# Zaten oturmus GUIDED'da baslatildiginda ayni kod 1.3 derece veriyor, yani
# kapi ucak GERCEKTEN oturana kadar tutmali.
YUMUSAK_BASLANGIC_S = 2.0    # kapi acildiktan sonra komutun rampalanma suresi
# =====================================================================

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
            "anti_windup_x", "anti_windup_y",
            # --- 2026-07-29 eklendi: durust PID teshisi icin ---
            # Her eksende HAM (clamp oncesi) komut + doygunluk bayragi olmadan
            # "komut ne kadar kuvvetli uygulandi" sorusu cevaplanamaz.
            "sqrt_area_px", "airspeed_ms", "groundspeed_ms", "vz_ms", "throttle_pct",
            "rate_error_deg", "p_term_rate", "i_term_rate", "d_term_rate", "heading_rate_dps",
            "cmd_head_raw_deg", "cmd_alt_raw_m", "sat_head", "sat_alt",
            # HEDEF VERISI BILEREK YOK: hedefin irtifasi/konumu gudum surecine
            # HIC girmez (testlere hile karismasin). O veriyi ayri calisan
            # tools/gercek_konum_logger.py toplar. Bu iki sutun onun yerine
            # kosuyu tanimlar ki hangi ayarla uculdugu logdan anlasilsin.
            "aim_pitch_deg", "mount_phys_deg", "lat", "lon",
            "menzil_est_m", "m_per_deg"
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

    # get_rakip_irtifa() KALDIRILDI (2026-07-29): hedefin irtifasi gudum
    # surecine girmemeli. Bkz. tools/gercek_konum_logger.py


class MavlinkManager(threading.Thread):
    def __init__(self, connection_str=DEFAULT_CONNECTION, target_sysid=DEFAULT_SYSID,
                 heartbeat_timeout=HEARTBEAT_TIMEOUT_S):
        super().__init__()
        self.daemon = True
        self.running = True
        self.lock = threading.Lock()

        self.current_mode = "UNKNOWN"

        self.current_yaw_rad = 0.0
        self.current_roll_rad = 0.0
        self.current_pitch_rad = 0.0
        self.current_alt_rel = 100.0  # Varsayılan başlangıç irtifası
        # BOOTSTRAP KORUMASI (2026-07-30): yukaridaki varsayilanlar telemetri
        # gelmeden once kullanilirsa ilk irtifa komutu gercek 50 m'ye karsi
        # 100+cmd olur -> param3=0 oldugu icin ANLIK ~+52 m basamak. Log
        # taramasinda hic tetiklenmemis ama pay 0.87 s'ye kadar inmis; mod
        # zaten GUIDED iken heartbeat GLOBAL_POSITION_INT'ten once gelirse
        # tetiklenir. Kontrol yolu bu iki bayrak dolmadan CALISMAZ.
        self.attitude_geldi = False
        self.pozisyon_geldi = False
        # PID/hiz grafikleri icin telemetri (VFR_HUD + GLOBAL_POSITION_INT).
        self.current_airspeed = float('nan')
        self.current_groundspeed = float('nan')
        self.current_throttle = float('nan')
        self.current_lat = float('nan')
        self.current_lon = float('nan')
        self.current_vx = float('nan')
        self.current_vy = float('nan')
        self.current_vz = float('nan')

        print(f"MAVLink bağlantısı: {connection_str} (beklenen SysID {target_sysid})...")
        try:
            self.master = mavutil.mavlink_connection(connection_str)
            # wait_heartbeat() süresizdir: yanlış porta bağlanınca kod sessizce
            # sonsuza kadar asılır. Zaman aşımlı + SysID doğrulamalı bekleme.
            heartbeat = None
            deadline = time.monotonic() + heartbeat_timeout
            while time.monotonic() < deadline:
                candidate = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
                if candidate is None:
                    continue
                if not self.master.probably_vehicle_heartbeat(candidate):
                    continue  # MAVProxy'nin kendi GCS heartbeat'i
                if candidate.get_srcSystem() == target_sysid:
                    heartbeat = candidate
                    break
            if heartbeat is None:
                raise TimeoutError(
                    f"{connection_str} üzerinde {heartbeat_timeout:.0f} sn içinde "
                    f"SysID {target_sysid} heartbeat'i gelmedi")
            self.master.target_system = heartbeat.get_srcSystem()
            self.master.target_component = heartbeat.get_srcComponent()
            print(f"Bağlantı Başarılı. Sistem ID: {self.master.target_system} "
                  f"(comp {self.master.target_component})")
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
                    msg = self.master.recv_match(type=['ATTITUDE', 'HEARTBEAT', 'GLOBAL_POSITION_INT', 'VFR_HUD'], blocking=False)

                if msg:
                    if msg.get_type() == 'ATTITUDE':
                        self.attitude_geldi = True
                        self.current_yaw_rad = msg.yaw
                        self.current_roll_rad = msg.roll
                        self.current_pitch_rad = msg.pitch
                    elif msg.get_type() == 'HEARTBEAT':
                        if self.master.flightmode:
                            self.current_mode = self.master.flightmode
                    elif msg.get_type() == 'GLOBAL_POSITION_INT':
                        self.pozisyon_geldi = True
                        self.current_alt_rel = msg.relative_alt / 1000.0
                        # PID grafikleri icin: yer hizi bilesenleri ve konum.
                        self.current_lat = msg.lat / 1e7
                        self.current_lon = msg.lon / 1e7
                        self.current_vx = msg.vx / 100.0
                        self.current_vy = msg.vy / 100.0
                        self.current_vz = msg.vz / 100.0
                    elif msg.get_type() == 'VFR_HUD':
                        # Hiz kontrolu eklenmeden ONCE bile taban cizgisi loglanir.
                        self.current_airspeed = msg.airspeed
                        self.current_groundspeed = msg.groundspeed
                        self.current_throttle = msg.throttle
                time.sleep(0.01)
            except:
                time.sleep(0.1)

    def send_heading_target(self, heading_deg, heading_rate_dps=3.0):
        with self.lock:
            try:
                self.master.mav.command_long_send(
                    self.master.target_system,
                    self.master.target_component,
                    mavutil.mavlink.MAV_CMD_GUIDED_CHANGE_HEADING,
                    0, 1, heading_deg, heading_rate_dps, 0, 0, 0, 0
                )
            except:
                pass

    def send_speed_target(self, speed_ms, accel=2.0):
        """43000: param1=0 (airspeed), param2=hiz, param3=ivme.

        YALNIZ TEST IZOLASYONU icin kullanilir (--sabit-hiz). Gudumun kendi
        hiz kontrolcusu YOK; dikey eksen tune edilirken hizin serbest
        birakilmasi "tirmanamadi mi yoksa hizlanamadi mi" ayrimini imkansiz
        kildigi icin hiz disaridan sabitlenir.
        """
        with self.lock:
            try:
                self.master.mav.command_long_send(
                    self.master.target_system, self.master.target_component,
                    mavutil.mavlink.MAV_CMD_GUIDED_CHANGE_SPEED,
                    0, 0, speed_ms, accel, 0, 0, 0, 0)
            except:
                pass

    def send_altitude_target(self, altitude_m):
        """MAV_CMD_GUIDED_CHANGE_ALTITUDE (43001). param3 = 0 BILINCLI.

        ArduPlane kaynagindan dogrulandi (GCS_MAVLink_Plane.cpp, 43001):
            if (is_zero(packet.param3)) target_alt_rate = 1000.0;
            else                        target_alt_rate = fabsf(param3);
        ve mode_guided.cpp'de her donguda:
            delta = (now - target_alt_time_ms)/1000       # son KOMUTTAN beri
            delta_amt = delta * target_alt_rate
            alt = constrain(hedef, onceki - delta_amt, onceki + delta_amt)

        Kritik: target_alt_time_ms HER KOMUTTA now'a cekilir. Biz 10 Hz komut
        gonderdigimiz icin delta ~0.1 s'de sabitlenir; param3 = R verilirse
        komut basina izin verilen degisim 0.1*R metreye DUSER. 1 Temmuz'da
        param3=10 idi -> komut basina ~1.3 m -> "+25 m tirman" fiilen
        uygulanmiyordu ve DIKEY EKSEN HIC TEST EDILMEMIS oldu.
        Dolayisiyla 10 Hz komut hizinda param3 MUTLAKA 0 kalmali; hiz sinirini
        biz kendi tarafimizda min/max_alt_change_m ile koyuyoruz.

        NOT: param2 konumunda duran eski 0.8 KALDIRILDI. 43001 handler'i
        param2'yi hic okumuyor (yalnizca frame, z ve param3). Orada durmasi
        "rampa verilmis" yanilgisi yaratiyordu; birinin onu param3'e tasimasi
        dikey ekseni yeniden olduren degisiklik olurdu.

        GUVENLIK: ArduPlane z == 0 ve z == -1 degerlerini DENIED ile reddeder;
        cagiran taraf target_alt'i 10 m'nin altina dusurmuyor.
        """
        with self.lock:
            try:
                self.master.mav.command_long_send(
                    self.master.target_system,
                    self.master.target_component,
                    mavutil.mavlink.MAV_CMD_GUIDED_CHANGE_ALTITUDE,
                    0,          # confirmation
                    0, 0, 0,    # param1, param2, param3(=0: rampa yok)
                    0, 0, 0,    # param4, param5, param6
                    altitude_m  # param7 -> COMMAND_INT.z (hedef irtifa)
                )
            except:
                pass

class AutopilotController:
    def __init__(self, connection_str=DEFAULT_CONNECTION, target_sysid=DEFAULT_SYSID,
                 mount_pitch_deg=None, mount_phys_pitch_deg=None, aim_pitch_deg=None,
                 sabit_heading=None, sabit_hiz=None):
        self.data_queue = queue.Queue()

        self.logger = FlightLogger()
        self.mavlink = MavlinkManager(connection_str, target_sysid)
        self.redis = RedisListener(self.data_queue)

        if self.mavlink.running and self.redis.running:
            self.mavlink.start()
            self.redis.start()
        else:
            exit(1)

        # --- YENİ PID VE KONTROL AYARLARI (HEADING & ALTITUDE) ---
        # "P oldukça hantal, I ve D 0" talebine uygun
        # Sistemin hedefe dönebilmesi için Kp değerlerini ciddi oranda artırdık
        self.Kp_heading = 3.0    # 1 derece hata = 4.5 derece hedefe sapma
        self.Ki_heading = 0.0
        self.Kd_heading = 0.06

        self.Kp_alt = 2.5       # 1 derece hata = 1.5 metre irtifa değişimi
        self.Ki_alt = 0.0
        self.Kd_alt = 0.05

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
        # IRTIFA KOMUT SINIRI — gercek hayatta kritik.
        # param3=0 gonderdigimiz icin (asagida gerekcesi) ArduPilot hedef
        # irtifayi RAMPASIZ, aninda kabul eder: target_alt_rate = 1000 m/s.
        # Yani tek gercek hiz sinirlayici BU CLAMP'tir. +-25 m'lik bir sicrama
        # TECS'e "25 m otede havuc" gosterir ve azami tirmanis/alcalma komutu
        # urettirir. Ucusta kanitlanmis surum (old_guidance/goat_gimbal_ucak.py)
        # asimetrik ve cok daha dar tutuyordu; ona donuldu:
        #   tirmanis <= +2 m, alcalma <= -5 m
        # Asimetri bilincli: ucak icin tirmanis alcalmadan daha pahali/yavas.
        self.max_alt_change_m = 2.0
        self.min_alt_change_m = -5.0

        # --- MENZIL NORMALIZASYONU (2026-07-29) ---
        # SORUN: dongu hatayi ACI olarak olcup komutu METRE olarak veriyor,
        # Kp_alt ise sabit 2.5 m/derece. Ama 1 derecenin metre karsiligi
        # menzille buyur: 50 m'de 0.87 m, 300 m'de 5.24 m. Sabit kazanc bu
        # yuzden YAKINDA fazla, UZAKTA yetersiz komut uretiyor.
        # Olculen kanit (T5, 2026-07-29): |ey| ortalamasi 100-300 m'de 1.23
        # deg ve doygunluk %23; 40-100 m'de 0.30 deg ve doygunluk %0;
        # 40 m altinda tepe hata 6.38 deg (10 m menzilde 6 deg = 1 m irtifa,
        # ama sabit kazanc 15 m komut istiyor).
        # COZUM: hatayi menzille metreye cevir -> dh = R * tan(hata).
        # Menzil YALNIZ KAMERADAN tahmin edilir (sqrt(alan) kalibrasyonu),
        # hedef telemetrisine dokunulmaz; test butunlugu korunur.
        # VARSAYILAN ARTIK 'menzil' (2026-07-29 A/B/C olcumu):
        # ayni senaryoda (hedef 50->30 m dalis, ort menzil 34 m)
        #   A eski aci     : |dz| 1.01 m, tepe -8.7 m, doygunluk %19.4
        #   C P1+P2+P3     : |dz| 0.73 m, tepe -5.4 m, doygunluk %7.4
        self.dikey_mod = 'menzil'       # 'aci' (eski) | 'menzil' (yeni)
        self.agl_olcekli_alcalma = True    # P1 (yalniz zemin emniyeti)
        self.kalite_deadzone = True        # P3
        self.deadzone_dar_deg = 0.15       # P3: gurultu tabani 25 m alti 0.019 deg
        self.menzil_k = 2119.67         # R = k / sqrt_area_px
        self.menzil_min_m = 5.0         # kalibrasyon 4 m'ye kadar gecerli;
                                        # 15 m tabani kisa menzilde eski sabit-kazanc
                                        # kusurunu geri getiriyordu
        self.menzil_max_m = 250.0
        self.menzil_tau_s = 2.0         # alcak gecirgen: DC offset yavas degissin
        self._menzil_filt = None
        # Boyutsuz kazanclar. Kp_h = 1.0 ankraji: R = 143 m'de yeni yasa eski
        # Kp_alt = 2.5 m/derece ile BIREBIR ayni komutu uretir
        # (143 * tan(1 deg) = 2.50). Yani mevcut tune bir menzilde korunur,
        # digerlerinde fiziksel olarak duzeltilir.
        # ANKRAJ DUZELTILDI (2026-07-29, log analizi): 143 m ankraji operasyonel
        # bandin (0-100 m) DISINDA kaliyordu ve orada kazanci medyan 0.27 katina
        # dusuruyordu -> OP_A'da ortalama hata 0.92 -> 1.13 m KOTULESIYORDU.
        # Ankraj R = 48 m (bandin ortasi): bugunku tune bandda KORUNUR
        # (orneklerin yalniz %5.9'u >0.25 m degisir), yalnizca band disinda
        # fiziksel olarak duzeltilir.
        self.Kp_h = 3.0                 # 143/48
        self.Ki_h = 0.0
        self.Kd_h = 0.06                # 0.05/(48*tan1deg), ayni ankraj
        self.deadzone_deg = 0.4

        self.prev_error_x, self.prev_error_y = 0.0, 0.0
        self.integral_error_x, self.integral_error_y = 0.0, 0.0
        self.prev_derivative_x, self.prev_derivative_y = 0.0, 0.0
        self.last_time = time.time()
        self._dt_gecmis = []
        self._pitch_gecmis = []
        self.last_cmd_send_time = 0.0

        self.last_target_time = 0.0
        self.last_target_heading = 0.0
        self.last_target_alt = 0.0
        self.last_heading_rate = self.min_heading_rate

        self.frame_counter = 0
        self.start_time = time.time()

        # --- DEVIR (AUTO->GUIDED) YUMUSATMA KAPISI (2026-07-30) ---
        # Olculen: devirde gaz kolu ILK GUDUM KOMUTUNDAN ONCE cokuyor
        # (t+0.27 s vs ilk komut t+0.47 s), pitch +-6 derece salaniyor 8-16 s.
        # Kameranin dikey yari-FOV'u 6.78 derece oldugu icin hedef kadrajdan
        # cikiyor: ilk 10 saniyede karelerin %17.7-18.5'i takipsiz.
        # AUTO->GUIDED gecen 4 kosunun HEPSI osile ediyor; zaten GUIDED'da
        # baslayan 4 kosuda sifir. Yani kaynak devir, gudum degil.
        # Kapi: devirden sonra ucak oturana kadar SABITLE (mevcut irtifayi ve
        # yonu komutla), PID'i calistirma. Boylece gudum kendi uretmedigi bir
        # gecici rejime karsi savasmiyor ve doygunluga girmiyor.
        # GERCEK DONANIMDA KAPATILABILIR: --devir-kapisi-kapat
        # Kapi sim'de olculen bir sorunu (devirde pitch salinimini bizim
        # buyutmemiz) hedefliyor; gercek ucakta devir davranisi farkli
        # olabilecegi icin kullanici kapatabilmeli.
        self.devir_kapisi = DEVIR_KAPISI
        self.engage_bekleme_s = DEVIR_BEKLEME_S
        self.engage_azami_s = 25.0      # tavan: sonsuza kadar bekleme
        self.engage_vz_esik = 0.3       # m/s — dikey hiz bunun altina inince hazir
        self.yumusak_baslangic_s = YUMUSAK_BASLANGIC_S
        self._engage_t = None
        self._engage_alt = None         # devirdeki irtifa KILITLENIR
        self._kapi_acildi_t = None

        # --- TEST IZOLASYONU (sequential tuning) ---
        # Dikey ekseni tek basina olcebilmek icin yatay eksen ve hiz
        # DONDURULUR. GUIDED'da komut gondermemek ISE YARAMAZ (ucak loiter'a
        # girer), bu yuzden "o anki deger" surekli yeniden gonderilir.
        #   sabit_heading = 'auto' -> baslangictaki yaw dondurulur
        #   sabit_hiz     = m/s    -> her komut turunda 43000 ile tekrarlanir
        self.sabit_heading = sabit_heading
        self.sabit_hiz = sabit_hiz      # sayi veya 'auto'
        self._sabit_hiz_ms = sabit_hiz if isinstance(sabit_hiz, (int, float)) else None
        self._sabit_heading_deg = None
        if isinstance(sabit_heading, (int, float)):
            self._sabit_heading_deg = float(sabit_heading)

        # --- KAMERA AYARLARI (Lütfen Gerçek Sisteme Göre Güncelleyin!) ---
        self.camera_width = 1920
        self.camera_height = 1080
        self.camera_hfov_rad = 0.42

        self.center_x = 1025  # 960
        self.center_y = 569   # 540
        self.fx = 4543
        self.fy = 4539

        # --- KAMERA MONTAJ AÇISI (gövde eksenine göre, yalnızca pitch) ---
        # Eski tek parametreli "mount_pitch_deg" (-6.0) aslinda iki ayri fiziksel
        # buyuklugun toplamiydi:
        #   (a) mount_phys_pitch_deg: GERCEK fiziksel montaj acisi (kamera ile
        #       otopilot arasindaki sabit aci, ~-1°). Govde eksenine baglidir;
        #       ucak yatinca kamera govdeyle BIRLIKTE yatar. Bu yuzden
        #       de-rotasyondan ONCE, govde cercevesinde uygulanir.
        #   (b) aim_pitch_deg: AOA (hucum acisi) telafisi (~-5°). "Hedef ufka
        #       gore su kadar yukarida/asagida dursun" demektir; UFKA
        #       baglidir. Bu yuzden de-rotasyondan SONRA, sanal (ufka hizali)
        #       cercevede uygulanmalidir. Eskiden (b) de (a) ile ayni yerde
        #       uygulandigi icin ucak yattikca δ*sin(roll) kadar sahte yatay
        #       hata uretiyordu.
        def _ry(deg):
            r = math.radians(deg)
            return np.array([
                [ math.cos(r), 0.0, math.sin(r)],
                [ 0.0,          1.0, 0.0         ],
                [-math.sin(r), 0.0, math.cos(r)]
            ])

        # SIMULASYON VARSAYILANI: kamera govdeye TAM PARALEL kabul edilir
        # (models/bumblebee/model.sdf ve models/emir_ucak_temp/model.sdf icinde
        # camera pose pitch = 0). Bu yuzden mount_phys = 0.0.
        # GERCEK UCAKTA olculen deger ~-1.0 derecedir (kamera ile otopilot
        # arasindaki montaj acisi; bazen 1 derece civari). Gercek donanimda
        # ucarken --mount-phys-pitch -1.0 verilmeli (veya bu satir -1.0
        # yapilmali). Gercek montaj acisi ucus logundan da olculebilir:
        # stab_error_x_deg ~ roll_deg regresyon egiminin arktanjanti = -aci.
        self.mount_phys_pitch_deg = 0.0
        # --- aim_pitch_deg NE DEMEK? (2026-07-29, sayisal olarak dogrulandi) ---
        # DIKKAT: sanal gimbal govde pitch'ini matematiksel olarak cikarir, ama
        # FIZIKSEL kamera hala govdeyle birlikte egilidir. Kadraj sinirli
        # oldugundan (dikey yari-FOV ~6.8 deg) "sanal cerceve merkezi" ile
        # "gercek kadraj merkezi" AYNI SEY DEGILDIR. Bu parametre tam olarak o
        # farki kapatan DC offset'tir; sanal gimbal AC (govde salinimi)
        # gurultusunu temizlerken bu DC bileseni elle veriyoruz.
        #
        # Olculen bagintilar (theta = govde pitch, eps = hedefin ufka gore
        # gercek yukselisi):
        #     ey    = -(eps + aim)          -> denge (ey=0):  eps = -aim
        #     hedefin HAM kadrajdaki yeri:   delta_raw = theta - eps
        # Hedefi GERCEK kadraj merkezine oturtmak icin delta_raw = 0 gerekir:
        #     eps = theta   =>   aim = -theta
        #
        # Sayisal dogrulama (fx/fy=4543/4539, 1920x1080, merkez 1025/569):
        #   theta=-2.77 (Erenimbus): aim=+2.77 -> ham y=569.0 (TAM MERKEZ),
        #                            aim= 0.00 -> ham y=349.4 (219 px yukarida)
        #   theta=+4.19 (Bumblebee): aim=-4.19 -> ham y=569.0 (TAM MERKEZ),
        #                            aim= 0.00 -> ham y=901.5 (alt kenara 178 px)
        # Kullanicinin bumblebee'de elle buldugu -6 bu bagintiyla uyumludur
        # (kanonik -4.19'un biraz otesi, hedefi biraz yukarida tutar).
        #
        # ERENIMBUS: bugunku TRACKING loglarinda medyan seyir pitch'i -2.77 deg
        # (n=2573, 20260729_112440) -> aim = +2.8.
        # Ucak trim/hiz degisirse seyir pitch'i de degisir; bu deger o zaman
        # yeniden olculmelidir (log medyani pitch_deg sutunundan).
        self.aim_pitch_deg = +2.8
        # Menzil sonumlemesi: bu menzilin altinda aim TAM uygulanir, ustunde
        # dogrusal olarak sifire iner. Menzili zaten biliyoruz (telemetri),
        # o yuzden ek maliyet yok. Olculdu: 81 m'de aim=+2.8 hedefi merkeze
        # oturtuyor (23 px sapma) ve kenar payini 349 -> 488 px'e cikariyor;
        # 265-300 m'de ayni deger 13-15 m irtifa farki isteyip clamp'i
        # doyuruyor (%35) ve hic oturmuyor.
        self.aim_tam_menzil_m = 120.0
        self.aim_sifir_menzil_m = 250.0
        if mount_phys_pitch_deg is not None:
            self.mount_phys_pitch_deg = float(mount_phys_pitch_deg)
        if aim_pitch_deg is not None:
            self.aim_pitch_deg = float(aim_pitch_deg)
        # Eski --mount-pitch (tek parametre) geriye uyumluluk: verilen deger
        # TOPLAMI ifade eder, aim = toplam - mount_phys olarak cozulur.
        if mount_pitch_deg is not None:
            self.aim_pitch_deg = float(mount_pitch_deg) - self.mount_phys_pitch_deg

        self.mount_pitch_deg = self.mount_phys_pitch_deg + self.aim_pitch_deg  # geriye donuk bilgi/log amacli
        print(f"[kamera] mount_phys={self.mount_phys_pitch_deg:+.2f} "
              f"aim={self.aim_pitch_deg:+.2f} (toplam {self.mount_pitch_deg:+.2f})")

        self.R_mount_phys = _ry(self.mount_phys_pitch_deg)
        self.R_aim = _ry(self.aim_pitch_deg)
        self._son_aim_deg = self.aim_pitch_deg

        self.K = np.array([
            [self.fx, 0,       self.center_x],
            [0,       self.fy, self.center_y],
            [0,       0,       1            ]
        ])
        self.K_inv = np.linalg.inv(self.K)

    def menzil_tahmin(self, target_area, dt):
        """Kameradan menzil tahmini: R = k / sqrt(alan), alcak gecirgen filtreli.

        Hedefin telemetrisi KULLANILMAZ. Kalibrasyon offline yapildi
        (tools/menzil_kalibrasyon.py, tutulan test RMSE 17.9 m, gecerli 4-275 m).
        Filtre bilincli yavas: bu bir DC duzeltmesi, gurultuyu kontrol
        yasasina tasimamali.
        """
        if not target_area or target_area <= 0:
            return self._menzil_filt
        ham = self.menzil_k / math.sqrt(target_area)
        ham = self.clamp(ham, self.menzil_min_m, self.menzil_max_m)
        if self._menzil_filt is None:
            self._menzil_filt = ham
        else:
            a = dt / (self.menzil_tau_s + dt) if dt > 0 else 0.0
            self._menzil_filt += a * (ham - self._menzil_filt)
        return self._menzil_filt

    def _aim_etkin_deg(self, menzil_est):
        """Menzil-farkinda aim: bant icinde tam, bant disinda sonumlu.

        aim sabit bir ACI ama metre karsiligi menzille buyur (R*tan(aim)).
        120 m'de 5.9 m (kolay), 300 m'de 14.7 m (clamp servis edemiyor,
        olculdu: %35 doygunluk, hic oturmuyor). Bu yuzden uzakta sonumleniyor.
        """
        if not self.aim_pitch_deg or menzil_est is None:
            return self.aim_pitch_deg
        r = float(menzil_est)
        if r <= self.aim_tam_menzil_m:
            k = 1.0
        elif r >= self.aim_sifir_menzil_m:
            k = 0.0
        else:
            k = ((self.aim_sifir_menzil_m - r)
                 / (self.aim_sifir_menzil_m - self.aim_tam_menzil_m))
        return self.aim_pitch_deg * k

    def _hiz_tohumla(self):
        """--sabit-hiz auto: devirdeki GERCEK seyir hizini bir kez yakala.

        AUTO'da 19.6 m/s ucan ucaga devirde 18 m/s komutlamak TECS'e 1.6 m/s
        attirmak demek; olculen devir salinimi bu farkla orantiliydi.
        """
        if self._sabit_hiz_ms is not None:
            return
        v = getattr(self.mavlink, 'current_airspeed', None)
        if v is not None and v == v and 5.0 < v < 40.0:
            self._sabit_hiz_ms = round(float(v), 2)
            print(f"[hiz] sabit hiz devirdeki seyir hizindan tohumlandi: "
                  f"{self._sabit_hiz_ms:.2f} m/s")

    def clamp(self, value, min_val, max_val):
        return max(min(value, max_val), min_val)

    def stabilize_pixel(self, obj_x, obj_y):
        p_raw = np.array([obj_x, obj_y, 1.0])
        r_cam = self.K_inv @ p_raw
        r_body = self.R_mount_phys @ (R_c_b @ r_cam)   # fiziksel montaj: de-rotasyondan ONCE

        roll_rad = self.mavlink.current_roll_rad
        pitch_rad = self.mavlink.current_pitch_rad
        R_stab = compute_R_b_e(roll_rad, pitch_rad, 0)
        # AIM de-rotasyondan SONRA uygulanir. Menzil-farkinda oldugu icin her
        # karede yeniden kurulur; menzil yavas degistigi icin bu ucuz.
        aim_e = self._aim_etkin_deg(self._menzil_filt)
        if aim_e != self._son_aim_deg:
            self._son_aim_deg = aim_e
            r_ = math.radians(aim_e)
            self.R_aim = np.array([[math.cos(r_), 0.0, math.sin(r_)],
                                   [0.0, 1.0, 0.0],
                                   [-math.sin(r_), 0.0, math.cos(r_)]])
        r_virt_body = self.R_aim @ (R_stab @ r_body)
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
                   aw_x, aw_y,
                   rate_error=0.0, p_rate=0.0, i_rate=0.0, d_rate=0.0, heading_rate_dps=0.0,
                   cmd_head_raw_deg=0.0, cmd_alt_raw_m=0.0, sat_head=0, sat_alt=0,
                   menzil_est_m=None, m_per_deg=None):
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
        m = self.mavlink
        nan = float('nan')
        f = lambda v, n=2: (f"{v:.{n}f}" if v is not None and v == v else "")
        row += [
            f"{math.sqrt(target_area):.2f}" if target_area and target_area > 0 else "0.00",
            f(getattr(m, 'current_airspeed', nan)), f(getattr(m, 'current_groundspeed', nan)),
            f(getattr(m, 'current_vz', nan)), f(getattr(m, 'current_throttle', nan), 1),
            f"{rate_error:.4f}", f"{p_rate:.6f}", f"{i_rate:.6f}", f"{d_rate:.6f}", f"{heading_rate_dps:.3f}",
            f"{cmd_head_raw_deg:.4f}", f"{cmd_alt_raw_m:.4f}", sat_head, sat_alt,
            f"{self._son_aim_deg:.3f}", f"{self.mount_phys_pitch_deg:.3f}",
            f(getattr(m, 'current_lat', nan), 7), f(getattr(m, 'current_lon', nan), 7),
            f(menzil_est_m, 1), f(m_per_deg, 4),
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
                                    self.mavlink.send_heading_target(self.last_target_heading, self.last_heading_rate)
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
                            self.integral_error_rate = 0.0

                            # Failsafe: Continue at current heading and altitude
                            cmd_sent = 0
                            if gorev == "Goruntulu" and self.mavlink.current_mode == "GUIDED":
                                if current_time - self.last_cmd_send_time >= 0.5:
                                    self.mavlink.send_heading_target(yaw_deg, self.min_heading_rate)
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
                # dt TABANI: sabit 0.001 s, ilk karede (last_time __init__'te
                # kuruldugu icin) 12-15 kat sisik turev uretiyordu — olculen
                # ham turev 1127-2817 deg/s. Artik taban kosan medyanin yarisi.
                ilk_kare = (self._dt_gecmis == [])
                ham_dt = current_time - self.last_time
                self._dt_gecmis.append(ham_dt)
                if len(self._dt_gecmis) > 60:
                    self._dt_gecmis.pop(0)
                med = sorted(self._dt_gecmis)[len(self._dt_gecmis) // 2]
                dt = max(max(0.001, 0.5 * med), min(ham_dt, 0.1))
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
                # ILK KAREDE TUREV YOK: prev_error 0 oldugu icin ilk ornek
                # butun hatayi basamak sanip 1127-2817 deg/s uretiyordu.
                # Taban dt korumasi da o karede henuz gecmis olmadigi icin
                # calismiyor -> dogrusu turevi hic hesaplamamak.
                if ilk_kare:
                    raw_deriv_x = raw_deriv_y = 0.0
                else:
                    raw_deriv_x = (stab_error_x_deg - self.prev_error_x) / dt
                    raw_deriv_y = (stab_error_y_deg - self.prev_error_y) / dt
                deriv_x = (alpha * raw_deriv_x) + ((1.0 - alpha) * self.prev_derivative_x)
                deriv_y = (alpha * raw_deriv_y) + ((1.0 - alpha) * self.prev_derivative_y)

                # İntegral
                if gorev == "Goruntulu" and self.mavlink.current_mode == "GUIDED":
                    if abs(stab_error_x_deg) < self.deadzone_deg: self.integral_error_x *= 0.99  # noqa
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
                # P3 — deadzone tespit kalitesine gore (2026-07-29).
                # Sabit ACISAL deadzone'un METRE bedeli menzille buyur:
                # 0.4 deg = 12 m'de 0.08 m ama 100 m'de 0.70 m. Olculen gurultu
                # tabani: 25 m alti 0.019 deg, 50-100 m 0.164 deg. Yani 0.4 deg
                # yakinda gurultunun 20 KATI -> gereksiz olu bant.
                # Bbox buyukse (yakin/temiz tespit) daralt, kucukse eski degeri koru.
                dz_now = self.deadzone_deg
                if self.kalite_deadzone and target_area and math.sqrt(target_area) >= 12.0:
                    dz_now = self.deadzone_dar_deg
                dz_x = 1 if abs(stab_error_x_deg) < dz_now else 0
                dz_y = 1 if abs(stab_error_y_deg) < dz_now else 0
                p_x = 0 if dz_x else stab_error_x_deg - math.copysign(dz_now, stab_error_x_deg)
                p_y = 0 if dz_y else stab_error_y_deg - math.copysign(dz_now, stab_error_y_deg)

                # --- 1. HEADING KONTROLÜ ---
                p_term_heading = p_x * self.Kp_heading
                i_term_heading = self.integral_error_x * self.Ki_heading
                d_term_heading = deriv_x * self.Kd_heading

                cmd_head_deg = p_term_heading + i_term_heading + d_term_heading
                cmd_head_ham_deg = cmd_head_deg  # clamp oncesi ham PID cikisi
                cmd_head_deg = self.clamp(cmd_head_deg, -self.max_heading_change_deg, self.max_heading_change_deg)
                sat_head = 1 if abs(cmd_head_ham_deg) > self.max_heading_change_deg else 0
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

                # --- 2. ALTITUDE KONTROLÜ ---
                # Hedef Y ekseninde aşağıdaysa (p_y pozitif), irtifa DÜŞÜRÜLMELİ
                menzil_est = self.menzil_tahmin(target_area, dt)
                if self.dikey_mod == 'menzil' and menzil_est:
                    # 1 derecelik acisal hatanin METRE karsiligi (menzile bagli)
                    m_per_deg = menzil_est * math.tan(math.radians(1.0))
                    p_term_alt = p_y * m_per_deg * self.Kp_h
                    i_term_alt = self.integral_error_y * m_per_deg * self.Ki_h
                    d_term_alt = deriv_y * m_per_deg * self.Kd_h
                else:
                    m_per_deg = float('nan')
                    p_term_alt = p_y * self.Kp_alt
                    i_term_alt = self.integral_error_y * self.Ki_alt
                    d_term_alt = deriv_y * self.Kd_alt

                cmd_alt_m = -1 * (p_term_alt + i_term_alt + d_term_alt)
                cmd_alt_ham_m = cmd_alt_m  # clamp oncesi ham PID cikisi
                # YUMUSAK BASLANGIC: kapi acilir acilmaz tam komut vermek,
                # yerlesme boyunca kadrajda birikmis hatayi BASAMAK olarak
                # uygulamak demek. Komut 0'dan 1'e rampalanir.
                if self._kapi_acildi_t is None:
                    self._kapi_acildi_t = current_time
                _ramp = 1.0
                if self.yumusak_baslangic_s > 0:
                    _ramp = min(1.0, (current_time - self._kapi_acildi_t)
                                / self.yumusak_baslangic_s)
                cmd_alt_m *= _ramp

                # P1 — YALNIZCA ZEMIN EMNIYETI (2026-07-29, kullanici karari).
                # Ilk surum tavani -5 -> -10 m'ye acmisti; gerekcesi sim'den
                # olculen "dikey hiz = 0.28 * komut" oraniydi. AMA bu oran
                # UCAK MODELINE BAGLI (TECS, tirmanma/alcalma performansi) ve
                # -5 m degeri GERCEK UCUSTA dogrulanmis. Sim olcumuyle gercek
                # degeri degistirmek dogru degil -> tavan -5 m'ye GERI ALINDI.
                # Korunan tek parca ucaktan bagimsiz olan emniyet: zemine
                # yaklasinca yetki kendiliginden daralir (16 m AGL'de -1 m,
                # 15 m'de sifir). Bu, tavani asla artirmaz, yalnizca azaltir.
                if self.agl_olcekli_alcalma:
                    alt_limit = -min(max(current_alt - 15.0, 1.0),
                                     abs(self.min_alt_change_m))
                else:
                    alt_limit = self.min_alt_change_m
                cmd_alt_m = self.clamp(cmd_alt_m, alt_limit, self.max_alt_change_m)
                sat_alt = 1 if (cmd_alt_ham_m > self.max_alt_change_m or
                                cmd_alt_ham_m < alt_limit) else 0

                target_alt = current_alt + cmd_alt_m
                # Zemin güvenliği (10 metre altına inmesini engelle)
                if target_alt < 10.0:
                    target_alt = 10.0

                # HEDEFIN IRTIFASI BURADA OKUNMUYOR (2026-07-29, kullanici karari):
                # gudum sureci hedefin gercek konum/irtifa verisine ERISMEMELI,
                # yoksa testlere farkinda olmadan hile karisir. Karsilastirma
                # verisi ayri calisan tools/gercek_konum_logger.py'den gelir ve
                # yalnizca ANALIZDE (tools/pid_grafik.py --gercek) birlestirilir.

                # Anti-windup
                aw_x, aw_y = 0, 0
                if abs(cmd_head_deg) >= self.max_heading_change_deg:
                    self.integral_error_x *= 0.9; aw_x = 1
                if cmd_alt_m >= self.max_alt_change_m or cmd_alt_m <= alt_limit:
                    self.integral_error_y *= 0.9; aw_y = 1

                command_sent = 0
                # --- DEVIR KAPISI + BOOTSTRAP KORUMASI ---
                aktif = (gorev == "Goruntulu" and self.mavlink.current_mode == "GUIDED")
                hazir_tel = (self.mavlink.attitude_geldi and self.mavlink.pozisyon_geldi)
                if not aktif:
                    self._engage_t = None
                    self._engage_alt = None
                    self._kapi_acildi_t = None
                elif self._engage_t is None:
                    self._engage_t = current_time
                    # Irtifayi KILITLE: yerlesme sirasinda current_alt
                    # komutlamak ucagin kendi hareketini kovalamak demek.
                    self._engage_alt = current_alt
                    self._kapi_acildi_t = None
                    print(f"[devir] GUIDED algilandi — ucak oturana kadar "
                          f"SABITLEME ({self.engage_bekleme_s:.1f}-{self.engage_azami_s:.1f} s)")
                yerlesiyor = False
                if aktif and self.devir_kapisi and self._engage_t is not None:
                    gecen = current_time - self._engage_t
                    vz = abs(getattr(self.mavlink, 'current_vz', 0.0) or 0.0)
                    if vz != vz:
                        vz = 0.0
                    self._pitch_gecmis.append(math.degrees(pitch_now))
                    if len(self._pitch_gecmis) > 80:
                        self._pitch_gecmis.pop(0)
                    pitch_oturdu = (len(self._pitch_gecmis) >= 40 and
                                    (max(self._pitch_gecmis[-40:])
                                     - min(self._pitch_gecmis[-40:])) < 1.5)
                    yerlesiyor = (gecen < self.engage_bekleme_s
                                  or vz > self.engage_vz_esik
                                  or not pitch_oturdu)
                    if gecen > self.engage_azami_s:
                        yerlesiyor = False
                if aktif and (yerlesiyor or not hazir_tel):
                    # Ucak devir gecicisinde (ya da telemetri henuz yok):
                    # PID'i CALISTIRMA, mevcut durumu komutla. Boylece gudum
                    # kendi uretmedigi bir salinima karsi savasmiyor.
                    if current_time - self.last_cmd_send_time >= 0.1 and hazir_tel:
                        hdg = (self._sabit_heading_deg if self._sabit_heading_deg is not None
                               else math.degrees(yaw_now) % 360.0)
                        self.mavlink.send_heading_target(hdg, self.min_heading_rate)
                        if self.sabit_hiz is not None:
                            self._hiz_tohumla()
                            if self._sabit_hiz_ms:
                                self.mavlink.send_speed_target(self._sabit_hiz_ms)
                        self.mavlink.send_altitude_target(self._engage_alt or current_alt)
                        self.last_cmd_send_time = current_time
                    # Kapi acilinca temiz baslamak icin durumu suurekli sifirla
                    self.integral_error_x = self.integral_error_y = 0.0
                    self.prev_error_x, self.prev_error_y = stab_error_x_deg, stab_error_y_deg
                    self.prev_derivative_x = self.prev_derivative_y = 0.0
                    self.prev_error_rate = 0.0
                    self.prev_derivative_rate = 0.0
                    self.last_target_alt = self._engage_alt or current_alt
                    self._log_state(
                        current_time, dt, gorev,
                        "TELEMETRI_BEKLENIYOR" if not hazir_tel else "DEVIR_YERLESME",
                        1, 0, queue_size, data_age_ms,
                        bbox_x, bbox_y, bbox_w, bbox_h, obj_x, obj_y, target_area,
                        stab_x, stab_y, stab_x - obj_x, stab_y - obj_y,
                        current_alt, roll_now, pitch_now, yaw_now,
                        raw_error_x_deg, raw_error_y_deg, stab_error_x_deg, stab_error_y_deg,
                        raw_error_x_deg, raw_error_y_deg, stab_error_x_deg, stab_error_y_deg,
                        0, 0, 0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, self.last_target_heading,
                        0, 0, 0, 0, 0, current_alt, 0, 0)
                    continue
                if current_time - self.last_cmd_send_time >= 0.1:
                    if aktif:
                        if self.sabit_heading is not None:
                            # YATAY EKSEN DONDURULDU (test izolasyonu):
                            # gudumun hesapladigi target_heading GONDERILMEZ.
                            if self._sabit_heading_deg is None:
                                self._sabit_heading_deg = math.degrees(yaw_now) % 360
                                print(f"[test] heading donduruldu: "
                                      f"{self._sabit_heading_deg:.1f} deg")
                            self.mavlink.send_heading_target(
                                self._sabit_heading_deg, self.min_heading_rate)
                        else:
                            self.mavlink.send_heading_target(target_heading, heading_rate)
                        if self.sabit_hiz is not None:
                            self._hiz_tohumla()
                            if self._sabit_hiz_ms:
                                self.mavlink.send_speed_target(self._sabit_hiz_ms)
                        self.mavlink.send_altitude_target(target_alt)
                        command_sent = 1

                        logging.info(
                            f"[{gorev}] OTONOM: Yön: {target_heading:.1f}° | "
                            f"Pitch: {math.degrees(pitch_now):+.2f}° | "
                            f"Ham y: {obj_y:.1f}px | Stab y: {stab_y:.1f}px | "
                            f"Ham Δirtifa: {cmd_alt_ham_m:+.2f}m | İrtifa: {target_alt:.1f}m")

                        # print(f"[{gorev}] OTONOM: HedefYön: {target_heading:.1f}° | Rate: {heading_rate:.1f}°/s | İrtifa: {target_alt:.1f}m")
                    else:
                        print(f"[{gorev}] BEKLEME - Hdf Yön: {target_heading:.1f}° | "
                              f"Pitch: {math.degrees(pitch_now):+.2f}° | Ham y: {obj_y:.1f}px | "
                              f"Stab y: {stab_y:.1f}px | Ham Δirtifa: {cmd_alt_ham_m:+.2f}m | "
                              f"İrtifa: {target_alt:.1f}m")

                    self.last_cmd_send_time = current_time

                self.last_target_heading = target_heading
                self.last_target_alt = target_alt
                self.last_heading_rate = heading_rate

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
                    aw_x, aw_y,
                    rate_error=rate_error, p_rate=p_term_rate, i_rate=i_term_rate,
                    d_rate=d_term_rate, heading_rate_dps=heading_rate,
                    cmd_head_raw_deg=cmd_head_ham_deg, cmd_alt_raw_m=cmd_alt_ham_m,
                    sat_head=sat_head, sat_alt=sat_alt,
                    menzil_est_m=menzil_est, m_per_deg=m_per_deg
                )

        except KeyboardInterrupt:
            print("\nKapatılıyor...")
            self.logger.close()
            self.mavlink.running = False
            self.redis.running = False

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Görüntülü güdüm (heading + irtifa)')
    parser.add_argument('--connect', default=DEFAULT_CONNECTION,
                        help=f'MAVLink bağlantısı (varsayılan: {DEFAULT_CONNECTION})')
    parser.add_argument('--sysid', type=int, default=DEFAULT_SYSID,
                        help=f'Beklenen araç SysID (varsayılan: {DEFAULT_SYSID})')
    parser.add_argument('--mount-pitch', type=float, default=None,
                        help='[ESKI/geriye uyumlu] Toplam kamera pitch telafisi (derece): '
                             'mount_phys + aim toplamini ifade eder, aim = deger - mount_phys '
                             'olarak cozulur. Verilmezse kod icindeki varsayilanlar '
                             '(mount_phys=-1.0, aim=-5.0, toplam=-6.0) kullanilir.')
    parser.add_argument('--mount-phys-pitch', type=float, default=None,
                        help='Kamera ile otopilot arasindaki GERCEK fiziksel montaj acisi '
                             '(derece, varsayilan 0.0 = simde kamera govdeye paralel). '
                             'Gercek ucakta ~-1.0 verilmeli. De-rotasyondan ONCE, govde '
                             'cercevesinde uygulanir.')
    parser.add_argument('--sabit-heading', default=None,
                        help="TEST IZOLASYONU: yatay ekseni dondur. Derece ver "
                             "veya 'auto' (baslangictaki yaw dondurulur). "
                             "Dikey eksen tune edilirken kullanilir.")
    parser.add_argument('--sabit-hiz', default=None,
                        help="TEST IZOLASYONU: hava hizini sabit tut (m/s) veya "
                             "'auto' (devirdeki gercek seyir hizindan tohumlanir). "
                             "'auto' onerilir: AUTO'da 19.6 m/s ucarken 18 "
                             "komutlamak devir salinimini buyutuyor. Zarf 9-22.")
    parser.add_argument('--dikey-mod', choices=('aci', 'menzil'), default=None,
                        help="Dikey eksen kontrol yasasi: 'aci' (eski, sabit "
                             "Kp_alt m/derece) veya 'menzil' (yeni, dh=R*tan(hata); "
                             "menzil kameradan tahmin edilir)")
    parser.add_argument('--devir-kapisi', dest='devir_kapisi', action='store_true', default=None,
                        help='AUTO->GUIDED devir yumusatma kapisi (VARSAYILAN ACIK): '
                             'ucak oturana kadar PID calismaz, irtifa kilitlenir')
    parser.add_argument('--devir-kapisi-kapat', dest='devir_kapisi', action='store_false',
                        help='Devir kapisini kapat (gercek donanimda gerekirse)')
    parser.add_argument('--devir-bekleme', type=float, default=None,
                        help='Devir kapisi asgari bekleme suresi (s, varsayilan 2.0)')
    parser.add_argument('--agl-alcalma', dest='agl_alcalma', action='store_true', default=None,
                        help='P1 (VARSAYILAN ACIK): zemin emniyeti — alcalma yetkisi '
                             '15 m AGL\'e yaklasirken daralir (tavan yine -5 m)')
    parser.add_argument('--agl-alcalma-kapat', dest='agl_alcalma', action='store_false',
                        help='P1 kapat, sabit -5 m clamp\'e don')
    parser.add_argument('--kalite-deadzone', dest='kalite_deadzone', action='store_true', default=None,
                        help='P3 (VARSAYILAN ACIK): bbox buyukken deadzone 0.4 -> 0.15 derece')
    parser.add_argument('--kalite-deadzone-kapat', dest='kalite_deadzone', action='store_false',
                        help='P3 kapat')
    parser.add_argument('--aim-pitch', type=float, default=None,
                        help='AOA (hucum acisi) telafisi (derece, varsayilan -6.0). Ufka '
                             'baglidir; de-rotasyondan SONRA uygulanir.')
    args = parser.parse_args()

    sh = args.sabit_heading
    if sh is not None and sh != 'auto':
        sh = float(sh)
    shz = args.sabit_hiz
    if shz is not None and shz != 'auto':
        shz = float(shz)
    gudum = AutopilotController(connection_str=args.connect, target_sysid=args.sysid,
                                mount_pitch_deg=args.mount_pitch,
                                mount_phys_pitch_deg=args.mount_phys_pitch,
                                aim_pitch_deg=args.aim_pitch,
                                sabit_heading=sh, sabit_hiz=shz)
    if args.dikey_mod:
        gudum.dikey_mod = args.dikey_mod
    if args.devir_kapisi is not None:
        gudum.devir_kapisi = args.devir_kapisi
    if args.devir_bekleme is not None:
        gudum.engage_bekleme_s = args.devir_bekleme
    print(f"[devir] kapi={'ACIK' if gudum.devir_kapisi else 'KAPALI'}"
          + (f"  bekleme={gudum.engage_bekleme_s:.1f} s" if gudum.devir_kapisi else ""))
    if args.agl_alcalma is not None:
        gudum.agl_olcekli_alcalma = args.agl_alcalma
    if args.kalite_deadzone is not None:
        gudum.kalite_deadzone = args.kalite_deadzone
    print(f"[dikey] mod={gudum.dikey_mod}" +
          (f"  Kp_h={gudum.Kp_h} Kd_h={gudum.Kd_h} k={gudum.menzil_k}"
           if gudum.dikey_mod == 'menzil' else f"  Kp_alt={gudum.Kp_alt}"))
    # Kosu ayarini CSV'nin yanina yaz: hangi ayarla ucruldugu sonradan tartisma
    # konusu olmasin (autoresearch: ayar belgelenmezse deneyler kiyaslanamaz).
    try:
        import json as _json
        _cfg = {
            'csv': gudum.logger.log_filename,
            'aim_pitch_deg': gudum.aim_pitch_deg,
            'mount_phys_pitch_deg': gudum.mount_phys_pitch_deg,
            'sabit_heading': sh, 'sabit_hiz': shz,
            'dikey_mod': gudum.dikey_mod, 'Kp_h': gudum.Kp_h, 'Kd_h': gudum.Kd_h,
            'agl_olcekli_alcalma': gudum.agl_olcekli_alcalma,
            'kalite_deadzone': gudum.kalite_deadzone,
            'deadzone_dar_deg': gudum.deadzone_dar_deg,
            'devir_kapisi': gudum.devir_kapisi,
            'engage_bekleme_s': gudum.engage_bekleme_s,
            'yumusak_baslangic_s': gudum.yumusak_baslangic_s,
            'menzil_k': gudum.menzil_k,
            'Kp_heading': gudum.Kp_heading, 'Ki_heading': gudum.Ki_heading,
            'Kd_heading': gudum.Kd_heading,
            'Kp_alt': gudum.Kp_alt, 'Ki_alt': gudum.Ki_alt, 'Kd_alt': gudum.Kd_alt,
            'Kp_rate': gudum.Kp_rate, 'Kd_rate': gudum.Kd_rate,
            'deadzone_deg': gudum.deadzone_deg,
            'max_alt_change_m': gudum.max_alt_change_m,
            'min_alt_change_m': gudum.min_alt_change_m,
            'max_heading_change_deg': gudum.max_heading_change_deg,
            'kamera': {'w': gudum.camera_width, 'h': gudum.camera_height,
                       'fx': gudum.fx, 'fy': gudum.fy,
                       'cx': gudum.center_x, 'cy': gudum.center_y},
        }
        with open(gudum.logger.log_filename + '.config.json', 'w') as _f:
            _json.dump(_cfg, _f, indent=2, ensure_ascii=False)
    except Exception as _e:
        print(f'[uyari] kosu ayari yazilamadi: {_e}')
    gudum.run()
