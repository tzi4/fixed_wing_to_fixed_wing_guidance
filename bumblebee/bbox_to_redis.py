#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import redis
import json
import time
import os
import atexit
import argparse
import datetime
import signal
import threading
import fnmatch
import math

# OpenCV varsayilan 16 thread ile ayni isi ~4x CPU'ya yapiyor (1080p olcum: 7.7 ms
# @411% CPU vs tek thread 10.3 ms @100% CPU); 30 Hz kamera icin tek thread zaten
# 97 fps kapasite veriyor, o yuzden varsayilan 1. BUMBLEBEE_CV_THREADS ile ezilir
# (0 veya negatif = OpenCV varsayilanina don; OpenCV'de setNumThreads(0) "tum
# cekirdekler" degil "sirali" demek oldugu icin -1'e cevriliyor).
_CV_THREADS = int(os.environ.get('BUMBLEBEE_CV_THREADS', '1'))
cv2.setNumThreads(_CV_THREADS if _CV_THREADS > 0 else -1)

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_VIDEO_DIR = os.path.join(SCRIPT_DIR, 'videos')
FONT = cv2.FONT_HERSHEY_SIMPLEX


class TemporalBBoxSelector:
    """Adaylar arasinda kareler arasi sureklilik ve parca birlestirme."""

    def __init__(self, track_timeout_s=0.50):
        self.track_timeout_s = float(track_timeout_s)
        self.last_bbox = None
        self.last_time = None

    @staticmethod
    def _center(bbox):
        x, y, w, h = bbox
        return x + w / 2.0, y + h / 2.0

    def select(self, candidates, image_w, image_h, now=None):
        now = time.monotonic() if now is None else float(now)
        if not candidates:
            if self.last_time is not None and now - self.last_time > self.track_timeout_s:
                self.last_bbox = self.last_time = None
            return None

        selected = None
        if self.last_bbox is not None and self.last_time is not None:
            elapsed = max(0.0, now - self.last_time)
            old_x, old_y = self._center(self.last_bbox)
            motion_gate = max(80.0, 4.0 * max(self.last_bbox[2:]),
                              80.0 + 600.0 * min(elapsed, self.track_timeout_s))
            scored = []
            for bbox in candidates:
                cx, cy = self._center(bbox)
                distance = math.hypot(cx - old_x, cy - old_y)
                if distance <= motion_gate:
                    scored.append((distance, bbox))
            if scored:
                selected = min(scored, key=lambda item: item[0])[1]

        if (selected is None and self.last_time is not None and
                now - self.last_time <= self.track_timeout_s):
            return None
        if selected is None:
            image_cx, image_cy = image_w / 2.0, image_h / 2.0
            selected = min(candidates, key=lambda bbox: math.hypot(
                self._center(bbox)[0] - image_cx,
                self._center(bbox)[1] - image_cy))

        selected_cx, selected_cy = self._center(selected)
        merge_gate = max(50.0, 3.0 * max(selected[2:]))
        nearby = [bbox for bbox in candidates if math.hypot(
            self._center(bbox)[0] - selected_cx,
            self._center(bbox)[1] - selected_cy) <= merge_gate]
        if len(nearby) > 1:
            x1 = min(b[0] for b in nearby)
            y1 = min(b[1] for b in nearby)
            x2 = max(b[0] + b[2] for b in nearby)
            y2 = max(b[1] + b[3] for b in nearby)
            selected = (x1, y1, x2 - x1, y2 - y1)

        self.last_bbox = selected
        self.last_time = now
        return selected


def draw_text(image, text, origin, color, scale=0.6, thickness=2):
    """Okunurluk icin once siyah kontur, sonra renkli metin.

    LINE_8 kullaniliyor: LINE_AA ayni metni ~4x pahali ciziyor (olcum: uzun
    durum satiri 1360 us -> 325 us). Siyah kontur okunurlugu zaten sagliyor,
    kenar yumusatmasi 30 Hz'de bosuna CPU yakiyordu.
    """
    cv2.putText(image, text, origin, FONT, scale, (0, 0, 0), thickness + 2, cv2.LINE_8)
    cv2.putText(image, text, origin, FONT, scale, color, thickness, cv2.LINE_8)


def text_width(text, scale=0.6, thickness=2):
    """draw_text'in kaplayacagi GERCEK genislik (px).

    Olcum siyah konturun kalinligiyla (thickness + 2) yapilir; renkli metin
    onun icinde kalir. Sabit piksel varsaymak yerine hep bu kullanilir.
    """
    return cv2.getTextSize(text, FONT, scale, thickness + 2)[0][0]


# --- "SU AN CALISAN KOD" OVERLAY AYARLARI --------------------------------
# Videoyu sonradan izlerken "burada hangi kod devredeydi" sorusunun cevabi.
# Yeni bir gudum/test script'i eklemek icin YALNIZ bu listeye satir eklenir;
# fnmatch kalibi kullanildigi icin '*' jokeri gecerlidir.
CODE_WATCH_PATTERNS = (
    'formation.py',
    'command_sender.py',
    'goat_cam_offset.py',
    'teva.py',                 # yarisma surumu (1 Hz telemetriden menzil)
    'goat_gimbal_ucak.py',
    'tzi_emir.py',
    'goruntulu_gudum*.py',
    'load_plan.py',
    'verify_flight.py',
    # --- test altyapisi: videoda "o an ne kosuyordu" gorunsun ---
    'test_kurulum.py',
    'hedef_rampa.py',
    'sunucu_simulator.py',
    'gercek_konum_logger.py',
)
CODE_SCAN_PERIOD_S = 1.0   # tarama periyodu: kare basina DEGIL, saniyede bir
CODE_REDIS_KEY = 'aktif_kod'  # bir script kendini acikca duyurmak isterse
CODE_MAX_NAMES = 3         # bundan fazlasi "+N" olarak ozetlenir
CODE_MAX_CHARS = 96        # 1080p'de satirin ekrandan tasmamasi icin
CODE_SELF_NAME = os.path.basename(__file__)  # kendi surecimizi listeleme
CODE_LINE_Y = 88           # "Kod: ..." satirinin taban cizgisi
CODE_LINE_X = 20           # sol kenar bosluğu
UCAK_RIGHT_MARGIN = 20     # ucak adinin sag kenara mesafesi
UCAK_MIN_GAP = 24          # iki metnin arasinda BIRAKILACAK EN AZ bosluk


# --- 3B KOORDINAT OVERLAY'I (SOL ALT) ------------------------------------
# Videoyu sonradan izlerken "o karede avci ve hedef NEREDEYDI" sorusunun cevabi.
# Kaynak YALNIZCA Redis'tir; bu surecte MAVLink baglantisi yoktur ve bilerek
# ACILMAZ (kamera yolu 30 Hz'de tek cekirdek butcesiyle calisiyor; ikinci bir
# link + heartbeat thread'i o butceyi bozar).
#   avci_telemetri  : kendi ucagimiz  (ayri bir yayinci doldurur)
#   rakip_telemetri : hedef ucak      (yarisma sunucusu formati)
# Ikisi de ayni JSON sekli:
#   {"konumBilgileri": [{"iha_enlem": .., "iha_boylam": .., "iha_irtifa": ..}]}
# Anahtar yoksa / bozuksa satir GIZLENMEZ, '-' ile cizilir: kadraj duzeni sabit
# kalir ve izleyici "veri gelmiyor" durumunu gorebilir.
AVCI_REDIS_KEY = 'avci_telemetri'
HEDEF_REDIS_KEY = 'rakip_telemetri'
TELEM_SCAN_PERIOD_S = 1.0   # okuma periyodu: kare basina DEGIL, saniyede bir
COORD_LINE_X = 20           # sol kenar bosluğu
COORD_BOTTOM_MARGIN = 20    # en alt satirin taban cizgisi ile alt kenar arasi
COORD_LINE_GAP = 28         # iki koordinat satiri arasi
COORD_PLACEHOLDER = '-'     # veri yoksa basilan isaret


def _kisa_timeoutlu_redis():
    """Overlay yollari icin AYRI Redis istemcisi.

    Yayin (publish) baglantisina dokunulmaz. Kisa timeout: Redis takilirsa
    overlay thread'i saniyelerce asili kalmasin.
    """
    try:
        return redis.Redis(host='localhost', port=6379, db=0,
                           socket_timeout=0.2, socket_connect_timeout=0.2)
    except Exception:
        return None


# --- OVERLAY'DEKI UCAK ADI ------------------------------------------------
# "Kod: ..." satirinin SAGINA, kadrajin sag kenarina hizali olarak o an ucan
# aracin adi yazilir. Kaynak sirasi:
#   1) BUMBLEBEE_UCAK ortam degiskeni (launcher bunu export eder),
#   2) BUMBLEBEE_HUNTER_MODEL yolundaki model klasoru,
#   3) hicbiri yoksa ad HIC yazilmaz (satir yalnizca "Kod: ..." kalir).
UCAK_ADLARI = {
    'bumblebee': 'Bumblebee',          # 10 kg+ kendi ucagimiz
    'emir_ucak_temp': 'Erenimbus',     # Emir'in 1.5 kg ucagi (A/B kontrol)
}


def resolve_aircraft_name():
    """Overlay'de gosterilecek ucak adi (yoksa None)."""
    name = os.environ.get('BUMBLEBEE_UCAK', '').strip()
    if name:
        return name
    path = os.environ.get('BUMBLEBEE_HUNTER_MODEL', '').strip()
    if not path:
        return None                    # belirsiz: ad yazilmaz
    # DIKKAT: yol icinde arama YAPILMAZ. Depo klasoru de 'bumblebee' oldugu
    # icin (.../gudum/bumblebee/models/emir_ucak_temp/model.sdf) parcalari
    # bastan taramak yanlis eslesme veriyordu. Model klasoru, model.sdf'in
    # DOGRUDAN ust klasorudur; yalnizca o dikkate alinir.
    model_dir = os.path.basename(os.path.dirname(os.path.normpath(path)))
    return UCAK_ADLARI.get(model_dir)


# --- HEDEF RENGI ---------------------------------------------------------
# 2026-07-28: hedef ucak KIRMIZI'dan MOR'a alindi (models/hedef_mor).
#
# NEDEN: kirmizi HSV penceresi arka planla CAKISIYORDU. Gazebo <sky> kubbesi
# ufuk bandinda BGR~(127,127,255) render ediyor; ucak yatinca bu bant tam
# kadraj genisliginde (w~1920) sahte hedef uretiyordu (26 Tem. dalis olayi).
# Zemin dokusu da siyirma acisinda kirmizi-uclu cizgi artefaktlari veriyordu.
# Cozum olarak dunyayi fakirlestirmek (sky/zemin gorselini silmek) yerine
# HEDEFIN RENGI arka planda bulunmayan bir tona alindi; boylece GERCEKCI
# dunyada (gokyuzu + pist + cim VAR) test yapilabiliyor.
#
# OLCUM (3 kayit, 900 ornek kare, 1.99e8 "kromatik" piksel; S>=70,V>=50):
#   H 110-114 (gokyuzu kubbesi) : %54.4  (ust 1/3'te %87.7)
#   H  45- 49 (cim)             : %26.1  (alt 1/3'te %54.4)
#   H  30- 34 (pist)            : %13.8
#   H   0-  4 (KIRMIZI pencere) : % 1.9   <-- eski hedef penceresi
#   H 140-160 (MOR pencere)     : % 0.0012  <-- ~sifir, ayrim payi ~1700x
# Mor pencereye giren o %0.0012'lik kalinti da dagilmis, DUSUK DOYGUNLUKLU
# (S ortancasi ~78) H.264 kroma gurultusudur; S alt siniri 120'ye cekilince
# %95'i eleniyor. Gercek hedef kaynakta RGB(1,0,1) -> S=255 oldugu icin
# golgede bile S>150 kalir.
#
# BUMBLEBEE_TARGET_COLOR=red ile eski kirmizi pencereye BIREBIR donulur.
TARGET_COLOR_WINDOWS = {
    # Gazebo/Purple = RGB(1,0,1) -> OpenCV HSV H=150. Tek pencere yeter.
    'purple': ((140, 160),),
    # Kirmizi H=0'da sarmal yaptigi icin iki pencere gerekir (eski davranis).
    'red': ((0, 10), (170, 180)),
}
TARGET_COLOR_SV = {
    'purple': (120, 60),   # (S_min, V_min) - kroma gurultusunu eler
    'red': (70, 50),       # eski kirmizi esikleri AYNEN korunur
}
DEFAULT_TARGET_COLOR = 'purple'


def build_color_ranges(color, s_min=None, v_min=None):
    """Renk adindan (lower, upper) HSV pencere listesi uretir.

    Bilinmeyen bir ad verilirse uyarip varsayilana doner: renk yazim hatasi
    yuzunden dedektor sessizce hic tespit etmemeli.
    """
    color = (color or '').strip().lower() or DEFAULT_TARGET_COLOR
    if color not in TARGET_COLOR_WINDOWS:
        print(f"UYARI: bilinmeyen BUMBLEBEE_TARGET_COLOR='{color}'; "
              f"gecerli degerler: {', '.join(sorted(TARGET_COLOR_WINDOWS))}. "
              f"'{DEFAULT_TARGET_COLOR}' kullanilacak.")
        color = DEFAULT_TARGET_COLOR
    default_s, default_v = TARGET_COLOR_SV[color]
    s_min = default_s if s_min is None else s_min
    v_min = default_v if v_min is None else v_min
    ranges = [(np.array([lo, s_min, v_min]), np.array([hi, 255, 255]))
              for lo, hi in TARGET_COLOR_WINDOWS[color]]
    return color, ranges


class ActiveCodeTracker:
    """O an calisan gudum/test script'lerini periyodik olarak tespit eder.

    Tarama arka planda ayri bir thread'de ~CODE_SCAN_PERIOD_S'de bir yapilir;
    kare isleme yolu yalnizca hazir bir string'i (self.text) okur, yani kare
    basina ek maliyet bir attribute erisimi kadardir (~0.1 us). Kaynak olarak
    /proc/<pid>/cmdline taranir (subprocess/pgrep fork'u yok).

    Redis'teki CODE_REDIS_KEY anahtari doluysa /proc sonucunun yerine o gecer.
    Redis erisimi de ayni thread'de, yine saniyede bir yapilir; yayin (publish)
    yolundaki baglantiya dokunulmamasi icin AYRI bir istemci kullanilir.
    """

    def __init__(self, redis_client=None, period=CODE_SCAN_PERIOD_S,
                 patterns=CODE_WATCH_PATTERNS):
        self.period = float(period)
        self.patterns = tuple(patterns)
        self.text = 'Kod: -'
        self.redis = redis_client
        self._redis_owned = False
        self._redis_warned = False
        self._stop = threading.Event()
        self._thread = None

    # --- ic yardimcilar -------------------------------------------------
    def _ensure_redis(self):
        if self.redis is not None:
            return self.redis
        self.redis = _kisa_timeoutlu_redis()
        self._redis_owned = self.redis is not None
        return self.redis

    def _redis_override(self):
        client = self._ensure_redis()
        if client is None:
            return None
        try:
            raw = client.get(CODE_REDIS_KEY)
        except Exception as exc:
            if not self._redis_warned:
                self._redis_warned = True
                print(f"UYARI: '{CODE_REDIS_KEY}' okunamadi, surec taramasina "
                      f"devam ediliyor ({exc}).")
            return None
        if raw is None:
            return None
        if isinstance(raw, bytes):
            raw = raw.decode('utf-8', 'replace')
        raw = raw.strip()
        return raw or None

    def scan_processes(self):
        """Calisan surecler icinde CODE_WATCH_PATTERNS eslesmelerini dondurur."""
        my_pid = os.getpid()
        found = set()
        try:
            entries = os.listdir('/proc')
        except OSError:
            return []
        for entry in entries:
            if not entry.isdigit() or int(entry) == my_pid:
                continue
            try:
                with open('/proc/' + entry + '/cmdline', 'rb') as fh:
                    raw = fh.read()
            except (OSError, IOError):
                continue          # surec bu arada oldu ya da izin yok
            if b'.py' not in raw:
                continue          # ucuz on eleme: surectlerin cogu burada elenir
            for token in raw.split(b'\0'):
                if not token.endswith(b'.py') or len(token.split()) != 1:
                    # Bosluk iceren token = `bash -c "... python3 x/formation.py"`
                    # gibi bir sarmalayici komut dizisi, gercek bir argv yolu degil.
                    continue
                name = os.path.basename(token.decode('utf-8', 'replace'))
                if name == CODE_SELF_NAME:
                    continue
                found.add(name)
        if not found:
            return []
        # Kalip sirasini koru: overlay satiri kareden kareye yer degistirmesin.
        ordered = []
        for pattern in self.patterns:
            for name in sorted(found):
                if name not in ordered and fnmatch.fnmatchcase(name, pattern):
                    ordered.append(name)
        return ordered

    @staticmethod
    def format_line(names):
        if not names:
            return 'Kod: -'
        shown = list(names[:CODE_MAX_NAMES])
        extra = len(names) - len(shown)
        text = 'Kod: ' + ', '.join(shown)
        if extra > 0:
            text += f" +{extra}"
        if len(text) > CODE_MAX_CHARS:
            text = text[:CODE_MAX_CHARS - 3] + '...'
        return text

    # --- dis arayuz -----------------------------------------------------
    def refresh(self):
        override = self._redis_override()
        if override:
            text = 'Kod: ' + override
            if len(text) > CODE_MAX_CHARS:
                text = text[:CODE_MAX_CHARS - 3] + '...'
        else:
            text = self.format_line(self.scan_processes())
        self.text = text          # tek atama: kare yolu her zaman tutarli okur
        return text

    def start(self):
        if self._thread is not None:
            return
        try:
            self.refresh()        # ilk kareler de dogru olsun
        except Exception as exc:
            print(f"UYARI: aktif kod taramasi basarisiz: {exc}")
        self._thread = threading.Thread(target=self._loop, name='aktif-kod', daemon=True)
        self._thread.start()

    def _loop(self):
        while not self._stop.wait(self.period):
            try:
                self.refresh()
            except Exception:
                pass              # overlay hicbir kosulda gudumu bozmamali

    def stop(self):
        self._stop.set()
        if self._redis_owned and self.redis is not None:
            try:
                self.redis.close()
            except Exception:
                pass


class TelemetriTakipci:
    """Avci ve hedef 3B konumunu Redis'ten periyodik okuyup HAZIR STRING tutar.

    ActiveCodeTracker ile BIREBIR ayni desen: Redis erisimi ve JSON parse'i
    arka planda ayri bir thread'de ~TELEM_SCAN_PERIOD_S'de bir yapilir. Kare
    isleme yolu yalnizca self.avci_text / self.hedef_text attribute'larini
    okur; kare basina JSON parse ya da Redis cagrisi YOKTUR (30 Hz x 2 anahtar
    = saniyede 60 gidis-donus olurdu, saniyede 2 ile yetiniyoruz).

    Kaynak bulunamazsa metin yine uretilir, degerler '-' olur: overlay duzeni
    kareden kareye ziplamaz ve veri yoklugu izleyiciye acikca gorunur.
    """

    def __init__(self, redis_client=None, period=TELEM_SCAN_PERIOD_S):
        self.period = float(period)
        self.redis = redis_client
        self._redis_owned = False
        self._warned = set()
        self._stop = threading.Event()
        self._thread = None
        # Ilk kareler icin bos (dash) satirlar hazir dursun.
        self.avci_text = self._bicimle('Avci ', (None, None, None))
        self.hedef_text = self._bicimle('Hedef', (None, None, None))

    # --- ic yardimcilar -------------------------------------------------
    def _ensure_redis(self):
        if self.redis is not None:
            return self.redis
        self.redis = _kisa_timeoutlu_redis()
        self._redis_owned = self.redis is not None
        return self.redis

    @staticmethod
    def _konum_ayikla(raw):
        """Ham Redis degerinden (enlem, boylam, irtifa); olmayan alan None."""
        if not raw:
            return (None, None, None)
        if isinstance(raw, bytes):
            raw = raw.decode('utf-8', 'replace')
        data = json.loads(raw)
        # Yarisma sunucusu formati: konumBilgileri dizisinin ILK elemani.
        # Bazi yayincilar alanlari kok seviyede duz veriyor; ikisi de kabul.
        konum = data
        if isinstance(data, dict):
            liste = data.get('konumBilgileri')
            if isinstance(liste, list) and liste and isinstance(liste[0], dict):
                konum = liste[0]
        if not isinstance(konum, dict):
            return (None, None, None)

        def _sayi(anahtar):
            try:
                return float(konum.get(anahtar))
            except (TypeError, ValueError):
                return None

        return (_sayi('iha_enlem'), _sayi('iha_boylam'), _sayi('iha_irtifa'))

    @staticmethod
    def _bicimle(etiket, konum):
        """Tek satirlik overlay metni. Eksik alan COORD_PLACEHOLDER olur.

        Enlem/boylam 6 hane: ~0.1 m cozunurluk, hedef ayirt etmek icin fazlasi
        gereksiz ve satiri uzatiyor.
        """
        enlem, boylam, irtifa = konum
        e = f"{enlem:.6f}" if enlem is not None else COORD_PLACEHOLDER
        b = f"{boylam:.6f}" if boylam is not None else COORD_PLACEHOLDER
        i = f"{irtifa:.0f} m" if irtifa is not None else COORD_PLACEHOLDER
        return f"{etiket}: Enl {e}  Boy {b}  Irt {i}"

    def _oku(self, client, key):
        try:
            return self._konum_ayikla(client.get(key))
        except Exception as exc:
            if key not in self._warned:
                self._warned.add(key)
                print(f"UYARI: '{key}' okunamadi/cozulemedi, koordinat satiri "
                      f"'{COORD_PLACEHOLDER}' gosterilecek ({exc}).")
            return (None, None, None)

    # --- dis arayuz -----------------------------------------------------
    def refresh(self):
        client = self._ensure_redis()
        bos = (None, None, None)
        avci = self._oku(client, AVCI_REDIS_KEY) if client is not None else bos
        hedef = self._oku(client, HEDEF_REDIS_KEY) if client is not None else bos
        # Tek atama (iki bagimsiz string): kare yolu her zaman tutarli okur.
        self.avci_text = self._bicimle('Avci ', avci)
        self.hedef_text = self._bicimle('Hedef', hedef)

    def start(self):
        if self._thread is not None:
            return
        try:
            self.refresh()        # ilk kareler de dogru olsun
        except Exception as exc:
            print(f"UYARI: koordinat okumasi basarisiz: {exc}")
        self._thread = threading.Thread(target=self._loop, name='koordinat', daemon=True)
        self._thread.start()

    def _loop(self):
        while not self._stop.wait(self.period):
            try:
                self.refresh()
            except Exception:
                pass              # overlay hicbir kosulda gudumu bozmamali

    def stop(self):
        self._stop.set()
        if self._redis_owned and self.redis is not None:
            try:
                self.redis.close()
            except Exception:
                pass


class VideoRecorder:
    """bbox cizili kareleri MP4/AVI'ye yazar.

    Display'den tamamen bagimsizdir (headless'ta da calisir). Codec once 'mp4v'
    (.mp4) denenir, VideoWriter acilmazsa 'XVID' (.avi) fallback'ine dusulur;
    ikisi de acilmazsa kayit sessizce bos dosya birakmak yerine devre disi kalir.
    """

    WARMUP_FRAMES = 5          # baslangictaki patlama kareleri olcumu bozuyor, atilir
    PROBE_FRAMES = 30          # fps otomatik olcumu icin ornek kare sayisi
    PROBE_MIN_S = 1.0          # en az bu kadar sureye yayilan bir olcum istiyoruz
    PROBE_TIMEOUT_S = 4.0      # yavas yayinda olcumu sonsuza kadar bekletme

    def __init__(self, path=None, fps=0.0):
        self.requested_path = path or None
        self.target_fps = float(fps) if fps and fps > 0 else 0.0
        self.writer = None
        self.path = None
        self.fourcc_name = None
        self.fps = 0.0
        self.size = None
        self.frames = 0
        self.dropped = 0
        self.failed = False
        self.closed = False
        self.lock = threading.Lock()
        self._probe_t0 = None
        self._probe_count = 0
        self._warmup_seen = 0
        self._last_write = 0.0

    # --- ic yardimcilar -------------------------------------------------
    def _resolve_path(self, suffix):
        if self.requested_path:
            path = os.path.abspath(os.path.expanduser(self.requested_path))
            if os.path.isdir(path):
                name = 'gudum_' + datetime.datetime.now().strftime('%Y%m%d_%H%M%S') + suffix
                return os.path.join(path, name)
            root, _ext = os.path.splitext(path)
            return root + suffix
        name = 'gudum_' + datetime.datetime.now().strftime('%Y%m%d_%H%M%S') + suffix
        return os.path.join(DEFAULT_VIDEO_DIR, name)

    def _open(self, size, fps):
        for fourcc_name, suffix in (('mp4v', '.mp4'), ('XVID', '.avi')):
            path = self._resolve_path(suffix)
            try:
                os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
            except OSError as exc:
                print(f"Video klasoru olusturulamadi ({path}): {exc}")
                self.failed = True
                return False
            writer = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*fourcc_name), fps, size)
            if writer.isOpened():
                self.writer = writer
                self.path = path
                self.fourcc_name = fourcc_name
                self.fps = fps
                self.size = size
                print(f"Video kaydi basladi: {path} ({fourcc_name}, {size[0]}x{size[1]}, {fps:.1f} fps)")
                return True
            writer.release()
            # Codec acilmadiysa OpenCV bazen 0 baytlik dosya birakir; temizle.
            try:
                if os.path.isfile(path) and os.path.getsize(path) == 0:
                    os.remove(path)
            except OSError:
                pass
            print(f"UYARI: '{fourcc_name}' codec'i acilmadi ({path}), sonraki codec deneniyor.")
        print("HATA: hicbir video codec'i acilamadi; kayit devre disi.")
        self.failed = True
        return False

    # --- dis arayuz -----------------------------------------------------
    def write(self, frame):
        if self.failed or self.closed or frame is None:
            return
        now = time.time()
        with self.lock:
            if self.closed:
                return
            if self.writer is None:
                height, width = frame.shape[:2]
                if self.target_fps > 0:
                    if not self._open((width, height), self.target_fps):
                        return
                else:
                    # fps verilmediyse gercek yayin hizini olc (ilk ~1.5 s atlanir).
                    if self._warmup_seen < self.WARMUP_FRAMES:
                        self._warmup_seen += 1
                        return
                    if self._probe_t0 is None:
                        self._probe_t0 = now
                        self._probe_count = 0
                        return
                    self._probe_count += 1
                    elapsed = now - self._probe_t0
                    enough = (self._probe_count >= self.PROBE_FRAMES and elapsed >= self.PROBE_MIN_S)
                    if not enough and elapsed < self.PROBE_TIMEOUT_S:
                        return
                    measured = (self._probe_count / elapsed) if elapsed > 0 else 30.0
                    fps = min(max(round(measured, 1), 1.0), 60.0)
                    if not self._open((width, height), fps):
                        return
            # Acik fps istendiyse kareleri o hiza seyrelt (gercek zamanli oynatma + kucuk dosya).
            if self.target_fps > 0 and self._last_write:
                if (now - self._last_write) < (0.9 / self.target_fps):
                    self.dropped += 1
                    return
            if self.size and (frame.shape[1], frame.shape[0]) != self.size:
                frame = cv2.resize(frame, self.size)
            self.writer.write(frame)
            self._last_write = now
            self.frames += 1

    def close(self):
        with self.lock:
            if self.closed:
                return
            self.closed = True
            writer, self.writer = self.writer, None
        if writer is not None:
            writer.release()
            size_bytes = os.path.getsize(self.path) if self.path and os.path.isfile(self.path) else 0
            print(f"Video kaydi kapatildi: {self.path} "
                  f"({self.frames} kare, {self.fps:.1f} fps, {size_bytes} bayt, "
                  f"{self.dropped} kare seyreltildi)")
        elif not self.failed and self.frames == 0:
            print("Video kaydi: hic kare yazilmadi (goruntu gelmedi mi?).")


class SimRedisDetector:
    def __init__(self, display=True, recorder=None, code_overlay=True,
                 coord_overlay=True):
        # Görselleştirme yalnız bir DISPLAY varsa ve kapatılmadıysa açılır.
        self.display = display
        # Video kaydı display'den bağımsızdır; headless'ta da çizim yapılır.
        self.recorder = recorder
        self.draw = display or (recorder is not None)
        # "Kod: ..." satırı yalnız çizim yapılıyorsa anlamlı; tarama thread'i de
        # yalnız o zaman başlar.
        self.code_tracker = ActiveCodeTracker() if (self.draw and code_overlay) else None
        # Sol alt 3B koordinat satirlari da yalniz cizim yapiliyorsa anlamli;
        # okuma thread'i de yalniz o zaman baslar (headless yayin yolu aynen kalir).
        self.telemetri = TelemetriTakipci() if (self.draw and coord_overlay) else None
        # --- REDIS BAĞLANTISI ---
        print("Redis sunucusuna bağlanılıyor...")
        try:
            self.r = redis.Redis(host='localhost', port=6379, db=0)
            # Sistemin görüntülü modda çalışması için görevi ayarlayalım
            self.r.set('gorev', 'Goruntulu')
            print("Redis bağlantısı başarılı! Yayın kanalı: 'tracker_bbox'")
        except Exception as e:
            print(f"Redis Bağlantı Hatası: {e}")
            exit(1)

        # --- ROS BAĞLANTISI ---
        rospy.init_node('sim_redis_detector', anonymous=True)
        self.bridge = CvBridge()
        # Simülasyondaki kamera topic'iniz (Eski kodunuzdan alındı)
        self.image_sub = rospy.Subscriber("/webcam/image_raw", Image, self.image_callback)
        print("ROS Node başlatıldı, /webcam/image_raw kanalından görüntü bekleniyor...")

        # --- RENK AYARLARI (varsayilan MOR; gerekce icin yukaridaki blok) ---
        # BUMBLEBEE_TARGET_COLOR=red -> eski kirmizi davranisa BIREBIR donus.
        # BUMBLEBEE_HSV_SMIN / _VMIN verilirse rengin varsayilan S/V alt
        # sinirini ezer (uzak mesafede hedef kacirilirsa gevsetmek icin).
        def _opt_int(name):
            raw = os.environ.get(name, '').strip()
            if not raw:
                return None
            try:
                return max(0, min(255, int(float(raw))))
            except ValueError:
                print(f"UYARI: {name} gecersiz ({raw}), yoksayildi.")
                return None

        self.target_color, self.color_ranges = build_color_ranges(
            os.environ.get('BUMBLEBEE_TARGET_COLOR', DEFAULT_TARGET_COLOR),
            _opt_int('BUMBLEBEE_HSV_SMIN'), _opt_int('BUMBLEBEE_HSV_VMIN'))
        pencere = ' + '.join(f"H[{lo[0]}-{hi[0]}] S>={lo[1]} V>={lo[2]}"
                             for lo, hi in self.color_ranges)
        print(f"Hedef rengi: {self.target_color.upper()}  ->  {pencere}")
        self.selector = TemporalBBoxSelector()

        # --- OPSIYONEL SEKIL KAPISI (VARSAYILAN KAPALI) ---
        # Gazebo render'i, ~2 km'den sonra yer duzleminde 1 piksel yuksekliginde,
        # duzenli araliklarla tekrar eden ve ucaklarin renklerini tasiyan cizgi
        # artefaktlari uretiyor. Dilate'den sonra bunlar h~5 px'lik
        # konturlara donusuyor ve en-buyuk-kontur secimi gercek hedef
        # yerine bu YER PARAZITINI seciyor. Iki zarf olcutu bunlari eler:
        #   BUMBLEBEE_MIN_BLOB_H    : bbox yuksekligi (px) alt siniri
        #   BUMBLEBEE_MIN_BLOB_FILL : kontur alani / bbox alani alt siniri
        #                             (egik ince cizgiler icin dusuk cikar)
        # Degiskenler verilmezse davranis BIREBIR eskisi gibidir.
        self.min_blob_h = float(os.environ.get('BUMBLEBEE_MIN_BLOB_H', '0') or 0)
        self.min_blob_fill = float(os.environ.get('BUMBLEBEE_MIN_BLOB_FILL', '0') or 0)
        if self.min_blob_h > 0 or self.min_blob_fill > 0:
            print(f"Sekil kapisi acik: min_h={self.min_blob_h:.0f} px, "
                  f"min_doluluk={self.min_blob_fill:.2f}")

        # --- "SU AN CALISAN KOD" SATIRI + UCAK ADI ---
        # Ucak adi hazir bir string; kare basina yalnizca cizilir (olcum
        # onbellekli). Olcum/publish yoluna etkisi SIFIR, yalniz canvas.
        self.ucak_label = None
        self._fit_cache = {}
        if self.draw:
            ucak = resolve_aircraft_name()
            self.ucak_label = f"[{ucak}]" if ucak else None
            print(f"Ucak adi overlay'i: {self.ucak_label or '- (BUMBLEBEE_UCAK verilmedi)'}")
        if self.code_tracker is not None:
            self.code_tracker.start()
            print(f"Aktif kod overlay'i acik ({CODE_SCAN_PERIOD_S:.0f} s'de bir tarama, "
                  f"'{CODE_REDIS_KEY}' anahtari onceliklidir) -> {self.code_tracker.text}")
        if self.telemetri is not None:
            self.telemetri.start()
            print(f"3B koordinat overlay'i acik ({TELEM_SCAN_PERIOD_S:.0f} s'de bir "
                  f"'{AVCI_REDIS_KEY}' / '{HEDEF_REDIS_KEY}' okunuyor)")
            print(f"  {self.telemetri.avci_text}")
            print(f"  {self.telemetri.hedef_text}")

    def _fit_code_line(self, code_text, w_img):
        """'Kod: ...' satirini ucak adiyla CAKISMAYACAK sekilde kirp.

        Ucak adi sag kenara hizalanir; koda kalan genislik olculur ve metin
        gerekirse '...' ile kisaltilir (format_line'daki '+N' ozetlemesinin
        piksel tabanli devami). Sonuc (kod_metni, ucak_x) olarak onbellege
        alinir: code_text saniyede bir degistigi icin kare basina maliyet
        bir sozluk aramasidir, cv2.getTextSize kare basina CAGRILMAZ.
        """
        key = (code_text, w_img)
        cached = self._fit_cache.get(key)
        if cached is not None:
            return cached
        if not self.ucak_label:
            result = (code_text, None)
        else:
            label_w = text_width(self.ucak_label)
            ucak_x = max(CODE_LINE_X, w_img - UCAK_RIGHT_MARGIN - label_w)
            avail = ucak_x - UCAK_MIN_GAP - CODE_LINE_X
            if avail <= 0:
                result = ('', ucak_x)          # kadraj cok dar: yalniz ucak adi
            elif text_width(code_text) <= avail:
                result = (code_text, ucak_x)
            else:
                lo, hi = 0, len(code_text)     # en uzun sigan on-ek: ikili arama
                while lo < hi:
                    mid = (lo + hi + 1) // 2
                    if text_width(code_text[:mid] + '...') <= avail:
                        lo = mid
                    else:
                        hi = mid - 1
                result = ((code_text[:lo] + '...') if lo else '', ucak_x)
        if len(self._fit_cache) > 32:          # sinirsiz buyumesin
            self._fit_cache.clear()
        self._fit_cache[key] = result
        return result

    def image_callback(self, data):
        try:
            # ROS mesajını OpenCV formatına çevir
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

        h_img, w_img, _ = cv_image.shape

        # --- Hedef Vuruş Alanı (Sarı Kutu) Hesaplaması ---
        # Yatayda %25, Dikeyde %10 boşluk bırakarak iç kutuyu tanımlıyoruz
        av_left = int(w_img * 0.25)
        av_right = int(w_img * 0.75)
        av_top = int(h_img * 0.10)
        av_bottom = int(h_img * 0.90)

        # Tespit HER ZAMAN ham kare üzerinde yapılır; çizimler ayrı bir kopyaya
        # (canvas) uygulanır. Böylece video kaydı/pencere açık olması yayınlanan
        # tracker_bbox verisini hiçbir şekilde etkilemez.
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        # Renk penceresi: mor tek, kirmizi iki parca (H=0'da sarmal yapar).
        lower, upper = self.color_ranges[0]
        mask = cv2.inRange(hsv, lower, upper)
        for lower, upper in self.color_ranges[1:]:
            mask = mask + cv2.inRange(hsv, lower, upper)
        mask = cv2.dilate(mask, None, iterations=2)

        # Konturları (şekilleri) bul
        contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Opsiyonel sekil kapisi (yukaridaki aciklamaya bak; varsayilan KAPALI).
        if contours and (self.min_blob_h > 0 or self.min_blob_fill > 0):
            kept = []
            for c in contours:
                _x, _y, _w, _h = cv2.boundingRect(c)
                if _h < self.min_blob_h:
                    continue
                if self.min_blob_fill > 0:
                    box_area = float(_w * _h)
                    if box_area <= 0 or (cv2.contourArea(c) / box_area) < self.min_blob_fill:
                        continue
                kept.append(c)
            contours = kept

        valid_detection = False
        kilitlenme_uygun = False
        bbox = []

        if len(contours) > 0:
            candidates = [cv2.boundingRect(c) for c in contours
                          if cv2.contourArea(c) > 10]
            selected = self.selector.select(candidates, w_img, h_img)

            if selected is not None:
                x, y, w, h = selected
                valid_detection = True

                # --- Kilitlenme Şartlarının Kontrolü ---
                # Şart 1: Tespit edilen hedef kutusu tamamen Sarı Kutunun (Av) içinde mi?
                is_inside = (x >= av_left) and ((x + w) <= av_right) and (y >= av_top) and ((y + h) <= av_bottom)

                # Şart 2: Hedef yatayda %5 veya dikeyde %5 alan kaplıyor mu?
                is_large_enough = (w >= (w_img * 0.05)) or (h >= (h_img * 0.05))

                # İki şart da sağlanıyorsa kilitlenme uygundur
                if is_inside and is_large_enough:
                    kilitlenme_uygun = True

                # Otopilotunuzun beklediği format için yatay kapsama hesapla
                horizontal_coverage = (w / w_img) * 100
                validity_flag = 1

                # Yeni otopilotunuz bu formatı bekliyor: [x, y, w, h, horizontal_cov, validity]
                bbox = [int(x), int(y), int(w), int(h), horizontal_coverage, validity_flag]

                # Redis üzerinden yayınla (json formatında — guidance kodu json.loads ile parse eder)
                self.r.publish('tracker_bbox', json.dumps(bbox))

        if not valid_detection:
            # Hedef yoksa boş veya validity=0 olan bir liste basabilirsiniz
            pass

        # --- ÇİZİM + OVERLAY (yalnız pencere ve/veya video kaydı için) ---
        if not self.draw:
            return

        canvas = cv_image.copy()
        # Hedef Vuruş Alanını (Av) çiz (Sarı Renk)
        cv2.rectangle(canvas, (av_left, av_top), (av_right, av_bottom), (0, 255, 255), 2)
        # Etiket kutunun ALTINA yazılır; üstte sol-üst overlay bloğuyla çakışıyordu.
        draw_text(canvas, "Hedef Vurus Alani", (av_left, min(av_bottom + 22, h_img - 6)),
                  (0, 255, 255), 0.5, 1)

        if valid_detection:
            x, y, w, h = bbox[0], bbox[1], bbox[2], bbox[3]
            cv2.rectangle(canvas, (x, y), (x + w, y + h), (0, 0, 255), 2)
            cv2.circle(canvas, (int(x + w / 2), int(y + h / 2)), 5, (0, 255, 0), -1)
            if kilitlenme_uygun:
                draw_text(canvas, "KILITLENME UYGUN",
                          (int(w_img / 2) - 150, int(h_img / 2) + 150), (0, 255, 0), 1.0, 3)

        # Sol üst köşe overlay'i: zaman damgası + tespit durumu (yalnız görüntüye,
        # yayınlanan bbox verisine dokunulmaz).
        # Tek now() cagrisi: iki ayri cagri saniye ile salise arasinda tutarsizlik
        # uretebiliyordu (23:59:59.99 -> 00:00:00.00).
        now = datetime.datetime.now()
        stamp = now.strftime('%Y-%m-%d %H:%M:%S.') + f"{now.microsecond // 10000:02d}"
        draw_text(canvas, stamp, (20, 32), (255, 255, 255), 0.6, 2)
        if valid_detection:
            status = (f"Hedef({self.target_color}): BULUNDU  x={bbox[0]} y={bbox[1]} w={bbox[2]} h={bbox[3]} "
                      f"kaps={bbox[4]:.1f}%" + ("  [KILIT UYGUN]" if kilitlenme_uygun else ""))
            color = (0, 255, 0)
        else:
            status = f"Hedef({self.target_color}): YOK"
            color = (0, 0, 255)
        draw_text(canvas, status, (20, 60), color, 0.6, 2)
        # O an calisan gudum/test script'leri (yalnizca canvas'a; hazir string,
        # tarama arka plan thread'inde).
        if self.code_tracker is not None:
            code_text, ucak_x = self._fit_code_line(self.code_tracker.text, w_img)
            draw_text(canvas, code_text, (CODE_LINE_X, CODE_LINE_Y), (255, 255, 0), 0.6, 2)
            if ucak_x is not None:
                draw_text(canvas, self.ucak_label, (ucak_x, CODE_LINE_Y), (255, 255, 0), 0.6, 2)

        # --- SOL ALT: AVCI VE HEDEF 3B KONUMU ---
        # Iki metin saniyede bir ARKA PLAN thread'inde uretilir (Redis get +
        # json.loads orada); burada SADECE CIZILIR. Yani kare basina eklenen
        # is = 2 attribute okumasi + 2 draw_text (4 putText, LINE_8).
        #
        # OLCUM (bu makine, 1080p, cv2 tek thread, 1500 tekrar):
        #   2 satir @ scale 0.6 / thickness 2 = 0.69 ms/kare
        #   -> 33.3 ms'lik 30 Hz kare butcesinin %2.1'i, ~%2 CPU
        #   -> ayni kareyi ureten tespit yoluna (cvtColor+inRange+dilate+
        #      findContours, olculen 8.8 ms) gore %8
        # Buyukluk mertebesi zaten her karede cizilen durum satirininkiyle ayni
        # (dosya basindaki not: LINE_8 ile 325 us/satir), yani yeni bir maliyet
        # sinifi getirmiyor. Ileride bant gerekirse thickness=1 olcumu yariya
        # indiriyor (0.41 ms) - okunurluk icin simdilik 2'de birakildi.
        # Kare basina Redis cagrisi ya da JSON parse'i YOKTUR.
        if self.telemetri is not None:
            hedef_y = h_img - COORD_BOTTOM_MARGIN
            draw_text(canvas, self.telemetri.avci_text,
                      (COORD_LINE_X, hedef_y - COORD_LINE_GAP), (255, 255, 255), 0.6, 2)
            draw_text(canvas, self.telemetri.hedef_text,
                      (COORD_LINE_X, hedef_y), (255, 255, 255), 0.6, 2)

        if self.recorder is not None:
            self.recorder.write(canvas)

        # Görüntüyü ekranda göster
        if self.display:
            cv2.imshow("Simulasyon Redis Dedektoru", canvas)
            cv2.waitKey(1)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Kamera → tracker_bbox (Redis) renk tespit köprüsü')
    parser.add_argument('--no-display', action='store_true',
                        help='OpenCV penceresini açma (headless). DISPLAY yoksa otomatik uygulanır.')
    parser.add_argument('--record', nargs='?', const='', default=None, metavar='DOSYA',
                        help='bbox çizili kareleri videoya kaydet. Dosya verilmezse '
                             'videos/gudum_YYYYmmdd_HHMMSS.mp4. BUMBLEBEE_VIDEO=1 ile aynı etki.')
    parser.add_argument('--record-fps', type=float, default=0.0, metavar='FPS',
                        help='Kayıt fps (varsayılan 0 = gelen yayından otomatik ölç). '
                             'Verilirse kareler bu hıza seyreltilir (daha küçük dosya).')
    parser.add_argument('--no-code-overlay', action='store_true',
                        help='"Kod: ..." satırını (o an çalışan güdüm/test script\'leri) '
                             'çizme. BUMBLEBEE_CODE_OVERLAY=0 ile aynı etki.')
    parser.add_argument('--no-coord-overlay', action='store_true',
                        help='Sol alttaki "Avci/Hedef" 3B koordinat satırlarını çizme. '
                             'BUMBLEBEE_COORD_OVERLAY=0 ile aynı etki.')
    args = parser.parse_args()

    # Kayıt: --record bayrağı VEYA BUMBLEBEE_VIDEO env değişkeni (launcher yolu).
    env_video = os.environ.get('BUMBLEBEE_VIDEO', '').strip().lower()
    record_enabled = (args.record is not None) or (env_video not in ('', '0', 'false', 'no', 'off'))
    env_fps = os.environ.get('BUMBLEBEE_VIDEO_FPS', '').strip()
    record_fps = args.record_fps
    if record_fps <= 0 and env_fps:
        try:
            record_fps = float(env_fps)
        except ValueError:
            print(f"UYARI: BUMBLEBEE_VIDEO_FPS geçersiz ({env_fps}), otomatik ölçüm kullanılacak.")

    recorder = VideoRecorder(args.record or None, record_fps) if record_enabled else None
    if recorder is not None:
        # Yarım kalmış MP4 üretmemek için üç ayrı güvenlik ağı: atexit, rospy
        # shutdown kancası ve finally. close() tekrar çağrımına karşı korumalı.
        atexit.register(recorder.close)

    # "Kod: ..." satırı VARSAYILAN AÇIK; --no-code-overlay veya
    # BUMBLEBEE_CODE_OVERLAY=0/false/no/off ile kapatılır.
    env_code = os.environ.get('BUMBLEBEE_CODE_OVERLAY', '').strip().lower()
    code_overlay = (not args.no_code_overlay) and (env_code not in ('0', 'false', 'no', 'off'))

    # Sol alt koordinat satırları da VARSAYILAN AÇIK; --no-coord-overlay veya
    # BUMBLEBEE_COORD_OVERLAY=0/false/no/off ile kapatılır.
    env_coord = os.environ.get('BUMBLEBEE_COORD_OVERLAY', '').strip().lower()
    coord_overlay = (not args.no_coord_overlay) and (env_coord not in ('0', 'false', 'no', 'off'))

    # DISPLAY yoksa ya da --no-display verildiyse pencere açma.
    display = (not args.no_display) and bool(os.environ.get('DISPLAY'))
    detector = None
    try:
        detector = SimRedisDetector(display=display, recorder=recorder,
                                    code_overlay=code_overlay,
                                    coord_overlay=coord_overlay)
        # rospy yalnız SIGINT'i yakalar; launcher/kapat.sh SIGTERM gönderdiğinde de
        # writer'ın düzgün release edilmesi için SIGTERM'i düzenli kapanışa çeviriyoruz.
        signal.signal(signal.SIGTERM, lambda _signum, _frame: rospy.signal_shutdown('SIGTERM'))
        if recorder is not None:
            rospy.on_shutdown(recorder.close)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        if detector is not None and detector.code_tracker is not None:
            detector.code_tracker.stop()
        if detector is not None and detector.telemetri is not None:
            detector.telemetri.stop()
        if recorder is not None:
            recorder.close()
        if display:
            cv2.destroyAllWindows()
