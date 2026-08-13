#!/usr/bin/env python3
"""
detectCV.py — OpenCV + SiamRPN Hedef Tespit & Takip Sistemi (v5)
=================================================================
OpenCV HSV renk filtreleme ile hedef uçağı tespit eder (YOLO yerine).
SiamRPN tracker ile kareler arası takip sağlar.
Redis 'tracker_bbox' kanalına tam uyumlu formatta yayın yapar.

Mimari (detect.sh pipeline'ının birebir OpenCV karşılığı):
  detect.sh:    YOLO → b_box KV → SiamRPN tracker → pub tracker_bbox
  detectCV.sh:  OpenCV renk → SiamRPN tracker → pub tracker_bbox

ÖNEMLİ: Redis'e HER ZAMAN tracker bbox'ı yayınlanır.
         OpenCV sadece tracker'ı başlatmak ve doğrulamak için kullanılır.
         Bu sayede coverage/bbox değerleri tutarlı kalır, dalgalanma olmaz.

v5 Değişiklikleri:
  - HER ZAMAN tracker bbox yayınlanır (OpenCV/tracker geçiş dalgalanması giderildi)
  - OpenCV rolü: tespit (init) + periyodik doğrulama (re-init)
  - Ekranda tek kutu (tracker bbox) — kafa karıştıran çift kutu kaldırıldı
  - detect.sh pipeline mantığının birebir karşılığı
"""

import os
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import redis
import time
import signal
import sys
import json

# Script'in bulunduğu klasör (detectCV_env/)
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

from siamrpn import TrackerSiamRPN


class OpenCVSiamRPNDetector:
    def __init__(self):
        # --- REDIS BAĞLANTISI ---
        print("=" * 60)
        print("  OpenCV + SiamRPN Hedef Tespit & Takip Sistemi v5")
        print("  OpenCV = bul/doğrula | SiamRPN = takip et/yayınla")
        print("=" * 60)
        print()
        print("[INIT] Redis sunucusuna bağlanılıyor...")
        try:
            self.r = redis.Redis(host='localhost', port=6379, db=0)
            self.r.ping()
            self.r.set('gorev', 'yok')
            print("[INIT] Redis bağlantısı başarılı!")
            print("[INIT] Redis 'gorev' = 'yok' olarak ayarlandı.")
        except Exception as e:
            print(f"[HATA] Redis Bağlantı Hatası: {e}")
            print("[HATA] Redis sunucusunun çalıştığından emin olun: redis-server")
            sys.exit(1)

        # --- SiamRPN TRACKER YÜKLEME ---
        print("[INIT] SiamRPN Tracker yükleniyor...")
        net_path = os.path.join(SCRIPT_DIR, 'model.pth')
        try:
            self.tracker = TrackerSiamRPN(net_path=net_path)
            print("[INIT] SiamRPN Tracker başarıyla yüklendi!")
        except Exception as e:
            print(f"[HATA] SiamRPN yüklenemedi: {e}")
            sys.exit(1)

        # --- TRACKER DURUMU ---
        self.tracker_initialized = False    # Tracker ilk bbox ile başlatıldı mı?
        self.last_cv_bbox = None            # Son OpenCV renk tespiti bbox'ı (x,y,w,h)
        self.last_tracker_bbox = None       # Son tracker bbox'ı (x,y,w,h)
        self.last_reinit_time = 0           # Son tracker re-init zamanı
        self.reinit_interval = 2.0          # OpenCV ile doğrulama/re-init periyodu (sn)
        self.last_cv_detect_time = 0        # Son başarılı OpenCV tespiti zamanı
        self.max_time_wo_cv = 5.0           # OpenCV tespiti olmadan max süre (sn)

        # --- ROS BAĞLANTISI ---
        rospy.init_node('opencv_siamrpn_detector', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/webcam/image_raw", Image, self.image_callback)
        print("[INIT] ROS Node 'opencv_siamrpn_detector' başlatıldı.")
        print("[INIT] /webcam/image_raw kanalından görüntü bekleniyor...")
        print()

        # --- MOR/MAGENTA RENK ARALIKLARI (HSV) ---
        self.lower_purple = np.array([135, 50, 50])
        self.upper_purple = np.array([175, 255, 255])

        # --- KAMERA ÇÖZÜNÜRLÜKLERİ ---
        self.frame_width = 640
        self.frame_height = 480

        # --- KENDİ UÇAĞINI MASKELEME ---
        self.bottom_mask_ratio = 0.0

        # --- İSTATİSTİK ---
        self.frame_count = 0
        self.detect_count = 0
        self.no_detect_count = 0
        self.last_fps_time = time.time()
        self.fps = 0.0
        self.last_print_time = 0

        # --- MİNİMUM ALAN FİLTRESİ ---
        self.min_contour_area = 20

        # --- VALİDASYON PARAMETRELERİ (tracker_allstar.py'den) ---
        self.max_bbox_area = self.frame_width * self.frame_height / 10
        self.horizontal_coverage = 0.0
        self.vertical_coverage = 0.0

        print("[AYAR] Tespit rengi: MOR/MAGENTA (Mavi hariç)")
        print(f"[AYAR] HSV Aralık: H={self.lower_purple[0]}-{self.upper_purple[0]}, "
              f"S={self.lower_purple[1]}-{self.upper_purple[1]}, "
              f"V={self.lower_purple[2]}-{self.upper_purple[2]}")
        print(f"[AYAR] Tracker re-init periyodu: {self.reinit_interval} sn")
        print(f"[AYAR] Max süre OpenCV tespitsiz: {self.max_time_wo_cv} sn")
        print(f"[AYAR] Min kontur alanı: {self.min_contour_area} px²")
        print()
        print("[HAZIR] Sistem hazır. OpenCV hedefi arayacak, bulunca SiamRPN başlayacak.")
        print("-" * 60)
        sys.stdout.flush()

    def _opencv_detect(self, cv_image, hsv):
        """OpenCV HSV renk filtresi ile mor hedef tespiti.
        Başarılıysa (x, y, w, h, area) tuple döner, değilse None."""
        mask = cv2.inRange(hsv, self.lower_purple, self.upper_purple)

        # Maskeleme (ratio=0 ise devre dışı)
        if self.bottom_mask_ratio > 0:
            h_img = cv_image.shape[0]
            cutoff_y = int(h_img * (1.0 - self.bottom_mask_ratio))
            mask[cutoff_y:, :] = 0

        # Morfolojik işlemler
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=1)

        # Kontur bulma
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:
            c = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(c)
            if area > self.min_contour_area:
                x, y, w, h = cv2.boundingRect(c)
                return (x, y, w, h, area), mask

        return None, mask

    def _check_distance_ok(self, cv_bbox, tracker_bbox):
        """OpenCV bbox ile tracker bbox arası mesafe kontrolü.
        tracker_allstar.py'deki check_if_distance_ok mantığı."""
        cx_cv = cv_bbox[0] + cv_bbox[2] / 2
        cy_cv = cv_bbox[1] + cv_bbox[3] / 2
        cx_tr = tracker_bbox[0] + tracker_bbox[2] / 2
        cy_tr = tracker_bbox[1] + tracker_bbox[3] / 2

        dist_x = abs(cx_cv - cx_tr)
        dist_y = abs(cy_cv - cy_tr)

        boundary_x = 0.1 * self.frame_width
        boundary_y = 0.1 * self.frame_height

        return dist_x < boundary_x and dist_y < boundary_y

    def _validate_bbox(self, bbox, frame_h, frame_w):
        """Tracker bbox geçerlilik kontrolü.
        tracker_allstar.py'deki rule_validation + area check mantığı."""
        x, y, w, h = bbox[:4]
        x2, y2 = x + w, y + h

        # --- ALAN KONTROLÜ ---
        bbox_area = w * h
        if bbox_area > self.max_bbox_area:
            return False, False

        # --- COVERAGE HESAPLA ---
        self.horizontal_coverage = (w / frame_w) * 100.0
        self.vertical_coverage = (h / frame_h) * 100.0

        # --- HITBOX KONTROLÜ ---
        hitbox_x1 = int(frame_w * 0.25)
        hitbox_y1 = int(frame_h * 0.1)
        hitbox_x2 = int(frame_w * 0.75)
        hitbox_y2 = int(frame_h * 0.9)

        is_within_hitbox = (x >= hitbox_x1 and y >= hitbox_y1 and
                            x2 <= hitbox_x2 and y2 <= hitbox_y2)

        covers_h = self.horizontal_coverage >= 5
        covers_v = self.vertical_coverage >= 5

        # OpenCV tespiti son max_time_wo_cv saniye içinde yapıldı mı?
        cv_time_valid = (time.time() - self.last_cv_detect_time) < self.max_time_wo_cv

        # guid_valid: cv zaman OK
        guid_valid = cv_time_valid
        # rule_valid: guid_valid + hitbox + coverage
        rule_valid = guid_valid and is_within_hitbox and covers_h and covers_v

        return rule_valid, guid_valid

    def image_callback(self, data):
        """ROS image callback — her kare için çağrılır.
        
        AKIŞ (detect.sh pipeline'ının aynısı):
        1. OpenCV renk tespiti çalışır (YOLO'nun yerini alır)
        2. Tespit varsa → tracker init/re-init (b_box KV yazımı gibi)
        3. Tracker.update() her karede çalışır
        4. Redis'e HER ZAMAN tracker bbox yayınlanır
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(f"[HATA] CvBridge: {e}")
            return

        self.frame_count += 1
        h_img, w_img = cv_image.shape[:2]
        self.frame_width = w_img
        self.frame_height = h_img
        self.max_bbox_area = w_img * h_img / 10

        # --- FPS HESAPLA ---
        now = time.time()
        elapsed = now - self.last_fps_time
        if elapsed >= 2.0:
            self.fps = self.frame_count / elapsed
            self.frame_count = 0
            self.last_fps_time = now

        # --- HSV DÖNÜŞÜM ---
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # =============================================
        # ADIM 1: OpenCV RENK TESPİTİ (= YOLO'nun rolü)
        # Sadece hedefi BUL, tracker'a BİLDİR.
        # =============================================
        cv_result, mask = self._opencv_detect(cv_image, hsv)
        cv_detected = cv_result is not None

        if cv_detected:
            cv_x, cv_y, cv_w, cv_h, cv_area = cv_result
            cv_bbox = (cv_x, cv_y, cv_w, cv_h)
            self.last_cv_bbox = cv_bbox
            self.last_cv_detect_time = now
            self.detect_count += 1

        # =============================================
        # ADIM 2: SiamRPN TRACKER (= tracker_allstar.py'nin rolü)
        # Tracker HER ZAMAN yayınlar. OpenCV sadece init/re-init.
        # =============================================
        tracker_box = None

        if not self.tracker_initialized:
            # --- Henüz başlatılmadı: OpenCV tespiti bekle ---
            if cv_detected:
                print(f"[TRACKER] İlk bbox ile SiamRPN başlatılıyor: {cv_bbox}")
                self.tracker.init(cv_image, cv_bbox)
                self.tracker_initialized = True
                self.last_reinit_time = now
        else:
            # --- Tracker çalışıyor: HER KAREDE update ---
            try:
                raw_box = self.tracker.update(cv_image)
                if raw_box is not None:
                    tracker_box = tuple(int(v) for v in raw_box)
                    self.last_tracker_bbox = tracker_box
            except Exception as e:
                print(f"[HATA] Tracker update: {e}")
                tracker_box = None

            # --- Periyodik doğrulama: OpenCV ile tracker'ı kontrol et ---
            # (tracker_allstar.py'deki "her 2 sn'de b_box kontrol" mantığı)
            if cv_detected and (now - self.last_reinit_time) >= self.reinit_interval:
                if tracker_box is not None:
                    if not self._check_distance_ok(cv_bbox, tracker_box):
                        # Tracker kaymış → OpenCV'ye güven, re-init
                        print(f"[TRACKER] Drift algılandı, re-init: {cv_bbox}")
                        self.tracker.init(cv_image, cv_bbox)
                        # Re-init sonrası hemen update et (tutarlı bbox)
                        try:
                            raw_box = self.tracker.update(cv_image)
                            if raw_box is not None:
                                tracker_box = tuple(int(v) for v in raw_box)
                                self.last_tracker_bbox = tracker_box
                        except Exception:
                            pass
                else:
                    # Tracker sonuç veremiyor → re-init
                    print(f"[TRACKER] Tracker sonuç yok, re-init: {cv_bbox}")
                    self.tracker.init(cv_image, cv_bbox)
                    try:
                        raw_box = self.tracker.update(cv_image)
                        if raw_box is not None:
                            tracker_box = tuple(int(v) for v in raw_box)
                            self.last_tracker_bbox = tracker_box
                    except Exception:
                        pass

                self.last_reinit_time = now

        # =============================================
        # ADIM 3: TRACKER BBOX YAYINLA (HER ZAMAN TRACKER)
        # detect.sh'de de tracker_allstar.py her zaman tracker_bbox yayınlar,
        # YOLO bbox'ı doğrudan yayınlamaz.
        # =============================================
        valid_detection = False

        if tracker_box is not None and self.tracker_initialized:
            x, y, w, h = tracker_box[:4]

            # Sınır kontrolü
            x = max(0, min(x, w_img - 1))
            y = max(0, min(y, h_img - 1))
            w = max(1, min(w, w_img - x))
            h = max(1, min(h, h_img - y))

            rule_valid, guid_valid = self._validate_bbox((x, y, w, h), h_img, w_img)

            if guid_valid and (x, y, w, h) != (0, 0, 0, 0):
                valid_detection = True
                horizontal_coverage = self.horizontal_coverage

                # Redis'e TRACKER bbox yayınla (her zaman aynı kaynak = stabil coverage)
                bbox_msg = [int(x), int(y), int(w), int(h), round(horizontal_coverage, 2)]
                self.r.publish('tracker_bbox', json.dumps(bbox_msg))
                self.r.set('horizontal_coverage', str(horizontal_coverage))

                # --- EKRANDA TEK KUTU ÇİZİMİ (tracker bbox) ---
                cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

                # Merkez crosshair
                cx, cy = int(x + w / 2), int(y + h / 2)
                cv2.circle(cv_image, (cx, cy), 5, (0, 255, 0), -1)
                cv2.line(cv_image, (cx - 15, cy), (cx + 15, cy), (0, 255, 0), 1)
                cv2.line(cv_image, (cx, cy - 15), (cx, cy + 15), (0, 255, 0), 1)

                # Bilgi metinleri
                area_val = w * h
                info_text = f"BBox: {w}x{h} | Area: {area_val}px"
                cv2.putText(cv_image, info_text, (x, y - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
                cov_text = f"Coverage: {horizontal_coverage:.1f}%"
                cv2.putText(cv_image, cov_text, (x, y + h + 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)

                # --- TERMINAL'E YAZDIR ---
                if self.detect_count % 5 == 0 or (now - self.last_print_time) > 0.5:
                    cv_status = "+" if cv_detected else "-"
                    print(f"[TAKIP] bbox=[{x},{y},{w},{h}] "
                          f"merkez=({cx},{cy}) "
                          f"alan={area_val}px "
                          f"coverage={horizontal_coverage:.1f}% "
                          f"cv={cv_status} "
                          f"FPS={self.fps:.1f}")
                    sys.stdout.flush()
                    self.last_print_time = now

        if not valid_detection:
            self.no_detect_count += 1
            if self.no_detect_count % 30 == 0:
                cv_age = now - self.last_cv_detect_time if self.last_cv_detect_time > 0 else -1
                trk_status = "ON" if self.tracker_initialized else "OFF"
                print(f"[ARAMA] Hedef bulunamadı | "
                      f"tracker={trk_status} "
                      f"cv_age={cv_age:.1f}s "
                      f"FPS={self.fps:.1f}")
                sys.stdout.flush()

        # =============================================
        # ADIM 4: EKRAN GÖSTERİMİ
        # =============================================

        # Durum çubuğu
        if valid_detection:
            status_color = (0, 255, 0)
            cv_indicator = " [CV+]" if cv_detected else " [CV-]"
            status_text = f"HEDEF: TAKIP{cv_indicator}"
        else:
            status_color = (0, 0, 255)
            status_text = "HEDEF: ARANIYOR..."
        cv2.putText(cv_image, status_text, (15, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)

        # FPS
        fps_text = f"FPS: {self.fps:.1f}"
        cv2.putText(cv_image, fps_text, (w_img - 130, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

        # Tracker durumu
        trk_text = f"Tracker: {'ON' if self.tracker_initialized else 'OFF'}"
        cv2.putText(cv_image, trk_text, (w_img - 160, 55),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

        # Versiyon
        cv2.putText(cv_image, "OpenCV+SiamRPN v5", (15, h_img - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)

        # Hitbox köşeleri
        rect_width = w_img // 2
        rect_height = int(h_img * 0.8)
        line_length = 30
        color2 = (0, 255, 255)

        top_left = (w_img // 2 - rect_width // 2, h_img // 2 - rect_height // 2)
        top_right = (w_img // 2 + rect_width // 2, h_img // 2 - rect_height // 2)
        bottom_left = (w_img // 2 - rect_width // 2, h_img // 2 + rect_height // 2)
        bottom_right = (w_img // 2 + rect_width // 2, h_img // 2 + rect_height // 2)

        cv2.line(cv_image, top_left, (top_left[0], top_left[1] + line_length), color2, 1)
        cv2.line(cv_image, top_left, (top_left[0] + line_length, top_left[1]), color2, 1)
        cv2.line(cv_image, top_right, (top_right[0], top_right[1] + line_length), color2, 1)
        cv2.line(cv_image, top_right, (top_right[0] - line_length, top_right[1]), color2, 1)
        cv2.line(cv_image, bottom_left, (bottom_left[0], bottom_left[1] - line_length), color2, 1)
        cv2.line(cv_image, bottom_left, (bottom_left[0] + line_length, bottom_left[1]), color2, 1)
        cv2.line(cv_image, bottom_right, (bottom_right[0], bottom_right[1] - line_length), color2, 1)
        cv2.line(cv_image, bottom_right, (bottom_right[0] - line_length, bottom_right[1]), color2, 1)

        # Pencereler
        cv2.imshow("OpenCV+SiamRPN Dedektoru", cv_image)

        mask_small = cv2.resize(mask, (320, 240))
        cv2.imshow("HSV Maske (Mor)", mask_small)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            rospy.signal_shutdown("Kullanıcı çıkışı (q)")


def signal_handler(sig, frame):
    print("\n[KAPANIŞ] Ctrl+C algılandı, kapatılıyor...")
    cv2.destroyAllWindows()
    sys.exit(0)


if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)
    try:
        detector = OpenCVSiamRPNDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
        print("[KAPANIŞ] OpenCV+SiamRPN Dedektörü kapatıldı.")
