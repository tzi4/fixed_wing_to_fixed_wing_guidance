#!/usr/bin/env python3

import argparse
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import redis
import json
import time
import math


class TemporalBBoxSelector:
    """Kırmızı adaylar arasında kareler arası sürekliliği korur.

    Uzak hedef birkaç piksel olabildiği için salt alan eşiği kullanmak yerine,
    son konuma yakınlık kapısı kullanılır. Takip tamamen kaybolduğunda merkezden
    yeniden edinim yapılır; böylece ufuk/zemin gibi büyük kırmızı lekeler hedef
    diye seçilmez.
    """

    def __init__(self, track_timeout_s=0.50):
        self.track_timeout_s = float(track_timeout_s)
        self.last_bbox = None
        self.last_time = None

    @staticmethod
    def _center(bbox):
        x, y, w, h = bbox
        return x + (w / 2.0), y + (h / 2.0)

    def select(self, candidates, image_w, image_h, now=None):
        now = time.monotonic() if now is None else float(now)
        if not candidates:
            if self.last_time is not None and now - self.last_time > self.track_timeout_s:
                self.last_bbox = None
                self.last_time = None
            return None

        selected = None
        if self.last_bbox is not None and self.last_time is not None:
            elapsed = max(0.0, now - self.last_time)
            old_x, old_y = self._center(self.last_bbox)
            # Dönüşteki görüntü hareketine izin ver, fakat bir karede
            # ekranın öbür tarafındaki lekeye atlamayı engelle.
            motion_gate = max(
                80.0,
                4.0 * max(self.last_bbox[2], self.last_bbox[3]),
                80.0 + (600.0 * min(elapsed, self.track_timeout_s)),
            )
            scored = []
            for bbox in candidates:
                cx, cy = self._center(bbox)
                distance = math.hypot(cx - old_x, cy - old_y)
                if distance <= motion_gate:
                    scored.append((distance, bbox))
            if scored:
                selected = min(scored, key=lambda item: item[0])[1]

        # Yakın zamanda izlenen hedef varsa kapı dışı bir adaya atlama.
        if selected is None and self.last_time is not None and \
                now - self.last_time <= self.track_timeout_s:
            return None

        if selected is None:
            image_cx, image_cy = image_w / 2.0, image_h / 2.0
            selected = min(
                candidates,
                key=lambda bbox: math.hypot(
                    self._center(bbox)[0] - image_cx,
                    self._center(bbox)[1] - image_cy,
                ),
            )

        # Uçağın kanat/gövde kırmızıları anti-aliasing nedeniyle ayrı
        # konturlara bölünebilir. Seçilen hedefin yakınındaki parçaları tek
        # bbox'ta birleştirerek boyutun kareden kareye yarıya düşmesini önle.
        selected_cx, selected_cy = self._center(selected)
        merge_gate = max(50.0, 3.0 * max(selected[2], selected[3]))
        nearby = [
            bbox for bbox in candidates
            if math.hypot(
                self._center(bbox)[0] - selected_cx,
                self._center(bbox)[1] - selected_cy,
            ) <= merge_gate
        ]
        if len(nearby) > 1:
            x1 = min(bbox[0] for bbox in nearby)
            y1 = min(bbox[1] for bbox in nearby)
            x2 = max(bbox[0] + bbox[2] for bbox in nearby)
            y2 = max(bbox[1] + bbox[3] for bbox in nearby)
            selected = (x1, y1, x2 - x1, y2 - y1)

        self.last_bbox = selected
        self.last_time = now
        return selected

class SimRedisDetector:
    def __init__(self, display=True):
        self.display = bool(display)
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

        # Kanat kenarları anti-aliasing nedeniyle daha düşük doygunluktadır.
        # Arka plan lekeleri aşağıdaki boyut/kenar/oran ve zaman kapılarıyla elenir.
        self.lower_red1 = np.array([0, 70, 50])
        self.upper_red1 = np.array([10, 255, 255])
        self.lower_red2 = np.array([170, 70, 50])
        self.upper_red2 = np.array([180, 255, 255])
        self.selector = TemporalBBoxSelector()

    def image_callback(self, data):
        try:
            # ROS mesajını OpenCV formatına çevir
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

        h_img, w_img, _ = cv_image.shape

        # --- YENİ EKLENEN: Hedef Vuruş Alanı (Sarı Kutu) Hesaplaması ---
        # Yatayda %25, Dikeyde %10 boşluk bırakarak iç kutuyu tanımlıyoruz
        av_left = int(w_img * 0.25)
        av_right = int(w_img * 0.75)
        av_top = int(h_img * 0.10)
        av_bottom = int(h_img * 0.90)

        # Maske, ekrana eklenecek durum çizimlerinden önce ham kareden üretilir.
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_red1, self.upper_red1) + \
               cv2.inRange(hsv, self.lower_red2, self.upper_red2)
        mask = cv2.dilate(mask, None, iterations=2)
        
        # Konturları (şekilleri) bul
        contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        candidates = []
        for contour in contours:
            if cv2.contourArea(contour) <= 4:
                continue
            x, y, w, h = cv2.boundingRect(contour)
            # Uçak hedefi yakında büyüyebilir; ancak ekranı kaplayan kırmızı
            # arka plan parçaları fiziksel olarak geçerli bir hedef değildir.
            if w > (w_img * 0.45) or h > (h_img * 0.45):
                continue
            if (w * h) > (w_img * h_img * 0.15):
                continue
            if x <= 1 or y <= 1 or x + w >= w_img - 1 or y + h >= h_img - 1:
                continue
            aspect_ratio = max(w / max(h, 1), h / max(w, 1))
            if aspect_ratio > 20.0:
                continue
            candidates.append((int(x), int(y), int(w), int(h)))

        selected_bbox = self.selector.select(candidates, w_img, h_img)
        valid_detection = selected_bbox is not None
        kilitlenme_uygun = False
        bbox = []

        if selected_bbox is not None:
            x, y, w, h = selected_bbox
                
            # --- YENİ EKLENEN: Kilitlenme Şartlarının Kontrolü ---
                # Şart 1: Tespit edilen kırmızı kutu tamamen Sarı Kutunun (Av) içinde mi?
            is_inside = (x >= av_left) and ((x + w) <= av_right) and (y >= av_top) and ((y + h) <= av_bottom)
                
                # Şart 2: Hedef yatayda %5 veya dikeyde %5 alan kaplıyor mu?
            is_large_enough = (w >= (w_img * 0.05)) or (h >= (h_img * 0.05))
                
                # İki şart da sağlanıyorsa kilitlenme uygundur
            if is_inside and is_large_enough:
                kilitlenme_uygun = True
                # -----------------------------------------------------

                # Otopilotunuzun beklediği format için yatay kapsama hesapla
            horizontal_coverage = (w / w_img) * 100
            validity_flag = 1
                
                # Yeni otopilotunuz bu formatı bekliyor: [x, y, w, h, horizontal_cov, validity]
            bbox = [int(x), int(y), int(w), int(h), horizontal_coverage, validity_flag]

                # Redis üzerinden yayınla (json formatında — guidance kodu json.loads ile parse eder)
            self.r.publish('tracker_bbox', json.dumps(bbox))

                # Ekranda çizim yap (Görseldeki gibi hedefi Kırmızı kutu içine alalım)
            cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 0, 255), 2)
            cv2.circle(cv_image, (int(x + w/2), int(y + h/2)), 5, (0, 255, 0), -1)

                # Kilitlenme uygunsa ekrana ortalı şekilde yazdır
            if kilitlenme_uygun:
                cv2.putText(cv_image, "KILITLENME UYGUN", (int(w_img/2) - 150, int(h_img/2) + 150),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 3)

        if not valid_detection:
            # Hedef yoksa boş veya validity=0 olan bir liste basabilirsiniz 
            pass

        if self.display:
            cv2.rectangle(cv_image, (av_left, av_top), (av_right, av_bottom), (0, 255, 255), 2)
            cv2.putText(cv_image, "Hedef Vurus Alani", (av_left, av_top - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            status_text = f"Hedef: {'BULUNDU' if valid_detection else 'YOK'}"
            color = (0, 255, 0) if valid_detection else (0, 0, 255)
            cv2.putText(cv_image, status_text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
            cv2.imshow("Simulasyon Redis Dedektoru", cv_image)
            cv2.waitKey(1)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='ROS kamera goruntusundeki kirmizi hedefi Redis\'e yayinlar.')
    parser.add_argument('--no-display', action='store_true',
                        help='OpenCV penceresi acmadan headless calis')
    args = parser.parse_args(rospy.myargv()[1:])
    try:
        detector = SimRedisDetector(display=not args.no_display)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
