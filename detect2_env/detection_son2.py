import time
import json
import torch
import redis
import struct
import numpy as np
import cv2
from ultralytics import YOLO

# Redis
redis_client = redis.Redis(host='localhost', port=6379, db=0)
DETECTION_CHANNEL = "tracker_bbox"  # tüketici bununla subscribe oluyor

# Cihaz
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

# YOLO model
yolo_model = YOLO("best.pt")
if device.type == "cuda":
    yolo_model.to(device)

CONF_TH = 0.5  # YOLO güven eşiği

def publish_detection(msg: dict):
    """tüketicinin beklediği JSON formatını kanala gönder"""
    try:
        redis_client.publish(DETECTION_CHANNEL, json.dumps(msg))
    except Exception as e:
        print(f"[WARN] Redis publish hata: {e}")

while True:
    try:
        # Redis'ten frame: ilk 8 byte >II (H,W), sonrası H*W*3 BGR
        frame_data = redis_client.get("frame")
        if frame_data is None:
            print("Redis'ten görüntü alınamadı!")
            time.sleep(0.01)
            continue

        h, w = struct.unpack(">II", frame_data[:8])
        frame = np.frombuffer(frame_data[8:], dtype=np.uint8).reshape(h, w, 3)

        # Inference
        res = yolo_model(frame, conf=CONF_TH, verbose=False)[0]
        boxes = res.boxes  # ultralytics.engine.results.Boxes

        frame_with_boxes = frame.copy()

        n = 0 if boxes is None or boxes.xyxy is None else int(boxes.xyxy.shape[0])
        ts_now = time.time()

        if n > 0:
            # En yüksek güvene sahip indeksi bul
            # (Boxes iterable değildir; tensörden argmax al)
            idx = int(torch.argmax(boxes.conf).item())
            x1, y1, x2, y2 = [int(v) for v in boxes.xyxy[idx].tolist()]
            conf = float(boxes.conf[idx].item())

            # KV yazımlar (takım uyumluluğu için korundu)
            redis_client.set("confidence", conf)
            redis_client.set("tarama", "False" if conf >= CONF_TH else "True")
            redis_client.set("gudum", "True" if conf >= CONF_TH else "False")
            bbox_list = [x1, y1, x2, y2]
            redis_client.set("b_box", str(bbox_list).encode("utf-8"))

            print(f"Tespit edilen en iyi nesnenin güven skoru: {conf:.2f}")
            if conf < CONF_TH:
                print("Güven skoru 0.5'ten küçük. Redis'e 'tarama: True' ve 'gudum: False' yazıldı.")
            else:
                print("Güven skoru 0.5'e eşit veya büyük. Redis'e 'tarama: False' ve 'gudum: True' yazıldı.")
            print(f"Redis'e bbox yazıldı: {bbox_list}")

            # Görsel üstüne çiz
            cv2.rectangle(frame_with_boxes, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame_with_boxes, f"Conf: {conf:.2f}", (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # PUB/SUB yayını — TÜKETİCİ İÇİN KRİTİK
            publish_detection({
                "source": "yolo",
                "bbox": [x1, y1, x2, y2],     # xyxy (tüketici bunu zaten xywh'e dönüştürüyor)
                "conf": conf,
                "detected": True,
                "ts": ts_now,
                "img_w": int(w),
                "img_h": int(h)
            })

        else:
            print("Nesne tespit edilmedi.")
            # KV yazımlar (takım için)
            redis_client.set("tarama", "False")
            redis_client.set("gudum", "False")
            redis_client.set("confidence", 0.0)

            # PUB/SUB yayını — YOLO saatini tazeler, tüketici 'no-detection'ı anlar
            publish_detection({
                "source": "yolo",
                "detected": False,
                "ts": ts_now,
                "img_w": int(w),
                "img_h": int(h)
            })

        # Gösterim
        cv2.imshow("YOLO Tespitleri", frame_with_boxes)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("Pencere kapatıldı, program sonlanıyor.")
            break

    except Exception as e:
        print(f"YOLO hata verdi: {e}")
        # Hata sonrası küçük bekleme
        time.sleep(0.01)
        continue

cv2.destroyAllWindows()
