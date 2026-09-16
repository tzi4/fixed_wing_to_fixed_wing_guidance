import time
import torch
import redis
import struct
import numpy as np
import cv2
from ultralytics import YOLO

# Redis
redis_client = redis.Redis(host='localhost', port=6379, db=0)
# NOTE: YOLO will NO longer broadcast to tracker_bbox.  Only writes to Redis KV (b_box, confidence, etc.).  Only tracker SiamRPN writes to the tracker_bbox channel.

# Device
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

# YOLO model
yolo_model = YOLO("v10_final.pt")
if device.type == "cuda":
    yolo_model.to(device)

CONF_TH = 0.5  # YOLO confidence threshold


while True:
    try:
        # Frame from Redis: first 8 bytes >II (H,W), after H*W*3 BGR
        frame_data = redis_client.get("frame")
        if frame_data is None:
            print("No image received from Redis!")
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
            # Find the index with the highest confidence (Boxes are not iterable; get argmax from tensor)
            idx = int(torch.argmax(boxes.conf).item())
            x1, y1, x2, y2 = [int(v) for v in boxes.xyxy[idx].tolist()]
            conf = float(boxes.conf[idx].item())

            # KV spellings (retained for team compatibility)
            redis_client.set("confidence", conf)
            redis_client.set("scan", "False" if conf >= CONF_TH else "True")
            redis_client.set("guidance", "True" if conf >= CONF_TH else "False")
            bbox_list = [x1, y1, x2, y2]
            redis_client.set("b_box", str(bbox_list).encode("utf-8"))

            print(f"Confidence score of the best detected object: {conf:.2f}")
            if conf < CONF_TH:
                print("Confidence score is less than 0.5. 'scan: True' and 'guidance: False' were written to Redis.")
            else:
                print("Confidence score is greater than or equal to 0.5. 'scan: False' and 'guidance: True' written to Redis.")
            print(f"bbox written to Redis: {bbox_list}")

            # Draw on image
            cv2.rectangle(frame_with_boxes, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame_with_boxes, f"Conf: {conf:.2f}", (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)



        else:
            print("No object detected.")
            # KV spellings (for team)
            redis_client.set("scan", "False")
            redis_client.set("guidance", "False")
            redis_client.set("confidence", 0.0)



        # impression
        cv2.imshow("YOLO detections", frame_with_boxes)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("The window is closed, the program terminates.")
            break

    except Exception as e:
        print(f"YOLO returned error: {e}")
        # Small wait after error
        time.sleep(0.01)
        continue

cv2.destroyAllWindows()
