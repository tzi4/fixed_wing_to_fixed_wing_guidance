
from siamrpn import TrackerSiamRPN
from redis_helper import RedisHelper
import ast
import time
import cv2
import threading
import logging
import os
import shutil
from datetime import datetime
import json
import numpy as np

class Track:
    def __init__(self):
        self.init_logger()
        net_path = 'model.pth'
        
        try:
            self.tracker = TrackerSiamRPN(net_path=net_path)
        except Exception as e:
            print(f"Tracker yüklenirken hata oluştu: {e}")
            exit(1)

        config_file = 'config2.json' if os.path.exists('config2.json') else 'config.json'
        with open(config_file) as f:
            data = json.load(f)
        self.rh = RedisHelper()
        self.last_b_box = None  # Son kullanılan b_box'ı saklamak için
        self.last_update_time = time.time()  # Son b_box kontrol zamanı
        self.counter = 0
        self.horizontal_coverage = 0.0
        self.vertical_coverage = 0.0

        self.W_frame = data['resolution']['w']
        self.H_frame = data['resolution']['h']

        self.is_local = data["uav_handler"]["is_local"]

        self.distance_boundry_for_x = (0.1 * self.W_frame)
        self.distance_boundry_for_y = (0.1 * self.H_frame)
        self.logger.debug('distance boundry for y:'+ str(self.distance_boundry_for_y))
        self.logger.debug('distance boundry for x:'+ str(self.distance_boundry_for_x))

        self.max_yolo_time = data['guidance']['MAX_TIME_WO_YOLO']
        self.max_bbox_area = self.W_frame * self.H_frame / 10

        self.yolo_time_validation = False
        self.last_yolo_time = time.time() - 600
        self.start_time = None
        self.elapsed_time = 0
        yolo_control_thread = threading.Thread(target=self.yolo_i_counter, daemon=True)
        yolo_control_thread.start()

    def init_logger(self):
        # Customcustom logger in order to log to both console and file
        self.logger = logging.Logger('GOAT')
        # Set the log level
        self.logger.setLevel(logging.DEBUG)
        # Create handlers
        c_handler = logging.StreamHandler()
        
        log_file_path = 'seq_system.log'
        old_logs_dir = "logs"
        
        if not os.path.exists(old_logs_dir):
            os.makedirs(old_logs_dir)
            
        if os.path.exists(log_file_path):
            # Generate a unique name for the log file in the old logs directory
            timestamp = datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
            new_log_file_name = f"GOAT_guidance_{timestamp}.log"

            new_log_file_path = os.path.join(old_logs_dir, new_log_file_name)
            # Move the log file to the old logs directory
            shutil.move(log_file_path, new_log_file_path)
            
        f_handler = logging.FileHandler(log_file_path)
        # Set levels for handlers
        c_handler.setLevel(logging.DEBUG)
        f_handler.setLevel(logging.DEBUG)

        # Create formatters and add it to handlers
        c_format = logging.Formatter('%(name)s - %(levelname)s - %(message)s')
        f_format = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(process)d - %(threadName)s - %(filename)s:%(lineno)d - %(funcName)s - %(message)s')
        c_handler.setFormatter(c_format)
        f_handler.setFormatter(f_format)

        # Add handlers to the self.logger
        self.logger.addHandler(c_handler)
        self.logger.addHandler(f_handler)

    def yolo_i_counter(self):
        while True:
            if time.time() - self.last_yolo_time > self.max_yolo_time:
                self.yolo_time_validation = False
            else:
                self.yolo_time_validation = True
            time.sleep(0.2)

    def check_if_distance_ok(self,yolo_bbox,tracker_box):
        # nokta
        x1, y1, w, h = tracker_box
        x2 = x1 + w
        y2 = y1 + h
        tracker_center = ([(x1+x2)/2, (y1+y2)/2])
        print(f"YOLO KUTUSU ==== {yolo_bbox}")
        x1,y1,x2,y2 = yolo_bbox
        yolo_center = ([(x1+x2)/2, (y1+y2)/2])

        # Unpack the points
        x_yolo, y_yolo = yolo_center
        x_tracker, y_tracker = tracker_center
        # Calculate the distance using the Pythagorean theorem
        tracker_delta_x_to_yolo = abs(x_tracker - x_yolo)
        tracker_delta_y_to_yolo = abs(y_tracker - y_yolo)

        # distance = np.sqrt(tracker_delta_x_to_yolo ** 2 + tracker_delta_y_to_yolo ** 2)
        self.logger.debug('yolo-tracker center distance (x,y): '+ str(tracker_delta_x_to_yolo)+ ','+ str(tracker_delta_y_to_yolo))

        if tracker_delta_x_to_yolo < self.distance_boundry_for_x and tracker_delta_y_to_yolo < self.distance_boundry_for_y: # 1280 720 için 90 lık 
            return True
        return False
    
    def calculate_bbox_area(self, tracker_box):
        x1, y1, w, h = tracker_box
        x2 = x1 + w
        y2 = y1 + h
        return (x2 - x1) * (y2 - y1)

    def check_if_area_ok(self,tracker_box,max_area):
        bbox_area = self.calculate_bbox_area(tracker_box)
        if bbox_area > max_area:
            return False
        return True
    
    def rule_validation(self, tracker_box, image_height, image_width):
        # Tracker kutusu bilgilerini aç
        x1, y1, w, h = tracker_box
        # Tracker kutusunun sağ alt köşesini hesapla
        x2, y2 = x1 + w, y1 + h

        # Hitbox sınırlarını belirle
        hitbox_x1, hitbox_y1, hitbox_x2, hitbox_y2 = int(image_width * 0.25), int(image_height * 0.1), int(image_width * 0.75), int(image_height * 0.9)

        # Tracker kutusunun hitbox içinde olup olmadığını kontrol et
        is_within_hitbox = x1 >= hitbox_x1 and y1 >= hitbox_y1 and x2 <= hitbox_x2 and y2 <= hitbox_y2


        self.target_width_yolo , self.target_height_yolo = tracker_box[2], tracker_box[3]
        self.horizontal_coverage = (self.target_width_yolo / image_width) * 100
        self.vertical_coverage = (self.target_height_yolo / image_height) * 100

        # Her bir yön için istenen yüzdeden büyük bir alan kaplayıp kaplamadığını kontrol et
        covers_more_than_5_percent_horizontally = self.horizontal_coverage >= 5
        covers_more_than_5_percent_vertically = self.vertical_coverage >= 5
        self.logger.debug('horizontal_coverage:'+str(self.horizontal_coverage))
        self.logger.debug('vertical_coverage:'+str(self.vertical_coverage))
        if is_within_hitbox and covers_more_than_5_percent_horizontally and covers_more_than_5_percent_vertically:
            return True
        else:
            return False
    
    def is_valid(self, yolo_bbox, tracker_box, height, width):
        """
        Tracker kutusu geçerlilik kontrolü.
        support_bbox yolo kutusu, height width frame özellikler, anlamlarına gelir.
        """
        rule_validation = False
        guid_validation = False

        is_rule_valid = False
        is_width_valid = True
        is_distance_valid = True

        # Rulebook validation
        is_rule_valid = self.rule_validation(tracker_box, height, width)

        # Area check
        is_area_valid = self.check_if_area_ok(tracker_box=tracker_box, max_area=self.max_bbox_area) 

        # Control with yolo box when present.
        if yolo_bbox:
            print(f"YOLO_BBOX === f{yolo_bbox}")
            is_distance_valid = self.check_if_distance_ok(yolo_bbox=yolo_bbox,tracker_box=tracker_box)

        self.logger.debug('area-distance-width-yolotimeval:'+str(is_area_valid)+ str(is_distance_valid)+str(is_width_valid)+ str(self.yolo_time_validation))
        if is_area_valid and is_distance_valid and self.yolo_time_validation:
            # eğer tracker büyük değilse (guidence yapılacak kadar) + yolo ile arasındaki mesafe oksa + time_wo_detection geçerliyse === 
            guid_validation = True
            if is_rule_valid:
                #üstüne kutu yüzde beşten büyük + hitbox'ın içindeyse === 
                rule_validation = True

        return rule_validation, guid_validation  
    
    def track(self):
        if self.is_local:
            frame = self.rh.from_redis('frame')
        else:
            frame_data = self.rh.r.get("frame")
            np_img = np.frombuffer(frame_data, dtype=np.uint8)
            frame = cv2.imdecode(np_img, cv2.IMREAD_COLOR)
        while frame is None:
            print("Frame Redis'ten alınamadı, bekleniyor...")
            time.sleep(1)
            if self.is_local:
                frame = self.rh.from_redis('frame')
            else:
                frame_data = self.rh.r.get("frame")
                if frame_data:
                    np_img = np.frombuffer(frame_data, dtype=np.uint8)
                    frame = cv2.imdecode(np_img, cv2.IMREAD_COLOR)
        
    
        b_box = self.rh.r.get("b_box")
        if b_box is None:
            print("Başlangıçta b_box bulunamadı, tekrar deneniyor")
            time.sleep(0.5)
            b_box = [1,1,1,1]
            self.tracker.init(frame, b_box)
        else:
            try:
                b_box = ast.literal_eval(b_box.decode('utf-8'))
                x1, y1, x2, y2 = map(int, b_box)
                self.last_b_box = (x1, y1, x2 - x1, y2 - y1)
                print(f"İlk BBox: {self.last_b_box}")
                self.tracker.init(frame, self.last_b_box)
            except ValueError:
                print("Başlangıç b_box formatı hatalı, çıkılıyor.")
                return

        prev_time = time.time()

        while True:
            if self.is_local:
                frame = self.rh.from_redis('frame')
            else:
                frame_data = self.rh.r.get("frame")
                np_img = np.frombuffer(frame_data, dtype=np.uint8)
                frame = cv2.imdecode(np_img, cv2.IMREAD_COLOR)
            if frame is None:
                print("Yeni frame alınamadı, bekleniyor...")
                time.sleep(0.1)
                continue
            
            current_time = time.time()
            fps = 1 / (current_time - prev_time)
            prev_time = current_time
            yolo_box = self.rh.r.get("b_box")
            if yolo_box is not None:
                self.last_yolo_time = time.time()
                yolo_box = ast.literal_eval(yolo_box.decode('utf-8'))
            if time.time() - self.last_update_time >= 2:
                new_b_box = self.rh.r.get("b_box")
                if new_b_box is not None:
                    try:
                        new_b_box = ast.literal_eval(new_b_box.decode('utf-8'))
                        x1, y1, x2, y2 = map(int, new_b_box)
                        new_b_box_tuple = (x1, y1, x2 - x1, y2 - y1)
                        
                        # **Eğer yeni b_box geldiyse, tracker'ı güncelle**
                        if new_b_box_tuple != self.last_b_box:
                            print(f"Yeni BBox bulundu, tracker güncelleniyor: {new_b_box_tuple}")
                            self.tracker.init(frame, new_b_box_tuple)
                            self.last_b_box = new_b_box_tuple
                    except ValueError:
                        print("Yeni b_box formatı hatalı, eski kutu ile devam ediliyor.")

                self.last_update_time = time.time()  # Zaman damgasını güncelle


            # **Takip işlemi sürekli devam etsin**
            tracker_box = self.tracker.update(frame)

            if tracker_box is None:
                print("Tracker güncellenemedi, eski kutu ile devam ediliyor.")
                time.sleep(0.1)
                continue

            t_box = [int(v) for v in tracker_box]
            print(f"Yeni takip koordinatları: {t_box}")
            #self.rh.r.set("tracker_bbox", str(t_box).encode('utf-8'))
            frame_h,frame_w = frame.shape[:2]
            rule_valid, guid_valid = self.is_valid(yolo_box,tracker_box,frame_h,frame_w)

            if guid_valid:
                # -----------------------
                # GUIDANCE MESSAGE
                if t_box != [0,0,0,0]:
                    t_box.append(self.horizontal_coverage)  # Ek veri ekliyorsanız
                    self.rh.r.publish('tracker_bbox', json.dumps(t_box))  # Mesajı JSON formatında gönderiyoruz.
                    t_box.pop()  # Eğer eklediğiniz veriyi geri almak istiyorsanız.
                    self.rh.r.set('horizontal_coverage',str(self.horizontal_coverage))
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)  # Kutuyu çiz
                # -----------------------
                self.logger.debug('rule valid:'+ str(rule_valid))

                if rule_valid:
                    if self.start_time is None:
                        self.start_time = time.time()  # İlk defa başlat
                    else:
                        self.elapsed_time = time.time() - self.start_time  # Geçen süreyi hesapla
                else:
                    self.start_time = None  # Valid değilse sıfırla
                    self.elapsed_time = 0

                if self.elapsed_time >= 4:
                    self.logger.debug("PAKET OLUŞTU, GÖREV TAMAM")

                # Sayaç değerini log'a yazdıralım
                self.logger.debug(f"Rule valid süresi: {self.elapsed_time:.2f} saniye")

            x, y, w, h = t_box
            x1 = x
            y1 = y
            x2 = x + w
            y2 = y + h
            # Crosshair yeni boyutları
            rect_width = frame_w // 2  # Ekranın yarısı
            rect_height = int(frame_h * 0.8)  # Ekranın %80'i
            line_length = 200

            # Dikdörtgenin yeni köşe noktalarını hesaplayalım
            top_left = (frame_w // 2 - rect_width // 2, frame_h // 2 - rect_height // 2)
            top_right = (frame_w // 2 + rect_width // 2, frame_h // 2 - rect_height // 2)
            bottom_left = (frame_w // 2 - rect_width // 2, frame_h // 2 + rect_height // 2)
            bottom_right = (frame_w // 2 + rect_width // 2, frame_h // 2 + rect_height // 2)
            
            color2 = (0, 255, 255)

            # Dikdörtgenin köşelerine çizgiler çizelim
            cv2.line(frame, top_left, (top_left[0], top_left[1] + line_length), color2, 5)
            cv2.line(frame, top_left, (top_left[0] + line_length, top_left[1]), color2, 5)

            cv2.line(frame, top_right, (top_right[0], top_right[1] + line_length), color2, 5)
            cv2.line(frame, top_right, (top_right[0] - line_length, top_right[1]), color2, 5)

            cv2.line(frame, bottom_left, (bottom_left[0], bottom_left[1] - line_length), color2, 5)
            cv2.line(frame, bottom_left, (bottom_left[0] + line_length, bottom_left[1]), color2, 5)

            cv2.line(frame, bottom_right, (bottom_right[0], bottom_right[1] - line_length), color2, 5)
            cv2.line(frame, bottom_right, (bottom_right[0] - line_length, bottom_right[1]), color2, 5)
            
            cv2.putText(frame, f"YATAY oran: %{int(self.horizontal_coverage)}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(frame, f"DIKEY oran: %{int(self.vertical_coverage)}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(frame, f"FPS: {int(fps)}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            rounded_time = round(self.elapsed_time, 1)
            cv2.putText(frame, f"SAYAC: {rounded_time}", (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            _, buffer = cv2.imencode(".jpg", frame)  # Görüntüyü JPEG formatına çevir
            self.rh.r.set("frame_tracker", buffer.tobytes())  # Redis'e bayt olarak gönder
            cv2.imshow("Tracking", frame) 
  
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            time.sleep(0.01)  

        cv2.destroyAllWindows()


    
if __name__ == '__main__':
    t = Track()
    t.track()
