
import redis
import struct
import numpy as np
import cv2
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time

try:
    pass
except Exception as e:
    print("ROS Modules not found. Running in real mode. Not simulation...")
    pass

class FramePublisher:
    last_start_time: int
    def __init__(self):
        self.r = redis.Redis(host='localhost', port=6379, db=0)
        print('redis connection established.')
        self.bridge = CvBridge()
        self.last_start_time = 0
        self.counter = 0

    def toRedis(self, name, frame):
        """Store given Numpy array 'frame' in Redis under key 'name'"""
        h, w = frame.shape[:2]
        shape = struct.pack('>II',h,w)
        encoded = shape + frame.tobytes()

        # Store encoded data in Redis
        #self.r.publish(name,encoded)

        # Use when loop usage needed
        self.r.set(name,encoded, px=1000)
        # self.r.publish(name,encoded)
        

    def run(self, sim=True):
        rospy.init_node('image_converter', anonymous=True)
        image_sub = rospy.Subscriber("/webcam/image_raw", Image, self.publish_frame)
        try:
            rospy.spin()
            print('running')
        except KeyboardInterrupt:
            print("Shutting down")
            #cv2.destroyAllWindows()

# frame_publisher.py içindeki run_cam metodunu bununla değiştirin

    def run_cam(self):
        # Kamera, döngünün DIŞINDA SADECE BİR KEZ başlatılmalıdır.
        camera = cv2.VideoCapture(2) 

        # Kameranın başarıyla açılıp açılmadığını kontrol et.
        if not camera.isOpened():
            print("HATA: Kamera (index 0) açılamadı veya bulunamadı.")
            print("Lütfen kameranın bağlı olduğundan ve başka bir program tarafından kullanılmadığından emin olun.")
            return # Kamera yoksa fonksiyondan çık.

        print("Kamera başarıyla açıldı. Görüntü akışı başlıyor...")
        while True:
            # camera.read() fonksiyonundan dönen iki değeri ayrı değişkenlere ata
            ret, frame = camera.read()

            # 'ret' değeri False ise (görüntü alınamadıysa) döngünün o adımını atla.
            if not ret:
                print("Uyarı: Kameradan görüntü karesi alınamadı.")
                break # Genellikle akışın sonu veya bir hata anlamına gelir, döngüyü sonlandır.

            # Görüntü başarıyla alındıysa Redis'e gönder
            self.toRedis('frame', frame)
            
            # CPU'yu %100 kullanmamak için çok kısa bir bekleme eklemek iyi bir pratiktir.
            time.sleep(0.01) # Saniyede yaklaşık 100 kareye denk gelir.

        # Program bittiğinde kamerayı serbest bırak.
        print("Görüntü akışı sonlandı.")
        camera.release()

    def publish_frame(self, frame):
        # FPS CONTROL
        
        start_time = time.perf_counter()
        elapsed_time_in_ms = (start_time - self.last_start_time)*1000
        # print('elapsed time: ', elapsed_time_in_ms)
        # if  elapsed_time_in_ms < 20: # 50 FPS
        #     return
        try:
            frame = self.bridge.imgmsg_to_cv2(frame, "bgr8")
        except CvBridgeError as e:
            print(e)
        self.toRedis('frame', frame)
        self.last_start_time = start_time
        cv2.imshow('img pub', frame)
        cv2.waitKey(1)
        # cv2.imwrite(f'frames/frame_{self.counter}.jpg', frame)
        # self.counter += 1
        # halt execution for current thread for code optimization
        time.sleep(0.01)


if __name__ == '__main__':
    fb = FramePublisher()
    fb.run()














# import redis
# import struct
# import numpy as np
# import cv2
# import rospy
# from std_msgs.msg import String
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge, CvBridgeError
# import time
# import threading

# try:
#     pass
# except Exception as e:
#     print("ROS Modules not found. Running in real mode. Not simulation...")
#     pass

# class FramePublisher:
#     last_start_time: int
#     def __init__(self):
#         self.r = redis.Redis(host='localhost', port=6379, db=0)
#         print('redis connection established.')
#         self.bridge = CvBridge()
#         self.last_start_time = 0
#         self.counter = 0

#     def toRedis(self, name, frame):
#         """Store given Numpy array 'frame' in Redis under key 'name'"""
#         h, w = frame.shape[:2]
#         shape = struct.pack('>II',h,w)
#         encoded = shape + frame.tobytes()

#         # Store encoded data in Redis
#         #self.r.publish(name,encoded)

#         # Use when loop usage needed
#         self.r.set(name,encoded, px=500)
#         # self.r.publish(name,encoded)
        

#     def run(self, sim=True):
#         rospy.init_node('image_converter', anonymous=True)
#         image_sub = rospy.Subscriber("/webcam/image_raw", Image, self.publish_frame)
#         try:
#             rospy.spin()
#             print('running')
#         except KeyboardInterrupt:
#             print("Shutting down")
#             cv2.destroyAllWindows()

#     def run_cam(self):
#         while True:
#             camera = cv2.VideoCapture(0)
#             frame = camera.read()
#             self.toRedis('frame', frame)
#             time.sleep(0.01)

#     def publish_frame(self, frame):
#         # FPS CONTROL
#         start_time = time.perf_counter()
#         elapsed_time_in_ms = (start_time - self.last_start_time)*1000
#         # print('elapsed time: ', elapsed_time_in_ms)
#         # if  elapsed_time_in_ms < 20: # 50 FPS
#         #     return
#         try:
#             frame = self.bridge.imgmsg_to_cv2(frame, "bgr8")
#         except CvBridgeError as e:
#             print(e)
#         self.toRedis('frame', frame)
#         self.last_start_time = start_time
#         cv2.imshow('img pub', frame)
#         cv2.waitKey(1)
#         # cv2.imwrite(f'frames/frame_{self.counter}.jpg', frame)
#         # self.counter += 1
#         # halt execution for current thread for code optimization
#         time.sleep(0.01)



# # def run_publisher():
# #     det = FramePublisher()
# #     det.run()

# # if __name__ == '__main__':
# #     detection_thread = threading.Thread(target=run_publisher)
# #     detection_thread.start()
# #     detection_thread.join()

# if __name__ == '__main__':
#     fb = FramePublisher()
#     fb.run()
