import redis
import struct
import numpy as np
import cv2
import json

class RedisHelper:
    def __init__(self):
        self.r = redis.Redis(host='localhost', port=6379, db=0)
        self.p = self.r.pubsub()
        self.p.subscribe('frame','control')
        import os
        config_file = 'config2.json' if os.path.exists('config2.json') else 'config.json'
        with open(config_file) as f:
            data = json.load(f)

        # ---------------------------------------- CONFIG SETTING IS IMPORTANT TO KEEP THE ACCOUNT ----------------------------------------
        self.app_sim = data['guidance']['APP_SIM']

    def convert_to_frame(self, frame_data):
        # 1. Try decoding as RAW (Struct Header >II + Bytes)
        try:
            if len(frame_data) > 8:
                h, w = struct.unpack('>II', frame_data[:8])
                expected_size = h * w * 3
                if len(frame_data) - 8 == expected_size:
                    frame = np.frombuffer(frame_data, dtype=np.uint8, offset=8).reshape(h, w, 3).copy()
                    return frame
        except Exception:
            pass

        # 2. Fallback to Encoded Image (cv2.imdecode)
        try:
            nparr = np.frombuffer(frame_data, np.uint8)
            frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            return frame
        except Exception as e:
            print(f"Error decoding frame: {e}")
            return None

    def convert_to_frame_normal(self,frame_data):
        h, w = struct.unpack('>II',frame_data[:8])
        frame = np.frombuffer(frame_data, dtype=np.uint8, offset=8).reshape(h,w,3).copy()
        return frame
    def from_redis_normal(self, n):
        """Retrieve Numpy array from Redis key 'n'"""
        encoded = self.r.get(n)
        if encoded is None:
            return None
        frame = self.convert_to_frame_normal(encoded)
        return frame

    def from_redis(self, n):
        """Retrieve Numpy array from Redis key 'n'"""
        encoded = self.r.get(n)
        if encoded is None:
            return None
        frame = self.convert_to_frame(encoded)
        return frame

    def from_redis_2(self, name):
        """Retrieve and decode a frame from Redis."""
        encoded = self.r.get(name)
        if encoded is None:
            return None

        # Unpack the shape of the frame
        shape = struct.unpack('>II', encoded[:8])
        h, w = shape

        # Decode the frame data
        frame_data = encoded[8:]
        frame = np.frombuffer(frame_data, dtype=np.uint8).reshape((h, w, 3))

        return frame

    def text_from_redis(self,n):
        encoded = self.r.get(n)
        return encoded

    def toRedis(self, name, frame):
        """Store given Numpy array 'frame' in Redis under key 'name'"""
        h, w = frame.shape[:2]
        shape = struct.pack('>II',h,w)
        encoded = shape + frame.tobytes()

        # Store encoded data in Redis
        #self.r.publish(name,encoded)

        # Use when loop usage needed
        self.r.set(name,encoded, px=500)
        # self.r.publish(name,encoded)
