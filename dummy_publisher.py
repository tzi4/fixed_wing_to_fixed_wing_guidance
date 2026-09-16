import redis
import time
import json
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("DummyInfo")

# Connect to Redis
try:
    r = redis.Redis(host='localhost', port=6379, db=0)
    channel = 'tracker_bbox'
    
    logger.info(f"Connected to Redis. Publishing dummy messages to '{channel}' every 0.1s.")

    # Fixed Bounding Box: [x, y, w, h]
    # Center (320, 240) for 640x480 resolution
    # x = 320 - 50 = 270
    # y = 240 - 50 = 190
    bbox_data = [270, 190, 100, 100] 
    
    while True:
        # Publish to channel
        r.publish(channel, json.dumps(bbox_data))
        
        time.sleep(0.1)
        
except KeyboardInterrupt:
    logger.info("Stopping publisher.")
except Exception as e:
    logger.error(f"Error: {e}")
