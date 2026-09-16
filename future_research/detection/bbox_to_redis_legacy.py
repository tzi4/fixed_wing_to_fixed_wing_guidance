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
    """Maintain continuity between red target candidates across frames.

A distant target may span only a few pixels. Select using a proximity
gate around the previous position instead of a fixed area threshold.
After tracking loss, reacquire near the image center to avoid selecting
large red horizon or ground regions.
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
            # Allow image movement in the rotation, but prevent jumping to the speck on the other side of the screen within a frame.
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

        # Do not jump to an off-gate island if a target has been tracked recently.
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

        # Red regions on the aircraft wings and fuselage can be divided into separate contours due to anti-aliasing. Prevent the size from being halved from frame to frame by combining pieces near the selected target into a single bbox.
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
        # --- REJECTION LINK ---
        print("Connecting to server Redis...")
        try:
            self.r = redis.Redis(host='localhost', port=6379, db=0)
            # Let's set the task for the system to work in display mode
            self.r.set('task', 'Visual')
            print("Redis connection successful! Broadcast channel: 'tracker_bbox'")
        except Exception as e:
            print(f"Redis Connection Error: {e}")
            exit(1)

        # --- ROS CONNECTION ---
        rospy.init_node('sim_redis_detector', anonymous=True)
        self.bridge = CvBridge()
        # Your camera topic in the simulation (Taken from your old code)
        self.image_sub = rospy.Subscriber("/webcam/image_raw", Image, self.image_callback)
        print("ROS Node started, waiting for image from /webcam/image_raw channel...")

        # Wing edges are less saturated due to anti-aliasing.  Background artifacts are eliminated with the following size, edge, ratio, and time gates.
        self.lower_red1 = np.array([0, 70, 50])
        self.upper_red1 = np.array([10, 255, 255])
        self.lower_red2 = np.array([170, 70, 50])
        self.upper_red2 = np.array([180, 255, 255])
        self.selector = TemporalBBoxSelector()

    def image_callback(self, data):
        try:
            # Convert message ROS to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

        h_img, w_img, _ = cv_image.shape

        # --- NEW ADDED: Target Hit Area (Yellow Box) Calculation --- We define the inner box by leaving 25% horizontal and 10% vertical space.
        av_left = int(w_img * 0.25)
        av_right = int(w_img * 0.75)
        av_top = int(h_img * 0.10)
        av_bottom = int(h_img * 0.90)

        # The mask is produced from the raw frame before the state drawings will be added to the screen.
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_red1, self.upper_red1) + \
               cv2.inRange(hsv, self.lower_red2, self.upper_red2)
        mask = cv2.dilate(mask, None, iterations=2)
        
        # Find contours (shapes)
        contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        candidates = []
        for contour in contours:
            if cv2.contourArea(contour) <= 4:
                continue
            x, y, w, h = cv2.boundingRect(contour)
            # The aircraft target may soon become larger; however, red background patches covering the screen are not a physically valid target.
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
        lock_eligible = False
        bbox = []

        if selected_bbox is not None:
            x, y, w, h = selected_bbox
                
            # --- NEW ADDED: Checking Lockout Conditions ---
                # Condition 1: Is the detected red box completely inside the Yellow Box (Prey)?
            is_inside = (x >= av_left) and ((x + w) <= av_right) and (y >= av_top) and ((y + h) <= av_bottom)
                
                # Condition 2: Does the target take up 5% horizontally or 5% vertically?
            is_large_enough = (w >= (w_img * 0.05)) or (h >= (h_img * 0.05))
                
                # Locking is appropriate if both conditions are met
            if is_inside and is_large_enough:
                lock_eligible = True
                # -----------------------------------------------------

                # Calculate horizontal coverage for the format your autopilot expects
            horizontal_coverage = (w / w_img) * 100
            validity_flag = 1
                
                # Your new autopilot expects this format: [x, y, w, h, horizontal_cov, validity]
            bbox = [int(x), int(y), int(w), int(h), horizontal_coverage, validity_flag]

                # Publish via Redis (in json format — parses guidance code with json.loads)
            self.r.publish('tracker_bbox', json.dumps(bbox))

                # Draw on the screen (Let's put the target in the Red box as in the image)
            cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 0, 255), 2)
            cv2.circle(cv_image, (int(x + w/2), int(y + h/2)), 5, (0, 255, 0), -1)

                # If the crash is OK, print centered on the screen
            if lock_eligible:
                cv2.putText(cv_image, "LOCKOUT FIT", (int(w_img/2) - 150, int(h_img/2) + 150),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 3)

        if not valid_detection:
            # If there is no target, you can start an empty list or a list with validity = 0.
            pass

        if self.display:
            cv2.rectangle(cv_image, (av_left, av_top), (av_right, av_bottom), (0, 255, 255), 2)
            cv2.putText(cv_image, "Target Hit Area", (av_left, av_top - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            status_text = f"Target: {'FOUND' if valid_detection else 'NONE'}"
            color = (0, 255, 0) if valid_detection else (0, 0, 255)
            cv2.putText(cv_image, status_text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
            cv2.imshow("Simulation Redis Detector", cv_image)
            cv2.waitKey(1)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='ROS broadcasts the red target in the camera image to Redis\'e.')
    parser.add_argument('--no-display', action='store_true',
                        help='OpenCV work headless without opening the window')
    args = parser.parse_args(rospy.myargv()[1:])
    try:
        detector = SimRedisDetector(display=not args.no_display)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
