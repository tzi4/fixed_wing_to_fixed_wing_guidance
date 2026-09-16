#!/usr/bin/env python3
"""
detectCV.py: OpenCV and SiamRPN target detection and tracking, version 5.

OpenCV HSV filtering detects the target aircraft. SiamRPN tracks it
between frames. Results use the existing Redis tracker_bbox schema.

Pipeline:
  detect.sh: YOLO -> b_box key/value -> SiamRPN -> tracker_bbox publication.
  detectCV.sh: OpenCV color detection -> SiamRPN -> tracker_bbox publication.

Always publish the tracker's bounding box. OpenCV initializes and
periodically validates or reinitializes the tracker. This keeps coverage
and bounding-box measurements consistent across detector/tracker updates.
The display shows one tracker box, following the detect.sh pipeline.
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

# The folder where the script is located (detectCV_env/)
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

from siamrpn import TrackerSiamRPN


class OpenCVSiamRPNDetector:
    def __init__(self):
        # --- REJECTION LINK ---
        print("=" * 60)
        print("  OpenCV + SiamRPN Target Detection & Tracking System v5")
        print("  OpenCV = detect/validate | SiamRPN = track/publish")
        print("=" * 60)
        print()
        print("[INIT] Connecting to server Redis...")
        try:
            self.r = redis.Redis(host='localhost', port=6379, db=0)
            self.r.ping()
            self.r.set('task', 'none')
            print("[INIT] Redis connection successful!")
            print("[INIT] Redis 'task' = 'none'.")
        except Exception as e:
            print(f"[ERROR] Redis Connection Error: {e}")
            print("[ERROR] Make duration_s that Redis server is running: redish-server")
            sys.exit(1)

        # --- SiamRPN TRACKER INSTALLATION ---
        print("[INIT] SiamRPN Tracker is loading...")
        net_path = os.path.join(SCRIPT_DIR, 'model.pth')
        try:
            self.tracker = TrackerSiamRPN(net_path=net_path)
            print("[INIT] SiamRPN Tracker has been successfully installed!")
        except Exception as e:
            print(f"[ERROR] Failed to load SiamRPN: {e}")
            sys.exit(1)

        # --- TRACKER STATUS ---
        self.tracker_initialized = False    # Was Tracker first launched with bbox?
        self.last_cv_bbox = None            # Latest OpenCV color detection bbox (x,y,w,h)
        self.last_tracker_bbox = None       # Last tracker bbox (x,y,w,h)
        self.last_reinit_time = 0           # Last tracker re-init time
        self.reinit_interval = 2.0          # Verification/re-init period (sec) with OpenCV
        self.last_cv_detect_time = 0        # Time of last successful OpenCV detection
        self.max_time_wo_cv = 5.0           # OpenCV without detection max duration (sec)

        # --- ROS CONNECTION ---
        rospy.init_node('opencv_siamrpn_detector', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/webcam/image_raw", Image, self.image_callback)
        print("[INIT] ROS Node 'opencv_siamrpn_detector' is started.")
        print("[INIT] Waiting for image from channel /webcam/image_raw...")
        print()

        # --- PURPLE/MAGENTA COLOR RANGES (HSV) ---
        self.lower_purple = np.array([135, 50, 50])
        self.upper_purple = np.array([175, 255, 255])

        # --- CAMERA RESOLUTIONS ---
        self.frame_width = 640
        self.frame_height = 480

        # --- MASKING YOUR OWN PLANE ---
        self.bottom_mask_ratio = 0.0

        # --- STATISTICS ---
        self.frame_count = 0
        self.detect_count = 0
        self.no_detect_count = 0
        self.last_fps_time = time.time()
        self.fps = 0.0
        self.last_print_time = 0

        # --- MINIMUM AREA FILTER ---
        self.min_contour_area = 20

        # --- VALIDATION PARAMETERS (from tracker_allstar.py) ---
        self.max_bbox_area = self.frame_width * self.frame_height / 10
        self.horizontal_coverage = 0.0
        self.vertical_coverage = 0.0

        print("[SETTING] Detection color: PURPLE/MAGENTA (Except Blue)")
        print(f"[SETTING] HSV Range: H={self.lower_purple[0]}-{self.upper_purple[0]}, "
              f"S={self.lower_purple[1]}-{self.upper_purple[1]}, "
              f"V={self.lower_purple[2]}-{self.upper_purple[2]}")
        print(f"[SETTING] Tracker re-init period: {self.reinit_interval} sec")
        print(f"[SETTING] Max duration OpenCV undetected: {self.max_time_wo_cv} sec")
        print(f"[SETTING] Min contour area: {self.min_contour_area} px²")
        print()
        print("[READY] The system is ready. OpenCV will search for the target, when found, SiamRPN will start.")
        print("-" * 60)
        sys.stdout.flush()

    def _opencv_detect(self, cv_image, hsv):
        """OpenCV Purple target detection with HSV color filter.         If successful (x, y, w, h, area) returns via tube, otherwise None."""
        mask = cv2.inRange(hsv, self.lower_purple, self.upper_purple)

        # Masking (disabled if ratio=0)
        if self.bottom_mask_ratio > 0:
            h_img = cv_image.shape[0]
            cutoff_y = int(h_img * (1.0 - self.bottom_mask_ratio))
            mask[cutoff_y:, :] = 0

        # Morphological processes
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=1)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:
            c = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(c)
            if area > self.min_contour_area:
                x, y, w, h = cv2.boundingRect(c)
                return (x, y, w, h, area), mask

        return None, mask

    def _check_distance_ok(self, cv_bbox, tracker_bbox):
        """OpenCV Distance control between bbox and tracker bbox.         check_if_distance_ok logic in tracker_allstar.py."""
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
        """Tracker bbox validity check.         rule_validation + area check logic in tracker_allstar.py."""
        x, y, w, h = bbox[:4]
        x2, y2 = x + w, y + h

        # --- AREA CONTROL ---
        bbox_area = w * h
        if bbox_area > self.max_bbox_area:
            return False, False

        # --- CALCULATE COVERAGE ---
        self.horizontal_coverage = (w / frame_w) * 100.0
        self.vertical_coverage = (h / frame_h) * 100.0

        # --- HITBOX CONTROL ---
        hitbox_x1 = int(frame_w * 0.25)
        hitbox_y1 = int(frame_h * 0.1)
        hitbox_x2 = int(frame_w * 0.75)
        hitbox_y2 = int(frame_h * 0.9)

        is_within_hitbox = (x >= hitbox_x1 and y >= hitbox_y1 and
                            x2 <= hitbox_x2 and y2 <= hitbox_y2)

        covers_h = self.horizontal_coverage >= 5
        covers_v = self.vertical_coverage >= 5

        # Was OpenCV detected in the last max_time_wo_cv seconds?
        cv_time_valid = (time.time() - self.last_cv_detect_time) < self.max_time_wo_cv

        # guid_valid: cv time OK
        guid_valid = cv_time_valid
        # rule_valid: guid_valid + hitbox + coverage
        rule_valid = guid_valid and is_within_hitbox and covers_h and covers_v

        return rule_valid, guid_valid

    def image_callback(self, data):
        """ROS image callback, called for each frame.

Following the detect.sh pipeline:
  1. OpenCV color detection takes the place of YOLO.
  2. A detection initializes or reinitializes the tracker.
  3. Tracker.update() runs on every frame.
  4. Always publish the tracker bounding box to Redis.
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(f"[ERROR] CvBridge: {e}")
            return

        self.frame_count += 1
        h_img, w_img = cv_image.shape[:2]
        self.frame_width = w_img
        self.frame_height = h_img
        self.max_bbox_area = w_img * h_img / 10

        # --- CALCULATE FPS ---
        now = time.time()
        elapsed = now - self.last_fps_time
        if elapsed >= 2.0:
            self.fps = self.frame_count / elapsed
            self.frame_count = 0
            self.last_fps_time = now

        # --- HSV CONVERSION ---
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # ============================= STEP 1: OpenCV COLOR DETECTION (= YOLO's role) Just FIND the target, NOTIFY it to the tracker.  ==============================================
        cv_result, mask = self._opencv_detect(cv_image, hsv)
        cv_detected = cv_result is not None

        if cv_detected:
            cv_x, cv_y, cv_w, cv_h, cv_area = cv_result
            cv_bbox = (cv_x, cv_y, cv_w, cv_h)
            self.last_cv_bbox = cv_bbox
            self.last_cv_detect_time = now
            self.detect_count += 1

        # ============================= STEP 2: SiamRPN TRACKER (= Role of tracker_allstar.py) Tracker ALWAYS broadcasts. OpenCV only init/re-init. =============================================
        tracker_box = None

        if not self.tracker_initialized:
            # --- Not started yet: wait for OpenCV detection ---
            if cv_detected:
                print(f"[TRACKER] Starting SiamRPN with first bbox: {cv_bbox}")
                self.tracker.init(cv_image, cv_bbox)
                self.tracker_initialized = True
                self.last_reinit_time = now
        else:
            # --- Tracker is running: update EVERY SCREEN ---
            try:
                raw_box = self.tracker.update(cv_image)
                if raw_box is not None:
                    tracker_box = tuple(int(v) for v in raw_box)
                    self.last_tracker_bbox = tracker_box
            except Exception as e:
                print(f"[ERROR] Tracker update: {e}")
                tracker_box = None

            # --- Periodic verification: check tracker with OpenCV --- ("Check b_box every 2 sec" logic in tracker_allstar.py)
            if cv_detected and (now - self.last_reinit_time) >= self.reinit_interval:
                if tracker_box is not None:
                    if not self._check_distance_ok(cv_bbox, tracker_box):
                        # Tracker shifted → trust OpenCV, re-init
                        print(f"[TRACKER] Drift detected, re-init: {cv_bbox}")
                        self.tracker.init(cv_image, cv_bbox)
                        # Update immediately after re-init (consistent bbox)
                        try:
                            raw_box = self.tracker.update(cv_image)
                            if raw_box is not None:
                                tracker_box = tuple(int(v) for v in raw_box)
                                self.last_tracker_bbox = tracker_box
                        except Exception:
                            pass
                else:
                    # Tracker fails to produce results → re-init
                    print(f"[TRACKER] Tracker no results, re-init: {cv_bbox}")
                    self.tracker.init(cv_image, cv_bbox)
                    try:
                        raw_box = self.tracker.update(cv_image)
                        if raw_box is not None:
                            tracker_box = tuple(int(v) for v in raw_box)
                            self.last_tracker_bbox = tracker_box
                    except Exception:
                        pass

                self.last_reinit_time = now

        # ============================================ STEP 3: PUBLISH TRACKER BBOX (ALWAYS TRACKER) Also in detect.sh, tracker_allstar.py always publishes tracker_bbox, it does not publish YOLO bbox directly.  ==============================================
        valid_detection = False

        if tracker_box is not None and self.tracker_initialized:
            x, y, w, h = tracker_box[:4]

            # border control
            x = max(0, min(x, w_img - 1))
            y = max(0, min(y, h_img - 1))
            w = max(1, min(w, w_img - x))
            h = max(1, min(h, h_img - y))

            rule_valid, guid_valid = self._validate_bbox((x, y, w, h), h_img, w_img)

            if guid_valid and (x, y, w, h) != (0, 0, 0, 0):
                valid_detection = True
                horizontal_coverage = self.horizontal_coverage

                # Broadcast TRACKER bbox to Redis (always same source = stable coverage)
                bbox_msg = [int(x), int(y), int(w), int(h), round(horizontal_coverage, 2)]
                self.r.publish('tracker_bbox', json.dumps(bbox_msg))
                self.r.set('horizontal_coverage', str(horizontal_coverage))

                # --- SINGLE BOX DRAWING ON THE SCREEN (tracker bbox) ---
                cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

                # Center crosshair
                cx, cy = int(x + w / 2), int(y + h / 2)
                cv2.circle(cv_image, (cx, cy), 5, (0, 255, 0), -1)
                cv2.line(cv_image, (cx - 15, cy), (cx + 15, cy), (0, 255, 0), 1)
                cv2.line(cv_image, (cx, cy - 15), (cx, cy + 15), (0, 255, 0), 1)

                # Information labels
                area_val = w * h
                info_text = f"BBox: {w}x{h} | Area: {area_val}px"
                cv2.putText(cv_image, info_text, (x, y - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
                cov_text = f"Coverage: {horizontal_coverage:.1f}%"
                cv2.putText(cv_image, cov_text, (x, y + h + 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)

                # --- PRINT TO TERMINAL ---
                if self.detect_count % 5 == 0 or (now - self.last_print_time) > 0.5:
                    cv_status = "+" if cv_detected else "-"
                    print(f"[FOLLOWING] bbox=[{x},{y},{w},{h}] "
                          f"center=({cx},{cy}) "
                          f"box_area={area_val}px "
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
                print(f"[SEARCH] Target not found | "
                      f"tracker={trk_status} "
                      f"cv_age={cv_age:.1f}s "
                      f"FPS={self.fps:.1f}")
                sys.stdout.flush()

        # ============================== STEP 4: SCREEN DISPLAY ==============================================================================

        # status bar
        if valid_detection:
            status_color = (0, 255, 0)
            cv_indicator = " [CV+]" if cv_detected else " [CV-]"
            status_text = f"TARGET: FOLLOW-UP{cv_indicator}"
        else:
            status_color = (0, 0, 255)
            status_text = "TARGET: WANTED..."
        cv2.putText(cv_image, status_text, (15, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)

        # FPS
        fps_text = f"FPS: {self.fps:.1f}"
        cv2.putText(cv_image, fps_text, (w_img - 130, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

        # Tracker status
        trk_text = f"Tracker: {'ON' if self.tracker_initialized else 'OFF'}"
        cv2.putText(cv_image, trk_text, (w_img - 160, 55),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

        # Version
        cv2.putText(cv_image, "OpenCV+SiamRPN v5", (15, h_img - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)

        # Hitbox corners
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

        # Windows
        cv2.imshow("OpenCV+SiamRPN Detector", cv_image)

        mask_small = cv2.resize(mask, (320, 240))
        cv2.imshow("HSV Mask (Purple)", mask_small)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            rospy.signal_shutdown("User exit (q)")


def signal_handler(sig, frame):
    print("\n[CLOSING] Ctrl+C detected, shutting down...")
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
        print("[OFF] OpenCV+SiamRPN Detector has been turned off.")
