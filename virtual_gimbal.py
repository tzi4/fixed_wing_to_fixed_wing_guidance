#Bbox is read via Redis, virtual gimbal combinations are applied and raw and virtual coordinates are printed to the terminal.
import numpy as np
import math
import json
import time
import redis
import threading
from pymavlink import mavutil

# ─── CONSTANTS ───
RESOLUTION_W = 640
RESOLUTION_H = 480
CAMERA_FOCAL_LENGTH = 467.7
UAV_PORT = '14553'

# Camera->Body rotation
R_c_b = np.array([[0, 0, 1],
                   [1, 0, 0],
                   [0, 1, 0]], dtype=float)
R_c_b_T = R_c_b.T


def compute_R_b_e(roll, pitch, yaw):
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp,    cp*sr,            cp*cr           ]
    ])


class VirtualGimbal:
    def __init__(self):
        # Redis
        self.r = redis.Redis(host='localhost', port=6379, db=0)
        self.p = self.r.pubsub()
        self.p.subscribe('tracker_bbox')
        print("[VirtualGimbal] Redis connection established, 'tracker_bbox' channel is being listened to.")

        # camera intrinsics
        self.W = RESOLUTION_W
        self.H = RESOLUTION_H
        self.center_x = self.W / 2
        self.center_y = self.H / 2

        f_oc = CAMERA_FOCAL_LENGTH
        self.K = np.array([
            [f_oc, 0, self.center_x],
            [0, f_oc, self.center_y],
            [0, 0, 1]
        ])
        self.K_inv = np.linalg.inv(self.K)

        # MAVLink (for attitudinal reading only)
        self.master = mavutil.mavlink_connection(f'udpin:127.0.0.1:{UAV_PORT}')
        self.master.wait_heartbeat()
        print(f"[VirtualGimbal] MAVLink connection established (sys={self.master.target_system}).")

        # MAVLink reader thread — keeps cache updated
        self._reader_thread = threading.Thread(
            target=self._mavlink_reader, daemon=True, name="MAVLinkReader")
        self._reader_thread.start()

        # Thread-safe repository for the latest bbox
        self.latest_bbox = None
        self.latest_bbox_time = None
        self.bbox_lock = threading.Lock()

    def _mavlink_reader(self):
        """It keeps the pymavlink cache updated by constantly making recv_match."""
        while True:
            try:
                self.master.recv_match(blocking=True, timeout=0.1)
            except Exception:
                time.sleep(0.01)

    # ─── Parse ───
    def _parse_bbox(self, data):
        """Parse a Redis message and return an (x, y, w, h) tuple or None."""
        if isinstance(data, (list, tuple)) and len(data) >= 4:
            x, y, w, h = data[:4]
            if w is not None:
                return (int(x), int(y), int(w), int(h))
        return None

    # ─── Virtual Gimbal Conversion ───
    def _apply_virtual_gimbal(self, bbox):
        """
        bbox: (x, y, w, h) — upper left corner + width/height Calculates raw (u, v) and virtual (u_virt, v_virt), send to terminal.
        """
        # Raw pixel center
        u_raw = bbox[0] + bbox[2] / 2.0
        v_raw = bbox[1] + bbox[3] / 2.0

        # Back-Projection: p_raw -> r_cam
        p_raw = np.array([u_raw, v_raw, 1.0])
        r_cam = self.K_inv @ p_raw

        # Camera -> Body
        r_body = R_c_b @ r_cam

        # Read attitude from the cache without blocking
        att_msg = self.master.messages.get('ATTITUDE', None)
        if att_msg:
            roll_rad = att_msg.roll
            pitch_rad = att_msg.pitch
        else:
            roll_rad, pitch_rad = 0.0, 0.0

        # Stabilization: Body -> Virtual Body (yaw=0)
        R_stab = compute_R_b_e(roll_rad, pitch_rad, 0)
        r_virt_body = R_stab @ r_body

        # Virtual Body -> Virtual Camera
        r_virt_cam = R_c_b_T @ r_virt_body

        # Re-Projection: r_virt_cam -> p_virt
        p_virt_hom = self.K @ r_virt_cam

        if p_virt_hom[2] != 0:
            u_virt = p_virt_hom[0] / p_virt_hom[2]
            v_virt = p_virt_hom[1] / p_virt_hom[2]
        else:
            u_virt, v_virt = 0.0, 0.0

        # head to terminal
        print(
            f"BBox: {bbox} | "
            f"Raw: u={u_raw:.1f}, v={v_raw:.1f} | "
            f"Virtual: u={u_virt:.1f}, v={v_virt:.1f} | "
            f"RPY(rad): roll={roll_rad:.3f}, pitch={pitch_rad:.3f}"
        )

    # ─── Redis Listener ───
    def _redis_listener(self):
        """Listen to Redis pub/sub, parse messages, and update latest_bbox."""
        for message in self.p.listen():
            if message['type'] != 'message':
                continue
            try:
                data = json.loads(message['data'].decode('utf-8'))
            except (json.JSONDecodeError, UnicodeDecodeError):
                continue

            bbox = self._parse_bbox(data)
            if bbox is not None:
                now = time.time()
                with self.bbox_lock:
                    self.latest_bbox = bbox
                    self.latest_bbox_time = now

    # ─── Processor ───
    def _processor(self):
        """On the 30 Hz it reads the latest bbox and applies virtual gimbal conversion."""
        period = 1.0 / 30.0
        last_processed_time = None

        while True:
            time.sleep(period)
            with self.bbox_lock:
                bbox = self.latest_bbox
                bbox_time = self.latest_bbox_time

            if bbox is not None and bbox_time != last_processed_time:
                last_processed_time = bbox_time
                self._apply_virtual_gimbal(bbox)

    # ─── Run ───
    def run(self):
        listener = threading.Thread(
            target=self._redis_listener, daemon=True, name="RedisListener")
        listener.start()

        processor = threading.Thread(
            target=self._processor, daemon=True, name="Processor")
        processor.start()

        print("[VirtualGimbal] Working...")
        listener.join()
        processor.join()


if __name__ == '__main__':
    sg = VirtualGimbal()
    sg.run()
