#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import redis
import json
import time
import os
import atexit
import argparse
import datetime
import signal
import threading
import fnmatch
import math

# OpenCV's default 16 threads used roughly four times the CPU in a 1080p
#test: 7.7 ms at 411% CPU versus 10.3 ms at 100% CPU with one thread.
#One thread supports about 97 fps, sufficient for the 30 Hz camera.
#Default to one thread and allow BUMBLEBEE_CV_THREADS to override it.
#Zero or a negative value restores OpenCV's automatic setting. Pass -1
#because setNumThreads(0) requests sequential execution in OpenCV.
_CV_THREADS = int(os.environ.get('BUMBLEBEE_CV_THREADS', '1'))
cv2.setNumThreads(_CV_THREADS if _CV_THREADS > 0 else -1)

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_VIDEO_DIR = os.path.join(SCRIPT_DIR, 'videos')
FONT = cv2.FONT_HERSHEY_SIMPLEX


class TemporalBBoxSelector:
    """Interframe continuity and fragment merging between candidates."""

    def __init__(self, track_timeout_s=0.50):
        self.track_timeout_s = float(track_timeout_s)
        self.last_bbox = None
        self.last_time = None

    @staticmethod
    def _center(bbox):
        x, y, w, h = bbox
        return x + w / 2.0, y + h / 2.0

    def select(self, candidates, image_w, image_h, now=None):
        now = time.monotonic() if now is None else float(now)
        if not candidates:
            if self.last_time is not None and now - self.last_time > self.track_timeout_s:
                self.last_bbox = self.last_time = None
            return None

        selected = None
        if self.last_bbox is not None and self.last_time is not None:
            elapsed = max(0.0, now - self.last_time)
            old_x, old_y = self._center(self.last_bbox)
            motion_gate = max(80.0, 4.0 * max(self.last_bbox[2:]),
                              80.0 + 600.0 * min(elapsed, self.track_timeout_s))
            scored = []
            for bbox in candidates:
                cx, cy = self._center(bbox)
                distance = math.hypot(cx - old_x, cy - old_y)
                if distance <= motion_gate:
                    scored.append((distance, bbox))
            if scored:
                selected = min(scored, key=lambda item: item[0])[1]

        if (selected is None and self.last_time is not None and
                now - self.last_time <= self.track_timeout_s):
            return None
        if selected is None:
            image_cx, image_cy = image_w / 2.0, image_h / 2.0
            selected = min(candidates, key=lambda bbox: math.hypot(
                self._center(bbox)[0] - image_cx,
                self._center(bbox)[1] - image_cy))

        selected_cx, selected_cy = self._center(selected)
        merge_gate = max(50.0, 3.0 * max(selected[2:]))
        nearby = [bbox for bbox in candidates if math.hypot(
            self._center(bbox)[0] - selected_cx,
            self._center(bbox)[1] - selected_cy) <= merge_gate]
        if len(nearby) > 1:
            x1 = min(b[0] for b in nearby)
            y1 = min(b[1] for b in nearby)
            x2 = max(b[0] + b[2] for b in nearby)
            y2 = max(b[1] + b[3] for b in nearby)
            selected = (x1, y1, x2 - x1, y2 - y1)

        self.last_bbox = selected
        self.last_time = now
        return selected


def draw_text(image, text, origin, color, scale=0.6, thickness=2):
    """Draw a black outline followed by colored text for legibility.

LINE_8 reduced the measured long status-line cost from 1360 to 325 us.
LINE_AA was about four times more expensive. The black outline provides
adequate readability without its extra cost at 30 Hz.
    """
    cv2.putText(image, text, origin, FONT, scale, (0, 0, 0), thickness + 2, cv2.LINE_8)
    cv2.putText(image, text, origin, FONT, scale, color, thickness, cv2.LINE_8)


def text_width(text, scale=0.6, thickness=2):
    """The ACTUAL width (px) that the draw_text will occupy.

    The measurement is made by the thickness of the black contour (thickness + 2); colored text fails in it. This is always used instead of assuming fixed pixels.
    """
    return cv2.getTextSize(text, FONT, scale, thickness + 2)[0][0]


# ACTIVE-SCRIPT OVERLAY SETTINGS
#The video records which guidance or test script was active.
#Add a row to this list for another script. Matching uses fnmatch,
#so the '*' wildcard is supported.
CODE_WATCH_PATTERNS = (
    'formation.py',
    'command_sender.py',
    'goat_cam_offset.py',
    'teva.py',                 # competition version (range from 1 Hz telemetry)
    'goat_gimbal_aircraft.py',
    'tzi_emir.py',
    'visual_guidance*.py',
    'load_plan.py',
    'verify_flight.py',
    # --- test infrastructure: show "what was running at that moment" in the video ---
    'test_setup.py',
    'target_ramp.py',
    'server_simulator.py',
    'ground_truth_logger.py',
)
CODE_SCAN_PERIOD_S = 1.0   # scan period: NOT per frame, every second
CODE_REDIS_KEY = 'active_code'  # If a script wants to announce itself explicitly
CODE_MAX_NAMES = 3         # more than that is summarized as "+N"
CODE_MAX_CHARS = 96        # To prevent the line from overflowing the screen in 1080p
CODE_SELF_NAME = os.path.basename(__file__)  # listing our own process
CODE_LINE_Y = 88           # baseline of the "Code: ..." line
CODE_LINE_X = 20           # left margin
AIRCRAFT_RIGHT_MARGIN = 20     # Distance of aircraft name from right edge
AIRCRAFT_MIN_GAP = 24          # MINIMUM space between two texts


# 3D POSITION OVERLAY AT THE LOWER LEFT
#The overlay records where the hunter and target were in each video frame.
#It reads only Redis. This process deliberately opens no MAVLink link:
#the camera pipeline runs at 30 Hz within a single-core budget, and a
#second link and heartbeat thread would add avoidable load.
#  hunter_telemetry: own-aircraft position, supplied by a separate publisher.
#  rakip_telemetri: target position, using the competition server schema.
#Both keys contain:
#  {"konumBilgileri": [{"iha_enlem": .., "iha_boylam": .., "iha_irtifa": ..}]}
#Missing or invalid data is displayed as '-' instead of hiding the row,
#keeping the layout stable and making missing telemetry visible.
HUNTER_REDIS_KEY = 'hunter_telemetry'
TARGET_REDIS_KEY = 'rakip_telemetri'
TELEM_SCAN_PERIOD_S = 1.0   # read period: NOT per frame, every second
COORD_LINE_X = 20           # left margin
COORD_BOTTOM_MARGIN = 20    # between the baseline of the bottom row and the bottom edge
COORD_LINE_GAP = 28         # spacing between the two coordinate lines
COORD_PLACEHOLDER = '-'     # sign printed if there is no data


def _redis_with_short_timeout():
    """SEPARATE Redis client for Overlay routes.

    The publication link remains untouched. Short timeout: If Redis hangs, the overlay thread should not hang for seconds.
    """
    try:
        return redis.Redis(host='localhost', port=6379, db=0,
                           socket_timeout=0.2, socket_connect_timeout=0.2)
    except Exception:
        return None


# AIRCRAFT NAME OVERLAY
#Place the aircraft name to the right of the Code line, aligned with the
#frame's right edge. Resolve it from:
#  1. BUMBLEBEE_AIRCRAFT, exported by the launcher.
#  2. The model directory in BUMBLEBEE_HUNTER_MODEL.
#  3. If neither identifies an aircraft, omit the name and keep the Code line.
AIRCRAFT_NAMES = {
    'bumblebee': 'Bumblebee',          # our 10 kg+ aircraft
    'emir_aircraft_temp': 'Erenimbus',     # Emir's 1.5 kg plane (A/B control)
}


def resolve_aircraft_name():
    """Aircraft name to be displayed at Overlay (or None)."""
    name = os.environ.get('BUMBLEBEE_AIRCRAFT', '').strip()
    if name:
        return name
    path = os.environ.get('BUMBLEBEE_HUNTER_MODEL', '').strip()
    if not path:
        return None                    # indefinite: no name written
    # Only inspect the immediate parent directory of model.sdf.
    #Searching every component of a path such as
    #.../guidance/bumblebee/models/emir_aircraft_temp/model.sdf would incorrectly
    #identify the repository directory bumblebee as the aircraft model.
    model_dir = os.path.basename(os.path.dirname(os.path.normpath(path)))
    return AIRCRAFT_NAMES.get(model_dir)


# TARGET COLOR
#The target uses the purple model models/purple_target. The previous red HSV
#window overlapped the background: the Gazebo sky dome renders a horizon
#band near BGR (127,127,255). During a roll, it produced false detections
#almost 1920 pixels wide. Grazing views of the ground also produced red
#line artifacts. A purple target permits tests with the sky, runway and
#grass present.
#
#Measured across three recordings, 900 frames and 1.99e8 chromatic pixels
#with S >= 70 and V >= 50:
#  H 110-114, sky: 54.4% overall and 87.7% in the upper third.
#  H 45-49, grass: 26.1% overall and 54.4% in the lower third.
#  H 30-34, runway: 13.8%.
#  H 0-4, red window: 1.9%.
#  H 140-160, purple window: 0.0012%, roughly 1700 times less overlap.
#The remaining purple-window pixels are scattered H.264 chroma noise
#with a median saturation near 78. Requiring S >= 120 removes 95% of
#that residue. The target material is RGB (1,0,1), with source saturation
#255, and remains above 150 in shadow.
#Set BUMBLEBEE_TARGET_COLOR=red to restore the original red window.
TARGET_COLOR_WINDOWS = {
    # Gazebo/Purple = RGB(1,0,1) -> OpenCV HSV H=150. One window is enough.
    'purple': ((140, 160),),
    # Since red spirals at H=0, two windows are required (old behavior).
    'red': ((0, 10), (170, 180)),
}
TARGET_COLOR_SV = {
    'purple': (120, 60),   # (S_min, V_min): suppress chroma noise
    'red': (70, 50),       # The old red thresholds are preserved AS IS
}
DEFAULT_TARGET_COLOR = 'purple'


def build_color_ranges(color, s_min=None, v_min=None):
    """Generates HSV window list from color name (lower, upper).

    Warn and use the default for an unknown color, so a typo cannot silently disable detection.
    """
    color = (color or '').strip().lower() or DEFAULT_TARGET_COLOR
    if color not in TARGET_COLOR_WINDOWS:
        print(f"WARNING: unknown BUMBLEBEE_TARGET_COLOR='{color}'; "
              f"valid values: {', '.join(sorted(TARGET_COLOR_WINDOWS))}. "
              f"'{DEFAULT_TARGET_COLOR}' will be used.")
        color = DEFAULT_TARGET_COLOR
    default_s, default_v = TARGET_COLOR_SV[color]
    s_min = default_s if s_min is None else s_min
    v_min = default_v if v_min is None else v_min
    ranges = [(np.array([lo, s_min, v_min]), np.array([hi, 255, 255]))
              for lo, hi in TARGET_COLOR_WINDOWS[color]]
    return color, ranges


class ActiveCodeTracker:
    """Identify currently running guidance and test scripts periodically.

A background thread scans /proc/<pid>/cmdline every CODE_SCAN_PERIOD_S,
without spawning a subprocess or pgrep. The frame path reads the cached
self.text attribute, adding about 0.1 microseconds per frame.

A nonempty CODE_REDIS_KEY value overrides the process-scan result.
The same background thread checks Redis once per second using a separate
client, leaving the frame publication connection untouched.
    """

    def __init__(self, redis_client=None, period=CODE_SCAN_PERIOD_S,
                 patterns=CODE_WATCH_PATTERNS):
        self.period = float(period)
        self.patterns = tuple(patterns)
        self.text = 'Code: -'
        self.redis = redis_client
        self._redis_owned = False
        self._redis_warned = False
        self._stop = threading.Event()
        self._thread = None

    # --- internal helpers -------------------------------------------------
    def _ensure_redis(self):
        if self.redis is not None:
            return self.redis
        self.redis = _redis_with_short_timeout()
        self._redis_owned = self.redis is not None
        return self.redis

    def _redis_override(self):
        client = self._ensure_redis()
        if client is None:
            return None
        try:
            raw = client.get(CODE_REDIS_KEY)
        except Exception as exc:
            if not self._redis_warned:
                self._redis_warned = True
                print(f"WARNING: Failed to read '{CODE_REDIS_KEY}', process scan "
                      f"in progress ({exc}).")
            return None
        if raw is None:
            return None
        if isinstance(raw, bytes):
            raw = raw.decode('utf-8', 'replace')
        raw = raw.strip()
        return raw or None

    def scan_processes(self):
        """Returns matches of CODE_WATCH_PATTERNS within running processes."""
        my_pid = os.getpid()
        found = set()
        try:
            entries = os.listdir('/proc')
        except OSError:
            return []
        for entry in entries:
            if not entry.isdigit() or int(entry) == my_pid:
                continue
            try:
                with open('/proc/' + entry + '/cmdline', 'rb') as fh:
                    raw = fh.read()
            except (OSError, IOError):
                continue          # The process is dead in the meantime or there is no permission
            if b'.py' not in raw:
                continue          # cheap ten eliminations: most processes are eliminated here
            for token in raw.split(b'\0'):
                if not token.endswith(b'.py') or len(token.split()) != 1:
                    # A wrapper script like token=`bash -c "... python3 x/formation.py"` with spaces is not a real argv path.
                    continue
                name = os.path.basename(token.decode('utf-8', 'replace'))
                if name == CODE_SELF_NAME:
                    continue
                found.add(name)
        if not found:
            return []
        # Preserve the pattern order: the overlay line does not move from frame to frame.
        ordered = []
        for pattern in self.patterns:
            for name in sorted(found):
                if name not in ordered and fnmatch.fnmatchcase(name, pattern):
                    ordered.append(name)
        return ordered

    @staticmethod
    def format_line(names):
        if not names:
            return 'Code: -'
        shown = list(names[:CODE_MAX_NAMES])
        extra = len(names) - len(shown)
        text = 'Code: ' + ', '.join(shown)
        if extra > 0:
            text += f" +{extra}"
        if len(text) > CODE_MAX_CHARS:
            text = text[:CODE_MAX_CHARS - 3] + '...'
        return text

    # --- public interface -----------------------------------------------------
    def refresh(self):
        override = self._redis_override()
        if override:
            text = 'Code: ' + override
            if len(text) > CODE_MAX_CHARS:
                text = text[:CODE_MAX_CHARS - 3] + '...'
        else:
            text = self.format_line(self.scan_processes())
        self.text = text          # single assignment: frame reads the path always consistently
        return text

    def start(self):
        if self._thread is not None:
            return
        try:
            self.refresh()        # Let the first squares be correct too
        except Exception as exc:
            print(f"WARNING: active code scan failed: {exc}")
        self._thread = threading.Thread(target=self._loop, name='active-code', daemon=True)
        self._thread.start()

    def _loop(self):
        while not self._stop.wait(self.period):
            try:
                self.refresh()
            except Exception:
                pass              # Overlay should not disturb my guidance under any circumstances

    def stop(self):
        self._stop.set()
        if self._redis_owned and self.redis is not None:
            try:
                self.redis.close()
            except Exception:
                pass


class TelemetryTracker:
    """Cache formatted hunter and target positions read periodically from Redis.

Following ActiveCodeTracker, a background thread reads Redis and parses
JSON every TELEM_SCAN_PERIOD_S. The frame path only reads self.hunter_text
and self.target_text. It performs no per-frame Redis calls or JSON parsing.
This uses two reads per second instead of sixty for two keys at 30 Hz.

When a source is missing, render '-' values rather than hiding the row.
The overlay layout stays stable and missing data remains visible.
    """

    def __init__(self, redis_client=None, period=TELEM_SCAN_PERIOD_S):
        self.period = float(period)
        self.redis = redis_client
        self._redis_owned = False
        self._warned = set()
        self._stop = threading.Event()
        self._thread = None
        # Leave blank (dash) lines ready for the first frames.
        self.hunter_text = self._format_value('Hunter ', (None, None, None))
        self.target_text = self._format_value('Target', (None, None, None))

    # --- internal helpers -------------------------------------------------
    def _ensure_redis(self):
        if self.redis is not None:
            return self.redis
        self.redis = _redis_with_short_timeout()
        self._redis_owned = self.redis is not None
        return self.redis

    @staticmethod
    def _extract_position(raw):
        """From the raw Redis value (latitude, longitude, altitude); non-area None."""
        if not raw:
            return (None, None, None)
        if isinstance(raw, bytes):
            raw = raw.decode('utf-8', 'replace')
        data = json.loads(raw)
        # Contest server format: FIRST element of the array konumBilgileri.  Some publishers export fields flat at the root level; both accepted.
        location_data = data
        if isinstance(data, dict):
            records = data.get('konumBilgileri')
            if isinstance(records, list) and records and isinstance(records[0], dict):
                location_data = records[0]
        if not isinstance(location_data, dict):
            return (None, None, None)

        def _number(key_name):
            try:
                return float(location_data.get(key_name))
            except (TypeError, ValueError):
                return None

        return (_number('iha_enlem'), _number('iha_boylam'), _number('iha_irtifa'))

    @staticmethod
    def _format_value(label, location_data):
        """Single line overlay text. The missing field becomes COORD_PLACEHOLDER.

        Latitude/longitude 6 digits: ~0.1 m resolution, more is unnecessary to distinguish targets and lengthens the line.
        """
        latitude, longitude, altitude = location_data
        e = f"{latitude:.6f}" if latitude is not None else COORD_PLACEHOLDER
        b = f"{longitude:.6f}" if longitude is not None else COORD_PLACEHOLDER
        i = f"{altitude:.0f} m" if altitude is not None else COORD_PLACEHOLDER
        return f"{label}: Lat {e}  Lon {b}  Alt {i}"

    def _read(self, client, key):
        try:
            return self._extract_position(client.get(key))
        except Exception as exc:
            if key not in self._warned:
                self._warned.add(key)
                print(f"WARNING: cannot read or decode '{key}'; coordinate line will show "
                      f"'{COORD_PLACEHOLDER}' ({exc}).")
            return (None, None, None)

    # --- public interface -----------------------------------------------------
    def refresh(self):
        client = self._ensure_redis()
        empty_value = (None, None, None)
        hunter = self._read(client, HUNTER_REDIS_KEY) if client is not None else empty_value
        target = self._read(client, TARGET_REDIS_KEY) if client is not None else empty_value
        # Single assignment (two independent strings): frame always reads the path consistently.
        self.hunter_text = self._format_value('Hunter ', hunter)
        self.target_text = self._format_value('Target', target)

    def start(self):
        if self._thread is not None:
            return
        try:
            self.refresh()        # Let the first squares be correct too
        except Exception as exc:
            print(f"WARNING: coordinate reading failed: {exc}")
        self._thread = threading.Thread(target=self._loop, name='coordinates', daemon=True)
        self._thread.start()

    def _loop(self):
        while not self._stop.wait(self.period):
            try:
                self.refresh()
            except Exception:
                pass              # Overlay should not disturb my guidance under any circumstances

    def stop(self):
        self._stop.set()
        if self._redis_owned and self.redis is not None:
            try:
                self.redis.close()
            except Exception:
                pass


class VideoRecorder:
    """Record frames with bounding boxes to MP4 or AVI.

Recording works independently of display, including in headless mode.
Try mp4v with .mp4 first, then XVID with .avi if VideoWriter fails to
open. If both fail, disable recording instead of leaving an empty file.
    """

    WARMUP_FRAMES = 5          # initial burst frames distort measurement, are discarded
    PROBE_FRAMES = 30          # sample frame count for fps automatic measurement
    PROBE_MIN_S = 1.0          # We want a measurement that lasts at least this long.
    PROBE_TIMEOUT_S = 4.0      # Do not hold measurement indefinitely on slow broadcast

    def __init__(self, path=None, fps=0.0):
        self.requested_path = path or None
        self.target_fps = float(fps) if fps and fps > 0 else 0.0
        self.writer = None
        self.path = None
        self.fourcc_name = None
        self.fps = 0.0
        self.size = None
        self.frames = 0
        self.dropped = 0
        self.failed = False
        self.closed = False
        self.lock = threading.Lock()
        self._probe_t0 = None
        self._probe_count = 0
        self._warmup_seen = 0
        self._last_write = 0.0

    # --- internal helpers -------------------------------------------------
    def _resolve_path(self, suffix):
        if self.requested_path:
            path = os.path.abspath(os.path.expanduser(self.requested_path))
            if os.path.isdir(path):
                name = 'guidance_' + datetime.datetime.now().strftime('%Y%m%d_%H%M%S') + suffix
                return os.path.join(path, name)
            root, _ext = os.path.splitext(path)
            return root + suffix
        name = 'guidance_' + datetime.datetime.now().strftime('%Y%m%d_%H%M%S') + suffix
        return os.path.join(DEFAULT_VIDEO_DIR, name)

    def _open(self, size, fps):
        for fourcc_name, suffix in (('mp4v', '.mp4'), ('XVID', '.avi')):
            path = self._resolve_path(suffix)
            try:
                os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
            except OSError as exc:
                print(f"Failed to create video folder ({path}): {exc}")
                self.failed = True
                return False
            writer = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*fourcc_name), fps, size)
            if writer.isOpened():
                self.writer = writer
                self.path = path
                self.fourcc_name = fourcc_name
                self.fps = fps
                self.size = size
                print(f"Video recording started: {path} ({fourcc_name}, {size[0]}x{size[1]}, {fps:.1f} fps)")
                return True
            writer.release()
            # If the codec is not opened, OpenCV sometimes leaves a 0-byte file; clear
            try:
                if os.path.isfile(path) and os.path.getsize(path) == 0:
                    os.remove(path)
            except OSError:
                pass
            print(f"WARNING: Codec '{fourcc_name}' was not opened ({path}), trying next codec.")
        print("ERROR: no video codec could be opened; recording is disabled.")
        self.failed = True
        return False

    # --- public interface -----------------------------------------------------
    def write(self, frame):
        if self.failed or self.closed or frame is None:
            return
        now = time.time()
        with self.lock:
            if self.closed:
                return
            if self.writer is None:
                height, width = frame.shape[:2]
                if self.target_fps > 0:
                    if not self._open((width, height), self.target_fps):
                        return
                else:
                    # If fps is not given, measure actual broadcast rate (first ~1.5 s is skipped).
                    if self._warmup_seen < self.WARMUP_FRAMES:
                        self._warmup_seen += 1
                        return
                    if self._probe_t0 is None:
                        self._probe_t0 = now
                        self._probe_count = 0
                        return
                    self._probe_count += 1
                    elapsed = now - self._probe_t0
                    enough = (self._probe_count >= self.PROBE_FRAMES and elapsed >= self.PROBE_MIN_S)
                    if not enough and elapsed < self.PROBE_TIMEOUT_S:
                        return
                    measured = (self._probe_count / elapsed) if elapsed > 0 else 30.0
                    fps = min(max(round(measured, 1), 1.0), 60.0)
                    if not self._open((width, height), fps):
                        return
            # If open fps is desired, dilute the frames to that level (real-time playback + small file).
            if self.target_fps > 0 and self._last_write:
                if (now - self._last_write) < (0.9 / self.target_fps):
                    self.dropped += 1
                    return
            if self.size and (frame.shape[1], frame.shape[0]) != self.size:
                frame = cv2.resize(frame, self.size)
            self.writer.write(frame)
            self._last_write = now
            self.frames += 1

    def close(self):
        with self.lock:
            if self.closed:
                return
            self.closed = True
            writer, self.writer = self.writer, None
        if writer is not None:
            writer.release()
            size_bytes = os.path.getsize(self.path) if self.path and os.path.isfile(self.path) else 0
            print(f"Video recording turned off: {self.path} "
                  f"({self.frames} frame, {self.fps:.1f} fps, {size_bytes} byte, "
                  f"{self.dropped} frame diluted)")
        elif not self.failed and self.frames == 0:
            print("Video recording: no frames were written (no image arrived?).")


class SimRedisDetector:
    def __init__(self, display=True, recorder=None, code_overlay=True,
                 coord_overlay=True):
        # The visualization is opened only if there is a DISPLAY and it is not closed.
        self.display = display
        # Video recording is independent of external playback; Drawing is also done in headless.
        self.recorder = recorder
        self.draw = display or (recorder is not None)
        # The "Code: ..." line is only meaningful if drawing; Only then does the scan thread start.
        self.code_tracker = ActiveCodeTracker() if (self.draw and code_overlay) else None
        # The lower left 3D coordinate lines are only meaningful if drawing;  Only then does the reading thread start (the headless broadcast method fails).
        self.telemetry = TelemetryTracker() if (self.draw and coord_overlay) else None
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

        # COLOR SETTINGS
        #Default to purple. BUMBLEBEE_TARGET_COLOR=red restores the red window.
        #BUMBLEBEE_HSV_SMIN and BUMBLEBEE_HSV_VMIN override the color's default
        #saturation and value thresholds, allowing relaxation at long range.
        def _opt_int(name):
            raw = os.environ.get(name, '').strip()
            if not raw:
                return None
            try:
                return max(0, min(255, int(float(raw))))
            except ValueError:
                print(f"WARNING: invalid {name} ({raw}), ignoring.")
                return None

        self.target_color, self.color_ranges = build_color_ranges(
            os.environ.get('BUMBLEBEE_TARGET_COLOR', DEFAULT_TARGET_COLOR),
            _opt_int('BUMBLEBEE_HSV_SMIN'), _opt_int('BUMBLEBEE_HSV_VMIN'))
        window_size = ' + '.join(f"H[{lo[0]}-{hi[0]}] S>={lo[1]} V>={lo[2]}"
                             for lo, hi in self.color_ranges)
        print(f"Target color: {self.target_color.upper()} -> {window_size}")
        self.selector = TemporalBBoxSelector()

        # OPTIONAL SHAPE FILTER, DISABLED BY DEFAULT
        #Beyond about 2 km, Gazebo can render regularly spaced ground artifacts
        #one pixel high in the aircraft colors. Dilation expands them into
        #contours about 5 px high, and largest-contour selection can choose them
        #instead of the target. Two optional thresholds reject these artifacts:
        #  BUMBLEBEE_MIN_BLOB_H: minimum bounding-box height in pixels.
        #  BUMBLEBEE_MIN_BLOB_FILL: minimum contour-area to box-area ratio,
        #                         which is small for thin diagonal lines.
        #Leaving both variables unset preserves the unfiltered behavior.
        self.min_blob_h = float(os.environ.get('BUMBLEBEE_MIN_BLOB_H', '0') or 0)
        self.min_blob_fill = float(os.environ.get('BUMBLEBEE_MIN_BLOB_FILL', '0') or 0)
        if self.min_blob_h > 0 or self.min_blob_fill > 0:
            print(f"Figure gate open: min_h={self.min_blob_h:.0f} px, "
                  f"min_doluluk={self.min_blob_fill:.2f}")

        # --- "CODE CURRENTLY WORKING" LINE + AIRPLANE NAME --- Airplane name is a ready-made string; Only drawn per frame (measurement cached). This affects only the canvas, leaving measurement and publication unchanged.
        self.aircraft_label = None
        self._fit_cache = {}
        if self.draw:
            aircraft = resolve_aircraft_name()
            self.aircraft_label = f"[{aircraft}]" if aircraft else None
            print(f"Aircraft name overlay: {self.aircraft_label or '- (BUMBLEBEE_AIRCRAFT not set)'}")
        if self.code_tracker is not None:
            self.code_tracker.start()
            print(f"Active code overlay on ({CODE_SCAN_PERIOD_S:.0f} one scan in s, "
                  f"Key '{CODE_REDIS_KEY}' takes precedence) -> {self.code_tracker.text}")
        if self.telemetry is not None:
            self.telemetry.start()
            print(f"3D coordinate overlay on ({TELEM_SCAN_PERIOD_S:.0f} "
                  f"'{HUNTER_REDIS_KEY}' / '{TARGET_REDIS_KEY}' being read)")
            print(f"  {self.telemetry.hunter_text}")
            print(f"  {self.telemetry.target_text}")

    def _fit_code_line(self, code_text, w_img):
        """Truncate the Code line to avoid overlapping the aircraft name.

The aircraft name is right-aligned. Measure the remaining width and
append '...' when truncation is needed, extending format_line's +N
summary. Cache by (code_text, aircraft_x). Since code_text changes once per
second, each frame performs a dictionary lookup rather than getTextSize.
        """
        key = (code_text, w_img)
        cached = self._fit_cache.get(key)
        if cached is not None:
            return cached
        if not self.aircraft_label:
            result = (code_text, None)
        else:
            label_w = text_width(self.aircraft_label)
            aircraft_x = max(CODE_LINE_X, w_img - AIRCRAFT_RIGHT_MARGIN - label_w)
            avail = aircraft_x - AIRCRAFT_MIN_GAP - CODE_LINE_X
            if avail <= 0:
                result = ('', aircraft_x)          # the frame is too narrow: only the plane name
            elif text_width(code_text) <= avail:
                result = (code_text, aircraft_x)
            else:
                lo, hi = 0, len(code_text)     # longest-fitting ten-add: binary search
                while lo < hi:
                    mid = (lo + hi + 1) // 2
                    if text_width(code_text[:mid] + '...') <= avail:
                        lo = mid
                    else:
                        hi = mid - 1
                result = ((code_text[:lo] + '...') if lo else '', aircraft_x)
        if len(self._fit_cache) > 32:          # bound cache growth
            self._fit_cache.clear()
        self._fit_cache[key] = result
        return result

    def image_callback(self, data):
        try:
            # Convert message ROS to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

        h_img, w_img, _ = cv_image.shape

        # --- Target Hit Area (Yellow Box) Calculation --- We define the inner box by leaving 25% horizontal and 10% vertical space.
        av_left = int(w_img * 0.25)
        av_right = int(w_img * 0.75)
        av_top = int(h_img * 0.10)
        av_bottom = int(h_img * 0.90)

        # Detection is ALWAYS done on the raw frame; drawings are applied to a separate copy (canvas). Thus, video recording/window open does not affect the broadcasted tracker_bbox output in any way.
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        # Color window: purple in one piece, red in two pieces (spirals at H=0).
        lower, upper = self.color_ranges[0]
        mask = cv2.inRange(hsv, lower, upper)
        for lower, upper in self.color_ranges[1:]:
            mask = mask + cv2.inRange(hsv, lower, upper)
        mask = cv2.dilate(mask, None, iterations=2)

        # Find contours (shapes)
        contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Optional shape gate (see description above; default OFF).
        if contours and (self.min_blob_h > 0 or self.min_blob_fill > 0):
            kept = []
            for c in contours:
                _x, _y, _w, _h = cv2.boundingRect(c)
                if _h < self.min_blob_h:
                    continue
                if self.min_blob_fill > 0:
                    box_area = float(_w * _h)
                    if box_area <= 0 or (cv2.contourArea(c) / box_area) < self.min_blob_fill:
                        continue
                kept.append(c)
            contours = kept

        valid_detection = False
        lock_eligible = False
        bbox = []

        if len(contours) > 0:
            candidates = [cv2.boundingRect(c) for c in contours
                          if cv2.contourArea(c) > 10]
            selected = self.selector.select(candidates, w_img, h_img)

            if selected is not None:
                x, y, w, h = selected
                valid_detection = True

                # --- Checking Lock Conditions --- Condition 1: Is the detected target box completely inside the Yellow Box (Av)?
                is_inside = (x >= av_left) and ((x + w) <= av_right) and (y >= av_top) and ((y + h) <= av_bottom)

                # Condition 2: Does the target take up 5% horizontally or 5% vertically?
                is_large_enough = (w >= (w_img * 0.05)) or (h >= (h_img * 0.05))

                # Locking is appropriate if both conditions are met
                if is_inside and is_large_enough:
                    lock_eligible = True

                # Calculate horizontal coverage for the format your autopilot expects
                horizontal_coverage = (w / w_img) * 100
                validity_flag = 1

                # Your new autopilot expects this format: [x, y, w, h, horizontal_cov, validity]
                bbox = [int(x), int(y), int(w), int(h), horizontal_coverage, validity_flag]

                # Publish via Redis (in json format — parses guidance code with json.loads)
                self.r.publish('tracker_bbox', json.dumps(bbox))

        if not valid_detection:
            # If there is no target, you can start an empty list or a list with validity = 0.
            pass

        # --- DRAW + OVERLAY (for window and/or video recording only) ---
        if not self.draw:
            return

        canvas = cv_image.copy()
        # Draw the Target Hit Area (Prey) (Yellow Color)
        cv2.rectangle(canvas, (av_left, av_top), (av_right, av_bottom), (0, 255, 255), 2)
        # The label is written on the BOTTOM of the box; at the top it overlapped with the upper-left overlay block.
        draw_text(canvas, "Target Hit Area", (av_left, min(av_bottom + 22, h_img - 6)),
                  (0, 255, 255), 0.5, 1)

        if valid_detection:
            x, y, w, h = bbox[0], bbox[1], bbox[2], bbox[3]
            cv2.rectangle(canvas, (x, y), (x + w, y + h), (0, 0, 255), 2)
            cv2.circle(canvas, (int(x + w / 2), int(y + h / 2)), 5, (0, 255, 0), -1)
            if lock_eligible:
                draw_text(canvas, "LOCKOUT FIT",
                          (int(w_img / 2) - 150, int(h_img / 2) + 150), (0, 255, 0), 1.0, 3)

        # Top left corner overlay: timestamp + detection status (only the image, the published bbox export is left untouched).  A single now() call: two separate calls could produce discrepancies between seconds and split-seconds (23:59:59.99 -> 00:00:00.00).
        now = datetime.datetime.now()
        stamp = now.strftime('%Y-%m-%d %H:%M:%S.') + f"{now.microsecond // 10000:02d}"
        draw_text(canvas, stamp, (20, 32), (255, 255, 255), 0.6, 2)
        if valid_detection:
            status = (f"Target({self.target_color}): FOUND x={bbox[0]} y={bbox[1]} w={bbox[2]} h={bbox[3]} "
                      f"coverage={bbox[4]:.1f}%" + ("  [LOCK AVAILABLE]" if lock_eligible else ""))
            color = (0, 255, 0)
        else:
            status = f"Target({self.target_color}): NONE"
            color = (0, 0, 255)
        draw_text(canvas, status, (20, 60), color, 0.6, 2)
        # guidance/test scripts currently running (canvas only; ready string, in scan background thread).
        if self.code_tracker is not None:
            code_text, aircraft_x = self._fit_code_line(self.code_tracker.text, w_img)
            draw_text(canvas, code_text, (CODE_LINE_X, CODE_LINE_Y), (255, 255, 0), 0.6, 2)
            if aircraft_x is not None:
                draw_text(canvas, self.aircraft_label, (aircraft_x, CODE_LINE_Y), (255, 255, 0), 0.6, 2)

        # LOWER-LEFT HUNTER AND TARGET POSITIONS
        #The background thread reads Redis, parses JSON and formats both rows
        #once per second. This frame path only draws them: two attribute reads
        #and two draw_text calls, using four LINE_8 putText operations.
        #
        #Measured on this machine at 1080p with one OpenCV thread, 1500 repeats:
        #  Two rows, scale 0.6 and thickness 2: 0.69 ms per frame.
        #  This is 2.1% of the 33.3 ms budget at 30 Hz, or roughly 2% CPU.
        #  It is about 8% of the 8.8 ms detection path comprising cvtColor,
        #  inRange, dilate and findContours.
        #The cost is comparable to the existing status row, measured at 325 us
        #per line with LINE_8. Thickness 1 reduces the two-row cost to 0.41 ms,
        #but thickness 2 is retained for legibility. There are no per-frame
        #Redis reads or JSON parsing calls.
        if self.telemetry is not None:
            target_y = h_img - COORD_BOTTOM_MARGIN
            draw_text(canvas, self.telemetry.hunter_text,
                      (COORD_LINE_X, target_y - COORD_LINE_GAP), (255, 255, 255), 0.6, 2)
            draw_text(canvas, self.telemetry.target_text,
                      (COORD_LINE_X, target_y), (255, 255, 255), 0.6, 2)

        if self.recorder is not None:
            self.recorder.write(canvas)

        # Show image on screen
        if self.display:
            cv2.imshow("Simulation Redis Detector", canvas)
            cv2.waitKey(1)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Camera → tracker_bbox (Redis) color detection bridge')
    parser.add_argument('--no-display', action='store_true',
                        help='Opening the OpenCV window (headless). If there is no DISPLAY, it is applied automatically.')
    parser.add_argument('--record', nargs='?', const='', default=None, metavar='FILE',
                        help='bbox save drawn frames to video. If the file is not given '
                             'videos/guidance_YYYYmmdd_HHMMSS.mp4. Same effect as BUMBLEBEE_VIDEO=1.')
    parser.add_argument('--record-fps', type=float, default=0.0, metavar='FPS',
                        help='Record fps (default 0 = auto measure from incoming broadcast). '
                             'If exported, the frames are diluted to this alignment (smaller file).')
    parser.add_argument('--no-code-overlay', action='store_true',
                        help='"Code:..." line (currently running guidance/test script\'s) '
                             'boot. Same effect as BUMBLEBEE_CODE_OVERLAY=0.')
    parser.add_argument('--no-coord-overlay', action='store_true',
                        help='Drawing the 3D coordinate lines "Hunter/Target" in the bottom left. '
                             'Same effect as BUMBLEBEE_COORD_OVERLAY=0.')
    args = parser.parse_args()

    # Record: --record flag OR BUMBLEBEE_VIDEO env variable (launcher path).
    env_video = os.environ.get('BUMBLEBEE_VIDEO', '').strip().lower()
    record_enabled = (args.record is not None) or (env_video not in ('', '0', 'false', 'no', 'off'))
    env_fps = os.environ.get('BUMBLEBEE_VIDEO_FPS', '').strip()
    record_fps = args.record_fps
    if record_fps <= 0 and env_fps:
        try:
            record_fps = float(env_fps)
        except ValueError:
            print(f"WARNING: BUMBLEBEE_VIDEO_FPS invalid ({env_fps}), automatic measurement will be used.")

    recorder = VideoRecorder(args.record or None, record_fps) if record_enabled else None
    if recorder is not None:
        # Three separate safety nets to avoid producing incomplete MP4s: atexit, rospy shutdown hook and finally. close() is protected against re-calling.
        atexit.register(recorder.close)

    # "Code:..." line DEFAULT ON; It is turned off with --no-code-overlay or BUMBLEBEE_CODE_OVERLAY=0/false/no/off.
    env_code = os.environ.get('BUMBLEBEE_CODE_OVERLAY', '').strip().lower()
    code_overlay = (not args.no_code_overlay) and (env_code not in ('0', 'false', 'no', 'off'))

    # Bottom left coordinate lines are also DEFAULT ON; It is turned off with --no-coord-overlay or BUMBLEBEE_COORD_OVERLAY=0/false/no/off.
    env_coord = os.environ.get('BUMBLEBEE_COORD_OVERLAY', '').strip().lower()
    coord_overlay = (not args.no_coord_overlay) and (env_coord not in ('0', 'false', 'no', 'off'))

    # If DISPLAY is not present or --no-display is given, do not open a window.
    display = (not args.no_display) and bool(os.environ.get('DISPLAY'))
    detector = None
    try:
        detector = SimRedisDetector(display=display, recorder=recorder,
                                    code_overlay=code_overlay,
                                    coord_overlay=coord_overlay)
        # rospy catches the lone SIGINT; When launcher/stop.sh sends SIGTERM, we turn SIGTERM into regular closing so that the writer is released properly.
        signal.signal(signal.SIGTERM, lambda _signum, _frame: rospy.signal_shutdown('SIGTERM'))
        if recorder is not None:
            rospy.on_shutdown(recorder.close)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        if detector is not None and detector.code_tracker is not None:
            detector.code_tracker.stop()
        if detector is not None and detector.telemetry is not None:
            detector.telemetry.stop()
        if recorder is not None:
            recorder.close()
        if display:
            cv2.destroyAllWindows()
