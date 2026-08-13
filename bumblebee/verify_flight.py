#!/usr/bin/env python3
"""Record MAVLink and ROS camera health, then emit CSV/JSON acceptance reports."""

import argparse
import csv
import json
import math
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path

from pymavlink import mavutil


CRITICAL_WORDS = ('crash', 'failsafe', 'ekf variance', 'ekf failsafe', 'internal error')


@dataclass
class VehicleState:
    port: int
    expected_sysid: int
    name: str
    heartbeat: bool = False
    armed_seen: bool = False
    auto_seen: bool = False
    airborne_seen: bool = False
    unexpected_disarm: bool = False
    nan_telemetry: bool = False
    wp_min: int = 999999
    wp_max: int = -1
    airspeeds: list = field(default_factory=list)
    max_alt: float = 0.0
    rel_alt: float = 0.0
    lat: float = 0.0
    lon: float = 0.0
    have_gps: bool = False
    roll: float = 0.0
    pitch: float = 0.0
    airspeed: float = 0.0
    mode: str = ''
    armed: bool = False
    attitude_bad_since: float = 0.0
    sustained_divergence: bool = False
    critical_messages: list = field(default_factory=list)


class CameraMonitor:
    def __init__(self, topic):
        self.topic = topic
        self.frames = 0
        self.width = 0
        self.height = 0
        self.first = None
        self.last = None
        self.max_gap = 0.0
        self.error = None
        self._previous = None
        try:
            import rospy
            from sensor_msgs.msg import Image
            self.rospy = rospy
            if not rospy.core.is_initialized():
                rospy.init_node('bumblebee_verify_flight', anonymous=True, disable_signals=True)
            self.subscriber = rospy.Subscriber(topic, Image, self._callback, queue_size=1)
        except Exception as exc:
            self.error = str(exc)

    def _callback(self, msg):
        now = time.monotonic()
        if self.first is None:
            self.first = now
        if self._previous is not None:
            self.max_gap = max(self.max_gap, now - self._previous)
        self._previous = now
        self.last = now
        self.frames += 1
        self.width, self.height = int(msg.width), int(msg.height)


class BboxMonitor:
    """tracker_bbox Redis pubsub kanalına gelen mesajları belirli bir pencerede sayar."""

    def __init__(self, channel='tracker_bbox', window=60.0):
        self.channel = channel
        self.window = window
        self.count = 0
        self.error = None
        self._stop = threading.Event()
        self._pubsub = None
        try:
            import redis
            self._redis = redis.Redis(host='localhost', port=6379, db=0)
            self._pubsub = self._redis.pubsub()
            self._pubsub.subscribe(channel)
            self._thread = threading.Thread(target=self._run, daemon=True)
            self._thread.start()
        except Exception as exc:
            self.error = str(exc)

    def _run(self):
        deadline = time.monotonic() + self.window
        while not self._stop.is_set() and time.monotonic() < deadline:
            try:
                message = self._pubsub.get_message(timeout=1.0)
            except Exception as exc:
                self.error = str(exc)
                break
            if message and message.get('type') == 'message':
                self.count += 1

    def stop(self):
        self._stop.set()
        if self._pubsub is not None:
            try:
                self._pubsub.close()
            except Exception:
                pass


def separation_3d(a, b):
    """İki aracın anlık 3B mesafesi (m); ikisi de havada (rel_alt>10) değilse None."""
    if not (a.have_gps and b.have_gps):
        return None
    if not (a.rel_alt > 10 and b.rel_alt > 10):
        return None
    lat_ref = math.radians((a.lat + b.lat) / 2.0)
    dnorth = (a.lat - b.lat) * 111320.0
    deast = (a.lon - b.lon) * 111320.0 * math.cos(lat_ref)
    ddown = a.rel_alt - b.rel_alt
    return math.sqrt(dnorth * dnorth + deast * deast + ddown * ddown)


def connect(port, sysid, timeout):
    master = mavutil.mavlink_connection(f'udpin:127.0.0.1:{port}', source_system=253, source_component=192)
    heartbeat = None
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        candidate = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if candidate is not None and candidate.get_srcSystem() == sysid:
            heartbeat = candidate
            break
    received_sysid = 0 if heartbeat is None else heartbeat.get_srcSystem()
    if heartbeat is None or received_sysid != sysid:
        master.close()
        raise RuntimeError(f"port {port}: SysID {sysid} bekleniyordu, {received_sysid} alındı")
    master.target_system = received_sysid
    master.target_component = heartbeat.get_srcComponent()
    for msg_id, hz in ((0, 2), (30, 10), (33, 5), (42, 5), (74, 5), (253, 2)):
        master.mav.command_long_send(sysid, master.target_component, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, msg_id, int(1e6 / hz), 0, 0, 0, 0, 0)
    return master


def finite(*values):
    return all(math.isfinite(float(value)) for value in values)


def consume(master, state, now):
    while True:
        msg = master.recv_match(blocking=False)
        if msg is None:
            return
        if msg.get_srcSystem() != state.expected_sysid:
            continue
        kind = msg.get_type()
        if kind == 'BAD_DATA':
            continue
        if kind == 'HEARTBEAT':
            state.heartbeat = True
            was_armed = state.armed
            state.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            state.mode = mavutil.mode_string_v10(msg)
            state.armed_seen |= state.armed
            state.auto_seen |= state.mode == 'AUTO'
            if was_armed and not state.armed and state.airborne_seen:
                state.unexpected_disarm = True
        elif kind == 'MISSION_CURRENT':
            seq = int(msg.seq)
            state.wp_min, state.wp_max = min(state.wp_min, seq), max(state.wp_max, seq)
        elif kind == 'VFR_HUD':
            if not finite(msg.airspeed, msg.alt):
                state.nan_telemetry = True
            state.airspeed = float(msg.airspeed)
            if state.rel_alt > 20:
                state.airspeeds.append(state.airspeed)
        elif kind == 'GLOBAL_POSITION_INT':
            if not finite(msg.relative_alt, msg.lat, msg.lon):
                state.nan_telemetry = True
            state.rel_alt = float(msg.relative_alt) / 1000.0
            state.max_alt = max(state.max_alt, state.rel_alt)
            state.airborne_seen |= state.rel_alt > 10
            lat, lon = float(msg.lat) / 1e7, float(msg.lon) / 1e7
            if finite(lat, lon) and (lat != 0.0 or lon != 0.0):
                state.lat, state.lon, state.have_gps = lat, lon, True
            if state.airborne_seen and state.armed and state.rel_alt < 1 and state.max_alt > 20:
                state.critical_messages.append('ground impact after takeoff')
        elif kind == 'ATTITUDE':
            if not finite(msg.roll, msg.pitch, msg.yaw):
                state.nan_telemetry = True
                continue
            state.roll, state.pitch = float(msg.roll), float(msg.pitch)
            bad = abs(state.roll) > math.radians(80) or abs(state.pitch) > math.radians(60)
            if bad and state.attitude_bad_since == 0:
                state.attitude_bad_since = now
            elif not bad:
                state.attitude_bad_since = 0
            elif now - state.attitude_bad_since > 5:
                state.sustained_divergence = True
        elif kind == 'STATUSTEXT':
            text = str(msg.text).strip('\x00')
            if any(word in text.lower() for word in CRITICAL_WORDS):
                state.critical_messages.append(text)


def vehicle_checks(state, require_waypoints):
    checks = {
        'heartbeat': state.heartbeat,
        'armed': state.armed_seen,
        'auto': state.auto_seen,
        'airborne': state.airborne_seen,
        'no_unexpected_disarm': not state.unexpected_disarm,
        'finite_telemetry': not state.nan_telemetry,
        'attitude_stable': not state.sustained_divergence,
        'no_critical_status': not state.critical_messages,
    }
    if state.name == 'bumblebee':
        checks['waypoint_progress'] = state.wp_max >= require_waypoints
        in_range = sum(12 <= speed <= 30 for speed in state.airspeeds)
        checks['airspeed_12_30'] = len(state.airspeeds) >= 5 and in_range / len(state.airspeeds) >= 0.8
        checks['altitude_bounded'] = state.max_alt <= 200
    return checks


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=180)
    parser.add_argument('--connect-timeout', type=float, default=90)
    parser.add_argument('--camera-topic', default='/webcam/image_raw')
    parser.add_argument('--report-dir', type=Path, default=Path(__file__).resolve().parent / 'reports' / 'manual')
    parser.add_argument('--require-waypoints', type=int, default=3)
    parser.add_argument('--ready-file', type=Path, help='MAVLink bağlantıları hazır olduğunda oluştur')
    args = parser.parse_args()
    args.report_dir.mkdir(parents=True, exist_ok=True)
    if args.ready_file:
        args.ready_file.unlink(missing_ok=True)

    camera = CameraMonitor(args.camera_topic)
    definitions = ((14553, 1, 'bumblebee'), (14561, 2, 'target'))
    states, masters = [], []
    try:
        for port, sysid, name in definitions:
            state = VehicleState(port, sysid, name)
            master = connect(port, sysid, args.connect_timeout)
            state.heartbeat = True
            states.append(state)
            masters.append(master)
    except Exception as exc:
        for master in masters:
            master.close()
        print(f"Bağlantı hatası: {exc}", file=sys.stderr)
        raise SystemExit(2)
    if args.ready_file:
        args.ready_file.parent.mkdir(parents=True, exist_ok=True)
        args.ready_file.write_text('ready\n', encoding='utf-8')

    stamp = time.strftime('%Y%m%d_%H%M%S')
    csv_path = args.report_dir / f'telemetry_{stamp}.csv'
    started = time.monotonic()
    next_sample = started
    bbox = BboxMonitor(window=min(60.0, args.duration))
    min_separation = None
    with csv_path.open('w', newline='', encoding='utf-8') as handle:
        writer = csv.DictWriter(handle, fieldnames=['elapsed', 'vehicle', 'sysid', 'armed', 'mode', 'wp', 'airspeed', 'relative_alt', 'roll_deg', 'pitch_deg'])
        writer.writeheader()
        while time.monotonic() - started < args.duration:
            now = time.monotonic()
            for master, state in zip(masters, states):
                consume(master, state, now)
            if len(states) == 2:
                sep = separation_3d(states[0], states[1])
                if sep is not None:
                    min_separation = sep if min_separation is None else min(min_separation, sep)
            if now >= next_sample:
                for state in states:
                    writer.writerow({'elapsed': round(now - started, 3), 'vehicle': state.name, 'sysid': state.expected_sysid, 'armed': int(state.armed), 'mode': state.mode, 'wp': state.wp_max, 'airspeed': round(state.airspeed, 3), 'relative_alt': round(state.rel_alt, 3), 'roll_deg': round(math.degrees(state.roll), 3), 'pitch_deg': round(math.degrees(state.pitch), 3)})
                handle.flush()
                next_sample = now + 0.5
            time.sleep(0.02)
    for master in masters:
        master.close()
    bbox.stop()

    separation_ok = min_separation is not None and min_separation >= 20.0
    camera_checks = {
        'subscriber_started': camera.error is None,
        'frames_received': camera.frames >= 10,
        'resolution_1920x1080': (camera.width, camera.height) == (1920, 1080),
        'continuous': camera.frames >= 10 and camera.max_gap <= 2.0 and camera.last is not None and time.monotonic() - camera.last <= 2.0,
    }
    vehicles = {}
    passed = all(camera_checks.values()) and separation_ok
    for state in states:
        checks = vehicle_checks(state, args.require_waypoints)
        passed &= all(checks.values())
        vehicles[state.name] = {
            'sysid': state.expected_sysid,
            'checks': checks,
            'wp_min': None if state.wp_min == 999999 else state.wp_min,
            'wp_max': state.wp_max,
            'max_relative_alt_m': round(state.max_alt, 3),
            'airspeed_samples': len(state.airspeeds),
            'critical_messages': state.critical_messages,
        }
    report = {
        'passed': bool(passed),
        'duration_s': args.duration,
        'min_separation_m': None if min_separation is None else round(min_separation, 3),
        'separation_ok': bool(separation_ok),
        'tracker_bbox_messages_60s': bbox.count,
        'tracker_bbox_error': bbox.error,
        'vehicles': vehicles,
        'camera': {'checks': camera_checks, 'frames': camera.frames, 'width': camera.width, 'height': camera.height, 'max_gap_s': round(camera.max_gap, 3), 'error': camera.error},
        'telemetry_csv': str(csv_path),
    }
    json_path = args.report_dir / f'verification_{stamp}.json'
    json_path.write_text(json.dumps(report, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
    print(json.dumps(report, indent=2, ensure_ascii=False))
    print(f"JSON: {json_path}")
    raise SystemExit(0 if passed else 1)


if __name__ == '__main__':
    main()
