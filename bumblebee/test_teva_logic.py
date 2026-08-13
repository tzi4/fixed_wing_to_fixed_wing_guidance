#!/usr/bin/env python3

import json
import math
import queue
import sys
import threading
import unittest
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from bbox_to_redis import TemporalBBoxSelector
from teva import (AutopilotController, CAMERA_PROFILES, MavlinkManager,
                  RedisListener, TargetLockTracker)


class CameraProfileTests(unittest.TestCase):
    def test_real_flight_reference_is_preserved_exactly(self):
        self.assertEqual(CAMERA_PROFILES['real'], {
            'fx': 4515.0, 'fy': 4510.0, 'cx': 1091.0, 'cy': 633.0,
            'mount_phys_deg': -1.0, 'aim_deg': -5.0,
        })

    def test_erenimbus_sim_profile_is_separate(self):
        self.assertEqual(CAMERA_PROFILES['sim'], {
            'fx': 4543.0, 'fy': 4539.0, 'cx': 1025.0, 'cy': 569.0,
            'mount_phys_deg': 0.0, 'aim_deg': 0.0,
        })


class TemporalBBoxSelectorTests(unittest.TestCase):
    def test_reacquires_nearest_image_center(self):
        selector = TemporalBBoxSelector()
        selected = selector.select(
            [(20, 20, 100, 100), (950, 530, 20, 20)], 1920, 1080, now=1.0)
        self.assertEqual(selected, (950, 530, 20, 20))

    def test_does_not_jump_to_distant_blob(self):
        selector = TemporalBBoxSelector(track_timeout_s=0.5)
        selector.select([(950, 530, 20, 20)], 1920, 1080, now=1.0)
        self.assertIsNone(
            selector.select([(10, 10, 300, 100)], 1920, 1080, now=1.05))

    def test_merges_nearby_airframe_fragments(self):
        selector = TemporalBBoxSelector()
        selected = selector.select(
            [(880, 240, 50, 20), (850, 242, 15, 8)], 1920, 1080, now=1.0)
        self.assertEqual(selected, (850, 240, 80, 20))


class TargetLockTrackerTests(unittest.TestCase):
    QUALIFYING = (800, 300, 100, 30)  # genislik 1920'nin %5'inden buyuk

    def test_requires_four_continuous_seconds(self):
        tracker = TargetLockTracker(required_s=4.0, max_sample_gap_s=0.25)
        for index in range(40):
            self.assertFalse(tracker.update(self.QUALIFYING, now=10 + index / 10))
        self.assertTrue(tracker.update(self.QUALIFYING, now=14.0))

    def test_zone_exit_and_long_gap_reset(self):
        tracker = TargetLockTracker(required_s=4.0, max_sample_gap_s=0.25)
        tracker.update(self.QUALIFYING, now=1.0)
        tracker.update(self.QUALIFYING, now=2.0)
        tracker.update((100, 300, 100, 30), now=2.1)
        self.assertEqual(tracker.progress_s, 0.0)
        tracker.update(self.QUALIFYING, now=3.0)
        tracker.update(None, now=3.3)
        self.assertEqual(tracker.progress_s, 0.0)

    def test_geometry_enforces_size_and_full_containment(self):
        tracker = TargetLockTracker()
        self.assertTrue(tracker.qualifies(self.QUALIFYING))
        self.assertFalse(tracker.qualifies((800, 300, 40, 30)))
        self.assertFalse(tracker.qualifies((470, 300, 100, 30)))


class RedisBBoxTests(unittest.TestCase):
    def test_accepts_json_and_clips_to_image(self):
        data = json.loads('[-10, 100, 40, 20, 2.0, 1]')
        self.assertEqual(RedisListener.parse_bbox(data), (0, 100, 30, 20))

    def test_rejects_invalid_or_nonfinite_bbox(self):
        self.assertIsNone(RedisListener.parse_bbox([1, 2, 3, 4, 0, 0]))
        self.assertIsNone(RedisListener.parse_bbox([1, 2, float('nan'), 4]))
        self.assertIsNone(RedisListener.parse_bbox([1, 2, -3, 4]))

    def test_latest_queue_overwrites_old_frame(self):
        listener = object.__new__(RedisListener)
        listener.data_queue = queue.Queue(maxsize=1)
        listener._put_latest({'frame': 1})
        listener._put_latest({'frame': 2})
        self.assertEqual(listener.data_queue.get_nowait(), {'frame': 2})


class HeadingCommandTests(unittest.TestCase):
    def test_heading_rate_is_converted_to_centripetal_acceleration(self):
        sent = []

        class FakeMav:
            def command_long_send(self, *args):
                sent.append(args)

        class FakeMaster:
            target_system = 1
            target_component = 1
            mav = FakeMav()

        manager = object.__new__(MavlinkManager)
        manager.master = FakeMaster()
        manager.lock = threading.Lock()
        manager.current_airspeed = 20.0
        manager.current_groundspeed = 20.0
        manager.commands_allowed = lambda: True
        self.assertTrue(manager.send_heading_target(123.0, 5.0))
        self.assertEqual(len(sent), 1)
        # command_long: target sys/comp, command, confirmation, param1..7
        self.assertAlmostEqual(sent[0][6], 20.0 * math.radians(5.0), places=6)


class SpeedControlTests(unittest.TestCase):
    @staticmethod
    def controller():
        class Telemetry:
            current_airspeed = 20.0

        ctl = object.__new__(AutopilotController)
        ctl.mavlink = Telemetry()
        ctl.camera_width = 1920.0
        ctl.camera_height = 1080.0
        ctl.Kp_speed = 0.30
        ctl.Ki_speed = 0.02
        ctl.Kd_speed = 0.0
        ctl.min_speed = 10.0
        ctl.max_speed = 22.0
        ctl.base_speed = 20.0
        ctl.target_coverage_pct = 6.0
        ctl.target_range_m = 52.0
        ctl.range_near_m = 45.0
        ctl.edge_speed_trim_ms = 1.5
        ctl.max_approach_delta_near_ms = 1.0
        ctl.max_approach_delta_far_ms = 2.0
        ctl.speed_integral_band = 7.0
        ctl.speed_slew_rate = 0.15
        ctl.coverage_alpha = 0.35
        ctl.filtered_coverage = 6.0
        ctl.integral_error_speed = 0.0
        ctl.prev_error_speed = 0.0
        ctl.prev_derivative_speed = 0.0
        ctl.last_speed = 20.0
        ctl._speed_seeded = False
        return ctl

    def test_small_target_accelerates_with_slew_limit(self):
        ctl = self.controller()
        command, coverage, *_ = ctl._speed_control(50, 20, 0.1, first_sample=True)
        self.assertLess(coverage, ctl.target_coverage_pct)
        self.assertGreater(command, 20.0)
        self.assertLessEqual(command, 20.2 + 1e-9)

    def test_large_target_decelerates(self):
        ctl = self.controller()
        command, coverage, *_ = ctl._speed_control(200, 80, 0.1, first_sample=True)
        self.assertGreater(coverage, ctl.target_coverage_pct)
        self.assertLess(command, 20.0)

    def test_near_telemetry_prevents_false_acceleration(self):
        ctl = self.controller()
        command, coverage, *_ = ctl._speed_control(
            30, 12, 0.1, first_sample=True,
            bbox_x=900, bbox_y=500, menzil_est=30.0)
        self.assertLess(coverage, ctl.target_coverage_pct)
        self.assertLess(command, 20.0)

    def test_edge_target_decelerates_even_when_small(self):
        ctl = self.controller()
        command, *_ = ctl._speed_control(
            50, 20, 0.1, first_sample=True,
            bbox_x=900, bbox_y=0, menzil_est=100.0)
        self.assertLess(command, 20.0)


class DevirGateTests(unittest.TestCase):
    @staticmethod
    def controller():
        ctl = object.__new__(AutopilotController)
        ctl.devir_kapisi = True
        ctl.engage_bekleme_s = 0.0
        ctl.engage_azami_s = 25.0
        ctl.engage_vz_esik = 0.3
        ctl._engage_t = None
        ctl._engage_alt = None
        ctl._engage_heading = None
        ctl._kapi_acildi_t = None
        ctl._pitch_gecmis = []
        ctl.last_target_alt = 0.0
        ctl.last_target_heading = 0.0
        ctl.speed_control_enabled = False
        ctl.mavlink = type('Telemetry', (), {'current_vz': 0.0})()
        return ctl

    def test_open_gate_is_latched_until_guided_ends(self):
        ctl = self.controller()
        for index in range(40):
            settling = ctl._update_devir_gate(
                10.0 + index * 0.05, True, True, 50.0, 0.0, 90.0)
        self.assertFalse(settling)
        opened_at = ctl._kapi_acildi_t
        ctl.mavlink.current_vz = 5.0
        self.assertFalse(ctl._update_devir_gate(
            13.0, True, True, 42.0, math.radians(10), 120.0))
        self.assertEqual(ctl._kapi_acildi_t, opened_at)
        ctl._update_devir_gate(14.0, False, True, 42.0, 0.0, 120.0)
        self.assertIsNone(ctl._kapi_acildi_t)


if __name__ == '__main__':
    unittest.main()
