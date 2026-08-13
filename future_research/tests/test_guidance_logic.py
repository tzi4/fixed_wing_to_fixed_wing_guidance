#!/usr/bin/env python3

import sys
import unittest
from pathlib import Path

FUTURE_DIR = Path(__file__).resolve().parents[1]
for source_dir in (FUTURE_DIR / 'guidance', FUTURE_DIR / 'detection'):
    if str(source_dir) not in sys.path:
        sys.path.insert(0, str(source_dir))

from bbox_to_redis import TemporalBBoxSelector
from tzi_emir import TargetLockTracker


class TemporalBBoxSelectorTests(unittest.TestCase):
    def test_reacquires_candidate_nearest_image_center(self):
        selector = TemporalBBoxSelector()
        selected = selector.select(
            [(20, 20, 100, 100), (630, 350, 10, 10)],
            1280, 720, now=1.0,
        )
        self.assertEqual(selected, (630, 350, 10, 10))

    def test_does_not_jump_to_distant_false_positive(self):
        selector = TemporalBBoxSelector(track_timeout_s=0.5)
        selector.select([(630, 350, 10, 10)], 1280, 720, now=1.0)
        selected = selector.select([(10, 10, 200, 100)], 1280, 720, now=1.05)
        self.assertIsNone(selected)

    def test_reacquires_after_timeout(self):
        selector = TemporalBBoxSelector(track_timeout_s=0.5)
        selector.select([(630, 350, 10, 10)], 1280, 720, now=1.0)
        selected = selector.select([(100, 100, 10, 10)], 1280, 720, now=1.6)
        self.assertEqual(selected, (100, 100, 10, 10))

    def test_merges_nearby_airframe_fragments(self):
        selector = TemporalBBoxSelector()
        selected = selector.select(
            [(580, 142, 31, 16), (563, 143, 7, 5)],
            1280, 720, now=1.0,
        )
        self.assertEqual(selected, (563, 142, 48, 16))


class TargetLockTrackerTests(unittest.TestCase):
    QUALIFYING = (500, 200, 70, 20)  # width >= 1280'in %5'i

    def test_requires_four_continuous_seconds(self):
        tracker = TargetLockTracker(required_s=4.0, max_sample_gap_s=0.25)
        self.assertFalse(tracker.update(self.QUALIFYING, now=10.0))
        for index in range(1, 40):
            self.assertFalse(tracker.update(self.QUALIFYING, now=10.0 + index / 10.0))
        self.assertTrue(tracker.update(self.QUALIFYING, now=14.0))
        self.assertTrue(tracker.acquired)

    def test_zone_exit_resets_progress(self):
        tracker = TargetLockTracker(required_s=4.0)
        tracker.update(self.QUALIFYING, now=1.0)
        tracker.update(self.QUALIFYING, now=2.0)
        tracker.update((0, 200, 70, 20), now=2.1)
        self.assertEqual(tracker.progress_s, 0.0)
        self.assertIsNone(tracker.started_at)

    def test_short_frame_gap_is_tolerated_but_long_gap_resets(self):
        tracker = TargetLockTracker(required_s=4.0, max_sample_gap_s=0.25)
        tracker.update(self.QUALIFYING, now=1.0)
        tracker.update(None, now=1.20)
        tracker.update(self.QUALIFYING, now=1.21)
        self.assertAlmostEqual(tracker.progress_s, 0.21)
        tracker.update(None, now=1.50)
        self.assertEqual(tracker.progress_s, 0.0)

    def test_size_and_full_containment_are_both_enforced(self):
        tracker = TargetLockTracker()
        self.assertFalse(tracker.qualifies((500, 200, 20, 20)))
        self.assertFalse(tracker.qualifies((300, 200, 70, 20)))
        self.assertTrue(tracker.qualifies(self.QUALIFYING))


if __name__ == '__main__':
    unittest.main()
