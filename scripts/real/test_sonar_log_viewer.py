"""Offline checks for the Surveyor log viewer."""

from pathlib import Path
import unittest

from sonar_log_viewer import cross_track_depth, load_raw_intensity, scan_svlog


ROOT = Path(__file__).resolve().parents[2]
SAMPLE_DIR = ROOT / "records" / "sonar" / "rover_20260910_SonarView"


class SonarLogViewerTests(unittest.TestCase):
    def test_surveyor_log_has_points_and_raw_tiles(self):
        path = SAMPLE_DIR / "2026-09-10-12-19.svlog"
        if not path.exists():
            self.skipTest("downloaded rover sample is not present")
        log = scan_svlog(path)
        self.assertGreater(len(log.pings), 700)
        self.assertGreater(log.total_points, 9000)
        self.assertTrue(log.pings[0].has_raw_intensity)

    def test_cross_track_projection_uses_radians(self):
        path = SAMPLE_DIR / "2026-09-10-12-19.svlog"
        if not path.exists():
            self.skipTest("downloaded rover sample is not present")
        log = scan_svlog(path)
        point = log.pings[0].points[0]
        projected = cross_track_depth(log.pings[0])[0]
        self.assertAlmostEqual(projected[0], point[1] * __import__("math").sin(point[0]), places=6)
        self.assertAlmostEqual(projected[1], -point[1] * __import__("math").cos(point[0]), places=6)

    def test_raw_intensity_is_lazy_and_rectangular(self):
        path = SAMPLE_DIR / "2026-09-10-12-19.svlog"
        if not path.exists():
            self.skipTest("downloaded rover sample is not present")
        log = scan_svlog(path)
        decoded = load_raw_intensity(log, log.pings[0])
        self.assertIsNotNone(decoded)
        matrix, range_start, range_end = decoded
        self.assertEqual(len(matrix), 200)
        self.assertEqual(len(matrix[0]), 128)
        self.assertLess(range_start, range_end)

    def test_detections_only_log_has_no_raw_intensity(self):
        path = SAMPLE_DIR / "2026-09-10-13-14.svlog"
        if not path.exists():
            self.skipTest("downloaded rover sample is not present")
        log = scan_svlog(path)
        self.assertTrue(log.pings)
        self.assertFalse(log.pings[0].has_raw_intensity)


if __name__ == "__main__":
    unittest.main()
