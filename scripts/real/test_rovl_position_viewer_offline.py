"""Completely offline tests for ROVL parsing, geometry and GUI construction."""

from __future__ import print_function

import math
import sys
import unittest
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))

from rovl_position_viewer import PositionTracker  # noqa: E402
from rovl_protocol import forward_geodesic, nmea_checksum, parse_gps_sentence, parse_rovl_sentence, relative_position  # noqa: E402


def sentence(body):
    return "$%s*%02X\r\n" % (body, nmea_checksum(body))


class ROVLPositionOfflineTests(unittest.TestCase):
    def test_usrth_parser_checksum_empty_and_future_fields(self):
        body = "USRTH,358.5,1.5,2.8,10.0,,37.2,2.8,,,,178.1,16,T,T,1,3313,B,-2,-2,FUTURE"
        message = parse_rovl_sentence(sentence(body))
        self.assertEqual(message["sr"], 10.0)
        self.assertIsNone(message["tb"])
        self.assertEqual(message["cb"], 37.2)
        self.assertEqual(message["te"], 2.8)
        self.assertEqual(message["ch"], 178.1)
        self.assertEqual(message["extra_fields"], ["FUTURE"])
        with self.assertRaises(ValueError):
            parse_rovl_sentence(sentence(body)[:-5] + "00\r\n")

    def test_range_elevation_and_bearing_conversion(self):
        message = {"sr": 10.0, "cb": 90.0, "te": 30.0}
        result = relative_position(message)
        self.assertAlmostEqual(result["horizontal_range_m"], 10.0 * math.cos(math.radians(30.0)))
        self.assertAlmostEqual(result["east_m"], result["horizontal_range_m"])
        self.assertAlmostEqual(result["north_m"], 0.0, places=8)

    def test_topside_forward_geodesic(self):
        lat, lon = forward_geodesic(45.0, 10.0, 90.0, 100.0)
        self.assertAlmostEqual(lat, 45.0, places=4)
        self.assertGreater(lon, 10.0)

    def test_gps_nmea_fix(self):
        body = "GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,"
        gps = parse_gps_sentence(sentence(body))
        self.assertTrue(gps["fix"])
        self.assertAlmostEqual(gps["lat"], 48.1173, places=4)
        self.assertAlmostEqual(gps["lon"], 11.5166667, places=4)

    def test_gui_builds_without_hardware(self):
        try:
            app = PositionTracker()
        except Exception as exc:
            self.skipTest("Tk GUI non disponibile: %s" % exc)
        app.update_idletasks()
        self.assertEqual(app.title(), "BlueROV2 Position Tracker")
        self.assertIsNone(app.rovl_worker)
        self.assertIsNone(app.gps_worker)
        app.destroy()


if __name__ == "__main__":
    unittest.main()
