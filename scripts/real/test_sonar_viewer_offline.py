"""Offline parser and GUI smoke tests; no network or sonar access."""

from __future__ import print_function

import struct
import sys
import unittest
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))
import sonar_viewer as viewer  # noqa: E402


class SonarViewerOfflineTests(unittest.TestCase):
    def test_omniscan_packet_and_profile_decoder(self):
        header = viewer.OMNI_PROFILE_HEADER.pack(
            7, 0, 20000, 123, 450000, 2, 4, 15000, 0, 0,
            0.001, 1.0, 0.0, -60.0, 0.0, 0.0,
        )
        payload = header + struct.pack("<4H", 0, 21845, 43690, 65535)
        packet = viewer.make_packet(viewer.OS_MONO_PROFILE, payload)
        self.assertEqual(packet[:2], b"BR")
        decoded = viewer.decode_omni_profile_payload(payload)
        self.assertEqual(decoded["ping_number"], 7)
        self.assertEqual(decoded["num_results"], 4)
        self.assertEqual(decoded["pwr_results"], [0, 21845, 43690, 65535])

    def test_profile_normalization(self):
        self.assertEqual(viewer.normalized_row([0, 5, 10], 0, 10), [0, 128, 255])
        self.assertEqual(viewer.normalized_row([], 0, 1), [])

    def test_gui_builds_without_connecting(self):
        try:
            app = viewer.SonarViewer()
        except Exception as exc:
            self.skipTest("Tk GUI non disponibile in questo ambiente: %s" % exc)
        app.update_idletasks()
        self.assertIn("dual sonar", app.title())
        self.assertIsNone(app.omni_worker)
        self.assertIsNone(app.ping_worker)
        app.destroy()


if __name__ == "__main__":
    unittest.main()
