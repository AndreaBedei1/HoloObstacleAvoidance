"""Offline safety, decoder, replay and GUI smoke tests for sonar_viewer."""

from __future__ import print_function

import math
import queue
import struct
import sys
import tempfile
import threading
import unittest
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))
import sonar_viewer as viewer  # noqa: E402
from surveyor_processing import CHANNEL_DATA_HEADER, END_PING, ATOF_HEADER  # noqa: E402


def channel_payload(ping_number, ch1, ch2, bins=4):
    values = []
    for channel in (ch1, ch2):
        for index in range(bins):
            values.extend([float(channel + index), float(index) * 0.1])
    header = CHANNEL_DATA_HEADER.pack(ping_number, 1.0, 0, 0, 100, ch1, ch2, bins)
    return header + struct.pack("<%df" % len(values), *values)


def end_payload(ping_number, start_m=0.0, end_m=10.0, bins=4):
    values = list(END_PING.unpack(bytes(END_PING.size)))
    values[1] = float(start_m)
    values[2] = float(end_m)
    values[6] = int(ping_number)
    values[13] = 5.0
    values[16] = int(bins)
    values[17] = 1
    values[-1] = 1_700_000_000_000
    return END_PING.pack(*values)


class SonarViewerOfflineTests(unittest.TestCase):
    def test_packet_chunk_parser_is_protocol_framed(self):
        packet = viewer.make_packet(3010, end_payload(7))
        buffer = bytearray(packet[:11])
        self.assertEqual(viewer.decode_packet_stream_chunk(buffer), [])
        buffer.extend(packet[11:])
        decoded = viewer.decode_packet_stream_chunk(buffer)
        self.assertEqual(decoded[0][0], 3010)
        self.assertEqual(decoded[0][1], end_payload(7))
        self.assertEqual(buffer, bytearray())

    def test_ping1d_profile_preserves_official_fields(self):
        profile = {
            "distance": 1234,
            "confidence": 87,
            "transmit_duration": 42,
            "ping_number": 19,
            "scan_start": 500,
            "scan_length": 3000,
            "gain_setting": 4,
            "profile_data": bytearray([0, 32, 128, 255]),
        }
        record = viewer.ping1d_profile_record(profile, profile)
        self.assertEqual(record["distance_m"], 1.234)
        self.assertEqual(record["confidence"], 87)
        self.assertEqual(record["scan_start_mm"], 500)
        self.assertEqual(record["scan_length_mm"], 3000)
        self.assertEqual(record["gain"], 4)
        self.assertEqual(record["profile"], [0, 32, 128, 255])

    def test_default_worker_has_hard_transmit_lock(self):
        events = queue.Queue()
        worker = viewer.SurveyorWorker("offline", 62312, events)
        self.assertTrue(worker.tx_locked)
        with self.assertRaises(PermissionError):
            worker.request_start({"range_m": 20})
        self.assertTrue(worker.commands.empty())

        class FakeDevice(object):
            def __init__(self):
                self.calls = []

            def control_set_ping_parameters(self, **kwargs):
                self.calls.append(kwargs)

        worker.device = FakeDevice()
        with self.assertRaises(PermissionError):
            worker._wet_start({"range_m": 20})
        self.assertEqual(worker.device.calls, [])

    def test_replay_decodes_3009_3012_3010_without_transmitting(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "synthetic.svlog"
            with path.open("wb") as stream:
                for ch1 in range(0, 16, 2):
                    stream.write(viewer.make_packet(3009, channel_payload(7, ch1, ch1 + 1)))
                atof = ATOF_HEADER.pack(0, 1_700_000_000_000, 0.1, 1500.0, 7, 5, 0.001, 0, 1, 0)
                atof += struct.pack("<ffII", math.radians(5.0), 0.004, 0, 0)
                stream.write(viewer.make_packet(3012, atof))
                stream.write(viewer.make_packet(3010, end_payload(7)))
            events = queue.Queue()
            viewer.replay_surveyor(path, events, threading.Event())
            records = []
            while not events.empty():
                kind, data = events.get()
                if kind == "surveyor_ping":
                    records.append(data)
            self.assertEqual(len(records), 1)
            self.assertEqual(records[0]["ping_number"], 7)
            self.assertEqual(records[0]["channel_data_status"], "AVAILABLE")
            self.assertEqual(records[0]["detection_count"], 1)
            self.assertEqual(len(records[0]["matrix"]), 81)
            self.assertEqual(len(records[0]["matrix"][0]), 4)

    def test_nearest_matching_uses_monotonic_time(self):
        samples = [{"host_monotonic_ns": 100}, {"host_monotonic_ns": 180}]
        self.assertEqual(viewer.nearest_sample(170, samples), samples[1])
        matched = viewer.match_surveyor_ping({"host_monotonic_ns": 110}, samples, samples)
        self.assertEqual(matched["rgb_frame"], samples[0])

    def test_session_recorder_creates_local_copies_and_shared_identity(self):
        with tempfile.TemporaryDirectory() as directory:
            session = viewer.SessionRecorder(Path(directory))
            session.write_surveyor_packet(viewer.make_packet(3010, end_payload(3)), 12, 13)
            session.write_surveyor_ping({"ping_number": 3, "host_monotonic_ns": 12, "host_utc_ns": 13, "points": []})
            session.write_ping1d({"host_monotonic_ns": 14, "host_utc_ns": 15, "distance_m": 1.2})
            session.close()
            metadata = __import__("json").loads((session.directory / "session.json").read_text())
            self.assertTrue((session.directory / "surveyor_raw.svlog").stat().st_size > 0)
            self.assertEqual(metadata["session_id"], session.session_id)
            self.assertEqual(metadata["closed"], True)
            ping_line = (session.directory / "surveyor_pings.jsonl").read_text().strip()
            self.assertIn(session.session_id, ping_line)

    def test_gui_builds_offline_with_tx_button_locked(self):
        try:
            app = viewer.SonarViewer(offline=True)
        except Exception as exc:
            self.skipTest("Tk GUI non disponibile in questo ambiente: %s" % exc)
        app.update_idletasks()
        self.assertIn("Multimodal Recorder", app.title())
        self.assertIsNone(app.surveyor_worker)
        self.assertIsNone(app.ping_worker)
        self.assertEqual(str(app.start_surveyor_button.cget("state")), "disabled")
        app.destroy()


if __name__ == "__main__":
    unittest.main()
