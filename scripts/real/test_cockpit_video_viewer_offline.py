"""Offline tests for Cockpit pairing, ASS parsing and GUI construction."""

import shutil
import tempfile
import unittest
from pathlib import Path

import tkinter as tk

from cockpit_video_viewer import (
    AssIndex,
    CockpitVideoViewer,
    _parse_key_values,
    export_telemetry_csv,
    pair_recordings,
    parse_ass_file,
)


ASS_TEXT = """[Script Info]
Title: Synthetic Cockpit telemetry
[V4+ Styles]
Format: Name, Fontname, Fontsize, PrimaryColour, SecondaryColour, OutlineColour, BackColour, Bold, Italic, Underline, StrikeOut, ScaleX, ScaleY, Spacing, Angle, BorderStyle, Outline, Shadow, Alignment, MarginL, MarginR, MarginV, Encoding
Style: Default,Arial,20,&H00FFFFFF,&H000000FF,&H00000000,&H64000000,0,0,0,0,100,100,0,0,1,1,0,2,10,10,10,1
[Events]
Format: Layer, Start, End, Style, Name, MarginL, MarginR, MarginV, Effect, Text
Dialogue: 0,0:00:00.00,0:00:03.00,Default,,0,0,0,,Depth: 4.28 m\\NHeading=173°
Dialogue: 0,0:00:01.00,0:00:02.50,Default,,0,0,0,,Roll: 2.1°\\NPitch: -4.3°
Dialogue: 0,0:00:04.00,0:00:05.00,Default,,0,0,0,,{\\an8}CUSTOM FIELD: active\\Nraw line without separator
"""


class CockpitVideoViewerOfflineTests(unittest.TestCase):
    def setUp(self):
        self.temp = Path(tempfile.mkdtemp(prefix="cockpit_viewer_test_"))
        (self.temp / "Cockpit Synthetic #abc.mkv").write_bytes(b"not a real video")
        (self.temp / "Cockpit Synthetic #abc.ass").write_text(ASS_TEXT, encoding="utf-8")
        (self.temp / "VIDEO WITHOUT ASS.mkv").write_bytes(b"video")
        (self.temp / "ASS WITHOUT VIDEO.ass").write_text(ASS_TEXT, encoding="utf-8")

    def tearDown(self):
        shutil.rmtree(self.temp, ignore_errors=True)

    def test_pairing_statuses(self):
        pairs = pair_recordings(self.temp)
        statuses = {pair.display_name: pair.status for pair in pairs}
        self.assertEqual(statuses["Cockpit Synthetic #abc.mkv"], "VIDEO OK · ASS OK")
        self.assertEqual(statuses["VIDEO WITHOUT ASS.mkv"], "VIDEO WITHOUT ASS")
        self.assertEqual(statuses["ASS WITHOUT VIDEO.ass"], "ASS WITHOUT VIDEO")

    def test_ass_parsing_and_markup(self):
        index = parse_ass_file(self.temp / "Cockpit Synthetic #abc.ass")
        self.assertEqual(len(index.events), 3)
        self.assertEqual(index.events[0].readable_text, "Depth: 4.28 m\nHeading=173°")
        self.assertEqual(index.events[0].fields, [("Depth", "4.28 m"), ("Heading", "173°")])
        self.assertNotIn("{\\an8}", index.events[2].readable_text)
        self.assertIn("raw line without separator", index.events[2].other_lines)

    def test_active_events_overlap_and_seek(self):
        index = parse_ass_file(self.temp / "Cockpit Synthetic #abc.ass")
        self.assertEqual([event.index for event in index.active_at(500)], [0])
        self.assertEqual([event.index for event in index.active_at(1500)], [0, 1])
        self.assertEqual([event.index for event in index.active_at(3500)], [])
        self.assertEqual([event.index for event in index.active_at(4500)], [2])

    def test_generic_key_value_and_unknown_lines(self):
        fields, other = _parse_key_values("Depth: 4.28 m|Heading=173°\nunknown")
        self.assertEqual(fields, [("Depth", "4.28 m"), ("Heading", "173°")])
        self.assertEqual(other, ["unknown"])

    def test_csv_export_does_not_change_ass(self):
        ass = self.temp / "Cockpit Synthetic #abc.ass"
        before = ass.read_bytes()
        index = parse_ass_file(ass)
        destination = self.temp / "telemetry.csv"
        export_telemetry_csv(index, destination)
        self.assertTrue(destination.exists())
        self.assertEqual(before, ass.read_bytes())

    def test_gui_constructs_without_video_playback(self):
        try:
            app = CockpitVideoViewer(self.temp, use_vlc=False)
        except tk.TclError as exc:
            self.skipTest(f"GUI display unavailable: {exc}")
        else:
            try:
                self.assertIsNotNone(app.recording_list)
                self.assertEqual(len(app.pairs), 3)
                self.assertIsInstance(app.ass_index, AssIndex)
            finally:
                app.destroy()


if __name__ == "__main__":
    unittest.main()
