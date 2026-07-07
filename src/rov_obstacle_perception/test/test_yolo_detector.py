"""Tests for the YOLO detector skeleton (pure helpers, no ultralytics)."""

from __future__ import annotations

import math
import unittest

from rov_obstacle_perception.yolo_obstacle_detector_node import (
    YoloDetection,
    center_x_to_bearing_rad,
    convert_detections,
    detection_risk,
    normalize_class_map,
)


def _det(**kwargs) -> YoloDetection:
    defaults = dict(
        class_name="boat",
        confidence=0.9,
        x_min=0.4,
        y_min=0.4,
        x_max=0.6,
        y_max=0.7,
    )
    defaults.update(kwargs)
    return YoloDetection(**defaults)


class NormalizeClassMapTest(unittest.TestCase):
    def test_parses_pairs_and_normalizes_keys(self):
        mapping = normalize_class_map(" Boat:anchor , SHIP : anchor ")
        self.assertEqual(mapping, {"boat": "anchor", "ship": "anchor"})

    def test_ignores_malformed_entries(self):
        self.assertEqual(normalize_class_map("no-colon, :empty, x:"), {})

    def test_empty_input(self):
        self.assertEqual(normalize_class_map(""), {})
        self.assertEqual(normalize_class_map(None), {})


class DetectionRiskTest(unittest.TestCase):
    def test_central_large_detection_is_high_risk(self):
        risk = detection_risk(0.5, 0.25, 1.0, risk_area_gain=4.0)
        self.assertGreaterEqual(risk, 0.9)

    def test_edge_small_detection_is_low_risk(self):
        risk = detection_risk(0.02, 0.001, 0.5, risk_area_gain=4.0)
        self.assertLess(risk, 0.2)

    def test_risk_is_clamped_to_unit_interval(self):
        self.assertLessEqual(detection_risk(0.5, 10.0, 1.0, 100.0), 1.0)
        self.assertGreaterEqual(detection_risk(1.5, -1.0, 0.0, 4.0), 0.0)


class ConvertDetectionsTest(unittest.TestCase):
    KW = dict(
        confidence_threshold=0.25,
        target_classes=[""],
        class_map={},
        horizontal_fov_deg=90.0,
        risk_area_gain=4.0,
    )

    def test_geometry_fields(self):
        fields = convert_detections([_det()], **self.KW)
        self.assertEqual(len(fields), 1)
        f = fields[0]
        self.assertAlmostEqual(f["center_x"], 0.5, places=6)
        self.assertAlmostEqual(f["center_y"], 0.55, places=6)
        self.assertAlmostEqual(f["width"], 0.2, places=6)
        self.assertAlmostEqual(f["height"], 0.3, places=6)
        self.assertAlmostEqual(f["apparent_area"], 0.06, places=6)
        self.assertAlmostEqual(
            f["bearing_rad"],
            center_x_to_bearing_rad(0.5, 90.0),
            places=6,
        )
        self.assertAlmostEqual(f["bearing_rad"], 0.0, places=6)

    def test_bearing_sign_matches_fake_detector_convention(self):
        left = convert_detections(
            [_det(x_min=0.0, x_max=0.2)], **self.KW
        )[0]
        right = convert_detections(
            [_det(x_min=0.8, x_max=1.0)], **self.KW
        )[0]
        self.assertLess(left["bearing_rad"], 0.0)
        self.assertGreater(right["bearing_rad"], 0.0)
        self.assertAlmostEqual(
            right["bearing_rad"], math.radians(90.0) * 0.4, places=6
        )

    def test_confidence_filter(self):
        fields = convert_detections([_det(confidence=0.1)], **self.KW)
        self.assertEqual(fields, [])

    def test_target_class_filter(self):
        kw = dict(self.KW)
        kw["target_classes"] = ["boat", "surfboard"]
        kept = convert_detections(
            [_det(class_name="Boat"), _det(class_name="person")], **kw
        )
        self.assertEqual(len(kept), 1)
        self.assertEqual(kept[0]["class_name"], "boat")

    def test_class_map_renames_for_planner(self):
        kw = dict(self.KW)
        kw["class_map"] = {"boat": "anchor"}
        fields = convert_detections([_det(class_name="boat")], **kw)
        self.assertEqual(fields[0]["class_name"], "anchor")

    def test_degenerate_boxes_dropped(self):
        fields = convert_detections(
            [_det(x_min=0.5, x_max=0.5)], **self.KW
        )
        self.assertEqual(fields, [])

    def test_out_of_range_boxes_clamped(self):
        fields = convert_detections(
            [_det(x_min=-0.5, x_max=0.5, y_min=-1.0, y_max=0.5)], **self.KW
        )
        f = fields[0]
        self.assertAlmostEqual(f["width"], 0.5, places=6)
        self.assertAlmostEqual(f["height"], 0.5, places=6)


class SkeletonSafetyTest(unittest.TestCase):
    def test_module_imports_without_ultralytics(self):
        # The node module itself must not require ultralytics at import time.
        import rov_obstacle_perception.yolo_obstacle_detector_node as module

        self.assertTrue(hasattr(module, "YoloObstacleDetectorNode"))

    def test_node_source_has_no_real_control(self):
        from pathlib import Path
        import rov_obstacle_perception.yolo_obstacle_detector_node as module

        text = Path(module.__file__).read_text(encoding="utf-8")
        self.assertNotIn("mavlink", text.lower())
        self.assertNotIn("thruster", text.lower())


if __name__ == "__main__":
    unittest.main()
