import math
from pathlib import Path
import unittest

from rov_obstacle_perception.fake_obstacle_detector_node import (
    SCENARIO_APPROACHING,
    SCENARIO_CENTRAL_STATIC,
    SCENARIO_CROSSING_LEFT_TO_RIGHT,
    SCENARIO_CROSSING_RIGHT_TO_LEFT,
    SCENARIO_LEFT_STATIC,
    SCENARIO_NONE,
    SCENARIO_RIGHT_STATIC,
    SUPPORTED_SCENARIOS,
    build_fake_obstacle_fields,
    center_x_to_bearing_rad,
)


class FakeDetectorScenarioTest(unittest.TestCase):
    def test_required_scenarios_are_supported(self):
        self.assertEqual(
            SUPPORTED_SCENARIOS,
            {
                SCENARIO_NONE,
                SCENARIO_CENTRAL_STATIC,
                SCENARIO_LEFT_STATIC,
                SCENARIO_RIGHT_STATIC,
                SCENARIO_CROSSING_LEFT_TO_RIGHT,
                SCENARIO_CROSSING_RIGHT_TO_LEFT,
                SCENARIO_APPROACHING,
            },
        )

    def test_center_x_to_bearing_rad_uses_horizontal_fov(self):
        self.assertAlmostEqual(center_x_to_bearing_rad(0.5, 90.0), 0.0)
        self.assertAlmostEqual(center_x_to_bearing_rad(0.0, 90.0), -math.pi / 4.0)
        self.assertAlmostEqual(center_x_to_bearing_rad(1.0, 90.0), math.pi / 4.0)

    def test_left_static_scenario_has_negative_bearing(self):
        fields = _fields_for_scenario(SCENARIO_LEFT_STATIC)

        self.assertLess(fields.center_x, 0.5)
        self.assertLess(fields.bearing_rad, 0.0)

    def test_right_static_scenario_has_positive_bearing(self):
        fields = _fields_for_scenario(SCENARIO_RIGHT_STATIC)

        self.assertGreater(fields.center_x, 0.5)
        self.assertGreater(fields.bearing_rad, 0.0)

    def test_config_contains_topic_and_fov_parameters(self):
        package_dir = Path(__file__).resolve().parents[1]
        text = (package_dir / "config" / "fake_detector_scenarios.yaml").read_text(
            encoding="utf-8"
        )

        self.assertIn('output_topic: "/perception/obstacles"', text)
        self.assertIn("horizontal_fov_deg: 90.0", text)


def _fields_for_scenario(scenario: str):
    return build_fake_obstacle_fields(
        scenario=scenario,
        elapsed_s=0.0,
        center_x=0.5,
        center_y=0.5,
        width=0.25,
        height=0.35,
        risk=0.8,
        horizontal_fov_deg=90.0,
    )


if __name__ == "__main__":
    unittest.main()
