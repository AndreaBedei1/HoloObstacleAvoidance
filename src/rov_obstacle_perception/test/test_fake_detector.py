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


if __name__ == "__main__":
    unittest.main()

