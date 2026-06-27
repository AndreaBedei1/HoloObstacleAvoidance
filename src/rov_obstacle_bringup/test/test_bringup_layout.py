from pathlib import Path
import unittest


class BringupLayoutTest(unittest.TestCase):
    def test_demo_launch_exists(self):
        package_dir = Path(__file__).resolve().parents[1]

        self.assertTrue((package_dir / "launch" / "obstacle_avoidance_demo.launch.py").exists())
        self.assertTrue((package_dir / "config" / "demo.yaml").exists())
        self.assertTrue(
            (package_dir / "rov_obstacle_bringup" / "nominal_cmd_publisher_node.py").exists()
        )

    def test_demo_launch_starts_required_nodes(self):
        package_dir = Path(__file__).resolve().parents[1]
        text = (package_dir / "launch" / "obstacle_avoidance_demo.launch.py").read_text(
            encoding="utf-8"
        )

        self.assertIn("fake_obstacle_detector_node", text)
        self.assertIn("local_avoidance_planner_node", text)
        self.assertIn("nominal_cmd_publisher_node", text)
        self.assertIn("risk_enter_threshold", text)
        self.assertIn("avoidance_yaw_rate", text)
        self.assertIn("rov_obstacle_bringup", text)
        self.assertIn("TimerAction", text)


if __name__ == "__main__":
    unittest.main()
