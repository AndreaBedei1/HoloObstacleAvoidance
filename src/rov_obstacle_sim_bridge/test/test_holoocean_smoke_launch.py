from pathlib import Path
import unittest


class HoloOceanSmokeLaunchLayoutTest(unittest.TestCase):
    def test_smoke_launch_file_exists(self):
        package_dir = Path(__file__).resolve().parents[1]

        self.assertTrue((package_dir / "launch" / "holoocean_pose_smoke.launch.py").exists())
        self.assertTrue((package_dir / "launch" / "holoocean_oracle_demo.launch.py").exists())

    def test_oracle_demo_uses_simulation_only_nodes(self):
        package_dir = Path(__file__).resolve().parents[1]
        text = (package_dir / "launch" / "holoocean_oracle_demo.launch.py").read_text(
            encoding="utf-8"
        )

        self.assertIn("simulated_rover_pose_publisher_node", text)
        self.assertIn("holoocean_obstacle_oracle_node", text)
        self.assertIn("local_avoidance_planner_node", text)
        self.assertIn("cmd_vel_safe_logger_node", text)
        self.assertNotIn("mavlink", text.lower())

    def test_oracle_config_uses_planner_safe_topic(self):
        package_dir = Path(__file__).resolve().parents[1]
        text = (package_dir / "config" / "holoocean_oracle.yaml").read_text(
            encoding="utf-8"
        )

        self.assertIn("input_topic: /planner/cmd_vel_safe", text)


if __name__ == "__main__":
    unittest.main()
