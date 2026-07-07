from pathlib import Path
import unittest


class HoloOceanSmokeLaunchLayoutTest(unittest.TestCase):
    def test_smoke_launch_file_exists(self):
        package_dir = Path(__file__).resolve().parents[1]

        self.assertTrue((package_dir / "launch" / "holoocean_pose_smoke.launch.py").exists())
        self.assertTrue((package_dir / "launch" / "holoocean_oracle_demo.launch.py").exists())
        self.assertTrue((package_dir / "launch" / "holoocean_anchor_avoidance.launch.py").exists())

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

    def test_anchor_launch_uses_generic_planner_and_no_real_control(self):
        package_dir = Path(__file__).resolve().parents[1]
        text = (package_dir / "launch" / "holoocean_anchor_avoidance.launch.py").read_text(
            encoding="utf-8"
        )

        self.assertIn("holoocean_bridge_node", text)
        self.assertIn("local_avoidance_planner_node", text)
        # The supported anchor path is the REAL custom mesh scenario.
        self.assertIn("custom_anchor_visible.yaml", text)
        self.assertNotIn("anchor_center_static.yaml", text)
        self.assertIn('"relay_oracle_topic": "/perception/obstacles"', text)
        self.assertNotIn("mavlink", text.lower())
        self.assertNotIn("thruster", text.lower())

    def test_oracle_avoidance_launch_relays_oracle_to_planner_input(self):
        package_dir = Path(__file__).resolve().parents[1]
        text = (package_dir / "launch" / "holoocean_oracle_avoidance.launch.py").read_text(
            encoding="utf-8"
        )

        self.assertIn("holoocean_bridge_node", text)
        self.assertIn('"relay_oracle_topic": "/perception/obstacles"', text)

    def test_custom_anchor_scenarios_are_the_main_path(self):
        package_dir = Path(__file__).resolve().parents[1]
        scenario_dir = package_dir / "config" / "holoocean_scenarios"

        for name in [
            "custom_anchor_visible.yaml",
            "custom_anchor_left.yaml",
            "custom_anchor_right.yaml",
            "custom_anchor_with_spheres.yaml",
        ]:
            self.assertTrue((scenario_dir / name).exists(), msg=name)

    def test_primitive_scenarios_moved_to_legacy(self):
        package_dir = Path(__file__).resolve().parents[1]
        scenario_dir = package_dir / "config" / "holoocean_scenarios"
        legacy_dir = scenario_dir / "legacy_primitives"

        for name in [
            "anchor_center_static.yaml",
            "anchor_left_static.yaml",
            "anchor_right_static.yaml",
            "anchor_partially_visible.yaml",
            "anchor_with_spheres.yaml",
            "sphere_front.yaml",
            "sphere_left.yaml",
            "sphere_right.yaml",
            "multi_sphere.yaml",
        ]:
            self.assertTrue((legacy_dir / name).exists(), msg=name)
            self.assertFalse(
                (scenario_dir / name).exists(),
                msg=f"{name} should live only in legacy_primitives/",
            )


if __name__ == "__main__":
    unittest.main()
