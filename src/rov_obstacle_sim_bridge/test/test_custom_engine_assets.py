"""Tests for the EXTERNAL modified engine integration (custom assets).

Pure-Python: no holoocean import, no ROS 2, no engine required.  Tests that
need the external engine folder skip gracefully when it is absent.
"""

from __future__ import annotations

import importlib.util
import math
import os
from pathlib import Path
import sys
import tempfile
import textwrap
import unittest


PACKAGE_DIR = Path(__file__).resolve().parents[1]
REPO_DIR = PACKAGE_DIR.parents[1]
SERVER_DIR = PACKAGE_DIR / "holoocean_server"
SCENARIO_DIR = PACKAGE_DIR / "config" / "holoocean_scenarios"
ENGINE_CONFIG = REPO_DIR / "config" / "custom_holoocean_engine.yaml"


def _load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


SERVER = _load_module(
    "holoocean_sim_server_for_custom_tests",
    SERVER_DIR / "holoocean_sim_server.py",
)
LAUNCHER = _load_module(
    "custom_engine_launcher_for_tests",
    SERVER_DIR / "custom_engine_launcher.py",
)
COMMANDS = _load_module(
    "custom_asset_commands_for_tests",
    SERVER_DIR / "custom_asset_commands.py",
)


class EngineConfigTest(unittest.TestCase):
    """config/custom_holoocean_engine.yaml loading and validation."""

    def test_repo_engine_config_structure_is_valid(self):
        cfg = LAUNCHER.load_engine_config(str(ENGINE_CONFIG))
        problems = LAUNCHER.validate_engine_config(cfg, check_paths=False)
        self.assertEqual(problems, [])
        self.assertEqual(cfg["external_engine"]["default_map"], "ExampleLevel")
        self.assertEqual(cfg["conda_env"], "ocean")
        self.assertEqual(cfg["assets"]["anchor_mesh"], "/Game/ancora.ancora")

    def test_missing_engine_section_is_reported(self):
        problems = LAUNCHER.validate_engine_config({}, check_paths=False)
        self.assertTrue(any("external_engine" in p for p in problems))

    def test_empty_field_is_reported(self):
        cfg = {
            "external_engine": {
                "ue_editor_exe": "",
                "uproject": "x.uproject",
                "default_map": "M",
            }
        }
        problems = LAUNCHER.validate_engine_config(cfg, check_paths=False)
        self.assertTrue(any("ue_editor_exe" in p for p in problems))

    def test_bad_launch_values_are_reported(self):
        cfg = {
            "external_engine": {
                "ue_editor_exe": "e.exe",
                "uproject": "p.uproject",
                "default_map": "M",
            },
            "launch": {"res_x": -1},
        }
        problems = LAUNCHER.validate_engine_config(cfg, check_paths=False)
        self.assertTrue(any("res_x" in p for p in problems))

    def test_missing_paths_are_reported_when_checking(self):
        cfg = {
            "external_engine": {
                "ue_editor_exe": "Z:/definitely/not/here.exe",
                "uproject": "Z:/definitely/not/here.uproject",
                "default_map": "M",
            }
        }
        problems = LAUNCHER.validate_engine_config(cfg, check_paths=True)
        self.assertEqual(len(problems), 2)

    def test_explicit_path_beats_env_var(self):
        with tempfile.TemporaryDirectory() as tmp:
            other = Path(tmp) / "other.yaml"
            other.write_text("external_engine: {}\n", encoding="utf-8")
            old = os.environ.get(LAUNCHER.ENGINE_CONFIG_ENV_VAR)
            os.environ[LAUNCHER.ENGINE_CONFIG_ENV_VAR] = str(other)
            try:
                resolved = LAUNCHER.resolve_engine_config_path(str(ENGINE_CONFIG))
                self.assertEqual(resolved, ENGINE_CONFIG.resolve())
                resolved_env = LAUNCHER.resolve_engine_config_path(None)
                self.assertEqual(resolved_env, other.resolve())
            finally:
                if old is None:
                    del os.environ[LAUNCHER.ENGINE_CONFIG_ENV_VAR]
                else:
                    os.environ[LAUNCHER.ENGINE_CONFIG_ENV_VAR] = old


class EngineCommandTest(unittest.TestCase):
    """build_engine_command produces a VISIBLE -game command line."""

    CFG = {
        "external_engine": {
            "ue_editor_exe": "C:/UE/UnrealEditor.exe",
            "uproject": "C:/proj/Holodeck.uproject",
            "default_map": "ExampleLevel",
        },
        "launch": {
            "windowed": True,
            "res_x": 1280,
            "res_y": 720,
            "ticks_per_sec": 30,
            "frames_per_sec": 30,
            "log_file": "logs/test_engine.log",
        },
    }

    def test_command_shape(self):
        cmd = LAUNCHER.build_engine_command(self.CFG)
        self.assertEqual(cmd[0], "C:/UE/UnrealEditor.exe")
        self.assertEqual(cmd[1], "C:/proj/Holodeck.uproject")
        self.assertEqual(cmd[2], "/Game/ExampleLevel")
        self.assertIn("-game", cmd)
        self.assertIn("-windowed", cmd)
        self.assertIn("-ResX=1280", cmd)
        self.assertIn("-ResY=720", cmd)
        self.assertIn("-TicksPerSec=30", cmd)
        self.assertIn("-FramesPerSec=30", cmd)

    def test_never_headless(self):
        cmd = " ".join(LAUNCHER.build_engine_command(self.CFG))
        for forbidden in ("-RenderOffScreen", "-nullrhi", "-unattended"):
            self.assertNotIn(forbidden.lower(), cmd.lower())

    def test_map_override(self):
        cmd = LAUNCHER.build_engine_command(self.CFG, map_name="OtherMap")
        self.assertIn("/Game/OtherMap", cmd)

    def test_log_redirected_inside_repo(self):
        log = LAUNCHER.engine_log_path(self.CFG)
        self.assertTrue(str(log).startswith(str(LAUNCHER.repo_root())))


class CustomScenarioLoadTest(unittest.TestCase):
    """load_config parses custom_engine + custom_assets sections."""

    def test_custom_anchor_visible_scenario(self):
        cfg = SERVER.load_config(str(SCENARIO_DIR / "custom_anchor_visible.yaml"))
        self.assertIsNotNone(cfg.custom_engine)
        self.assertTrue(cfg.custom_engine.enabled)
        self.assertEqual(cfg.custom_engine.world, "ExampleLevel")
        self.assertEqual(cfg.custom_engine.agent_type, "HoveringAUV")
        self.assertTrue(cfg.show_viewport)

        self.assertEqual(len(cfg.custom_assets), 1)
        anchor = cfg.custom_assets[0]
        self.assertEqual(anchor.class_name, "anchor")
        self.assertEqual(anchor.mesh_asset, "/Game/ancora.ancora")
        self.assertTrue(anchor.spawned_at_runtime)
        self.assertGreater(anchor.radius_m, 0.0)
        self.assertIsNotNone(anchor.half_extents_m)

        # The engine config referenced by the scenario resolves to the repo one.
        resolved = os.path.normpath(
            os.path.join(cfg.config_dir, cfg.custom_engine.engine_config)
        )
        self.assertEqual(Path(resolved), ENGINE_CONFIG)

    def test_all_custom_scenarios_load(self):
        for name in (
            "custom_anchor_visible.yaml",
            "custom_anchor_left.yaml",
            "custom_anchor_right.yaml",
            "custom_anchor_with_spheres.yaml",
        ):
            cfg = SERVER.load_config(str(SCENARIO_DIR / name))
            self.assertTrue(cfg.custom_engine.enabled, msg=name)
            self.assertGreaterEqual(len(cfg.custom_assets), 1, msg=name)

    def test_legacy_primitive_scenarios_have_no_custom_engine(self):
        cfg = SERVER.load_config(
            str(SCENARIO_DIR / "legacy_primitives" / "sphere_front.yaml")
        )
        self.assertIsNone(cfg.custom_engine)
        self.assertEqual(cfg.custom_assets, [])

    def test_custom_assets_without_engine_rejected(self):
        with tempfile.NamedTemporaryFile(
            "w", suffix=".yaml", delete=False, encoding="utf-8"
        ) as fh:
            fh.write(textwrap.dedent("""
                custom_assets:
                  - name: a
                    mesh_asset: "/Game/x.x"
                    relative_position: [1.0, 0.0, 0.0]
            """))
            path = fh.name
        try:
            with self.assertRaises(ValueError):
                SERVER.load_config(path)
        finally:
            os.unlink(path)

    def test_bad_mesh_path_rejected(self):
        with tempfile.NamedTemporaryFile(
            "w", suffix=".yaml", delete=False, encoding="utf-8"
        ) as fh:
            fh.write(textwrap.dedent("""
                custom_engine: {enabled: true}
                custom_assets:
                  - name: a
                    mesh_asset: "ancora.ancora"
                    relative_position: [1.0, 0.0, 0.0]
            """))
            path = fh.name
        try:
            with self.assertRaises(ValueError):
                SERVER.load_config(path)
        finally:
            os.unlink(path)

    def test_static_asset_requires_absolute_position(self):
        with tempfile.NamedTemporaryFile(
            "w", suffix=".yaml", delete=False, encoding="utf-8"
        ) as fh:
            fh.write(textwrap.dedent("""
                custom_engine: {enabled: true}
                custom_assets:
                  - name: a
                    mesh_asset: "/Game/x.x"
                    relative_position: [1.0, 0.0, 0.0]
                    spawned_at_runtime: false
            """))
            path = fh.name
        try:
            with self.assertRaises(ValueError):
                SERVER.load_config(path)
        finally:
            os.unlink(path)


class CustomAssetPlanTest(unittest.TestCase):
    """build_custom_asset_plan geometry (oracle projection inputs)."""

    def _config(self, **asset_kwargs):
        defaults = dict(
            name="anchor_real",
            class_name="anchor",
            mesh_asset="/Game/ancora.ancora",
            relative_position=(10.0, 0.0, 0.0),
            rotation=(0.0, 0.0, 0.0),
            scale=12.0,
            radius_m=1.8,
            half_extents_m=(0.5, 1.5, 2.0),
            spawned_at_runtime=True,
        )
        defaults.update(asset_kwargs)
        return SERVER.SimConfig(
            custom_engine=SERVER.CustomEngineSpec(enabled=True),
            custom_assets=[SERVER.CustomAssetSpec(**defaults)],
        )

    def test_relative_placement_facing_plus_x(self):
        cfg = self._config()
        plan, oracle = SERVER.build_custom_asset_plan(cfg, 0.0, 0.0, -69.0, 0.0)
        self.assertEqual(len(plan), 1)
        self.assertEqual(plan[0].position, (10.0, 0.0, -69.0))
        self.assertEqual(oracle[0]["class_name"], "anchor")
        self.assertEqual(oracle[0]["position"], [10.0, 0.0, -69.0])
        self.assertAlmostEqual(oracle[0]["radius_m"], 1.8)

    def test_relative_placement_rotates_with_yaw(self):
        cfg = self._config(relative_position=(10.0, 0.0, 0.0))
        yaw = math.radians(90.0)  # facing +y (left)
        plan, _ = SERVER.build_custom_asset_plan(cfg, 0.0, 0.0, 0.0, yaw)
        x, y, z = plan[0].position
        self.assertAlmostEqual(x, 0.0, places=6)
        self.assertAlmostEqual(y, 10.0, places=6)
        self.assertAlmostEqual(z, 0.0, places=6)
        # Spawn yaw follows the rover yaw so the mesh faces consistently.
        self.assertAlmostEqual(plan[0].rotation[2], 90.0, places=4)

    def test_absolute_placement_ignores_rover(self):
        cfg = self._config(
            relative_position=None,
            absolute_position=(-93.55, 38.38, -69.6),
        )
        plan, oracle = SERVER.build_custom_asset_plan(
            cfg, 123.0, -456.0, 7.0, math.radians(37.0)
        )
        self.assertEqual(plan[0].position, (-93.55, 38.38, -69.6))
        self.assertEqual(oracle[0]["position"], [-93.55, 38.38, -69.6])

    def test_axis_aligned_bounds_from_half_extents(self):
        cfg = self._config(half_extents_m=(0.5, 1.5, 2.0))
        _, oracle = SERVER.build_custom_asset_plan(cfg, 0.0, 0.0, 0.0, 0.0)
        bounds = oracle[0]["bounds"]
        self.assertAlmostEqual(bounds["min"][0], 10.0 - 0.5, places=6)
        self.assertAlmostEqual(bounds["max"][0], 10.0 + 0.5, places=6)
        self.assertAlmostEqual(bounds["min"][1], -1.5, places=6)
        self.assertAlmostEqual(bounds["max"][1], 1.5, places=6)
        self.assertAlmostEqual(bounds["min"][2], -2.0, places=6)
        self.assertAlmostEqual(bounds["max"][2], 2.0, places=6)

    def test_yawed_bounds_still_contain_center(self):
        cfg = self._config(rotation=(0.0, 0.0, 45.0))
        _, oracle = SERVER.build_custom_asset_plan(cfg, 0.0, 0.0, 0.0, 0.0)
        bounds = oracle[0]["bounds"]
        for axis in range(3):
            self.assertLess(bounds["min"][axis], oracle[0]["position"][axis])
            self.assertGreater(bounds["max"][axis], oracle[0]["position"][axis])
        # 45 deg yaw grows the xy extents (0.5,1.5) -> sqrt(2)*avg-ish box.
        expected_half_xy = (0.5 + 1.5) / math.sqrt(2.0)
        self.assertAlmostEqual(
            bounds["max"][0] - oracle[0]["position"][0], expected_half_xy, places=5
        )

    def test_static_asset_kept_in_oracle_but_not_spawned(self):
        cfg = self._config(
            relative_position=None,
            absolute_position=(1.0, 2.0, 3.0),
            spawned_at_runtime=False,
        )
        plan, oracle = SERVER.build_custom_asset_plan(cfg, 0.0, 0.0, 0.0, 0.0)
        self.assertFalse(plan[0].spawned_at_runtime)
        self.assertEqual(len(oracle), 1)


class CustomScenarioCfgTest(unittest.TestCase):
    """build_custom_scenario_cfg (HoloOcean attach dict)."""

    def _sim_config(self):
        return SERVER.SimConfig(
            agent_name="auv0",
            camera_sensor="FrontCamera",
            ticks_per_sec=30,
            frames_per_sec=30,
            camera_width=512,
            camera_height=512,
            custom_engine=SERVER.CustomEngineSpec(
                enabled=True,
                world="ExampleLevel",
                agent_location=(-101.55, 38.38, -69.0),
                agent_yaw_deg=0.0,
            ),
        )

    def test_scenario_dict_shape(self):
        scenario = SERVER.build_custom_scenario_cfg(self._sim_config())
        self.assertEqual(scenario["world"], "ExampleLevel")
        self.assertEqual(scenario["main_agent"], "auv0")
        agent = scenario["agents"][0]
        self.assertEqual(agent["agent_type"], "HoveringAUV")
        self.assertEqual(agent["location"], [-101.55, 38.38, -69.0])
        sensor_types = {s["sensor_type"] for s in agent["sensors"]}
        self.assertLessEqual(
            {"PoseSensor", "VelocitySensor", "DepthSensor", "RGBCamera"},
            sensor_types,
        )
        camera = [s for s in agent["sensors"] if s["sensor_type"] == "RGBCamera"][0]
        self.assertEqual(camera["sensor_name"], "FrontCamera")
        self.assertEqual(camera["configuration"]["CaptureWidth"], 512)

    def test_frames_and_ticks_always_present(self):
        # Regression: holoocean.make() falls back to interactive input()
        # when these keys are missing, hanging non-interactive runs.
        scenario = SERVER.build_custom_scenario_cfg(self._sim_config())
        self.assertIn("ticks_per_sec", scenario)
        self.assertIn("frames_per_sec", scenario)
        cfg = self._sim_config()
        cfg.frames_per_sec = False
        scenario2 = SERVER.build_custom_scenario_cfg(cfg)
        self.assertIs(scenario2["frames_per_sec"], False)

    def test_requires_enabled_custom_engine(self):
        with self.assertRaises(ValueError):
            SERVER.build_custom_scenario_cfg(SERVER.SimConfig())


class SpawnAssetParamsTest(unittest.TestCase):
    """custom_asset_commands.spawn_asset_params (pure)."""

    def test_parameter_order_matches_engine_contract(self):
        nums, strings = COMMANDS.spawn_asset_params(
            position=(1.0, 2.0, 3.0),
            rotation=(4.0, 5.0, 6.0),
            scale=(7.0, 8.0, 9.0),
            mesh_asset="/Game/ancora.ancora",
            label="anchor_01",
            units="meters",
        )
        self.assertEqual(nums, [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0])
        self.assertEqual(strings, ["/Game/ancora.ancora", "anchor_01", "meters"])

    def test_wrong_lengths_rejected(self):
        with self.assertRaises(ValueError):
            COMMANDS.spawn_asset_params((1.0, 2.0), (0, 0, 0), (1, 1, 1), "/G/x.x")


class StaleEngineParsingTest(unittest.TestCase):
    """parse_engine_process_lines — stale engine PID matching (pure)."""

    UPROJECT = "C:/Users/x/Desktop/Holo/MondoTest/HoloOcean/engine/Holodeck.uproject"

    def test_matches_full_path_any_slashes(self):
        lines = [
            r"123|C:\UE\UnrealEditor.exe C:\Users\x\Desktop\Holo\MondoTest\HoloOcean\engine\Holodeck.uproject /Game/ExampleLevel -game",
            "456|C:/UE/UnrealEditor.exe C:/Users/x/Desktop/Holo/MondoTest/HoloOcean/engine/Holodeck.uproject /Game/M -game",
        ]
        pids = LAUNCHER.parse_engine_process_lines(lines, self.UPROJECT)
        self.assertEqual(pids, [123, 456])

    def test_ignores_other_projects_without_name_match(self):
        lines = ["789|C:/UE/UnrealEditor.exe C:/other/Different.uproject -game"]
        self.assertEqual(
            LAUNCHER.parse_engine_process_lines(lines, self.UPROJECT), []
        )

    def test_name_fallback_matches(self):
        # A manually-started engine may use a different path spelling; the
        # project file name is distinctive enough to treat it as ours.
        lines = ["222|UnrealEditor.exe D:/copy/engine/HOLODECK.UPROJECT -game"]
        self.assertEqual(
            LAUNCHER.parse_engine_process_lines(lines, self.UPROJECT), [222]
        )

    def test_garbage_lines_are_skipped(self):
        lines = ["", "no-separator", "abc|has separator but bad pid", None]
        self.assertEqual(
            LAUNCHER.parse_engine_process_lines(lines, self.UPROJECT), []
        )

    def test_cleanup_without_uproject_is_noop(self):
        self.assertEqual(LAUNCHER.cleanup_stale_engines({"external_engine": {}}), 0)


class ExternalEngineOptionalTest(unittest.TestCase):
    """Path checks that only run when the external folder is present."""

    def test_external_paths_when_available(self):
        cfg = LAUNCHER.load_engine_config(str(ENGINE_CONFIG))
        if not LAUNCHER.external_engine_available(cfg):
            self.skipTest(
                "external modified HoloOcean engine not present on this machine"
            )
        engine = cfg["external_engine"]
        self.assertTrue(Path(engine["uproject"]).is_file())
        self.assertTrue(Path(engine["ue_editor_exe"]).is_file())
        population = Path(engine.get("world_population_json", ""))
        if str(population):
            self.assertTrue(population.is_file())


if __name__ == "__main__":
    unittest.main()
