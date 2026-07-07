"""Tests for HoloOcean sim-server semantic object configuration.

These tests exercise pure loader/spawn-plan helpers only. They do not import or
start HoloOcean.
"""

from __future__ import annotations

import importlib.util
from pathlib import Path
import sys
import tempfile
import textwrap
import unittest


PACKAGE_DIR = Path(__file__).resolve().parents[1]
REPO_DIR = PACKAGE_DIR.parents[1]
SERVER_PATH = PACKAGE_DIR / "holoocean_server" / "holoocean_sim_server.py"
SCENARIO_DIR = PACKAGE_DIR / "config" / "holoocean_scenarios"
# Primitive scenarios are legacy: kept only for loader/oracle regressions.
LEGACY_DIR = SCENARIO_DIR / "legacy_primitives"


def _load_server_module():
    spec = importlib.util.spec_from_file_location("holoocean_sim_server_for_tests", SERVER_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


SERVER = _load_server_module()


class CoordinateConventionDocsTest(unittest.TestCase):
    def test_no_forward_right_up_comments_remain(self):
        paths = [
            REPO_DIR / "README.md",
            SERVER_PATH,
            LEGACY_DIR / "sphere_front.yaml",
        ]
        for path in paths:
            text = path.read_text(encoding="utf-8")
            self.assertNotIn("right_m", text, msg=str(path))
            self.assertNotIn("[forward, right, up", text, msg=str(path))

    def test_sphere_front_documents_forward_left_up(self):
        text = (LEGACY_DIR / "sphere_front.yaml").read_text(encoding="utf-8")
        self.assertIn("[forward_m, left_m, up_m]", text)


class SemanticObjectConfigTest(unittest.TestCase):
    def test_loads_anchor_parts(self):
        cfg = SERVER.load_config(str(LEGACY_DIR / "anchor_center_static.yaml"))

        self.assertEqual(len(cfg.semantic_objects), 1)
        anchor = cfg.semantic_objects[0]
        self.assertEqual(anchor.name, "anchor_center")
        self.assertEqual(anchor.class_name, "anchor")
        self.assertEqual(anchor.relative_position, (10.0, 0.0, 0.0))
        self.assertGreaterEqual(len(anchor.parts), 6)
        self.assertEqual(anchor.parts[0].prop_type, "box")

    def test_build_spawn_plan_aggregates_anchor(self):
        cfg = SERVER.load_config(str(LEGACY_DIR / "anchor_center_static.yaml"))
        spawns, oracle = SERVER.build_spawn_plan(cfg, 0.0, 0.0, 0.0, 0.0)

        self.assertGreaterEqual(len(spawns), 6)
        self.assertEqual(len(oracle), 1)
        anchor = oracle[0]
        self.assertEqual(anchor["class_name"], "anchor")
        self.assertEqual(anchor["part_count"], len(spawns))
        self.assertIn("bounds", anchor)
        self.assertGreater(anchor["radius_m"], 1.0)
        self.assertTrue(all(spawn.semantic_parent == "anchor_center" for spawn in spawns))

    def test_debug_mode_can_publish_primitive_detections(self):
        with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False) as fh:
            fh.write(
                textwrap.dedent(
                    """
                    oracle:
                      debug_primitive_detections: true
                    semantic_objects:
                      - name: anchor_debug
                        class_name: anchor
                        relative_position: [5.0, 0.0, 0.0]
                        parts:
                          - name: stem
                            prop_type: box
                            relative_position: [0.0, 0.0, 0.0]
                            radius_m: 1.0
                            scale: [0.3, 0.3, 2.0]
                          - name: tip
                            prop_type: sphere
                            relative_position: [0.0, 1.0, -1.0]
                            radius_m: 0.3
                            scale: 0.6
                    """
                )
            )
            fh.flush()
            cfg = SERVER.load_config(fh.name)

        spawns, oracle = SERVER.build_spawn_plan(cfg, 0.0, 0.0, 0.0, 0.0)

        self.assertEqual(len(spawns), 2)
        self.assertEqual(len(oracle), 3)
        self.assertEqual(oracle[0]["class_name"], "anchor")
        self.assertTrue(any(o["class_name"] == "anchor_part" for o in oracle[1:]))

    def test_anchor_scenario_yaml_parses(self):
        for name in [
            "anchor_center_static.yaml",
            "anchor_left_static.yaml",
            "anchor_right_static.yaml",
            "anchor_partially_visible.yaml",
            "anchor_with_spheres.yaml",
        ]:
            with self.subTest(name=name):
                cfg = SERVER.load_config(str(LEGACY_DIR / name))
                spawns, oracle = SERVER.build_spawn_plan(cfg, 0.0, 0.0, 0.0, 0.0)
                self.assertGreater(len(spawns), 0)
                self.assertTrue(any(o["class_name"] == "anchor" for o in oracle))

    def test_sphere_scenario_regression(self):
        cfg = SERVER.load_config(str(LEGACY_DIR / "sphere_front.yaml"))
        spawns, oracle = SERVER.build_spawn_plan(cfg, 0.0, 0.0, 0.0, 0.0)

        self.assertEqual(len(cfg.semantic_objects), 0)
        self.assertEqual(len(spawns), 1)
        self.assertEqual(len(oracle), 1)
        self.assertEqual(oracle[0]["class_name"], "sphere")
        self.assertNotIn("bounds", oracle[0])

    def test_constructing_server_does_not_import_holoocean(self):
        cfg = SERVER.load_config(str(LEGACY_DIR / "anchor_center_static.yaml"))
        server = SERVER.HolooceanSimServer(cfg, verbose=False)

        self.assertIsNone(server.env)
        self.assertIsNone(server.agent)


if __name__ == "__main__":
    unittest.main()
