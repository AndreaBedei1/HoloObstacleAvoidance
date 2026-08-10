"""Unit tests for the sim-server command watchdog (no engine, no ROS).

Contract: if no fresh cmd_vel frame arrives within cmd_timeout_s, the
commanded body velocity is zeroed and stays zero until a fresh command
arrives. A server that never received any command idles at zero without
counting as a watchdog trip. Timeout <= 0 disables the watchdog.
"""

from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path

PACKAGE_DIR = Path(__file__).resolve().parents[1]
SERVER_DIR = PACKAGE_DIR / "holoocean_server"


def _load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


srv_mod = _load_module("holoocean_sim_server_wd_test",
                       SERVER_DIR / "holoocean_sim_server.py")


def make_server(timeout=1.0):
    cfg = srv_mod.SimConfig(cmd_timeout_s=timeout)
    return srv_mod.HolooceanSimServer(cfg, verbose=False)


CMD = {"surge": 0.4, "sway": 0.1, "heave": 0.0, "yaw_rate": 0.2}


class TestCommandWatchdog(unittest.TestCase):
    def test_no_command_ever_idles_without_trip(self):
        s = make_server()
        tripped = s.apply_command_watchdog(now_s=100.0)
        self.assertFalse(tripped)
        self.assertEqual(s.cmd["surge"], 0.0)

    def test_fresh_command_not_zeroed(self):
        s = make_server(timeout=1.0)
        s.apply_command(CMD, now_s=10.0)
        tripped = s.apply_command_watchdog(now_s=10.5)
        self.assertFalse(tripped)
        self.assertAlmostEqual(s.cmd["surge"], 0.4)
        self.assertAlmostEqual(s.cmd["yaw_rate"], 0.2)

    def test_stale_command_zeroed(self):
        s = make_server(timeout=1.0)
        s.apply_command(CMD, now_s=10.0)
        tripped = s.apply_command_watchdog(now_s=11.5)
        self.assertTrue(tripped)
        for key in ("surge", "sway", "heave", "roll_rate", "pitch_rate",
                    "yaw_rate"):
            self.assertEqual(s.cmd[key], 0.0, key)

    def test_boundary_exactly_at_timeout_not_tripped(self):
        s = make_server(timeout=1.0)
        s.apply_command(CMD, now_s=10.0)
        self.assertFalse(s.apply_command_watchdog(now_s=11.0))

    def test_recovery_on_fresh_command(self):
        s = make_server(timeout=1.0)
        s.apply_command(CMD, now_s=10.0)
        self.assertTrue(s.apply_command_watchdog(now_s=12.0))
        self.assertTrue(s._watchdog_active)
        s.apply_command(CMD, now_s=12.1)
        self.assertFalse(s.apply_command_watchdog(now_s=12.2))
        self.assertFalse(s._watchdog_active)
        self.assertAlmostEqual(s.cmd["surge"], 0.4)

    def test_watchdog_stays_tripped_while_stale(self):
        s = make_server(timeout=1.0)
        s.apply_command(CMD, now_s=10.0)
        for now in (12.0, 13.0, 14.0):
            self.assertTrue(s.apply_command_watchdog(now_s=now))
        self.assertEqual(s.cmd["surge"], 0.0)

    def test_timeout_zero_disables(self):
        s = make_server(timeout=0.0)
        s.apply_command(CMD, now_s=10.0)
        self.assertFalse(s.apply_command_watchdog(now_s=1000.0))
        self.assertAlmostEqual(s.cmd["surge"], 0.4)

    def test_default_config_has_watchdog_enabled(self):
        cfg = srv_mod.SimConfig()
        self.assertEqual(cfg.cmd_timeout_s, 1.0)


if __name__ == "__main__":
    unittest.main()
