#!/usr/bin/env python
"""Launch the EXTERNAL modified HoloOcean engine (visible) and wait.

Thin CLI wrapper around ``custom_engine_launcher`` so .bat scripts can start
the engine window on its own.  Prints the exact command line used, then keeps
the console attached until Ctrl+C (the engine window itself stays open when
this wrapper is killed).

Works in any Python environment that has PyYAML (conda ``ocean`` included).
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(
    0, str(_REPO_ROOT / "src" / "rov_obstacle_sim_bridge" / "holoocean_server")
)

from custom_engine_launcher import (  # noqa: E402
    engine_log_path,
    launch_engine,
    load_engine_config,
    validate_engine_config,
)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--engine-config", default=None)
    parser.add_argument("--map", default=None, help="override the default map")
    parser.add_argument("--no-wait", action="store_true",
                        help="return immediately instead of monitoring")
    args = parser.parse_args()

    cfg = load_engine_config(args.engine_config)
    problems = validate_engine_config(cfg, check_paths=True)
    if problems:
        print("[launch] invalid engine config:")
        for p in problems:
            print("  -", p)
        return 2

    proc = launch_engine(cfg, map_name=args.map)
    print(f"[launch] engine PID {proc.pid}; log: {engine_log_path(cfg)}")
    if args.no_wait:
        return 0
    print("[launch] engine window is up once the map finishes loading "
          "(~1 min). Ctrl+C leaves the window running.")
    try:
        while proc.poll() is None:
            time.sleep(2.0)
        print(f"[launch] engine exited with code {proc.returncode}")
        return int(proc.returncode or 0)
    except KeyboardInterrupt:
        print("[launch] detached; engine window keeps running")
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
