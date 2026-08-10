# BlueROV2 in HoloOcean — Integration Notes

## Agent verification (2026-08-10, holoocean 2.3.0, env `holoocean_joystick`)

- `holoocean.agents.BlueROV2` **is the BlueROV2 Heavy**: 8 thrusters,
  vectored-6-DOF — the same topology as the real vehicle
  (`FRAME_CONFIG=2`, see `docs/REAL_BLUEROV2_HARDWARE_INVENTORY.md`).
- Control schemes: 0 = 8 thruster forces (used by us), 1 = PID waypoint,
  2 = accelerations; custom/Fossen dynamics hooks exist engine-side.
- Verified working on THIS machine with the prebuilt `Ocean` package
  (`SimpleUnderwater` world): spawns, ticks, and returns
  RGBCamera(512×512×4) / IMUSensor(2×3) / DVLSensor(7) / DepthSensor /
  PoseSensor(4×4) / VelocitySensor(3) under scheme-0 commands.
- The custom Unreal world (cooked `ancora/mina/siluro`) is NOT on this
  machine; scientific scenarios therefore run on stock worlds with primitive
  stand-in obstacles until the custom world is transferred (blocker B1).
  `build_custom_scenario_cfg` accepts `agent_type: BlueROV2` unchanged for
  when it is.

## Engine ground truth on thruster geometry (IMPORTANT)

The Python agent docstring's `thruster_d`/`thruster_p` matrices are
**wrong** (misordered and sign-flipped). The authoritative source is the
engine C++ (`Source/Holodeck/Agents/Private/BlueROV2.cpp::ApplyThrusters`,
`Public/BlueROV2.h::thrusterLocations`), read from the local HoloOcean-2.3.0
source tree:

- Thrusters 0–3: vertical, +force pushes **up**; positions
  (±0.12, ±0.2181, +0.0809) m in the client body frame (relative to COM,
  `Perfect=true`).
- Thrusters 4–7: horizontal 45° pods at (±0.1562, ±0.0988, 0.0) m; +force
  pushes **forward-left** for t4/t6 and **forward-right** for t5/t7 (all
  four have +x components).
- Client/ROS body frame: x forward, **y left**, z up (right-handed) —
  HoloOcean's client conversion flips UE's y. This matches ROS REP-103, so
  no additional frame conversion is needed for Twist commands.
- Per-thruster clamp engine-side: ±28.75 N (`BR_MAX_THRUST`).
- Vehicle constants: m=11.5 kg, neutral buoyancy (`Perfect=true`), COB 5 cm
  above COM, linear damping 1.0, angular damping 0.75.

The wrong-docstring geometry produced *zero net surge* in open-loop probes;
the corrected geometry reproduces every probe observation. Probe data:
`experiments/simulation/frame_probe/probe_results.json`
(`scripts/probe_bluerov2_frames.py`).

## Scenario configuration

New config path for stock worlds (`holoocean:` section):

```yaml
holoocean:
  stock_world: SimpleUnderwater   # any world in the installed Ocean package
  stock_package: Ocean
  agent_type: BlueROV2
  camera_socket: CameraSocket
sim:
  motion_model: dynamics          # teleport | hold | dynamics
dynamics:                         # bluerov2_dynamics.DynamicsConfig fields
  command_latency_s: 0.0
```

Reference scenario:
`src/rov_obstacle_sim_bridge/config/holoocean_scenarios/bluerov2_dynamics_smoke.yaml`.

Teleport is retained ONLY for legacy regression scenarios; scientific
scenarios must set `motion_model: dynamics` (`load_config` validates the
value). Primitive obstacles (`spawn_prop`) work in stock worlds unchanged.
