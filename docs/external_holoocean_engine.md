# External modified HoloOcean engine — inspection notes

This document records what was found in the external, **read-only** folder
`C:\Users\andrea.bedei3\Desktop\Holo\MondoTest` and how this repository uses
it. Nothing in that folder is ever modified by this repo; all integration
code, configs and launchers live inside `HoloObstacleAvoidance`.

## Layout of the external folder

```text
MondoTest/
├── HoloOcean/
│   ├── main.py                  # previous manual workflow: spawn 1533-object population
│   ├── client/                  # HoloOcean Python client source (v2.2.2)
│   ├── create_dataset/          # generate_world_population.py (octree -> population JSON)
│   ├── engine/                  # MODIFIED UE 5.3.2 project
│   │   ├── Holodeck.uproject    # <- the modified engine (editor project, NOT packaged)
│   │   ├── Binaries/Win64/UnrealEditor-Holodeck.dll   # compiled editor module
│   │   ├── Content/
│   │   │   ├── ExampleLevel.umap        # custom underwater world
│   │   │   ├── ancora.uasset            # REAL anchor mesh  -> /Game/ancora.ancora
│   │   │   ├── mina.uasset              # mine mesh         -> /Game/mina.mina
│   │   │   ├── siluro.uasset            # torpedo mesh      -> /Game/siluro.siluro
│   │   │   ├── Megascans/               # rocks / seaweed / coral assets
│   │   │   └── Config/
│   │   │       ├── world_population.json            # 1533 spawns (240 mina, 200 siluro,
│   │   │       │                                    #  200 ancora, 332 rocks, 560 seaweed, 1 coral)
│   │   │       └── runtime_world_commands_README.md # docs for the C++ spawn commands
│   │   ├── Octrees/ExampleLevel/        # seabed octree data (used to place assets)
│   │   └── Source/Holodeck/             # C++ mods (see below)
│   └── HoloOceanDataset/                # sonar recognition training experiments
└── materials.csv
```

## The engine modifications

The UE project adds three **world commands** (registered in
`Source/Holodeck/ClientCommands/Private/CommandFactory.cpp`) plus a spawner
actor:

| Command | Purpose |
| --- | --- |
| `SpawnAsset` | Spawn any static mesh (e.g. `/Game/ancora.ancora`) at runtime |
| `ClearSpawned` | Destroy all runtime-spawned actors |
| `RespawnFromConfig` | Clear + spawn a whole JSON population |

`SpawnAsset` parameters (from `SpawnAssetCommand.cpp`):

* 9 numbers: `x, y, z, roll, pitch, yaw, scale_x, scale_y, scale_z`
* 1–3 strings: `mesh_asset_path`, optional `actor_label`, optional `units`
  (`"ue"` = UE centimetres; `"m" / "meters" / "client"` = client metres via
  `ConvertLinearVector(ClientToUE)`, i.e. ×100 with the Y axis flipped)

The spawner (`ARuntimeRowSpawner`, `Source/Holodeck/Utils/`) is created on
demand — no actor needs to be placed in the level.

The `HolodeckOn` command-line gate in `HolodeckGameMode.cpp` is commented
out, so the Holodeck shared-memory server **always** starts in `-game` mode.

## Custom anchors: spawnable, not static

The custom worlds contain **no** baked-in anchors. Every anchor / mine /
torpedo is spawned **at runtime** with `SpawnAsset`. The previous manual
workflow was:

1. Start the engine visibly (UE editor `-game` mode, window 1280×720):

   ```text
   UnrealEditor.exe <...>/engine/Holodeck.uproject /Game/ExampleLevel -game
       -windowed -ResX=1280 -ResY=720 -TicksPerSec=30 -FramesPerSec=30
   ```

2. In the conda `ocean` env run `python main.py`, which attaches with
   `holoocean.make(scenario_cfg=..., start_world=False)` and enqueues one
   `SpawnAsset` command per population entry (batches of 120/tick).

## Environment

* **conda `ocean`** (`C:\Users\andrea.bedei3\.conda\envs\ocean`): Python 3.9,
  `holoocean 2.3.0` (installed from `Desktop/HoloOcean-2.3.0/client`), numpy,
  opencv, pywin32, pyyaml. The 2.3.0 client is shared-memory compatible with
  the modified 2.2.2-based engine (`shmem.py`/`holooceanclient.py` identical).
* **Unreal Editor 5.3** at `C:\Program Files\Epic Games\UE_5.3` (stock; the
  modifications are entirely inside the external project).
* The **packaged** HoloOcean 2.3.0 "Ocean" worlds in
  `%LOCALAPPDATA%\holoocean\2.3.0` are the *stock* engine used by the
  primitive scenarios — they do **not** contain `SpawnAsset` or the custom
  meshes (verified by binary search of the exe/pak).

## Client attach mechanics (why our launcher works)

* With `start_world=False`, `holoocean.make` opens the engine's named shared
  memory / semaphores (`Global\HOLODECK_SEMAPHORE_*` with an empty UUID) —
  matching an engine launched without `--HolodeckUUID`.
* The engine creates those objects at map start; the client retries until
  they exist (see `custom_engine_launcher.attach_holoocean`).
* `scenario_cfg` must contain `ticks_per_sec` **and** `frames_per_sec`,
  otherwise `holoocean.make` falls back to an interactive `input()` prompt
  and non-interactive runs die with `EOFError`.
* The client's scenario agents (HoveringAUV + PoseSensor / VelocitySensor /
  DepthSensor / RGBCamera) are spawned by the engine on attach.
* **One attach per engine start.** The engine releases its "server ready"
  semaphore exactly once at map start, and the attach path waits on it with
  an INFINITE timeout — attaching to a window that a previous client already
  used deadlocks both sides.  `custom_engine_launcher.attach_holoocean`
  probes the semaphore first (`engine_ready_signal_pending`) and reports a
  stale window instead of hanging.  `--engine-running` therefore only works
  with a freshly opened window (e.g. `start_custom_holoocean_visible.bat`
  that nothing has attached to yet).
* `spawn_prop` (primitive props) is **not** supported by the modified
  engine: `SpawnProp` is a level-blueprint command that `ExampleLevel` does
  not implement. Distractor primitives in custom worlds are spawned through
  `SpawnAsset` with `/Engine/BasicShapes/*` meshes instead.
* **Camera pixel order is BGRA**, not RGBA: the engine fills the shared
  camera buffer through an `FColor*` (`HolodeckCamera.cpp`), and UE's
  `FColor` memory layout is B,G,R,A — the client docstring saying "RGBA" is
  wrong. Consumers must take channels `2,1,0` to get RGB (the sim server
  does this before advertising `rgb8`).

## ExampleLevel geometry notes

* Client coordinates, metres, REP-103 (+x fwd, +y left, +z up).
* Water surface ≈ −25 m; seabed between ≈ −26 m and −76 m (from
  `world_population.json` octree metadata).
* Core octree area (real seabed data): UE X ∈ [−18432, 24576] cm,
  Y ∈ [−17408, 16384] cm.
* Population anchors use raw entry scale × ⅔ ≈ 7–14 (Unreal actor scale) —
  our scenarios use 12.3.
* Reference anchor site used by this repo (entry `ancora_007`):
  client (−93.55, 38.38, −69.60) m; rover start 8–12 m behind it.
