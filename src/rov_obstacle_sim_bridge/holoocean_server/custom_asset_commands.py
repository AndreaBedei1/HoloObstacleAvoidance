"""Direct HoloOcean commands for the EXTERNAL modified engine.

The modified engine registers ``SpawnAsset`` / ``ClearSpawned`` /
``RespawnFromConfig`` in its C++ ``CommandFactory``.  They MUST be sent as
*direct* commands (``Command.set_command_type``), exactly like the external
``main.py`` does.

Do **not** use ``env.send_world_command`` for these: that wraps the name in a
``CustomCommand`` which is routed to the level blueprint's
``ExecuteCustomCommand`` event — and the custom worlds' blueprint does not
implement these names, which makes the engine raise a *fatal* error
("World cannot execute given command") and exit.

The helpers import ``holoocean`` lazily so this module can be imported (and
unit-tested for its pure parts) outside the conda ``ocean`` environment.
"""

from __future__ import annotations

from typing import Any, Optional, Sequence


def _direct_command(command_type: str,
                    num_params: Sequence[float],
                    string_params: Sequence[str]):
    from holoocean.command import Command

    cmd = Command()
    cmd.set_command_type(command_type)
    cmd.add_number_parameters([float(v) for v in num_params])
    for s in string_params:
        cmd.add_string_parameters(str(s))
    return cmd


def spawn_asset_params(
    position: Sequence[float],
    rotation: Sequence[float],
    scale: Sequence[float],
    mesh_asset: str,
    label: str = "",
    units: str = "meters",
) -> tuple[list[float], list[str]]:
    """Pure helper returning (num_params, string_params) for ``SpawnAsset``.

    ``units="meters"`` means client metres (REP-103, +y left); the engine
    converts with ``ConvertLinearVector(ClientToUE)``.  Unit-testable without
    holoocean installed.
    """
    if len(position) != 3 or len(rotation) != 3 or len(scale) != 3:
        raise ValueError("position, rotation and scale must have 3 elements")
    nums = [
        float(position[0]), float(position[1]), float(position[2]),
        float(rotation[0]), float(rotation[1]), float(rotation[2]),
        float(scale[0]), float(scale[1]), float(scale[2]),
    ]
    strings = [str(mesh_asset), str(label), str(units)]
    return nums, strings


def enqueue_spawn_asset(
    env: Any,
    position: Sequence[float],
    rotation: Sequence[float],
    scale: Sequence[float],
    mesh_asset: str,
    label: str = "",
    units: str = "meters",
) -> None:
    """Queue a ``SpawnAsset`` on the environment (executes on next tick)."""
    nums, strings = spawn_asset_params(
        position, rotation, scale, mesh_asset, label, units
    )
    env._enqueue_command(_direct_command("SpawnAsset", nums, strings))


def enqueue_clear_spawned(env: Any) -> None:
    """Queue a ``ClearSpawned`` (removes all runtime-spawned actors)."""
    env._enqueue_command(_direct_command("ClearSpawned", [], []))


def enqueue_respawn_from_config(
    env: Any,
    config_path: str,
    path_is_absolute: bool = False,
) -> None:
    """Queue a ``RespawnFromConfig`` with a JSON population file."""
    env._enqueue_command(
        _direct_command(
            "RespawnFromConfig",
            [],
            [config_path, "true" if path_is_absolute else "false"],
        )
    )
