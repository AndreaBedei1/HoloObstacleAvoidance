"""Load and validate a nested S0-S3 calibration profile.

WHY this exists: the calibration ladder only supports the headline claim
(paper/contributions.md C1) if each rung differs from the previous one by
exactly ONE class of effect.  Nothing in a set of YAML files enforces that
by itself: a knob touched at two levels, or a plausible-looking number
quietly filling an unmeasured slot, would break the attribution without
producing any error.  This loader is that enforcement.

It merges s0_historical.yaml -> s1_observation.yaml -> s2_timing.yaml ->
s3_vehicle.yaml and refuses the profile unless:

  * the parent chain is intact and S0 is the immutable base;
  * the sections each level declares in `owns` are pairwise disjoint, and a
    level only overrides sections it owns;
  * NO leaf key is overridden at two different levels (the double-counting
    check the ladder depends on);
  * every S0 leaf, and every non-null override, carries provenance;
  * every null carries a TODO naming the measurement that would fill it.

Nulls are legal in a profile - they are how "not measured yet" is written
down - but `--require-measured` (or require_measured=True) rejects any
profile that still contains one, so an unmeasured value can never become an
experimental condition by accident.

Usage::

    python scripts/calibration/load_profile.py S2
    python scripts/calibration/load_profile.py S3 --json
    python scripts/calibration/load_profile.py S3 --require-measured
    python scripts/calibration/load_profile.py S1 --todo

Unit test: scripts/calibration/test_load_profile.py
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

import yaml

REPO = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
DEFAULT_CONFIG_DIR = os.path.join(REPO, "config", "calibration")

LEVEL_FILES = [
    ("S0", "s0_historical.yaml"),
    ("S1", "s1_observation.yaml"),
    ("S2", "s2_timing.yaml"),
    ("S3", "s3_vehicle.yaml"),
]

ALLOWED_TOP_KEYS = {"profile", "values", "overrides", "provenance", "todo",
                    "evidence"}


class ProfileError(ValueError):
    """Raised when a profile chain violates a ladder rule."""


@dataclass
class Profile:
    """Result of merging a validated chain."""

    level: str
    chain: List[str]
    values: Dict[str, Any]
    provenance: Dict[str, str]
    todo: Dict[str, str]
    # Leaf keys whose merged value is null (unmeasured), in ladder order.
    unmeasured: List[str] = field(default_factory=list)
    # Leaf keys introduced above S0 (new model structure, not in the
    # historical configuration), reported so growth stays visible.
    introduced: List[Tuple[str, str]] = field(default_factory=list)
    # Keys that HAVE a value but still carry a TODO (weak support).
    advisory: List[Tuple[str, str]] = field(default_factory=list)
    # level -> owned sections.
    ownership: Dict[str, List[str]] = field(default_factory=dict)


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------

def _is_branch(value: Any) -> bool:
    """A non-empty mapping is a branch; everything else is a leaf."""
    return isinstance(value, dict) and bool(value)


def flatten(tree: Any, prefix: str = "") -> Dict[str, Any]:
    """Flatten nested dicts to dotted leaf paths (lists stay leaves)."""
    out: Dict[str, Any] = {}
    if not _is_branch(tree):
        if prefix:
            out[prefix] = tree
        return out
    for key, value in tree.items():
        path = f"{prefix}.{key}" if prefix else str(key)
        if _is_branch(value):
            out.update(flatten(value, path))
        else:
            out[path] = value
    return out


def deep_merge(base: Any, over: Any) -> Any:
    """Merge `over` onto `base`.

    A mapping merges key by key; anything else (including an explicit null,
    which is how a level says "this whole subtree must be re-derived")
    replaces the base value outright.
    """
    if _is_branch(base) and _is_branch(over):
        merged = dict(base)
        for key, value in over.items():
            merged[key] = (deep_merge(base[key], value)
                           if key in base else value)
        return merged
    return over


def resolve(mapping: Dict[str, str], key: str) -> Optional[str]:
    """Look a dotted key up allowing an ancestor prefix to cover a subtree."""
    if key in mapping:
        return mapping[key]
    parts = key.split(".")
    for cut in range(len(parts) - 1, 0, -1):
        prefix = ".".join(parts[:cut])
        if prefix in mapping:
            return mapping[prefix]
    return None


def _section(key: str) -> str:
    return key.split(".", 1)[0]


# ---------------------------------------------------------------------------
# loading + validation
# ---------------------------------------------------------------------------

def _read_level(config_dir: str, level_id: str,
                filename: str) -> Dict[str, Any]:
    path = os.path.join(config_dir, filename)
    if not os.path.isfile(path):
        raise ProfileError(f"{level_id}: missing profile file {path}")
    with open(path, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    if not isinstance(data, dict):
        raise ProfileError(f"{level_id}: {filename} is not a YAML mapping")

    unknown = set(data) - ALLOWED_TOP_KEYS
    if unknown:
        raise ProfileError(
            f"{level_id}: unknown top-level key(s) {sorted(unknown)} in "
            f"{filename}; allowed: {sorted(ALLOWED_TOP_KEYS)}")

    profile = data.get("profile") or {}
    if profile.get("id") != level_id:
        raise ProfileError(
            f"{filename}: profile.id is {profile.get('id')!r}, expected "
            f"{level_id!r}")
    return data


def _validate_chain(levels: List[Dict[str, Any]]) -> None:
    """S0 is the immutable base; every other level names its parent file."""
    s0 = levels[0]
    prof0 = s0["profile"]
    if prof0.get("parent") is not None:
        raise ProfileError("S0 must not declare a parent")
    if not prof0.get("base_level"):
        raise ProfileError("S0 must declare base_level: true")
    if prof0.get("status") != "IMMUTABLE":
        raise ProfileError("S0 must be marked IMMUTABLE")
    if "values" not in s0:
        raise ProfileError("S0 must provide a `values` block")
    if "overrides" in s0:
        raise ProfileError("S0 must not provide an `overrides` block")

    for index in range(1, len(levels)):
        level = levels[index]
        level_id = level["profile"]["id"]
        expected_parent = LEVEL_FILES[index - 1][1]
        if level["profile"].get("parent") != expected_parent:
            raise ProfileError(
                f"{level_id}: parent is "
                f"{level['profile'].get('parent')!r}, expected "
                f"{expected_parent!r}")
        if "values" in level:
            raise ProfileError(
                f"{level_id}: only S0 may declare `values`; use `overrides`")
        if "overrides" not in level:
            raise ProfileError(f"{level_id}: missing `overrides` block")


def _validate_ownership(levels: List[Dict[str, Any]]) -> Dict[str, List[str]]:
    """Owned sections must be disjoint, and overrides must stay in them."""
    ownership: Dict[str, List[str]] = {}
    seen: Dict[str, str] = {}
    for level in levels:
        level_id = level["profile"]["id"]
        owns = list(level["profile"].get("owns") or [])
        if not owns:
            raise ProfileError(f"{level_id}: profile.owns must be non-empty")
        ownership[level_id] = owns
        for section in owns:
            if section in seen:
                raise ProfileError(
                    f"section {section!r} is owned by both {seen[section]} "
                    f"and {level_id}; effect classes must not overlap")
            seen[section] = level_id

    for level in levels[1:]:
        level_id = level["profile"]["id"]
        owns = set(ownership[level_id])
        for key in flatten(level.get("overrides") or {}):
            section = _section(key)
            if section not in owns:
                owner = seen.get(section, "no level")
                raise ProfileError(
                    f"{level_id} overrides {key!r} but section "
                    f"{section!r} is owned by {owner}")
    return ownership


def _validate_no_double_override(levels: List[Dict[str, Any]]) -> None:
    """The core ladder rule: one knob, one level."""
    owner: Dict[str, str] = {}
    for level in levels[1:]:
        level_id = level["profile"]["id"]
        for key in flatten(level.get("overrides") or {}):
            if key in owner:
                raise ProfileError(
                    f"key {key!r} is overridden at both {owner[key]} and "
                    f"{level_id}; each effect belongs to exactly one level")
            owner[key] = level_id


def _validate_documentation(levels: List[Dict[str, Any]]) -> None:
    """Provenance for every value, a TODO for every null."""
    s0_flat = flatten(levels[0]["values"])
    s0_prov = levels[0].get("provenance") or {}
    for key, value in s0_flat.items():
        if value is None:
            raise ProfileError(
                f"S0 leaf {key!r} is null; S0 records what the system does "
                f"today and cannot contain unmeasured values")
        if resolve(s0_prov, key) is None:
            raise ProfileError(f"S0 leaf {key!r} has no provenance entry")

    for level in levels[1:]:
        level_id = level["profile"]["id"]
        flat = flatten(level.get("overrides") or {})
        prov = level.get("provenance") or {}
        todo = level.get("todo") or {}
        for key, value in flat.items():
            if value is None:
                if resolve(todo, key) is None:
                    raise ProfileError(
                        f"{level_id}: {key!r} is null with no todo naming "
                        f"the measurement that would fill it")
            elif resolve(prov, key) is None:
                raise ProfileError(
                    f"{level_id}: {key!r} has a value but no provenance")
        for key in todo:
            if not any(k == key or k.startswith(key + ".") for k in flat):
                raise ProfileError(
                    f"{level_id}: todo key {key!r} matches no override")
        for key in prov:
            if not any(k == key or k.startswith(key + ".") for k in flat):
                raise ProfileError(
                    f"{level_id}: provenance key {key!r} matches no override")


def load_profile(level: str = "S3",
                 config_dir: Optional[str] = None,
                 require_measured: bool = False) -> Profile:
    """Merge and validate the chain S0..`level`."""
    level = str(level).upper()
    ids = [lid for lid, _ in LEVEL_FILES]
    if level not in ids:
        raise ProfileError(f"unknown level {level!r}; expected one of {ids}")
    config_dir = config_dir or DEFAULT_CONFIG_DIR
    wanted = LEVEL_FILES[:ids.index(level) + 1]

    levels = [_read_level(config_dir, lid, name) for lid, name in wanted]
    _validate_chain(levels)
    ownership = _validate_ownership(levels)
    _validate_no_double_override(levels)
    _validate_documentation(levels)

    values: Dict[str, Any] = levels[0]["values"]
    provenance: Dict[str, str] = dict(levels[0].get("provenance") or {})
    todo: Dict[str, str] = {}
    introduced: List[Tuple[str, str]] = []
    advisory: List[Tuple[str, str]] = []

    s0_keys = set(flatten(levels[0]["values"]))
    for level_data in levels[1:]:
        level_id = level_data["profile"]["id"]
        overrides = level_data.get("overrides") or {}
        flat = flatten(overrides)
        for key, value in flat.items():
            if key not in s0_keys:
                introduced.append((level_id, key))
            if value is not None and resolve(
                    level_data.get("todo") or {}, key) is not None:
                advisory.append((level_id, key))
        values = deep_merge(values, overrides)
        provenance.update(level_data.get("provenance") or {})
        todo.update(level_data.get("todo") or {})

    merged_flat = flatten(values)
    unmeasured = [k for k, v in merged_flat.items() if v is None]

    if require_measured and unmeasured:
        raise ProfileError(
            f"{level} still has {len(unmeasured)} unmeasured value(s): "
            f"{', '.join(unmeasured)}")

    return Profile(
        level=level,
        chain=[name for _, name in wanted],
        values=values,
        provenance=provenance,
        todo=todo,
        unmeasured=unmeasured,
        introduced=introduced,
        advisory=advisory,
        ownership=ownership,
    )


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _report(profile: Profile) -> str:
    lines = [f"profile {profile.level}: {' -> '.join(profile.chain)}"]
    lines.append("ownership:")
    for level_id, sections in profile.ownership.items():
        lines.append(f"  {level_id}: {', '.join(sections)}")
    lines.append(f"leaves: {len(flatten(profile.values))}, "
                 f"unmeasured: {len(profile.unmeasured)}")
    if profile.introduced:
        lines.append("introduced above S0:")
        for level_id, key in profile.introduced:
            lines.append(f"  {level_id} {key}")
    if profile.advisory:
        lines.append("valued but flagged (todo present):")
        for level_id, key in profile.advisory:
            lines.append(f"  {level_id} {key}")
    if profile.unmeasured:
        lines.append("unmeasured (null):")
        for key in profile.unmeasured:
            lines.append(f"  {key}")
    return "\n".join(lines)


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("level", nargs="?", default="S3",
                        help="S0 | S1 | S2 | S3 (default S3)")
    parser.add_argument("--config-dir", default=DEFAULT_CONFIG_DIR)
    parser.add_argument("--json", action="store_true",
                        help="print the merged values as JSON")
    parser.add_argument("--yaml", action="store_true",
                        help="print the merged values as YAML")
    parser.add_argument("--todo", action="store_true",
                        help="print the outstanding measurements")
    parser.add_argument("--require-measured", action="store_true",
                        help="fail if any value is still null")
    args = parser.parse_args(argv)

    try:
        profile = load_profile(args.level, args.config_dir,
                               require_measured=args.require_measured)
    except ProfileError as exc:
        print(f"INVALID PROFILE: {exc}", file=sys.stderr)
        return 2

    if args.json:
        print(json.dumps(profile.values, indent=2, sort_keys=False))
    elif args.yaml:
        print(yaml.safe_dump(profile.values, sort_keys=False))
    elif args.todo:
        for key in profile.unmeasured:
            text = " ".join(str(resolve(profile.todo, key) or "").split())
            print(f"{key}: {text}")
    else:
        print(_report(profile))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
