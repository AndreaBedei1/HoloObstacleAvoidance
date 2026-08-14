"""Freeze the real-ROV pilot dataset into an auditable, labelled manifest.

WHY: experiments/real/ holds every artifact produced during the 2026-08-14
wet runs. Those runs were CALIBRATION/PILOT ONLY -- the pool geometry, the
detector and the engagement thresholds were all still being tuned between
runs, so nothing in that folder is admissible as final evaluation evidence.
The risk is not that someone lies about it later; it is that six months from
now nobody remembers which folder was pilot and which was the real campaign,
and a directory of plausible-looking logs quietly becomes "results". This
manifest is the record that makes that mistake impossible: a per-artifact
sha256 pins the bytes, and an explicit label pins the provenance.

WHY the git cross-check: a manifest built by walking the filesystem records
only what survived to today. That is precisely the wrong thing to freeze --
if aborted or inconvenient runs were cleaned up, a filesystem-only manifest
launders the survivors into a clean-looking record. So the manifest is
reconciled against git in both directions and reports, as first-class data:
  - artifacts tracked in HEAD but MISSING from the worktree (the pilot
    record is incomplete on disk; recover with `git checkout -- <path>`),
  - artifacts on disk but UNTRACKED (simply not committed yet),
  - artifacts on disk but GITIGNORED (excluded by rule, so git holds no
    copy at all -- this manifest's sha256 is their only integrity record).
An empty reconciliation is a positive result; a non-empty one is a finding
the reader must see before using the dataset.

Read-only: this script hashes and lists files. It never writes into, moves,
deletes or restores anything under experiments/real/ except the manifest it
generates, and it touches no hardware.

Usage:
    python scripts/analysis/build_pilot_manifest.py [--check]

    --check  Re-verify an existing manifest against the worktree and exit
             non-zero on any drift. Nothing is rewritten.
"""

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import os
import subprocess
import sys

REPO = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
REAL = os.path.join(REPO, "experiments", "real")
MANIFEST = os.path.join(REAL, "PILOT_MANIFEST.json")

# Everything under experiments/real is pilot data; the label only records
# WHICH KIND of pilot activity produced it, so a reader can tell a thruster
# characterisation log from a closed-loop avoidance run at a glance.
#
#   PILOT_CALIBRATION - wet closed-loop / dataset-capture runs whose numbers
#                       look like results but are calibration only.
#   BRINGUP           - platform characterisation: does the vehicle respond,
#                       what is its authority, does it hold station.
#   DEVELOPMENT       - software/algorithm iteration: detector checks,
#                       visual-servo tuning, supervised move tests.
LABELS = {
    "missions": "PILOT_CALIBRATION",
    "avoid_runs": "PILOT_CALIBRATION",
    "anchor_dataset": "PILOT_CALIBRATION",
    "approach_dataset": "PILOT_CALIBRATION",
    "motion_tests": "BRINGUP",
    "station_keep": "BRINGUP",
    "yaw_authority": "BRINGUP",
    "servo": "DEVELOPMENT",
    "supervised": "DEVELOPMENT",
}

# Unknown directories fall back to the most restrictive label rather than
# being dropped, so a folder added later can never default into looking
# like admissible evaluation data.
DEFAULT_LABEL = "PILOT_CALIBRATION"

# Generated documentation, not captured artifacts.
EXCLUDED = {"PILOT_MANIFEST.json", "README.md"}

STATEMENT = (
    "EVERY ARTIFACT LISTED IN THIS MANIFEST IS PILOT/CALIBRATION ONLY. "
    "These are the 2026-08-14 wet runs, recorded while pool geometry, the "
    "visual detector and the engagement thresholds were still being tuned "
    "between runs. NONE of this data may enter final evaluation statistics, "
    "reported results, tables or plots in the paper. It is admissible only "
    "as calibration input, qualitative illustration, or provenance history. "
    "Final statistics must come from a frozen post-calibration protocol run "
    "after this manifest's git SHA."
)


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for chunk in iter(lambda: fh.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def iso(ts):
    return dt.datetime.fromtimestamp(
        ts, dt.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def git(*args):
    """Run a git command in REPO; return stripped stdout or None."""
    try:
        out = subprocess.run(("git",) + args, cwd=REPO, capture_output=True,
                             text=True, timeout=60)
    except (OSError, subprocess.SubprocessError):
        return None
    return out.stdout.strip() if out.returncode == 0 else None


def label_for(rel):
    return LABELS.get(rel.split("/")[0], DEFAULT_LABEL)


def walk_worktree():
    """Every file under experiments/real, as repo-relative posix paths."""
    found = []
    for root, dirs, files in os.walk(REAL):
        dirs[:] = [d for d in dirs if d != ".git"]
        for name in sorted(files):
            full = os.path.join(root, name)
            rel = os.path.relpath(full, REAL).replace(os.sep, "/")
            if rel in EXCLUDED:
                continue
            found.append((rel, full))
    return sorted(found)


def tracked_blobs():
    """Map repo-relative-to-REAL path -> git blob sha at HEAD."""
    out = git("ls-tree", "-r", "HEAD", "--", "experiments/real")
    if not out:
        return {}
    blobs = {}
    for line in out.splitlines():
        meta, _, path = line.partition("\t")
        parts = meta.split()
        if len(parts) < 3 or parts[1] != "blob":
            continue
        rel = path[len("experiments/real/"):]
        blobs[rel] = parts[2]
    return blobs


def build():
    entries = []
    counts = {}
    total_bytes = 0
    for rel, full in walk_worktree():
        st = os.stat(full)
        lab = label_for(rel)
        counts[lab] = counts.get(lab, 0) + 1
        total_bytes += st.st_size
        entries.append({
            "path": rel,
            "label": lab,
            "size_bytes": st.st_size,
            "sha256": sha256(full),
            "mtime": iso(st.st_mtime),
        })

    on_disk = {e["path"] for e in entries}
    blobs = tracked_blobs()
    missing = sorted(set(blobs) - on_disk - EXCLUDED)

    # Not-in-HEAD splits into two very different situations: never committed
    # (an oversight, fixable with `git add`) and excluded by .gitignore (a
    # deliberate policy for bulky media -- but it means git holds no copy,
    # so this manifest is the only integrity record those files have).
    others = git("ls-files", "--others", "--exclude-standard",
                 "experiments/real") or ""
    prefix = "experiments/real/"
    untracked = sorted(
        p[len(prefix):] for p in others.splitlines()
        if p.startswith(prefix) and p[len(prefix):] in on_disk)
    ignored = sorted(on_disk - set(blobs) - set(untracked))

    dirty = git("status", "--porcelain")
    manifest = {
        "PILOT_ONLY_NOTICE": STATEMENT,
        "generated_utc": dt.datetime.now(
            dt.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "git_sha": git("rev-parse", "HEAD"),
        # A manifest taken from a dirty tree is not reproducible from the SHA
        # alone, so say so rather than implying a clean provenance.
        "git_worktree_clean": (dirty == ""),
        "root": "experiments/real",
        "labels_used": LABELS,
        "default_label_for_unknown_dirs": DEFAULT_LABEL,
        "totals": {
            "artifacts": len(entries),
            "bytes": total_bytes,
            "by_label": dict(sorted(counts.items())),
        },
        "integrity": {
            "tracked_but_missing_from_worktree": [
                {"path": p, "git_blob": blobs[p], "label": label_for(p),
                 "recover": "git checkout -- experiments/real/" + p}
                for p in missing
            ],
            "on_disk_but_untracked": [
                {"path": p, "label": label_for(p)} for p in untracked
            ],
            "on_disk_but_gitignored": [
                {"path": p, "label": label_for(p)} for p in ignored
            ],
            "note": (
                "A non-empty tracked_but_missing_from_worktree means the "
                "pilot record on disk is INCOMPLETE: those artifacts exist "
                "in git history but not in the working tree, so any count "
                "taken from the filesystem alone is a survivorship-filtered "
                "subset. on_disk_but_untracked artifacts are simply not "
                "committed yet. on_disk_but_gitignored artifacts are "
                "excluded from git by rule (bulky media), so no copy exists "
                "in history and the sha256 recorded here is their only "
                "integrity record -- back them up outside git."
            ),
        },
        "artifacts": entries,
    }
    return manifest


def check(manifest):
    """Re-hash the worktree and report drift against a stored manifest."""
    stored = {e["path"]: e for e in manifest["artifacts"]}
    drift = []
    for rel, full in walk_worktree():
        if rel not in stored:
            drift.append("ADDED    " + rel)
            continue
        if sha256(full) != stored[rel]["sha256"]:
            drift.append("MODIFIED " + rel)
    seen = {rel for rel, _ in walk_worktree()}
    for rel in sorted(set(stored) - seen):
        drift.append("REMOVED  " + rel)
    return drift


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--check", action="store_true",
                    help="verify the existing manifest; do not rewrite it")
    args = ap.parse_args()

    if not os.path.isdir(REAL):
        print("ERROR: not found: " + REAL, file=sys.stderr)
        return 2

    if args.check:
        if not os.path.exists(MANIFEST):
            print("ERROR: no manifest at " + MANIFEST, file=sys.stderr)
            return 2
        with open(MANIFEST, encoding="utf-8") as fh:
            drift = check(json.load(fh))
        if drift:
            print("DRIFT vs manifest (%d):" % len(drift))
            for d in drift:
                print("  " + d)
            return 1
        print("OK: worktree matches manifest.")
        return 0

    manifest = build()
    with open(MANIFEST, "w", encoding="utf-8") as fh:
        json.dump(manifest, fh, indent=2)
        fh.write("\n")

    t = manifest["totals"]
    print("Wrote %s" % MANIFEST)
    print("  git SHA      : %s (worktree clean: %s)"
          % (manifest["git_sha"], manifest["git_worktree_clean"]))
    print("  artifacts    : %d (%.1f MiB)"
          % (t["artifacts"], t["bytes"] / 1048576.0))
    for lab, n in t["by_label"].items():
        print("    %-18s %d" % (lab, n))

    integ = manifest["integrity"]
    miss = integ["tracked_but_missing_from_worktree"]
    untr = integ["on_disk_but_untracked"]
    if miss:
        print("  WARNING: %d artifact(s) tracked in HEAD are MISSING from "
              "the worktree." % len(miss))
        dirs = sorted({os.path.dirname(m["path"]) for m in miss})
        for d in dirs:
            print("    missing: %s" % d)
        print("  The on-disk pilot record is INCOMPLETE. Restore with:")
        print("    git checkout -- experiments/real")
    if untr:
        print("  NOTE: %d artifact(s) on disk are untracked (not committed "
              "yet)." % len(untr))
    ign = integ["on_disk_but_gitignored"]
    if ign:
        print("  NOTE: %d artifact(s) are gitignored: no copy exists in git "
              "history, so this manifest is their only integrity record."
              % len(ign))
    return 0


if __name__ == "__main__":
    sys.exit(main())
