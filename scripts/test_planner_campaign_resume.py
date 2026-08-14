"""Tests for the Phase-8 campaign RESUME semantics.

These protect completed experimental runs. A regression here can silently
destroy a multi-hour campaign or, worse, re-run an algorithm failure and
quietly improve the result — so the invariants are asserted, not assumed:

  1. an algorithm failure (collision, abort) is a VALID outcome and is never
     re-run (protocol section 4);
  2. a technical-invalid run is NOT a valid outcome and is re-run;
  3. a lost or corrupt manifest is recoverable from per-run artifacts;
  4. re-running a tuple never silently overwrites the previous attempt;
  5. the manifest write is atomic.

Run: python -m pytest scripts/test_planner_campaign_resume.py
"""

from __future__ import annotations

import importlib.util
import json
import os
import tempfile

_spec = importlib.util.spec_from_file_location(
    "rpc", os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "run_planner_campaign.py"))
rpc = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(rpc)


# --- 1/2: what counts as a completed experimental outcome ------------------

def test_completed_run_is_valid():
    assert rpc.has_valid_outcome({"ok": True, "metrics": {"a": 1}})


def test_algorithm_failure_is_valid_and_never_rerun():
    """A collision is a RESULT. Re-running it would bias the campaign."""
    collided = {"ok": True, "metrics": {"collision": True,
                                        "min_clearance_m": 0.05}}
    assert rpc.has_valid_outcome(collided)


def test_technical_invalid_is_not_valid():
    assert not rpc.has_valid_outcome(
        {"ok": True, "metrics": {"a": 1}, "technical_invalid": True})
    assert not rpc.has_valid_outcome(
        {"ok": True, "metrics": {"a": 1}, "also_retry_invalid": True})


def test_incomplete_run_is_not_valid():
    assert not rpc.has_valid_outcome({"ok": False, "metrics": {"a": 1}})
    assert not rpc.has_valid_outcome({"ok": True})


# --- 3: recovery ------------------------------------------------------------

def _write_run(root: str, name: str, metrics: dict) -> None:
    d = os.path.join(root, "runs", name)
    os.makedirs(d, exist_ok=True)
    with open(os.path.join(d, "validation.json"), "w") as f:
        json.dump(metrics, f)


def test_missing_campaign_dir_yields_nothing():
    with tempfile.TemporaryDirectory() as tmp:
        assert rpc.load_prior(os.path.join(tmp, "absent")) == []


def test_corrupt_manifest_recovers_from_run_artifacts():
    with tempfile.TemporaryDirectory() as tmp:
        _write_run(tmp, "F0_dwa_3", {"collision": False,
                                     "min_clearance_m": 0.7,
                                     "state_sequence": []})
        with open(os.path.join(tmp, "manifest.json"), "w") as f:
            f.write("{ truncated by a kill")
        prior = rpc.load_prior(tmp)
        assert len(prior) == 1
        rec = prior[0]
        assert (rec["scenario"], rec["planner"], rec["run"]) == ("F0", "dwa", 3)
        assert rec.get("recovered_from_run_dir") is True
        assert rpc.has_valid_outcome(rec)


def test_manifest_and_artifacts_do_not_double_count():
    with tempfile.TemporaryDirectory() as tmp:
        _write_run(tmp, "F0_dwa_3", {"collision": False,
                                     "state_sequence": []})
        with open(os.path.join(tmp, "manifest.json"), "w") as f:
            json.dump({"results": [{"scenario": "F0", "planner": "dwa",
                                    "run": 3, "ok": True,
                                    "metrics": {"collision": False}}]}, f)
        assert len(rpc.load_prior(tmp)) == 1


# --- 4: nothing is silently overwritten ------------------------------------

def test_preserve_partial_never_overwrites():
    with tempfile.TemporaryDirectory() as tmp:
        d = os.path.join(tmp, "runs", "F0_dwa_3")
        os.makedirs(d)
        with open(os.path.join(d, "validation.json"), "w") as f:
            f.write('{"first": true}')

        rpc.preserve_partial(d)
        first = os.path.join(d, "superseded_1", "validation.json")
        assert os.path.isfile(first)
        assert os.path.isfile(os.path.join(d, "superseded_1", "REASON.txt"))
        assert not os.path.isfile(os.path.join(d, "validation.json"))

        with open(os.path.join(d, "validation.json"), "w") as f:
            f.write('{"second": true}')
        rpc.preserve_partial(d)
        assert os.path.isfile(os.path.join(d, "superseded_2",
                                           "validation.json"))
        with open(first) as f:                      # untouched by round 2
            assert json.load(f) == {"first": True}


def test_preserve_partial_on_empty_or_missing_dir_is_noop():
    with tempfile.TemporaryDirectory() as tmp:
        rpc.preserve_partial(os.path.join(tmp, "absent"))      # no raise
        d = os.path.join(tmp, "empty")
        os.makedirs(d)
        rpc.preserve_partial(d)
        assert os.listdir(d) == []


# --- 5: atomic manifest ----------------------------------------------------

def test_write_manifest_is_atomic():
    with tempfile.TemporaryDirectory() as tmp:
        p = os.path.join(tmp, "manifest.json")
        rpc.write_manifest(p, {"results": [1, 2, 3]})
        with open(p) as f:
            assert json.load(f)["results"] == [1, 2, 3]
        assert not os.path.exists(p + ".tmp")


if __name__ == "__main__":
    # Standalone runner: the experiment machine's default interpreter has no
    # pytest, and this suite must stay runnable there.
    import sys
    tests = [(n, f) for n, f in sorted(globals().items())
             if n.startswith("test_") and callable(f)]
    failed = []
    for name, fn in tests:
        try:
            fn()
            print(f"  PASS  {name}")
        except Exception as exc:                       # noqa: BLE001
            print(f"  FAIL  {name}: {exc!r}")
            failed.append(name)
    print(f"\n{len(tests) - len(failed)}/{len(tests)} passed")
    sys.exit(1 if failed else 0)
