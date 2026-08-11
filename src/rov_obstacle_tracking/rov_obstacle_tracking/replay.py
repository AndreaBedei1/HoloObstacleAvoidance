"""Deterministic offline replay for temporal estimators (no ROS, no Unreal).

Dataset format: JSONL, one record per upstream frame SLOT at the nominal
detector rate. Every record distinguishes the three upstream conditions:

  {"t": <float>,
   "message_present": bool,     # false => SILENCE (no on_message call)
   "detection_present": bool,   # message_present and zero detections
                                #   => fresh-EMPTY array
   "meas": {class_name, confidence, cx, cy, w, h} | null,
   "gt":   {present: bool, cx, cy, w, h} | null,   # EVALUATION-ONLY
   "quality": {...} | null}     # optional image-quality signals

Ground truth is consumed ONLY by the evaluation layer; the estimator sees
exclusively `meas` via DetectionEvent. Replay is fully deterministic:
same file + same estimator config => identical outputs.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

from .temporal_core import (
    Detection,
    DetectionEvent,
    EstimatorOutput,
    TemporalEstimator,
)


@dataclass
class ReplayRecord:
    t: float
    message_present: bool
    detection_present: bool
    meas: Optional[Dict[str, Any]]
    gt: Optional[Dict[str, Any]]
    quality: Optional[Dict[str, Any]] = None

    @classmethod
    def from_json(cls, line: str) -> "ReplayRecord":
        d = json.loads(line)
        return cls(
            t=float(d["t"]),
            message_present=bool(d["message_present"]),
            detection_present=bool(d.get("detection_present", False)),
            meas=d.get("meas"),
            gt=d.get("gt"),
            quality=d.get("quality"),
        )

    def to_json(self) -> str:
        return json.dumps({
            "t": round(self.t, 6),
            "message_present": self.message_present,
            "detection_present": self.detection_present,
            "meas": self.meas,
            "gt": self.gt,
            "quality": self.quality,
        })


def load_dataset(path: str) -> List[ReplayRecord]:
    records = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line:
                records.append(ReplayRecord.from_json(line))
    return records


def save_dataset(records: List[ReplayRecord], path: str) -> None:
    with open(path, "w") as f:
        for r in records:
            f.write(r.to_json() + "\n")


@dataclass
class TickResult:
    t: float
    output: EstimatorOutput
    gt: Optional[Dict[str, Any]]          # gt at nearest record (eval only)
    upstream_message: bool                # a message arrived since last tick
    upstream_detection: bool


def run_replay(estimator: TemporalEstimator, records: List[ReplayRecord],
               output_rate_hz: float = 30.0) -> List[TickResult]:
    """Feed records chronologically; tick the estimator at a fixed rate.

    Message events and output ticks are interleaved by timestamp exactly as
    the ROS node would experience them (message callback, then timer).
    """
    if not records:
        return []
    results: List[TickResult] = []
    dt = 1.0 / output_rate_hz
    t_tick = records[0].t
    idx = 0
    t_end = records[-1].t + dt
    last_gt = None
    msg_since_tick = False
    det_since_tick = False
    while t_tick <= t_end:
        # Deliver all messages with t <= t_tick.
        while idx < len(records) and records[idx].t <= t_tick + 1e-9:
            rec = records[idx]
            last_gt = rec.gt
            if rec.message_present:
                msg_since_tick = True
                dets = []
                if rec.detection_present and rec.meas is not None:
                    m = rec.meas
                    dets = [Detection(
                        class_name=str(m["class_name"]),
                        confidence=float(m["confidence"]),
                        cx=float(m["cx"]), cy=float(m["cy"]),
                        w=float(m["w"]), h=float(m["h"]),
                    )]
                    det_since_tick = True
                estimator.on_message(DetectionEvent(t=rec.t, detections=dets))
            idx += 1
        out = estimator.tick(t_tick)
        results.append(TickResult(
            t=t_tick, output=out, gt=last_gt,
            upstream_message=msg_since_tick,
            upstream_detection=det_since_tick,
        ))
        msg_since_tick = False
        det_since_tick = False
        t_tick += dt
    return results
