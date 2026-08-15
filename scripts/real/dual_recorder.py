"""Continuous dual-stream video recording for the final validation runs.

Every final run must preserve BOTH raw video streams, not only derived
trajectories and tables: the onboard BlueROV2 RGB (what the detector
actually saw) and the overhead RealSense RGB (what the vehicle actually
did). A CSV of positions cannot be re-analysed with a different detector,
cannot show a reviewer why a detection failed, and cannot be checked
against the claimed geometry. The videos can.

SYNCHRONIZATION. Both writers stamp every frame with the SAME system
clock (`time.time()`, the clock every other log in this project uses),
in a sidecar index next to each video:

    onboard_index.jsonl   {"i": frame_number, "t": wall_clock_seconds}
    overhead_index.jsonl  {"i": frame_number, "t": wall_clock_seconds}

so any record from the ground truth, the perception chain, the planner,
the adapter or the vehicle telemetry joins to a frame by timestamp, and
the two videos join to each other. Video containers are not trusted for
timing: their frame rate is nominal and their timestamps are rewritten
by the encoder, so the index is the authority.

OPTIONAL HARD SYNC MARKER. The ROV lights are visible from above. If a
flash is requested, the recorder logs the commanded flash times to
`sync_markers.jsonl`; the flash is visible in the overhead video and its
onboard glow in the onboard video, giving an INDEPENDENT check that the
two indices really are on the same clock — a software-only claim would
otherwise be untestable.

DROPPED FRAMES ARE RECORDED, NOT HIDDEN. Each index line is written when
a frame is actually encoded, so a gap in `i` versus wall time is visible
in the data rather than silently interpolated.

Nothing here commands the vehicle. The lights flash only if the caller
passes a callback that does it.
"""

from __future__ import annotations

import json
import os
import threading
import time
from typing import Callable, Optional

import cv2


class _StreamWriter:
    """One video stream: encoder thread + timestamp index."""

    def __init__(self, path_base: str, size, fps: float, name: str):
        self.name = name
        self.fps = fps
        self.video_path = path_base + ".mp4"
        self.index_path = path_base + "_index.jsonl"
        self._writer = cv2.VideoWriter(
            self.video_path, cv2.VideoWriter_fourcc(*"mp4v"), fps, size)
        if not self._writer.isOpened():
            raise RuntimeError(f"cannot open video writer for {name}")
        self._index = open(self.index_path, "w")
        self._lock = threading.Lock()
        self.frames = 0
        self.first_t = None
        self.last_t = None

    def write(self, frame, t_wall: float) -> None:
        with self._lock:
            self._writer.write(frame)
            self._index.write(json.dumps(
                {"i": self.frames, "t": round(t_wall, 4)}) + "\n")
            if self.first_t is None:
                self.first_t = t_wall
            self.last_t = t_wall
            self.frames += 1

    def close(self) -> dict:
        with self._lock:
            self._writer.release()
            self._index.flush()
            self._index.close()
        span = (None if self.first_t is None
                else round(self.last_t - self.first_t, 3))
        return {"stream": self.name, "video": os.path.basename(
            self.video_path), "index": os.path.basename(self.index_path),
            "frames": self.frames, "span_s": span,
            "measured_fps": (None if not span or span <= 0
                             else round(self.frames / span, 2)),
            "nominal_fps": self.fps}


class DualRecorder:
    """Record the onboard and overhead streams for one run.

    `onboard_source` and `overhead_source` are callables returning
    (frame, t_frame) and (frame) respectively; they are polled in their
    own threads so a slow encoder never stalls the control loop.
    """

    def __init__(self, session_dir: str,
                 onboard_source: Optional[Callable] = None,
                 overhead_source: Optional[Callable] = None,
                 onboard_fps: float = 12.0,
                 overhead_fps: float = 8.0,
                 downscale: float = 0.5):
        os.makedirs(session_dir, exist_ok=True)
        self.dir = session_dir
        self.onboard_source = onboard_source
        self.overhead_source = overhead_source
        self.onboard_fps = onboard_fps
        self.overhead_fps = overhead_fps
        self.downscale = downscale
        self._stop = threading.Event()
        self._threads = []
        self._writers = {}
        self._markers = open(
            os.path.join(session_dir, "sync_markers.jsonl"), "w")
        self.t_start = None

    def _loop(self, name, source, fps, is_overhead):
        period = 1.0 / max(1.0, fps)
        while not self._stop.is_set():
            t0 = time.time()
            try:
                got = source()
            except Exception:
                time.sleep(period)
                continue
            frame = got[0] if isinstance(got, tuple) else got
            if frame is None:
                time.sleep(period)
                continue
            t_wall = time.time()
            if self.downscale and self.downscale != 1.0:
                frame = cv2.resize(
                    frame, (int(frame.shape[1] * self.downscale),
                            int(frame.shape[0] * self.downscale)))
            w = self._writers.get(name)
            if w is None:
                w = _StreamWriter(os.path.join(self.dir, name),
                                  (frame.shape[1], frame.shape[0]),
                                  fps, name)
                self._writers[name] = w
            w.write(frame, t_wall)
            dt = period - (time.time() - t0)
            if dt > 0:
                time.sleep(dt)

    def start(self) -> None:
        self.t_start = time.time()
        self.mark("recording_start")
        if self.onboard_source is not None:
            t = threading.Thread(target=self._loop,
                                 args=("onboard", self.onboard_source,
                                       self.onboard_fps, False),
                                 daemon=True)
            t.start()
            self._threads.append(t)
        if self.overhead_source is not None:
            t = threading.Thread(target=self._loop,
                                 args=("overhead", self.overhead_source,
                                       self.overhead_fps, True),
                                 daemon=True)
            t.start()
            self._threads.append(t)

    def mark(self, label: str, **extra) -> float:
        """Log a synchronization or event marker on the common clock."""
        t = time.time()
        self._markers.write(json.dumps(
            {"t": round(t, 4), "label": label, **extra}) + "\n")
        self._markers.flush()
        return t

    def flash_sync(self, lights: Callable[[float], None],
                   n: int = 2, on_s: float = 0.4,
                   off_s: float = 0.4) -> None:
        """Flash the lights and log the times: an INDEPENDENT check that
        the two video indices share a clock. Only called if the caller
        supplies a lights callback, so this module never actuates on its
        own."""
        for k in range(n):
            lights(1.0)
            self.mark("sync_flash_on", index=k)
            time.sleep(on_s)
            lights(0.0)
            self.mark("sync_flash_off", index=k)
            time.sleep(off_s)

    def stop(self) -> dict:
        self.mark("recording_stop")
        self._stop.set()
        for t in self._threads:
            t.join(timeout=5)
        stats = {"session_dir": self.dir,
                 "t_start": self.t_start,
                 "t_stop": time.time(),
                 "clock": "time.time() system clock, shared by every log "
                          "in this run",
                 "streams": [w.close() for w in self._writers.values()]}
        self._markers.close()
        with open(os.path.join(self.dir, "recording.json"), "w") as f:
            json.dump(stats, f, indent=2)
        return stats


def self_test() -> int:
    """Exercise the writer and the index with synthetic frames only."""
    import tempfile

    import numpy as np
    d = tempfile.mkdtemp(prefix="dualrec_")
    seq = {"i": 0}

    def fake_onboard():
        seq["i"] += 1
        img = np.full((360, 640, 3), seq["i"] % 255, dtype=np.uint8)
        return img, time.time()

    def fake_overhead():
        img = np.full((360, 640, 3), 128, dtype=np.uint8)
        return img

    rec = DualRecorder(d, fake_onboard, fake_overhead,
                       onboard_fps=10, overhead_fps=5, downscale=1.0)
    rec.start()
    rec.mark("test_event", note="mid-recording marker")
    time.sleep(2.0)
    stats = rec.stop()
    ok = True
    for s in stats["streams"]:
        idx = os.path.join(d, s["index"])
        vid = os.path.join(d, s["video"])
        n_idx = sum(1 for _ in open(idx))
        exists = os.path.isfile(vid) and os.path.getsize(vid) > 0
        good = n_idx == s["frames"] and exists and s["frames"] > 3
        ok &= good
        print(f"  [{'ok' if good else 'FAIL'}] {s['stream']}: "
              f"{s['frames']} frames, index lines {n_idx}, "
              f"measured {s['measured_fps']} fps, video {exists}")
    markers = sum(1 for _ in open(os.path.join(d, "sync_markers.jsonl")))
    print(f"  [{'ok' if markers >= 3 else 'FAIL'}] markers: {markers}")
    print("SELF-TEST:", "PASS" if ok and markers >= 3 else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    import sys
    if "--self-test" in sys.argv:
        raise SystemExit(self_test())
    print(__doc__)
