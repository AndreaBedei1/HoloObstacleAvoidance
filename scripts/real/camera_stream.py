"""Persistent low-latency reader for the BlueROV2 H.264 UDP stream.

Opening a fresh VideoCapture per frame costs 3-5 s (RTP/H.264 sync), far
too slow for closed-loop control. This keeps ONE capture open and a
reader thread draining it, so `latest()` always returns the newest
decoded frame with no buffer backlog.

The FFMPEG protocol whitelist must be in the environment BEFORE OpenCV
loads; `ensure_env()` re-execs the process once if needed.
"""

from __future__ import annotations

import os
import sys
import threading
import time

_OPTS = "protocol_whitelist;file,rtp,udp|fflags;nobuffer|flags;low_delay"

SDP = """v=0
o=- 0 0 IN IP4 127.0.0.1
s=BlueROV2 UDP Stream 0
c=IN IP4 0.0.0.0
t=0 0
m=video 5600 RTP/AVP 96
a=rtpmap:96 H264/90000
"""


def ensure_env():
    if os.environ.get("OPENCV_FFMPEG_CAPTURE_OPTIONS") != _OPTS:
        os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = _OPTS
        os.execv(sys.executable, [sys.executable] + sys.argv)


class CameraStream:
    def __init__(self, warmup_s: float = 8.0):
        import cv2
        sdp_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "bluerov_5600.sdp")
        with open(sdp_path, "w") as f:
            f.write(SDP)
        self.cap = cv2.VideoCapture(sdp_path, cv2.CAP_FFMPEG)
        if not self.cap.isOpened():
            raise RuntimeError("cannot open UDP 5600 stream")
        self._lock = threading.Lock()
        self._frame = None
        self._t = 0.0
        self._n = 0
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        t0 = time.time()
        while time.time() - t0 < warmup_s and self._frame is None:
            time.sleep(0.1)
        if self._frame is None:
            raise RuntimeError("no frames decoded from UDP 5600")

    def _loop(self):
        while not self._stop.is_set():
            ok, fr = self.cap.read()
            if ok:
                with self._lock:
                    self._frame = fr
                    self._t = time.time()
                    self._n += 1
            else:
                time.sleep(0.01)

    def latest(self):
        with self._lock:
            if self._frame is None:
                return None, 0.0
            return self._frame.copy(), self._t

    @property
    def frames_decoded(self):
        return self._n

    def close(self):
        self._stop.set()
        self._thread.join(timeout=2)
        self.cap.release()
