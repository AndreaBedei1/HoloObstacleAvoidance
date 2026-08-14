"""Grab frames from the BlueROV2 onboard camera (H.264 RTP on UDP 5600).

BlueOS 'UDP Stream 0' pushes RTP/H.264 to this PC (192.168.2.1:5600).
OpenCV's FFMPEG backend decodes it through a local SDP description —
no GStreamer needed on Windows.

Usage:
    python scripts/real/camera_grab.py [--frames N] [--out DIR]
                                       [--video seconds]
"""

import argparse
import os
import sys
import time

# The FFMPEG whitelist must be in the environment BEFORE the OpenCV DLL
# loads (setting it inside an already-running interpreter is ignored):
# re-exec ourselves once with it set.
_OPTS = "protocol_whitelist;file,rtp,udp|fflags;nobuffer|flags;low_delay"
if os.environ.get("OPENCV_FFMPEG_CAPTURE_OPTIONS") != _OPTS:
    os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = _OPTS
    os.execv(sys.executable, [sys.executable] + sys.argv)

SDP = """v=0
o=- 0 0 IN IP4 127.0.0.1
s=BlueROV2 UDP Stream 0
c=IN IP4 0.0.0.0
t=0 0
m=video 5600 RTP/AVP 96
a=rtpmap:96 H264/90000
"""


def open_capture():
    import cv2
    sdp_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            "bluerov_5600.sdp")
    with open(sdp_path, "w") as f:
        f.write(SDP)
    cap = cv2.VideoCapture(sdp_path, cv2.CAP_FFMPEG)
    return cap


def main() -> int:
    import cv2
    ap = argparse.ArgumentParser()
    ap.add_argument("--frames", type=int, default=3)
    ap.add_argument("--video", type=float, default=0.0,
                    help="record N seconds of video instead")
    ap.add_argument("--out", default="visualizations/real_rov_camera")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    cap = open_capture()
    if not cap.isOpened():
        print("FAILED to open UDP 5600 stream (is the vehicle camera on?)")
        return 1

    t0 = time.time()
    n = 0
    fps_probe = []
    writer = None
    last = time.time()
    while True:
        ok, frame = cap.read()
        now = time.time()
        if not ok:
            if now - t0 > 15:
                print("no frames after 15 s")
                return 1
            continue
        fps_probe.append(now - last)
        last = now
        n += 1
        if args.video > 0:
            if writer is None:
                h, w = frame.shape[:2]
                writer = cv2.VideoWriter(
                    os.path.join(args.out, time.strftime(
                        "rov_%Y%m%d_%H%M%S.mp4")),
                    cv2.VideoWriter_fourcc(*"mp4v"), 25, (w, h))
                print(f"recording {w}x{h}...")
            writer.write(frame)
            if now - t0 >= args.video:
                break
        else:
            path = os.path.join(args.out, time.strftime(
                f"frame_%Y%m%d_%H%M%S_{n}.png"))
            cv2.imwrite(path, frame)
            print("saved", path, frame.shape)
            if n >= args.frames:
                break
    if writer is not None:
        writer.release()
    cap.release()
    if len(fps_probe) > 5:
        import statistics
        med = statistics.median(fps_probe[5:])
        print(f"measured frame interval median {med*1000:.1f} ms "
              f"(~{1.0/med:.1f} fps), frames {n}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
