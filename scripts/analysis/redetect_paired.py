"""Re-run the CURRENT anchor detector over the saved paired frames.

The paired dataset was recorded in morning haze, when the detector used
absolute grey-level thresholds and returned zero detections in all 44
records that carry ground truth. Those thresholds were later made
contrast-relative. Because every frame was written to disk, the dataset
does not have to be re-collected: the corrected detector is replayed
over the stored images and paired with the ground truth that was
recorded at the same timestamp.

This is only legitimate because the ground truth is INDEPENDENT of the
detector — it comes from the overhead camera — so replaying the detector
cannot contaminate it. The output is written to a NEW file; the original
recording is never modified.
"""

from __future__ import annotations

import glob
import json
import os
import sys

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "real"))
_ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..")

from camera_stream import ensure_env  # noqa: E402

ensure_env()

import cv2  # noqa: E402

from anchor_detect import detect_anchor  # noqa: E402

DATA = os.path.join(_ROOT, "experiments", "real", "paired_dataset")


def main() -> int:
    total = redetected = found_now = found_before = 0
    for path in sorted(glob.glob(os.path.join(DATA, "*",
                                              "position_*.json"))):
        with open(path) as f:
            d = json.load(f)
        base = os.path.dirname(path)
        for r in d["records"]:
            total += 1
            if r.get("det", {}).get("found"):
                found_before += 1
            ff = r.get("frame_file")
            if not ff:
                continue
            img_path = os.path.join(base, ff.replace("\\", "/"))
            if not os.path.isfile(img_path):
                continue
            img = cv2.imread(img_path)
            if img is None:
                continue
            det = detect_anchor(img)
            redetected += 1
            r["det_replay"] = {
                "found": bool(det.get("found")),
                "score": det.get("score"),
                "center_x": det.get("center_x"),
                "center_y": det.get("center_y"),
                "width": det.get("width"),
                "height": det.get("height"),
                "bbox_px": det.get("bbox_px"),
                "mean_dark": det.get("mean_dark"),
            }
            if det.get("found"):
                found_now += 1
        d.setdefault("meta", {})["replay"] = {
            "detector": "scripts/real/anchor_detect.detect_anchor",
            "why": "original recording made in morning haze with absolute "
                   "thresholds; detector later made contrast-relative",
            "ground_truth_untouched": True,
        }
        out = path.replace("position_", "position_replay_")
        with open(out, "w") as f:
            json.dump(d, f, indent=2)
    print("record %d | fotogrammi rieseguiti %d" % (total, redetected))
    print("rilevamenti prima %d -> ora %d" % (found_before, found_now))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
