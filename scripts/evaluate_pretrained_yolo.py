#!/usr/bin/env python
"""Evaluate a PRETRAINED YOLO model on saved custom-anchor frames.

Answers one question for the report: can stock COCO weights see our anchor
(or anything useful) in the HoloOcean frames, or is a light fine-tune on the
`anchor` class required?

Runs in the pixi ROS env (or any env with `ultralytics` installed)::

    C:\\dev\\lyrical\\.pixi\\envs\\default\\python.exe scripts\\evaluate_pretrained_yolo.py

Outputs:
  - logs/yolo_pretrained_eval.json       (per-image detections + verdict)
  - visualizations/yolo_eval_<name>.png  (annotated copies)

Exit codes: 0 = evaluation ran; 3 = ultralytics not installed.
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_IMAGES = [
    REPO_ROOT / "visualizations" / "custom_anchor_frame0001.png",
    REPO_ROOT / "visualizations" / "custom_anchor_probe_00.png",
    REPO_ROOT / "visualizations" / "custom_anchor_probe_02.png",
]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", default="yolov8n.pt")
    parser.add_argument("--conf", type=float, default=0.10,
                        help="low threshold on purpose: we want to see even "
                             "weak guesses on the anchor")
    parser.add_argument("--images", nargs="*", default=None)
    args = parser.parse_args()

    try:
        from ultralytics import YOLO
    except ImportError as exc:
        print(f"[yolo-eval] ultralytics not installed: {exc}")
        print("[yolo-eval] install with: python -m pip install ultralytics")
        return 3

    images = [Path(p) for p in (args.images or DEFAULT_IMAGES)]
    images = [p for p in images if p.is_file()]
    if not images:
        print("[yolo-eval] no input images found; run the closed loop first")
        return 2

    print(f"[yolo-eval] loading {args.model} (COCO pretrained)")
    model = YOLO(args.model)

    out_dir = REPO_ROOT / "visualizations"
    report = {
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "model": args.model,
        "confidence_threshold": args.conf,
        "images": [],
    }
    any_detection = False
    anchorish = False
    for path in images:
        result = model.predict(source=str(path), conf=args.conf,
                               device="cpu", verbose=False)[0]
        entries = []
        for box in result.boxes:
            cls_id = int(box.cls[0])
            name = str(result.names.get(cls_id, cls_id))
            conf = float(box.conf[0])
            entries.append({"class": name, "confidence": round(conf, 3)})
            any_detection = True
            if name.lower() in ("anchor",):
                anchorish = True
        annotated = out_dir / f"yolo_eval_{path.stem}.png"
        result.save(filename=str(annotated))
        report["images"].append(
            {"image": str(path), "detections": entries, "annotated": str(annotated)}
        )
        print(f"[yolo-eval] {path.name}: "
              f"{entries if entries else 'NO detections'}")

    report["verdict"] = {
        "pretrained_detects_anchor_class": anchorish,
        "pretrained_detects_anything": any_detection,
        "recommendation": (
            "use pretrained as-is" if anchorish else
            "light fine-tuning on the 'anchor' class is required "
            "(COCO has no anchor class); see training/yolo_anchor/README.md"
        ),
    }
    report_path = REPO_ROOT / "logs" / "yolo_pretrained_eval.json"
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(f"[yolo-eval] report -> {report_path}")
    print(f"[yolo-eval] verdict: {report['verdict']['recommendation']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
