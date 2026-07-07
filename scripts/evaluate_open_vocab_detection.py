#!/usr/bin/env python
"""Quick open-vocabulary detection check on custom underwater object frames."""

from __future__ import annotations

import argparse
import importlib.util
import json
import time
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_PROMPTS = [
    "anchor",
    "underwater anchor",
    "naval mine",
    "underwater mine",
    "torpedo",
    "underwater torpedo",
]


def default_images() -> list[Path]:
    candidates = [
        REPO_ROOT / "visualizations" / "custom_anchor_frame0001.png",
        REPO_ROOT / "visualizations" / "custom_anchor_frame0200.png",
        REPO_ROOT / "datasets" / "custom_underwater_objects" / "images" / "test" / "test_000000.png",
        REPO_ROOT / "datasets" / "custom_underwater_objects" / "images" / "test" / "test_000001.png",
        REPO_ROOT / "datasets" / "custom_underwater_objects" / "images" / "test" / "test_000002.png",
        REPO_ROOT / "datasets" / "custom_underwater_objects" / "images" / "val" / "val_000000.png",
        REPO_ROOT / "datasets" / "custom_underwater_objects" / "images" / "val" / "val_000001.png",
        REPO_ROOT / "datasets" / "custom_underwater_objects" / "images" / "val" / "val_000002.png",
    ]
    return [p for p in candidates if p.is_file()]


def run_yolo_world(
    images: list[Path],
    prompts: list[str],
    *,
    model_name: str,
    conf: float,
    out_dir: Path,
) -> dict[str, Any]:
    try:
        from ultralytics import YOLOWorld
    except Exception as exc:
        return {
            "available": False,
            "reason": f"ultralytics YOLOWorld unavailable: {exc}",
        }

    try:
        model = YOLOWorld(model_name)
        model.set_classes(prompts)
    except Exception as exc:
        return {
            "available": False,
            "reason": f"failed to load {model_name}: {exc}",
        }

    out_dir.mkdir(parents=True, exist_ok=True)
    results = []
    total = 0
    for image in images:
        result = model.predict(source=str(image), conf=conf, device="cpu", verbose=False)[0]
        detections = []
        names = result.names
        for box in result.boxes:
            cls_id = int(box.cls[0])
            xyxy = [round(float(v), 2) for v in box.xyxy[0].tolist()]
            detections.append(
                {
                    "class": str(names.get(cls_id, cls_id)),
                    "confidence": round(float(box.conf[0]), 4),
                    "xyxy": xyxy,
                }
            )
        total += len(detections)
        annotated = out_dir / f"yoloworld_{image.stem}.jpg"
        result.save(filename=str(annotated))
        results.append(
            {
                "image": str(image),
                "detections": detections,
                "annotated": str(annotated),
            }
        )
    return {
        "available": True,
        "model": model_name,
        "confidence_threshold": conf,
        "prompts": prompts,
        "total_detections": total,
        "images": results,
    }


def grounding_dino_status() -> dict[str, Any]:
    if importlib.util.find_spec("groundingdino") is None:
        return {
            "available": False,
            "reason": (
                "groundingdino is not installed in the local Python environment; "
                "skipped to avoid a heavy/unstable install during the quick check"
            ),
        }
    return {
        "available": False,
        "reason": (
            "groundingdino package is present, but this repository has no pinned "
            "Grounding DINO weights/config; skipped rather than introducing an "
            "untracked heavyweight dependency path"
        ),
    }


def verdict(yolo_report: dict[str, Any]) -> str:
    if not yolo_report.get("available"):
        return "not usable: YOLO-World did not run locally"
    detections = int(yolo_report.get("total_detections", 0))
    if detections <= 0:
        return "not usable zero-shot on this quick sample"
    threshold = float(yolo_report.get("confidence_threshold", 1.0))
    if detections <= 1 and threshold < 0.05:
        return (
            "not usable zero-shot; only one weak very-low-confidence detection "
            "was found, so it is not reliable for pseudo-labeling"
        )
    return "usable only for rough pseudo-labeling; verify manually before trusting labels"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", default="yolov8s-world.pt")
    parser.add_argument("--conf", type=float, default=0.10)
    parser.add_argument("--out-dir", type=Path, default=REPO_ROOT / "visualizations" / "open_vocab_eval")
    parser.add_argument("--report-json", type=Path, default=REPO_ROOT / "logs" / "open_vocab_detection_eval.json")
    parser.add_argument("--images", nargs="*", default=None)
    parser.add_argument("--prompts", nargs="*", default=DEFAULT_PROMPTS)
    args = parser.parse_args()

    images = [Path(p) for p in args.images] if args.images else default_images()
    images = [p for p in images if p.is_file()]
    if not images:
        print("[open-vocab] no images found; generate dataset first")
        return 2

    yolo = run_yolo_world(
        images,
        list(args.prompts),
        model_name=args.model,
        conf=args.conf,
        out_dir=args.out_dir,
    )
    report = {
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "images": [str(p) for p in images],
        "yolo_world": yolo,
        "grounding_dino": grounding_dino_status(),
        "verdict": verdict(yolo),
    }
    args.report_json.parent.mkdir(parents=True, exist_ok=True)
    args.report_json.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
