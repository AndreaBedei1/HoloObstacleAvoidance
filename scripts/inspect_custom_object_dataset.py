#!/usr/bin/env python
"""Inspect a generated custom underwater object YOLO dataset."""

from __future__ import annotations

import argparse
import json
import random
import statistics
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any

import yaml
from PIL import Image, ImageDraw, ImageFont


REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_DATASET = REPO_ROOT / "datasets" / "custom_underwater_objects"
DEFAULT_PREVIEW_DIR = REPO_ROOT / "visualizations" / "dataset_preview"


COLORS = {
    0: (240, 80, 60),
    1: (80, 210, 120),
    2: (70, 150, 245),
}


def load_dataset_yaml(dataset_root: Path) -> dict[str, Any]:
    path = dataset_root / "dataset.yaml"
    if not path.is_file():
        raise FileNotFoundError(f"dataset.yaml not found: {path}")
    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    names = data.get("names", {})
    if isinstance(names, list):
        names = {idx: name for idx, name in enumerate(names)}
    else:
        names = {int(k): str(v) for k, v in names.items()}
    data["names"] = names
    return data


def parse_label_file(path: Path) -> tuple[list[dict[str, Any]], list[str]]:
    labels = []
    problems = []
    if not path.is_file():
        return labels, [f"missing label file: {path}"]
    text = path.read_text(encoding="utf-8").strip()
    if not text:
        return labels, ["empty label file"]
    for line_no, line in enumerate(text.splitlines(), start=1):
        parts = line.split()
        if len(parts) != 5:
            problems.append(f"line {line_no}: expected 5 fields, got {len(parts)}")
            continue
        try:
            cls = int(parts[0])
            cx, cy, w, h = (float(v) for v in parts[1:])
        except ValueError as exc:
            problems.append(f"line {line_no}: non-numeric value ({exc})")
            continue
        invalid = []
        if cls not in (0, 1, 2):
            invalid.append("class_id")
        if not (0.0 <= cx <= 1.0):
            invalid.append("center_x")
        if not (0.0 <= cy <= 1.0):
            invalid.append("center_y")
        if not (0.0 < w <= 1.0):
            invalid.append("width")
        if not (0.0 < h <= 1.0):
            invalid.append("height")
        if cx - w / 2 < -1e-6 or cx + w / 2 > 1.0 + 1e-6:
            invalid.append("x_extent")
        if cy - h / 2 < -1e-6 or cy + h / 2 > 1.0 + 1e-6:
            invalid.append("y_extent")
        if invalid:
            problems.append(f"line {line_no}: invalid {','.join(invalid)}")
            continue
        labels.append({"class_id": cls, "center_x": cx, "center_y": cy, "width": w, "height": h})
    return labels, problems


def summarize(values: list[float]) -> dict[str, float]:
    if not values:
        return {}
    ordered = sorted(values)
    n = len(ordered)

    def q(p: float) -> float:
        idx = min(n - 1, max(0, round((n - 1) * p)))
        return round(float(ordered[idx]), 6)

    return {
        "min": round(float(min(ordered)), 6),
        "p25": q(0.25),
        "median": round(float(statistics.median(ordered)), 6),
        "p75": q(0.75),
        "max": round(float(max(ordered)), 6),
    }


def draw_preview(
    image_path: Path,
    labels: list[dict[str, Any]],
    names: dict[int, str],
    out_path: Path,
) -> None:
    image = Image.open(image_path).convert("RGB")
    draw = ImageDraw.Draw(image)
    width, height = image.size
    try:
        font = ImageFont.truetype("arial.ttf", 13)
    except Exception:
        font = ImageFont.load_default()
    for label in labels:
        cls = int(label["class_id"])
        cx = float(label["center_x"]) * width
        cy = float(label["center_y"]) * height
        bw = float(label["width"]) * width
        bh = float(label["height"]) * height
        x0 = max(0.0, cx - bw / 2)
        y0 = max(0.0, cy - bh / 2)
        x1 = min(float(width - 1), cx + bw / 2)
        y1 = min(float(height - 1), cy + bh / 2)
        color = COLORS.get(cls, (255, 255, 255))
        draw.rectangle([x0, y0, x1, y1], outline=color, width=3)
        text = names.get(cls, str(cls))
        text_box = draw.textbbox((x0, y0), text, font=font)
        tw = text_box[2] - text_box[0]
        th = text_box[3] - text_box[1]
        draw.rectangle([x0, max(0, y0 - th - 4), x0 + tw + 6, y0], fill=color)
        draw.text((x0 + 3, max(0, y0 - th - 3)), text, fill=(0, 0, 0), font=font)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    image.save(out_path)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-root", type=Path, default=DEFAULT_DATASET)
    parser.add_argument("--preview-dir", type=Path, default=DEFAULT_PREVIEW_DIR)
    parser.add_argument("--samples", type=int, default=36)
    parser.add_argument("--seed", type=int, default=17)
    parser.add_argument("--edge-threshold", type=float, default=0.03)
    parser.add_argument("--report-json", type=Path, default=None)
    args = parser.parse_args()

    dataset_root = args.dataset_root.resolve()
    data = load_dataset_yaml(dataset_root)
    names = data["names"]
    rng = random.Random(args.seed)

    records = []
    class_counts = Counter()
    image_class_counts: dict[int, int] = Counter()
    split_image_counts = Counter()
    split_object_counts = Counter()
    widths = []
    heights = []
    areas = []
    edge_objects = 0
    empty_labels = []
    invalid_files = {}

    for split in ("train", "val", "test"):
        image_dir = dataset_root / "images" / split
        label_dir = dataset_root / "labels" / split
        for image_path in sorted(image_dir.glob("*.png")):
            label_path = label_dir / f"{image_path.stem}.txt"
            labels, problems = parse_label_file(label_path)
            split_image_counts[split] += 1
            if problems:
                invalid_files[str(label_path.relative_to(dataset_root))] = problems
            if not labels:
                empty_labels.append(str(label_path.relative_to(dataset_root)))
            present = set()
            for label in labels:
                cls = int(label["class_id"])
                class_counts[cls] += 1
                split_object_counts[split] += 1
                present.add(cls)
                widths.append(float(label["width"]))
                heights.append(float(label["height"]))
                areas.append(float(label["width"]) * float(label["height"]))
                x0 = float(label["center_x"]) - float(label["width"]) / 2.0
                x1 = float(label["center_x"]) + float(label["width"]) / 2.0
                y0 = float(label["center_y"]) - float(label["height"]) / 2.0
                y1 = float(label["center_y"]) + float(label["height"]) / 2.0
                if (
                    x0 <= args.edge_threshold
                    or y0 <= args.edge_threshold
                    or x1 >= 1.0 - args.edge_threshold
                    or y1 >= 1.0 - args.edge_threshold
                ):
                    edge_objects += 1
            for cls in present:
                image_class_counts[cls] += 1
            records.append((image_path, labels))

    args.preview_dir.mkdir(parents=True, exist_ok=True)
    selected = rng.sample(records, min(args.samples, len(records))) if records else []
    previews = []
    for idx, (image_path, labels) in enumerate(selected):
        out_path = args.preview_dir / f"preview_{idx:03d}_{image_path.stem}.jpg"
        draw_preview(image_path, labels, names, out_path)
        previews.append(str(out_path))

    report = {
        "dataset_root": str(dataset_root),
        "splits": {
            split: {
                "images": int(split_image_counts[split]),
                "objects": int(split_object_counts[split]),
            }
            for split in ("train", "val", "test")
        },
        "class_counts": {names.get(cls, str(cls)): int(count) for cls, count in sorted(class_counts.items())},
        "images_with_class": {
            names.get(cls, str(cls)): int(count) for cls, count in sorted(image_class_counts.items())
        },
        "box_distribution": {
            "width": summarize(widths),
            "height": summarize(heights),
            "area": summarize(areas),
        },
        "edge_objects": int(edge_objects),
        "edge_object_fraction": round(edge_objects / max(1, sum(class_counts.values())), 6),
        "empty_label_files": empty_labels,
        "invalid_label_files": invalid_files,
        "preview_files": previews,
    }

    report_json = args.report_json or (args.preview_dir / "inspection_report.json")
    report_json.parent.mkdir(parents=True, exist_ok=True)
    report_json.write_text(json.dumps(report, indent=2, sort_keys=True), encoding="utf-8")

    print(json.dumps(report, indent=2, sort_keys=True))
    if empty_labels or invalid_files:
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
