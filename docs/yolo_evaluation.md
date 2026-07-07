# Pretrained YOLO evaluation on custom-anchor frames

Date: 2026-07-06 — `scripts/evaluate_pretrained_yolo.py`, model `yolov8n.pt`
(COCO pretrained, ultralytics 8.4.90, torch 2.12.1 CPU, pixi ROS env).
Inputs: color-corrected frames from the visible custom-anchor simulation.
Raw results: `logs/yolo_pretrained_eval.json`; annotated copies:
`visualizations/yolo_eval_*.png`.

| Image | Anchor in view | Pretrained result (conf ≥ 0.10) |
| --- | --- | --- |
| `custom_anchor_frame0001.png` | yes, ~3 m tall at 12 m, centered | **no detections at all** |
| `custom_anchor_probe_00.png` | yes, huge (~9 m) at 8 m | `airplane` 0.63 (misclassification) |
| `custom_anchor_probe_02.png` | yes, huge (~9 m) at 8 m | `airplane` 0.59 (misclassification) |

## Verdict

- COCO simply has **no `anchor` class**; the model cannot name it.
- At operational distances (the avoidance-relevant case) the pretrained model
  sees **nothing** — not even a wrong class — so a `class_map` remap (e.g.
  `airplane:anchor`) is NOT a viable stopgap: it only fires when the anchor
  already fills the frame.
- **Recommendation: light fine-tuning of `yolov8n.pt` focused on the single
  `anchor` class**, using oracle-labeled simulation frames (zero manual
  annotation — the oracle already publishes normalized YOLO-style boxes).
  Plan and commands: `training/yolo_anchor/README.md`.

## Pipeline readiness

`yolo_obstacle_detector_node` (rov_obstacle_perception) already subscribes to
`/camera/front/image_raw` and publishes `/perception/obstacles`; point its
`model_path` at the fine-tuned weights and start the bridge with
`relay_oracle_topic:=''` to switch the planner from oracle to visual
detections. The planner itself needs no change.
