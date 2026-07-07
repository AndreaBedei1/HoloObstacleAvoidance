# YOLO Custom Objects Results

Status: first supervised simulation-only detector trained and evaluated.

## Training

Weights:

```text
training/yolo_custom_objects/runs/yolov8n_custom_underwater/weights/best.pt
```

Training config:

```text
training/yolo_custom_objects/train_config.yaml
training/yolo_custom_objects/dataset.yaml
training/yolo_custom_objects/README.md
```

Command used:

```bat
python -c "from pathlib import Path; from ultralytics import YOLO; root=Path.cwd(); model=YOLO('yolov8n.pt'); model.train(data=str(root/'training/yolo_custom_objects/dataset.yaml'), epochs=15, imgsz=512, batch=8, device='cpu', workers=0, project=str(root/'training/yolo_custom_objects/runs'), name='yolov8n_custom_underwater', exist_ok=True)"
```

Environment:

- Ultralytics: `8.4.23`
- Torch: `2.10.0+cpu`
- CUDA: unavailable
- Model: pretrained `yolov8n.pt`, not trained from scratch

## Test Metrics

Command:

```bat
python -c "from pathlib import Path; from ultralytics import YOLO; root=Path.cwd(); model=YOLO(str(root/'training/yolo_custom_objects/runs/yolov8n_custom_underwater/weights/best.pt')); model.val(data=str(root/'training/yolo_custom_objects/dataset.yaml'), split='test', imgsz=512, device='cpu', workers=0, project=str(root/'training/yolo_custom_objects/runs'), name='yolov8n_custom_underwater_test', exist_ok=True)"
```

Metrics source:

```text
logs/yolo_custom_objects_test_metrics.json
```

| Class | Precision | Recall | mAP50 | mAP50-95 |
|---|---:|---:|---:|---:|
| all | 0.976 | 0.952 | 0.964 | 0.862 |
| anchor | 0.969 | 0.926 | 0.940 | 0.805 |
| mine | 0.999 | 0.980 | 0.988 | 0.926 |
| torpedo | 0.959 | 0.950 | 0.963 | 0.854 |

CPU inference speed on test validation: about `16.2 ms/image`.

## Qualitative Outputs

Annotated outputs:

```text
visualizations/yolo_custom_objects_eval/named_qualitative/
visualizations/yolo_custom_objects_eval/qualitative/
```

Summary JSON:

```text
logs/yolo_custom_objects_qualitative.json
```

Observed examples:

- `custom_anchor_frame0001.png`: detected `anchor` at confidence `0.951`.
- `custom_anchor_frame0200.png`: no detection at confidence threshold `0.25`.
- Test split samples include successful anchor, mine, and torpedo detections.

## ROS 2 Integration

Updated:

```text
src/rov_obstacle_perception/config/yolo_detector.yaml
src/rov_obstacle_perception/launch/yolo_detector.launch.py
src/rov_obstacle_sim_bridge/launch/holoocean_yolo_avoidance.launch.py
scripts/run_custom_anchor_closed_loop.py
scripts/run_custom_anchor_closed_loop.bat
```

The detector subscribes to:

```text
/camera/front/image_raw
```

The detector publishes:

```text
/perception/obstacles
```

The YOLO closed-loop launch disables the oracle relay:

```text
"relay_oracle_topic": ""
```

The planner remains unchanged and continues to consume `/perception/obstacles`.

## Limitations

- Dataset is intentionally single-object, suspended, and non-occluded after visual QA showed invalid labels for occlusion and seabed intersections.
- Generalization to seabed-touching objects, multi-object occlusion, and clutter is not covered yet.
- The closed-loop frame `custom_anchor_frame0200.png` had no YOLO detection at `0.25`, so temporal stability still needs tuning.
