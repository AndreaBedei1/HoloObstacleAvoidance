# YOLO Custom Underwater Objects

Dataset:

```text
datasets/custom_underwater_objects
```

Classes:

```text
0 anchor
1 mine
2 torpedo
```

Training uses pretrained `yolov8n.pt`; this is not training from scratch.

## Environment

Validated local environment:

- Python: system Python with `ultralytics 8.4.23`
- Torch: `2.10.0+cpu`
- CUDA: unavailable, so training runs on CPU

## Train

```bat
python -c "from pathlib import Path; from ultralytics import YOLO; root=Path.cwd(); model=YOLO('yolov8n.pt'); model.train(data=str(root/'training/yolo_custom_objects/dataset.yaml'), epochs=15, imgsz=512, batch=8, device='cpu', workers=0, project=str(root/'training/yolo_custom_objects/runs'), name='yolov8n_custom_underwater', exist_ok=True)"
```

## Evaluate

```bat
python -c "from pathlib import Path; from ultralytics import YOLO; root=Path.cwd(); model=YOLO(str(root/'training/yolo_custom_objects/runs/yolov8n_custom_underwater/weights/best.pt')); model.val(data=str(root/'training/yolo_custom_objects/dataset.yaml'), split='test', imgsz=512, device='cpu', workers=0, project=str(root/'training/yolo_custom_objects/runs'), name='yolov8n_custom_underwater_test', exist_ok=True)"
```

Best weights path after training:

```text
training/yolo_custom_objects/runs/yolov8n_custom_underwater/weights/best.pt
```

The local Ultralytics install does not expose `ultralytics.__main__`, so use
the Python API commands above instead of `python -m ultralytics`. Absolute
project paths are used intentionally; relative `project=` paths were nested
under `runs/detect/` by this Ultralytics version.
