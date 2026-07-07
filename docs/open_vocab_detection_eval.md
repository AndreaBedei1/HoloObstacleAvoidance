# Open-Vocabulary Detection Quick Check

Inputs:

- saved custom-anchor simulation frames;
- generated custom-object dataset samples from `datasets/custom_underwater_objects`.

Prompts:

```text
anchor
underwater anchor
naval mine
underwater mine
torpedo
underwater torpedo
```

## YOLO-World

Command:

```bat
python scripts\evaluate_open_vocab_detection.py --model yolov8s-world.pt --conf 0.01
```

Artifacts:

- report: `logs/open_vocab_detection_eval.json`
- annotated images: `visualizations/open_vocab_eval/`

Result:

- `yolov8s-world.pt` loaded successfully after Ultralytics auto-installed its
  CLIP dependency.
- At `conf=0.10`: zero detections.
- At `conf=0.01`: one weak detection on `custom_anchor_frame0001.png`
  (`underwater anchor`, confidence `0.011`).
- Generated anchor/mine/torpedo samples had no useful detections.

Verdict: **not usable zero-shot** for this simulation dataset. The single weak
detection is not reliable enough for pseudo-labeling without manual review.

## Grounding DINO

Skipped. `groundingdino` is not installed locally, and installing/pinning the
package plus weights would be too heavy for the intended quick check.

## Decision

Proceed with supervised YOLO fine-tuning from pretrained YOLO weights using
the generated labels.
