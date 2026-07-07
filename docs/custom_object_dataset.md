# Custom Underwater Object Dataset

Status: full clean dataset generated and inspected. Earlier generated datasets
were deleted after visual QA showed low rover height and incorrect boxes.

## Current Generation Rules

The generator uses only the custom Unreal assets from
`config/custom_holoocean_engine.yaml`:

- `anchor` -> `/Game/ancora.ancora`
- `mine` -> `/Game/mina.mina`
- `torpedo` -> `/Game/siluro.siluro`

Current constraints:

- one object per image;
- object suspended in the water column;
- deep background sites only (`min_site_depth_m=65`);
- camera roughly 20-30 m above the seabed;
- object bottom at least 12 m above the seabed;
- no edge/cut-off boxes;
- labels extracted from rendered pixels, not just configured oracle bounds.
- rendered foreground components smaller than 120 px are rejected.

The label path is now:

```text
background frame without object
  -> spawn one custom mesh
  -> foreground frame with object
  -> foreground/background image difference
  -> connected component box
  -> YOLO label
```

The geometric oracle projection is used only as a broad candidate ROI for the
difference mask. This fixes the torpedo case where configured bounds covered
only part of the rendered mesh.

## Generation Command

```bat
C:\Users\andrea.bedei3\.conda\envs\ocean\python.exe scripts\generate_custom_object_dataset.py ^
  --output-dir datasets\custom_underwater_objects ^
  --train-count 1500 ^
  --val-count 300 ^
  --test-count 300 ^
  --overwrite ^
  --progress-every 50 ^
  --max-attempts-per-sample 260
```

Inspection command:

```bat
python scripts\inspect_custom_object_dataset.py ^
  --dataset-root datasets\custom_underwater_objects ^
  --preview-dir visualizations\dataset_preview ^
  --samples 60
```

## Dataset Result

- Train: 1500 images, 1500 objects.
- Val: 300 images, 300 objects.
- Test: 300 images, 300 objects.
- Class distribution: `anchor=700`, `mine=700`, `torpedo=700`.
- Camera clearance above seabed: min `20.03 m`, median `25.03 m`, max `30.00 m`.
- Object center clearance above seabed: min `22.22 m`, median `31.02 m`, max `44.30 m`.
- Haze alpha: min `0.00`, median `0.225`, max `0.449`.
- Label source: `rendered_background_difference`.
- Empty label files: 0.
- Invalid label files: 0.
- Edge objects: 0.
- Box width min/median/max: `0.041 / 0.113 / 0.607`.
- Box height min/median/max: `0.043 / 0.111 / 0.650`.
- Box area min/median/max: `0.002987 / 0.012806 / 0.252789`.

Preview overlays are in `visualizations/dataset_preview/`.

## Known Limitations

- Multi-object scenes and occlusions are intentionally disabled for now.
- Objects are suspended and kept clearly above the seabed.
- The dataset does not yet cover seabed-touching cases, hidden objects, or
  heavily cluttered scenes.
- This is a first detector dataset, not a photorealistic domain-randomization
  pass.
