# YOLO anchor fine-tune (prepared, not started)

Plan for the light fine-tuning step focused on the `anchor` class.
No new architecture: start from COCO-pretrained `yolov8n.pt` and fine-tune.

## Why fine-tuning is needed

COCO has no `anchor` class (verdict from the pretrained evaluation in
`docs/yolo_evaluation.md`): a stock model cannot name an anchor and at best
occasionally fires an unrelated class on it. The detector node
(`yolo_obstacle_detector_node`) therefore runs in skeleton mode / mapped-class
mode until fine-tuned weights exist.

## Minimal recipe (when we decide to run it)

1. **Collect frames + labels from the simulation oracle (no manual labeling).**
   Run the visible custom-anchor closed loop while saving camera frames; for
   each frame, write the oracle detection as a YOLO label line
   `0 <center_x> <center_y> <width> <height>` (the oracle already publishes
   exactly these normalized values on `/perception/obstacles_oracle`).
   Vary anchor side/scale/rotation using the `custom_anchor_*.yaml` scenarios
   plus new spawn sites from the external `world_population.json`.
   A few hundred frames with ~10-20% held out for `val` is enough for a
   first pass.

2. **Fine-tune (CPU works for nano; GPU optional):**

   ```bat
   C:\dev\lyrical\.pixi\envs\default\python.exe -m ultralytics ^
     detect train model=yolov8n.pt data=training\yolo_anchor\anchor_dataset.yaml ^
     epochs=30 imgsz=512 device=cpu
   ```

   (equivalently: `yolo detect train model=yolov8n.pt data=... epochs=30 imgsz=512`)

3. **Wire the weights into the ROS 2 node** by setting `model_path` in
   `src/rov_obstacle_perception/config/yolo_detector.yaml` to
   `runs/detect/train/weights/best.pt` and `class_map: "0:anchor"` is not
   needed (the dataset names the class `anchor` directly).

4. **Run the visual pipeline** with the bridge relay disabled so the planner
   consumes YOLO detections instead of the oracle:

   ```bat
   ros2 launch rov_obstacle_sim_bridge holoocean_oracle_avoidance.launch.py &  rem relay on (oracle) — baseline
   ros2 launch rov_obstacle_perception yolo_detector.launch.py               & rem detector; start bridge with relay_oracle_topic:=''
   ```

Everything stays simulation-only.
