# Dual live sonar diagnostics

The Windows app in `scripts/real/sonar_viewer.py` shows both real sonars in a
single window:

- **Left — Cerulean Omniscan 450 SS**: TCP `192.168.2.86:51200`, one acoustic
  1-D profile per ping, rendered as a time-stacked waterfall plus the latest
  intensity-vs-range profile.
- **Right — Blue Robotics Ping1D**: official `brping.Ping1D` through BlueOS
  PingProxy UDP `192.168.2.2:9090`, with distance, confidence and the optional
  full echo profile.

The app uses `bluerobotics-ping` for both devices. Connecting does not start
Omniscan pinging. The Omniscan is started only by **Start Omniscan** and is
stopped with `enable=0` before disconnect/close if the app started it.

## Install and run on Windows

From the repository root:

```bat
py -m pip install -r requirements_sonar.txt
python scripts\real\test_sonar_viewer_offline.py
python scripts\test_real_sonar_connections.py
scripts\real\start_sonar_viewer.bat
```

Close SonarView and PingViewer before the first real test. The connection
diagnostic does not start Omniscan; it only opens/closes its TCP socket and
initializes Ping1D through PingProxy.

The diagnostic recorder writes JSONL files below
`records\sonar\<timestamp>\sonar_frames.jsonl`, containing timestamps,
Ping1D distance/confidence/profile and Omniscan profiles. Screenshots are
saved separately with **Save screenshot**.

The implementation follows the official [Omniscan 450 Ping-Python guide](https://docs.ceruleansonar.com/c/omniscan-450/getting-started-with-ping-python)
and the official [Blue Robotics ping-python API](https://github.com/bluerobotics/ping-python).
