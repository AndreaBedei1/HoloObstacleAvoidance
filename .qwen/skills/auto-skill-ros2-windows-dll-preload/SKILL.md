---
name: ros2-windows-dll-preload
description: Preload spdlog.dll via ctypes before importing rclpy to fix LoadLibrary error 126 on Windows with pixi-installed ROS 2 Lyrical
source: auto-skill
extracted_at: '2026-06-26T14:04:40.779Z'
---

# ROS 2 Python Node DLL Preloading on Windows (pixi / ROS 2 Lyrical)

## Problem

On Windows with ROS 2 installed via pixi, running any `rclpy` node fails with:

```
RCLError: failed to load shared library 'rcl_logging_spdlog.dll' due to LoadLibrary error: 126
```

**Root cause:** `spdlog.dll` lives at `<pixi-env>\Library\bin\spdlog.dll`, but that directory is **not** on `PATH` after sourcing `setup.bat`. The `rcl_logging_spdlog.dll` loader can't find its dependency.

A secondary issue: system Python 3.14 won't work — ROS 2 Lyrical binaries are compiled for cp312 (Python 3.12). Using the wrong Python version produces:

```
ModuleNotFoundError: No module named 'rclpy._rclpy_pybind11'
```

## Solution

### 1. Use pixi's Python 3.12, not system Python

Always invoke nodes with the pixi environment Python:

```batch
set PYTHON_EXE=C:\dev\lyrical\.pixi\envs\default\python.exe
```

Verify with `%PYTHON_EXE% --version` → should report `Python 3.12.x`.

### 2. Preload spdlog.dll via ctypes before importing rclpy

Add this **before any `import rclpy`**:

```python
import ctypes
import os

spdlog_path = r"C:\dev\lyrical\.pixi\envs\default\Library\bin\spdlog.dll"
if os.path.exists(spdlog_path):
    ctypes.CDLL(spdlog_path)  # loads into process, satisfies rcl_logging_spdlog.dll dependency
```

### 3. Auto-detect spdlog.dll relative to sys.executable (portable pattern)

For scripts that may run from different environments:

```python
import os
import sys

def find_spdlog_dll():
    python_dir = os.path.dirname(sys.executable)
    library_bin = os.path.join(python_dir, "..", "Library", "bin", "spdlog.dll")
    if os.path.exists(library_bin):
        return library_bin
    return None

spdlog_path = find_spdlog_dll()
if spdlog_path:
    ctypes.CDLL(spdlog_path)
```

### 4. rclpy.init() is synchronous in Lyrical (not async)

`rclpy.init()` returns an `InitContextManager` context manager — **do not** use `await`:

```python
# Correct:
rclpy.init()
# ... node work ...
rclpy.shutdown()

# Wrong — raises TypeError: object InitContextManager can't be used in 'await' expression:
await rclpy.init()
```

### 5. Zenoh router must be running

ROS 2 Lyrical defaults to the Zenoh RMW. Any node will silently fail (exit code 1) if no Zenoh router is available. Start one before launching nodes:

```batch
start "Zenoh Router" /B ros2 run rmw_zenoh_cpp rmw_zenohd
timeout /t 3 /nobreak >nul  REM wait for router to initialize
```

## How to apply

Use this pattern whenever:
- Running ROS 2 Python nodes on Windows with pixi-installed Lyrical
- Creating standalone validation scripts that import `rclpy` outside of `ros2 run`
- Writing batch test runners that launch multiple ROS 2 nodes
- Debugging `LoadLibrary error: 126` or `STATUS_STACK_OVERFLOW` (exit code 3221225477) crashes

## Common pitfalls

| Symptom | Cause | Fix |
|---|---|---|
| `LoadLibrary error: 126` for `rcl_logging_spdlog.dll` | `spdlog.dll` not in PATH | Preload with `ctypes.CDLL()` |
| `ModuleNotFoundError: rclpy._rclpy_pybind11` | Wrong Python version (3.14 vs 3.12) | Use pixi Python |
| Nodes crash silently, exit code 1 | No Zenoh router running | Start `rmw_zenohd` first |
| Exit code 3221225477 (stack overflow) | Node receives YAML parameters with no matching section | Remove unused parameters from launch file |
| Launch file edits not taking effect | colcon caches launch files in `install/` | Rebuild or copy source launch file to `install/` |
