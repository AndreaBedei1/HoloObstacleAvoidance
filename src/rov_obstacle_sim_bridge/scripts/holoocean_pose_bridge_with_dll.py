# -*- coding: utf-8 -*-
"""
Preload required DLLs before importing rclpy to fix Windows DLL dependency issues.
This allows ROS 2 nodes to run without the "LoadLibrary error: 126" for spdlog.
"""
import ctypes
import os
import sys

# Find the pixi environment and preload spdlog.dll
def find_spdlog_dll():
    """Find spdlog.dll in the ROS 2 installation."""
    # Common locations for ROS 2 Lyrical on Windows
    candidates = [
        r"C:\dev\lyrical\.pixi\envs\default\Library\bin\spdlog.dll",
        os.path.join(os.environ.get("ROS_HOME", "~"), ".pixi", "envs", "default", "Library", "bin", "spdlog.dll"),
    ]
    
    for path in candidates:
        if os.path.exists(path):
            return path
    
    # Try to find it relative to the Python executable
    python_dir = os.path.dirname(sys.executable)
    library_bin = os.path.join(python_dir, "..", "Library", "bin", "spdlog.dll")
    if os.path.exists(library_bin):
        return library_bin
    
    return None

# Preload spdlog if found
spdlog_path = find_spdlog_dll()
if spdlog_path:
    try:
        ctypes.CDLL(spdlog_path)
        print(f"[dll_preload] Loaded {spdlog_path}", file=sys.stderr)
    except OSError as e:
        print(f"[dll_preload] Warning: Failed to load {spdlog_path}: {e}", file=sys.stderr)
else:
    print("[dll_preload] Warning: spdlog.dll not found, rclpy may fail to initialize", file=sys.stderr)

# Now import rclpy and run the node
if __name__ == "__main__":
    # Import after DLL preloading
    import rclpy
    from rov_obstacle_sim_bridge.holoocean_pose_bridge_node import main
    
    print("[dll_preload] rclpy imported successfully", file=sys.stderr)
    main()
