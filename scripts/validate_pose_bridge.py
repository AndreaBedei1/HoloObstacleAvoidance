# -*- coding: utf-8 -*-
"""
Simple validation script for HoloOcean smoke bridge.
This script validates that the pose bridge publishes messages correctly.
"""
import ctypes
import os
import sys
import time

# Preload spdlog.dll before importing rclpy
spdlog_path = r"C:\dev\lyrical\.pixi\envs\default\Library\bin\spdlog.dll"
if os.path.exists(spdlog_path):
    try:
        ctypes.CDLL(spdlog_path)
        print(f"[validate] Loaded {spdlog_path}")
    except OSError as e:
        print(f"[validate] Warning: Failed to load {spdlog_path}: {e}")

import rclpy
from geometry_msgs.msg import PoseStamped


def test_pose_bridge():
    """Test that the pose bridge publishes messages."""
    rclpy.init()
    
    received_count = 0
    last_pose = None
    
    def pose_callback(msg):
        nonlocal received_count, last_pose
        received_count += 1
        last_pose = msg
        print(f"[validate] Received pose #{received_count}: position=({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}, {msg.pose.position.z:.2f})")
    
    # Create subscriber
    node = rclpy.create_node('pose_bridge_validator')
    subscriber = node.create_subscription(
        PoseStamped,
        '/sim/rov_pose',
        pose_callback,
        10
    )
    
    print("[validate] Subscribed to /sim/rov_pose, waiting for messages...")
    
    # Wait for messages (timeout after 10 seconds)
    start_time = time.time()
    try:
        while time.time() - start_time < 10 and received_count < 5:
            rclpy.spin_once(node, timeout_sec=1.0)
    except KeyboardInterrupt:
        pass
    
    # Cleanup
    node.destroy_node()
    rclpy.shutdown()
    
    # Report results
    print(f"\n[validate] Test Results:")
    print(f"  Messages received: {received_count}")
    print(f"  Expected: >= 5 messages in 10 seconds")
    
    if received_count >= 5:
        print("[validate] SUCCESS: Pose bridge is publishing correctly!")
        return True
    else:
        print("[validate] FAILURE: Not enough messages received")
        return False


if __name__ == '__main__':
    try:
        success = test_pose_bridge()
        sys.exit(0 if success else 1)
    except Exception as e:
        print(f"[validate] Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
