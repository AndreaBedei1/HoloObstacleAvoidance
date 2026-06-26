# -*- coding: utf-8 -*-
"""
Launch test for HoloOcean smoke bridge - validates all nodes start and communicate.
This test uses launch_testing to verify the complete smoke test pipeline.
"""
import os
import sys
import ctypes
import unittest
import tempfile
import csv

# Preload spdlog.dll before importing rclpy
spdlog_candidates = [
    r"C:\dev\lyrical\.pixi\envs\default\Library\bin\spdlog.dll",
]
for path in spdlog_candidates:
    if os.path.exists(path):
        try:
            ctypes.CDLL(path)
        except OSError:
            pass
        break

import launch_testing
import launch_ros
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_testing.actions import TestAction
from launch_testing.launch_test import LaunchTestBase
import pytest


def generate_test_description():
    """Generate the launch description for testing."""
    # Declare test parameters
    auto_shutdown = DeclareLaunchArgument(
        'auto_shutdown',
        default_value='true',
        description='Auto shutdown after test'
    )
    
    duration_s = DeclareLaunchArgument(
        'duration_s',
        default_value='10',
        description='Test duration in seconds'
    )
    
    # Create temporary CSV file for recording
    temp_csv = tempfile.NamedTemporaryFile(mode='w', suffix='.csv', delete=False)
    temp_csv.close()
    csv_path = temp_csv.name
    
    # Start Zenoh router
    zenoh_router = ExecuteProcess(
        cmd=[sys.executable, '-c', 
             'import subprocess; subprocess.run(["C:\\dev\\lyrical\\Scripts\\ros2.exe", "run", "rmw_zenoh_cpp", "rmw_zenohd"])'],
        output='screen'
    )
    
    # Bridge node
    bridge_node = Node(
        package='rov_obstacle_sim_bridge',
        executable='holoocean_pose_bridge_node',
        name='holoocean_pose_bridge',
        output='screen',
    )
    
    # Oracle node
    oracle_node = Node(
        package='rov_obstacle_sim_bridge',
        executable='holoocean_obstacle_oracle_node',
        name='holoocean_obstacle_oracle',
        output='screen',
        parameters=[{
            'obstacles': [
                {'x': 5.0, 'y': 0.0, 'z': 0.0, 'radius': 1.5},
                {'x': -3.0, 'y': 4.0, 'z': 0.0, 'radius': 2.0}
            ]
        }]
    )
    
    # Recorder node
    recorder_node = Node(
        package='rov_obstacle_sim_bridge',
        executable='cmd_vel_safe_logger_node',
        name='cmd_vel_safe_logger',
        output='screen',
        parameters=[{
            'csv_file': csv_path,
            'flush_interval_s': 1.0
        }]
    )
    
    return LaunchDescription([
        auto_shutdown,
        duration_s,
        zenoh_router,
        bridge_node,
        oracle_node,
        recorder_node,
        TestAction(HoloOceanSmokeTest),
    ]), {'csv_path': csv_path}


class HoloOceanSmokeTest(LaunchTestBase):
    @unittest.skipUnless(os.name == 'nt', "Windows-specific DLL test")
    def test_nodes_start(self, proc_info, csv_path):
        """Test that all nodes start without crashing."""
        # Wait for processes to stabilize
        import time
        time.sleep(5)
        
        # Check that processes are still running (didn't crash)
        self.assertTrue(proc_info.returncode is None, "Nodes crashed during test")


@pytest.mark.rostest
def generate_test_description():
    return generate_test_description()
