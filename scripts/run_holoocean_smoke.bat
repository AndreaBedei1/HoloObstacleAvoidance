@echo off
REM HoloOcean smoke test launcher - handles ROS 2 DLL dependencies on Windows
REM This script properly sets up the environment for running ROS 2 nodes

REM Source ROS 2 environment
call "C:\dev\lyrical\setup.bat"
call "C:\Users\andrea.bedei3\Desktop\HoloObstacleAvoidance\install\setup.bat"

REM Add pixi Library/bin to PATH for DLL dependencies (spdlog, etc.)
set PATH=C:\dev\lyrical\.pixi\envs\default\Library\bin;%PATH%

REM Use pixi Python 3.12 (matches ROS 2 binaries)
set PYTHON_EXE=C:\dev\lyrical\.pixi\envs\default\python.exe

echo Starting HoloOcean smoke test...
echo Using Python: %PYTHON_EXE%

REM Start Zenoh router in background
start "Zenoh Router" /B %PYTHON_EXE% -c "import subprocess; subprocess.run(['C:\dev\lyrical\Scripts\ros2.exe', 'run', 'rmw_zenoh_cpp', 'rmw_zenohd'])"

REM Wait for router to initialize
timeout /t 3 /nobreak >nul

REM Run the launch file
%PYTHON_EXE% -c "import sys; sys.path.insert(0, r'C:\Users\andrea.bedei3\Desktop\HoloObstacleAvoidance\src\rov_obstacle_sim_bridge'); from rov_obstacle_sim_bridge.holoocean_pose_bridge_node import main; main()"
