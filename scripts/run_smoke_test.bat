@echo off
REM Comprehensive HoloOcean smoke test runner
REM This script validates the complete smoke bridge pipeline

echo ========================================
echo HoloOcean Smoke Test Runner
echo ========================================
echo.

REM Set up environment
call "C:\dev\lyrical\setup.bat"
call "C:\Users\andrea.bedei3\Desktop\HoloObstacleAvoidance\install\setup.bat"

REM Use pixi Python 3.12
set PYTHON_EXE=C:\dev\lyrical\.pixi\envs\default\python.exe

echo Starting Zenoh router...
start "Zenoh Router" /B %PYTHON_EXE% -c "import subprocess; subprocess.run(['C:\dev\lyrical\Scripts\ros2.exe', 'run', 'rmw_zenoh_cpp', 'rmw_zenohd'])"

echo Waiting for router to initialize...
timeout /t 3 /nobreak >nul

echo Starting pose bridge node...
start "Pose Bridge" /B %PYTHON_EXE% -c "import ctypes; ctypes.CDLL(r'C:\dev\lyrical\.pixi\envs\default\Library\bin\spdlog.dll'); import sys; sys.path.insert(0, r'C:\Users\andrea.bedei3\Desktop\HoloObstacleAvoidance\src\rov_obstacle_sim_bridge'); from rov_obstacle_sim_bridge.holoocean_pose_bridge_node import main; main()"

echo Waiting for nodes to stabilize...
timeout /t 5 /nobreak >nul

echo Running validation...
%PYTHON_EXE% scripts\validate_pose_bridge.py

if %ERRORLEVEL% EQU 0 (
    echo.
    echo ========================================
    echo TEST PASSED!
    echo ========================================
) else (
    echo.
    echo ========================================
    echo TEST FAILED!
    echo ========================================
)

REM Cleanup background processes
taskkill /F /IM "python.exe" /FI "WINDOWTITLE eq Zenoh Router*" 2>nul
taskkill /F /IM "python.exe" /FI "WINDOWTITLE eq Pose Bridge*" 2>nul

exit /b %ERRORLEVEL%
