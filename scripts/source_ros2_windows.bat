@echo off
setlocal

if "%ROS2_ROOT%"=="" set "ROS2_ROOT=C:\dev\lyrical"
if "%ROS_DISTRO_EXPECTED%"=="" set "ROS_DISTRO_EXPECTED=lyrical"
if "%PIXI_ENV_ROOT%"=="" set "PIXI_ENV_ROOT=%ROS2_ROOT%\.pixi\envs\default"
if "%COLCON_DEFAULTS_FILE%"=="" set "COLCON_DEFAULTS_FILE=%~dp0..\colcon_defaults_windows.yaml"

if not exist "%ROS2_ROOT%\local_setup.bat" (
  echo [FAIL] ROS 2 setup file not found: %ROS2_ROOT%\local_setup.bat
  exit /b 1
)

if not exist "%PIXI_ENV_ROOT%\python.exe" (
  echo [FAIL] Pixi Python not found: %PIXI_ENV_ROOT%\python.exe
  echo Run scripts\setup_ros2_windows.bat first.
  exit /b 1
)

endlocal & (
  set "ROS2_ROOT=%ROS2_ROOT%"
  set "ROS_DISTRO_EXPECTED=%ROS_DISTRO_EXPECTED%"
  set "PIXI_ENV_ROOT=%PIXI_ENV_ROOT%"
  set "COLCON_DEFAULTS_FILE=%COLCON_DEFAULTS_FILE%"
  set "COLCON_PYTHON_EXECUTABLE=%PIXI_ENV_ROOT%\python.exe"
  set "PATH=%PIXI_ENV_ROOT%;%PIXI_ENV_ROOT%\Library\mingw-w64\bin;%PIXI_ENV_ROOT%\Library\usr\bin;%PIXI_ENV_ROOT%\Library\bin;%PIXI_ENV_ROOT%\Scripts;%PIXI_ENV_ROOT%\bin;%PATH%"
  call "%ROS2_ROOT%\local_setup.bat"
)
