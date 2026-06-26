@echo off
setlocal

if "%ROS2_ROOT%"=="" set "ROS2_ROOT=C:\dev\lyrical"
if "%ROS_DISTRO_EXPECTED%"=="" set "ROS_DISTRO_EXPECTED=lyrical"
if "%PIXI_EXE%"=="" set "PIXI_EXE=%USERPROFILE%\.pixi\bin\pixi.exe"
if "%EXPECTED_PYTHON_VERSION%"=="" set "EXPECTED_PYTHON_VERSION=3.12.3"

echo HoloObstacleAvoidance ROS 2 Windows setup
echo ROS2_ROOT=%ROS2_ROOT%
echo ROS_DISTRO_EXPECTED=%ROS_DISTRO_EXPECTED%
echo EXPECTED_PYTHON_VERSION=%EXPECTED_PYTHON_VERSION%
echo.

if not exist "%PIXI_EXE%" (
  echo [FAIL] pixi was not found at %PIXI_EXE%
  echo Install pixi from https://pixi.sh before running this script.
  exit /b 1
)

if not exist "%ROS2_ROOT%\pixi.toml" (
  echo [FAIL] ROS 2 pixi.toml not found under %ROS2_ROOT%
  echo Install or unpack ROS 2 Lyrical to %ROS2_ROOT% first.
  exit /b 1
)

if not exist "%ROS2_ROOT%\local_setup.bat" (
  echo [FAIL] ROS 2 local_setup.bat not found under %ROS2_ROOT%
  exit /b 1
)

pushd "%ROS2_ROOT%"
"%PIXI_EXE%" install
if errorlevel 1 exit /b 1

if exist preinstall_setup_windows.py (
  "%PIXI_EXE%" run python preinstall_setup_windows.py
  if errorlevel 1 exit /b 1
)
popd

call "%~dp0source_ros2_windows.bat"
if errorlevel 1 exit /b 1

python "%~dp0preflight_ros2_windows.py"
exit /b %ERRORLEVEL%

