@echo off
setlocal

rem Machine-specific roots are overridable via environment variables so the same
rem repo works on the lab machine (pixi at C:\dev\lyrical) and the experiment
rem machine (binary install at C:\dev\ros2_lyrical + conda env ros2_lyrical).
if "%ROS2_ROOT%"=="" (
  if exist "C:\dev\lyrical\local_setup.bat" (
    set "ROS2_ROOT=C:\dev\lyrical"
  ) else if exist "C:\dev\ros2_lyrical\local_setup.bat" (
    set "ROS2_ROOT=C:\dev\ros2_lyrical"
  ) else (
    set "ROS2_ROOT=C:\dev\lyrical"
  )
)
if "%ROS_DISTRO_EXPECTED%"=="" set "ROS_DISTRO_EXPECTED=lyrical"
if "%PIXI_ENV_ROOT%"=="" (
  if exist "%ROS2_ROOT%\.pixi\envs\default\python.exe" (
    set "PIXI_ENV_ROOT=%ROS2_ROOT%\.pixi\envs\default"
  ) else if exist "%USERPROFILE%\miniconda3\envs\ros2_lyrical\python.exe" (
    set "PIXI_ENV_ROOT=%USERPROFILE%\miniconda3\envs\ros2_lyrical"
  ) else (
    set "PIXI_ENV_ROOT=%ROS2_ROOT%\.pixi\envs\default"
  )
)

if not exist "%ROS2_ROOT%\local_setup.bat" (
  echo [FAIL] ROS 2 setup file not found: %ROS2_ROOT%\local_setup.bat
  exit /b 1
)

if not exist "%PIXI_ENV_ROOT%\python.exe" (
  echo [FAIL] Python env not found: %PIXI_ENV_ROOT%\python.exe
  echo Run scripts\setup_ros2_windows.bat first.
  exit /b 1
)

rem Generate a machine-local colcon defaults file so cmake always receives the
rem Python interpreter actually present on this machine (the tracked
rem colcon_defaults_windows.yaml hardcoded the lab machine's pixi path).
if "%COLCON_DEFAULTS_FILE%"=="" (
  set "COLCON_DEFAULTS_FILE=%~dp0..\colcon_defaults_local.yaml"
  > "%~dp0..\colcon_defaults_local.yaml" (
    echo build:
    echo   merge-install: true
    echo   cmake-args:
    echo     - -DPython3_EXECUTABLE=%PIXI_ENV_ROOT:\=/%/python.exe
    echo     - -DPYTHON_EXECUTABLE=%PIXI_ENV_ROOT:\=/%/python.exe
    echo test:
    echo   merge-install: true
  )
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
