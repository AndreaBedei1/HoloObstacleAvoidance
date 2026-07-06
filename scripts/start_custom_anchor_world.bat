@echo off
REM ---------------------------------------------------------------------------
REM Start the custom-anchor world: EXTERNAL modified engine (visible window)
REM + HoloOcean sim server (conda ocean) serving the two-process TCP bridge.
REM
REM The sim server launches the engine automatically (custom_engine.auto_launch
REM in the scenario YAML), attaches, spawns the REAL /Game/ancora.ancora mesh
REM via the engine's SpawnAsset world command, and then listens on
REM 127.0.0.1:47654 for the ROS 2 bridge node.
REM
REM Usage:
REM   scripts\start_custom_anchor_world.bat [scenario_yaml] [extra sim-server args]
REM
REM Defaults to the custom_anchor_visible.yaml scenario.  Add --engine-running
REM to attach to an engine window you already started with
REM scripts\start_custom_holoocean_visible.bat.
REM ---------------------------------------------------------------------------
setlocal
set "REPO_ROOT=%~dp0.."
set "CONDA_ENV=ocean"
set "SCENARIO=%~1"
if "%SCENARIO%"=="" set "SCENARIO=%REPO_ROOT%\src\rov_obstacle_sim_bridge\config\holoocean_scenarios\custom_anchor_visible.yaml"
if not "%~1"=="" shift

if not exist "%SCENARIO%" (
  echo [FAIL] scenario not found: %SCENARIO%
  exit /b 1
)

set "OCEAN_PY=%USERPROFILE%\.conda\envs\%CONDA_ENV%\python.exe"

echo [start] scenario: %SCENARIO%
if exist "%OCEAN_PY%" (
  "%OCEAN_PY%" "%REPO_ROOT%\src\rov_obstacle_sim_bridge\holoocean_server\holoocean_sim_server.py" ^
    --config "%SCENARIO%" --serve %1 %2 %3 %4 %5
) else (
  conda run --no-capture-output -n %CONDA_ENV% python ^
    "%REPO_ROOT%\src\rov_obstacle_sim_bridge\holoocean_server\holoocean_sim_server.py" ^
    --config "%SCENARIO%" --serve %1 %2 %3 %4 %5
)
exit /b %ERRORLEVEL%
