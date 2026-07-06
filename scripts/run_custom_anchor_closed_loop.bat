@echo off
REM ---------------------------------------------------------------------------
REM ONE COMMAND: full VISIBLE closed-loop run with the REAL custom anchor.
REM
REM   external modified engine (visible window)  <- launched by sim server
REM   + HoloOcean sim server (conda ocean, TCP 47654)
REM   + zenoh router + ROS 2 bridge/nominal/planner nodes
REM   + validator (metrics JSON + PNG camera frames into visualizations\)
REM
REM Usage:
REM   scripts\run_custom_anchor_closed_loop.bat [scenario_yaml] [extra args]
REM
REM Extra args are forwarded to run_custom_anchor_closed_loop.py, e.g.
REM   --engine-running   reuse an already-open engine window
REM   --keep-engine      leave the engine window open afterwards
REM   --duration-s 60    longer validation window
REM ---------------------------------------------------------------------------
setlocal
set "REPO_ROOT=%~dp0.."

call "%~dp0source_ros2_windows.bat"
if errorlevel 1 exit /b 1
call "%REPO_ROOT%\install\setup.bat"
if errorlevel 1 (
  echo [FAIL] workspace not built: run scripts\setup_ros2_windows.bat first
  exit /b 1
)

set "SCENARIO_ARG="
if not "%~1"=="" (
  set "SCENARIO_ARG=--scenario %~1"
  shift
)

"%PIXI_ENV_ROOT%\python.exe" "%REPO_ROOT%\scripts\run_custom_anchor_closed_loop.py" %SCENARIO_ARG% %1 %2 %3 %4 %5 %6
exit /b %ERRORLEVEL%
