@echo off
REM ---------------------------------------------------------------------------
REM Start ONLY the EXTERNAL modified HoloOcean engine in a VISIBLE window.
REM
REM Reads paths from config\custom_holoocean_engine.yaml (or the file pointed
REM to by HOLO_CUSTOM_ENGINE_CONFIG).  The external engine folder is used
REM strictly read-only; the engine log is written into this repo's logs\.
REM
REM Usage:
REM   scripts\start_custom_holoocean_visible.bat [MapName]
REM
REM The engine window then waits for a HoloOcean client (sim server or
REM capture script) to attach.  No headless mode is supported on purpose.
REM ---------------------------------------------------------------------------
setlocal
set "REPO_ROOT=%~dp0.."
set "CONDA_ENV=ocean"
set "MAP_ARG="
if not "%~1"=="" set "MAP_ARG=--map %~1"

set "OCEAN_PY=%USERPROFILE%\.conda\envs\%CONDA_ENV%\python.exe"
if exist "%OCEAN_PY%" (
  "%OCEAN_PY%" "%REPO_ROOT%\scripts\launch_custom_engine.py" %MAP_ARG%
) else (
  conda run --no-capture-output -n %CONDA_ENV% python "%REPO_ROOT%\scripts\launch_custom_engine.py" %MAP_ARG%
)
exit /b %ERRORLEVEL%
