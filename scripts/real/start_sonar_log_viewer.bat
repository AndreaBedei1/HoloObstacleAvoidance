@echo off
cd /d "%~dp0\..\.."
where pythonw >nul 2>nul
if %ERRORLEVEL% EQU 0 (
    start "" pythonw scripts\real\sonar_log_viewer.py
) else (
    python scripts\real\sonar_log_viewer.py
)
