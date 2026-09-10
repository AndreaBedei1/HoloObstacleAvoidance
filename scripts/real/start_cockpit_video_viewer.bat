@echo off
cd /d "%~dp0\..\.."
where pythonw >nul 2>nul
if %ERRORLEVEL% EQU 0 (
    start "" pythonw scripts\real\cockpit_video_viewer.py
) else (
    python scripts\real\cockpit_video_viewer.py
)
