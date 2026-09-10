@echo off
setlocal
cd /d "%~dp0\..\.."
python scripts\real\sonar_viewer.py %*
endlocal
