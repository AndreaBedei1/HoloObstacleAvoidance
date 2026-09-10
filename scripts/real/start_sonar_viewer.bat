@echo off
setlocal
cd /d "%~dp0\..\.."
py scripts\real\sonar_viewer.py %*
endlocal
