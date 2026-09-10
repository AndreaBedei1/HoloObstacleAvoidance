@echo off
setlocal
cd /d "%~dp0\..\.."
py scripts\real\rovl_position_viewer.py %*
endlocal
