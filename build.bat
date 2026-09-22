@echo off
rem One-click build (double-click to run): calls build.ps1 and pauses at the end
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0build.ps1"
echo.
pause
