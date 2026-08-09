@echo off
setlocal
cd /d "%~dp0"

echo Starting Betaflight SITL + local web configurator...
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0start-bf.ps1"

echo.
echo Done. Keep this window open if you want to see the SITL console logs,
echo or close it - the server and simulator keep running in the background.
echo.
pause
