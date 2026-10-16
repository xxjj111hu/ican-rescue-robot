@echo off
title iCAN Simulation Launcher
setlocal

set "DISTRO=Ubuntu-22.04"
REM WSL-side path: pure ASCII, no wslpath conversion needed.
set "WSWSL=/home/xxjj/ican/demo_shoot_ws/demo_shoot_ws"

echo ============================================
echo   iCAN Simulation Launcher
echo ============================================
echo.
echo Distro : %DISTRO%
echo Path   : %WSWSL%
echo.
echo Loading ROS 2 + iCAN simulation environment...
echo Gazebo will open in 1-3 minutes on the first run.
echo Press Ctrl+C in this window to stop the simulation.
echo.

wsl.exe -d %DISTRO% --cd "%WSWSL%" -- bash -lc "bash start_sim.sh"

echo.
echo ============================================
echo   Simulation stopped.
echo ============================================
pause
