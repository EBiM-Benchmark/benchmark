@echo off
REM One-click start on Windows: creates the conda env on first run, then
REM launches the teleop. Flags pass through, e.g.:  start.bat --input vr
setlocal
cd /d "%~dp0robotiq_duo_full_scene_minimal_core"

where conda >nul 2>nul
if errorlevel 1 (
    echo [start] conda not found. Install Miniconda first:
    echo         https://docs.conda.io/en/latest/miniconda.html
    echo         then reopen this terminal and run start.bat again.
    pause
    exit /b 1
)

where python >nul 2>nul
if errorlevel 1 (
    call conda run --live-stream -n base python start.py %*
) else (
    python start.py %*
)
