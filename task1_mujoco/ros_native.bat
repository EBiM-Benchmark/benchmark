@echo off
REM Native Windows ROS 2 (RoboStack) launcher for the eval stack.
REM
REM Hardens PATH before anything ROS loads: other tools' directories (base
REM conda, Docker, Git) carry same-named older DLLs (OpenSSL, zstd, ...)
REM that shadow RoboStack's and kill rclpy with
REM   "DLL load failed while importing _rclpy_pybind11" (proc not found).
REM Then applies the RoboStack activation env and runs whatever follows:
REM
REM   ros_native.bat python robotiq_duo_full_scene_minimal_core\main.py --input keyboard --mnet
REM   ros_native.bat python robotiq_duo_full_scene_minimal_core\main.py --input vr --mnet
REM   ros_native.bat ros2 run mnet_client local_test
REM
REM One-time env setup (see README):
REM   conda create -n ros-humble --override-channels -c robostack-staging ^
REM       -c conda-forge python=3.11 ros-humble-ros-base ros-humble-cv-bridge ^
REM       colcon-common-extensions      (CONDA_CHANNEL_PRIORITY=strict)
REM   then pip install the sim deps into it and colcon-build ros_ws (below).
setlocal
if not defined EBIM_ROS_ENV set "EBIM_ROS_ENV=C:\miniconda\envs\ros-humble"
if not exist "%EBIM_ROS_ENV%\python.exe" (
    echo [ros_native] RoboStack env not found at %EBIM_ROS_ENV%
    echo [ros_native] create it first, or set EBIM_ROS_ENV to its path
    exit /b 1
)
set "CONDA_PREFIX=%EBIM_ROS_ENV%"
set "PATH=%CONDA_PREFIX%;%CONDA_PREFIX%\Library\bin;%CONDA_PREFIX%\Scripts;C:\Windows\System32;C:\Windows"
set "QT_PLUGIN_PATH=%CONDA_PREFIX%\Library\plugins"
call "%CONDA_PREFIX%\Library\local_setup.bat"
set PYTHONHOME=
set "ROS_OS_OVERRIDE=conda:win64"
set "ROS_ETC_DIR=%CONDA_PREFIX%\Library\etc\ros"
set "AMENT_PREFIX_PATH=%CONDA_PREFIX%\Library"
set "AMENT_PYTHON_EXECUTABLE=%CONDA_PREFIX%\python.exe"
REM shared-memory transport for the ~1MB camera frames (UDP loopback
REM collapses to ~1 fps under reliable fragmented traffic on Windows)
set "FASTRTPS_DEFAULT_PROFILES_FILE=%~dp0robotiq_duo_full_scene_minimal_core\release\fastdds_shm.xml"
REM overlay: the locally built mnet client workspace (ros_ws), if present
if exist "%~dp0ros_ws\install\local_setup.bat" call "%~dp0ros_ws\install\local_setup.bat"
%*
