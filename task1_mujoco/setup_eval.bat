@echo off
REM One-click setup for the scored evaluation on Windows (RoboStack, no
REM Docker). Practice mode (no --mnet) needs none of this - see start.bat.
REM Safe to re-run any time (after git pull, or to rebuild ros_ws): every
REM step below checks before it acts.
REM
REM   setup_eval.bat
REM   ros_native.bat python robotiq_duo_full_scene_minimal_core\main.py --input keyboard --mnet
REM   ros_native.bat ros2 run mnet_client local_test
setlocal enabledelayedexpansion
cd /d "%~dp0"
if not defined EBIM_ROS_ENV set "EBIM_ROS_ENV=C:\miniconda\envs\ros-humble"

if exist "%EBIM_ROS_ENV%\python.exe" (
    echo [setup] ros-humble env already exists at %EBIM_ROS_ENV% - skipping conda create
) else (
    echo [setup] creating the ros-humble conda env - this downloads several GB, a few minutes...
    call conda create -y -n ros-humble --override-channels -c robostack-staging -c conda-forge ^
        python=3.11 ros-humble-ros-base ros-humble-cv-bridge colcon-common-extensions
    if errorlevel 1 (
        echo [setup] conda create failed - see the error above
        exit /b 1
    )
)

echo [setup] installing sim + client Python dependencies...
call conda run -n ros-humble pip install mujoco==3.9.0 "numpy>=1.24,<2" glfw==2.10.0 ^
    pygame==2.6.1 "pillow>=10" pyopenxr==1.1.5301 PyOpenGL==3.1.10 openvr==2.12.1401 ^
    opencv-python "pydantic>=2,<3" requests tqdm pupil-apriltags pybullet
if errorlevel 1 (
    echo [setup] pip install failed - see the error above
    exit /b 1
)

echo [setup] copying the client and ros_teleop publishers into ros_ws\src...
xcopy /E /I /Y /Q mnet_client-ros_2 ros_ws\src\mnet_client >nul
xcopy /E /I /Y /Q teleop_ros2\keyboard_teleop_publisher ros_ws\src\keyboard_teleop_publisher >nul
xcopy /E /I /Y /Q teleop_ros2\gamepad_teleop_publisher ros_ws\src\gamepad_teleop_publisher >nul

echo [setup] colcon build (a few minutes on first run)...
pushd ros_ws
call "%~dp0ros_native.bat" colcon build --merge-install
if errorlevel 1 (
    popd
    echo [setup] colcon build failed - see the error above
    exit /b 1
)
popd

echo [setup] applying cross-platform fixes (file_dir, stdin)...
call "%~dp0ros_native.bat" python "%~dp0robotiq_duo_full_scene_minimal_core\release\mnet_client_postpatch.py" "%~dp0ros_ws" "%~dp0mnet_out_native"
if errorlevel 1 (
    echo [setup] post-build patch failed - see the error above
    exit /b 1
)

echo.
echo [setup] done. Results will land in mnet_out_native\. Run the eval with:
echo   ros_native.bat python robotiq_duo_full_scene_minimal_core\main.py --input keyboard --mnet
echo   ros_native.bat ros2 run mnet_client local_test
echo Re-run this script any time after pulling updates or before a submission.
