# Duo Full Scene Minimal-Core Teleop

Mobile dual-FR3 + Robotiq 2F-85 cable-routing teleop in the full board /
fixture / room scene. One entry point, three input methods:

```powershell
cd D:\mujoco_test\robotiq_duo_full_scene_minimal_core
conda run -n mujoco python main.py                      # keyboard (default)
conda run -n mujoco python main.py --input gamepad      # gamepad
conda run -n mujoco python main.py --input vr           # VR (OpenXR, no Steam)
```

Smoke test (headless, no devices needed):

```powershell
conda run -n mujoco python main.py --no-viewer
conda run -n mujoco python main.py --input vr --no-viewer
```

Each mode has its own flags — see `python main.py --help` and
`python main.py --input vr --help`.

## Layout

- `main.py` — entry point; dispatches `--input` and loads modes lazily, so a
  broken VR module never affects keyboard/gamepad.
- `teleop/` — all code, one module per concern (module map in
  `teleop/__init__.py`):
  - `config.py` every tunable constant; `cli.py` per-module argument groups
  - `scene.py` / `robot_arm.py` / `grasping.py` / `base_drive.py` /
    `session.py` — the shared simulation core
  - `input_keyboard.py`, `input_gamepad.py`, `run_desktop.py` — desktop modes
  - `vr_mapping.py`, `vr_openxr.py`, `vr_steamvr.py`, `run_vr.py` — VR mode
- `duo_full_scene_grasp.xml` + `assets/` — the scene.
- `duo_full_scene_gamepad_demo.py`, `duo_full_scene_vr_demo.py` — deprecated
  shims forwarding to `main.py` (old commands keep working).

## Controls

Keyboard / gamepad (`teleop/run_desktop.py` prints the full reminder):

- Share / `7`: mobile base mode; L1 / `8`: left arm; R1 / `9`: right arm
- left stick / arrow keys: translate base or active TCP in SCREEN directions
  (up = away from you, left = your left)
- right stick + D-pad left/right: TCP rotation; `R` toggles keyboard rotate
  mode (arrows = yaw/pitch, PageUp/PageDown = roll)
- L2/R2 or PageUp/PageDown: spine in base mode, TCP Z in arm mode;
  Home/End turn the base left/right
- Circle / `G`: close gripper (pad-contact force servo); Cross / `V`: open
- button 7/8 or `-`/`=`: slower/faster; `B` contact dump; `N` collision view
- `F` / `H`: report task finished / skipped (only in `--mnet` eval mode)

VR (`--input vr`, Quest 2 over Meta Quest Link, or any OpenXR runtime):

- hold GRIP: clutch — the controller drives that hand's arm (mirrored when
  facing the robot; `--facing behind` for same-side)
- TRIGGER: close gripper; A/X: open
- RIGHT stick: base X/Y; LEFT stick: yaw / spine
- stick click: left = speed up, right = slow down
- gripper contact anywhere rumbles the controller
- default is monitor-view; `--hmd-view` floats a stereo sim screen in the
  headset

### VR runtimes per OS

The VR code is pure OpenXR — it talks to whatever runtime is active, on any
OS; only the runtime setup differs:

- **Windows + Quest 2/3**: Meta Quest Link app (Link cable or Air Link),
  set as the active OpenXR runtime in its settings.
- **Ubuntu + Quest 2/3**: [WiVRn](https://github.com/WiVRn/WiVRn)
  (recommended, open source, no Steam: install the server via Flathub, the
  client on the headset, pair over WiFi/USB — it registers itself as the
  active OpenXR runtime), or ALVR + SteamVR. Requires an X11 session (under
  Wayland run via XWayland / `GLFW_PLATFORM=x11`).
- **Index / Vive (any OS)**: SteamVR is the OpenXR runtime.
- Controllers are covered by the suggested-binding profiles in
  `teleop/vr_openxr.py` (Touch / Index / Vive / khr-simple).

If the sim logs `FormFactorUnavailable`, the headset streaming app is not
connected or the headset is not being worn.

## ManipulationNet eval (`--mnet`)

`teleop/mnet_bridge.py` makes the sim look like a robot system to the
official mnet-client (`../mnet_client-ros_2`, part of this repo): it
publishes the evidence camera as `sensor_msgs/Image` (default
`/mujoco/camera/image_raw`, 30 fps) plus CameraInfo, follows
`/mnet_client/ongoing_task` and `/mnet_client/board_configuration`, and
reports task results via the Trigger services when you press `F` (finished)
/ `H` (skipped) in the viewer window. Works with every input method:
`python main.py --input vr --mnet`.

The evidence camera is `mnet_overhead`: a ceiling-mounted camera in the
scene XML directly above the board center, looking straight down (lens
dropped below the room's pendant lamp), board centered in frame with the
gripper workspace visible — as the benchmark requires. `--mnet-camera
viewer` publishes the operator's desktop view instead.

Our board implements the **Tier2** layout (2 wire adapters, 1 C-clip, 4
round pegs); every other announced tier is auto-reported as skipped
(`--mnet-tier`, default Tier2 — no manual `H` needed). When the client announces a tier it
also publishes the slightly RANDOMIZED fixture coordinates
(`test_coordinates`, each fixture off by at most one grid cell ~2.5 cm);
`teleop/mnet_board.py` automatically moves the sim fixtures to match and
re-lays the cable (disable with `--no-mnet-randomize`). Non-Tier2
configurations are detected and ignored.

Requires ROS 2 (`rclpy`) in the sim's Python environment — on Windows that
means WSL2/Ubuntu or a RoboStack conda env; without it the bridge disables
itself and teleop runs normally. Setup:

1. Fill `mnet_client-ros_2/config/team_config.json`: `camera_image_topic`
   = `/mujoco/camera/image_raw` (or pass `--mnet-camera-topic`),
   `autonomy_level` = 0 (teleoperation), `file_dir` = somewhere writable.
2. Start the sim first (`--mnet`) so the camera topic has a publisher, then
   `ros2 run mnet_client local_test` (task: `cable_management`), or
   `submission` for a real attempt.
3. Route the cable per the announced Tier task, press `F` when done with
   each task, `H` to skip; type FINISH in the client terminal to end.
