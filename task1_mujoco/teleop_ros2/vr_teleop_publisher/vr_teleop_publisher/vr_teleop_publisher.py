"""VR teleop publisher for the EBiM task-1 MuJoCo simulator, standalone
ROS 2 node.

Publishes RAW OpenXR controller state - pose, grip, trigger, buttons,
sticks - on /left|right/vr_hand (std_msgs/Float32MultiArray, 15 floats; the
namespaces name the HAND). Deliberately NO control math here: the clutch
anchor, VR->screen mapping, hand->arm mirroring (--facing), servo gains and
gripper edge logic all run in the simulator (--input ros_teleop) through the
exact same code as its local VR mode, so the feel is identical and this node
stays a dumb device reader like the keyboard/gamepad publishers.

Message layout (see the sim's teleop/config.py, ROS_TELEOP_VR_HAND_TOPIC):
  [0] valid  [1:4] pos xyz (m, VR standing space)  [4:8] quat wxyz
  [8] grip  [9] trigger  [10] a  [11] b  [12:14] stick xy  [14] stick_click

Needs on THIS machine: pyopenxr + PyOpenGL + glfw + numpy, and an active
OpenXR runtime with the headset connected (Quest Link on Windows, WiVRn on
Linux). The OpenXR backend is loaded from the simulator checkout's
teleop/vr_openxr.py (self-contained, no sim install needed) - found via
$EBIM_SIM_DIR, /ws/sim (the Docker image), or the repo layout relative to
this file.

--pattern publishes a synthetic engaged left hand moving in a slow circle
for a few seconds instead of reading a headset (self-test without any VR
hardware; the mapped arm should visibly follow).

Not yet wired: controller haptics (contact rumble exists only in the sim's
local modes) and stick-click speed levels.
"""

from __future__ import annotations

import argparse
import importlib.util
import math
import os
import time

import numpy as np
import rclpy
from std_msgs.msg import Float32MultiArray

PUBLISH_HZ = 90.0
SIDES = ("left", "right")
MSG_LEN = 15


def _find_vr_openxr():
    """Load the sim's self-contained teleop/vr_openxr.py by file path."""
    here = os.path.dirname(os.path.abspath(__file__))
    candidates = []
    if os.environ.get("EBIM_SIM_DIR"):
        candidates.append(os.environ["EBIM_SIM_DIR"])
    candidates += [
        "/ws/sim",  # the Docker image
        os.path.join(os.getcwd(), "robotiq_duo_full_scene_minimal_core"),
        # source layout: teleop_ros2/vr_teleop_publisher/vr_teleop_publisher/
        os.path.normpath(os.path.join(here, "..", "..", "..", "robotiq_duo_full_scene_minimal_core")),
    ]
    for sim_dir in candidates:
        path = os.path.join(sim_dir, "teleop", "vr_openxr.py")
        if os.path.isfile(path):
            spec = importlib.util.spec_from_file_location("vr_openxr", path)
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            return module
    raise RuntimeError(
        "cannot find the simulator's teleop/vr_openxr.py - run from the repo "
        "root, or set EBIM_SIM_DIR to the robotiq_duo_full_scene_minimal_core directory"
    )


def _mat_to_quat_wxyz(m: np.ndarray) -> np.ndarray:
    """3x3 rotation matrix -> unit quaternion (wxyz), pure numpy."""
    t = float(m[0, 0] + m[1, 1] + m[2, 2])
    if t > 0.0:
        s = math.sqrt(t + 1.0) * 2.0
        q = np.array(
            [0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s]
        )
    elif m[0, 0] >= m[1, 1] and m[0, 0] >= m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        q = np.array(
            [(m[2, 1] - m[1, 2]) / s, 0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s]
        )
    elif m[1, 1] >= m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        q = np.array(
            [(m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s]
        )
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        q = np.array(
            [(m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s]
        )
    return q / max(float(np.linalg.norm(q)), 1e-9)


class VrTeleopPublisher:
    def __init__(self, node) -> None:
        self.node = node
        self.hand_pubs = {s: node.create_publisher(Float32MultiArray, f"/{s}/vr_hand", 10) for s in SIDES}

    def publish_hand(self, side: str, hand) -> None:
        msg = Float32MultiArray()
        data = [0.0] * MSG_LEN
        if hand is not None and hand.valid:
            quat = _mat_to_quat_wxyz(np.asarray(hand.rot, dtype=np.float64))
            data[0] = 1.0
            data[1:4] = [float(v) for v in hand.pos]
            data[4:8] = [float(v) for v in quat]
            data[8] = float(hand.grip)
            data[9] = float(hand.trigger)
            data[10] = 1.0 if hand.a else 0.0
            data[11] = 1.0 if hand.b else 0.0
            data[12:14] = [float(hand.stick[0]), float(hand.stick[1])]
            data[14] = 1.0 if hand.stick_click else 0.0
        msg.data = data
        self.hand_pubs[side].publish(msg)


def _run_pattern(pub: VrTeleopPublisher, node, seconds: float) -> None:
    """Synthetic engaged left hand tracing a slow circle - no headset needed.
    The sim anchors the clutch at the first engaged pose it sees, so the
    mapped arm should trace a matching (scaled) circle."""
    node.get_logger().info(f"--pattern: engaged left hand circling for {seconds:.0f}s")
    end = time.perf_counter() + seconds
    t0 = time.perf_counter()
    while rclpy.ok() and time.perf_counter() < end:
        t = time.perf_counter() - t0
        msg = Float32MultiArray()
        data = [0.0] * MSG_LEN
        data[0] = 1.0
        data[1] = 0.05 * math.sin(0.8 * t)  # VR x
        data[2] = 0.05 * (1.0 - math.cos(0.8 * t))  # VR y (up)
        data[4] = 1.0  # identity quat wxyz
        data[8] = 1.0  # grip engaged
        msg.data = data
        pub.hand_pubs["left"].publish(msg)
        pub.publish_hand("right", None)
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(1.0 / PUBLISH_HZ)


def main(args=None) -> None:
    if os.name == "nt":
        try:
            import ctypes

            ctypes.windll.winmm.timeBeginPeriod(1)
        except Exception:
            pass

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--pattern",
        type=float,
        nargs="?",
        const=5.0,
        default=None,
        help="publish a synthetic engaged-hand sequence for N seconds (default 5) and exit",
    )
    cli, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = rclpy.create_node("vr_teleop_publisher")
    pub = VrTeleopPublisher(node)

    try:
        if cli.pattern is not None:
            _run_pattern(pub, node, cli.pattern)
            return
        try:
            vr_openxr = _find_vr_openxr()
        except Exception as exc:
            node.get_logger().error(str(exc))
            return
        try:
            with vr_openxr.OpenXRTeleop((64, 64)) as vr:
                vr.start()
                node.get_logger().info(
                    "OpenXR session up - publishing /left|right/vr_hand; "
                    "squeeze a grip to drive the mapped arm in the sim"
                )
                while rclpy.ok() and vr.alive():
                    hands = vr.get_hands()
                    for side in SIDES:
                        pub.publish_hand(side, hands.get(side))
                    rclpy.spin_once(node, timeout_sec=0.0)
                    time.sleep(1.0 / PUBLISH_HZ)
                vr.stop()
                if vr.error is not None:
                    node.get_logger().error(f"xr thread ended with: {vr.error}")
        except Exception as exc:
            node.get_logger().error(f"openxr session failed: {exc}")
            node.get_logger().error(
                "checklist: 1) headset streaming app connected (Quest Link / WiVRn) "
                "2) it is the active OpenXR runtime 3) WEAR the headset 4) controllers awake"
            )
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
