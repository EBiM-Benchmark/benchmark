"""Publish gripper TCP poses from the loaded Isaac USD stage."""

from __future__ import annotations

import os
import time


class IsaacGripperPosePublisher:
    def __init__(
        self,
        node,
        pose_stamped_type,
        stage,
        *,
        publish_rate_hz=60.0,
        frame_id="world",
    ):
        self._node = node
        self._pose_stamped_type = pose_stamped_type
        self._stage = stage
        self._frame_id = str(frame_id)
        self._publish_period = 1.0 / max(float(publish_rate_hz), 1.0)
        self._next_publish_time = 0.0
        self._publishers = {
            "left": self._node.create_publisher(pose_stamped_type, "/isaac/left_gripper_pose", 10),
            "right": self._node.create_publisher(pose_stamped_type, "/isaac/right_gripper_pose", 10),
        }
        self._prim_paths = {
            "left": self._resolve_tcp_prim_path("left"),
            "right": self._resolve_tcp_prim_path("right"),
        }
        for side, prim_path in self._prim_paths.items():
            if prim_path:
                print(f"Publishing {side} gripper TCP pose from {prim_path}")
            else:
                print(f"Warning: Could not resolve {side} gripper TCP prim path; pose topic will be silent.")

    def publish(self, force=False):
        now = time.monotonic()
        if not force and now < self._next_publish_time:
            return
        self._next_publish_time = now + self._publish_period

        try:
            from pxr import Usd, UsdGeom
        except Exception as error:
            print(f"Warning: Could not import pxr for gripper pose publishing: {error}")
            return

        for side, prim_path in self._prim_paths.items():
            if not prim_path:
                continue
            prim = self._stage.GetPrimAtPath(prim_path)
            if not prim or not prim.IsValid():
                continue
            try:
                mat = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                translation = mat.ExtractTranslation()
                rotation = mat.ExtractRotationQuat()
                imag = rotation.GetImaginary()
            except Exception as error:
                print(f"Warning: Failed to compute {side} gripper TCP pose from {prim_path}: {error}")
                continue

            msg = self._pose_stamped_type()
            msg.header.stamp = self._node.get_clock().now().to_msg()
            msg.header.frame_id = self._frame_id
            msg.pose.position.x = float(translation[0])
            msg.pose.position.y = float(translation[1])
            msg.pose.position.z = float(translation[2])
            msg.pose.orientation.x = float(imag[0])
            msg.pose.orientation.y = float(imag[1])
            msg.pose.orientation.z = float(imag[2])
            msg.pose.orientation.w = float(rotation.GetReal())
            self._publishers[side].publish(msg)

    def _resolve_tcp_prim_path(self, side):
        env_key = f"ISAAC_{side.upper()}_GRIPPER_TCP_PRIM_PATH"
        explicit = str(os.environ.get(env_key, "")).strip()
        if explicit:
            prim = self._stage.GetPrimAtPath(explicit)
            if prim and prim.IsValid():
                return explicit
            print(f"Warning: {env_key}={explicit} is not a valid prim path.")

        exact_names = (
            f"{side}_fr3v2_hand_tcp",
            f"{side}_fr3v2_1_hand_tcp",
            f"{side}_fr3_hand_tcp",
            f"{side}_hand_tcp",
        )
        best_suffix_match = None
        for prim in self._stage.Traverse():
            name = prim.GetName()
            path = str(prim.GetPath())
            if name in exact_names:
                return path
            if side in path and name.endswith("hand_tcp"):
                best_suffix_match = path

        return best_suffix_match
