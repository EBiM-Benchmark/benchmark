"""Visualize Newton cable point clouds as a USD BasisCurves prim in Isaac."""

from __future__ import annotations

import threading


class IsaacCablePointCloudVisualizer:
    def __init__(
        self,
        node,
        point_cloud_type,
        stage,
        *,
        topic="/cable/body_centers",
        prim_path="/World/NewtonCable/curve_0",
        width_m=0.004,
    ):
        self._node = node
        self._stage = stage
        self._topic = str(topic)
        self._prim_path = str(prim_path)
        self._width_m = float(width_m)
        self._lock = threading.Lock()
        self._points = None
        self._version = 0
        self._applied_version = -1
        self._curve = None
        self._subscription = self._node.create_subscription(
            point_cloud_type,
            self._topic,
            self._on_points,
            10,
        )
        print(f"Newton cable visualizer listening on {self._topic} -> {self._prim_path}")

    def _on_points(self, msg):
        points = [(float(p.x), float(p.y), float(p.z)) for p in msg.points]
        with self._lock:
            self._points = points
            self._version += 1

    def update(self):
        with self._lock:
            if self._points is None or self._version == self._applied_version:
                return
            points = list(self._points)
            version = self._version

        if len(points) < 2:
            return

        try:
            from pxr import Gf, Sdf, UsdGeom, Vt
        except Exception as error:
            print(f"Warning: Could not import pxr for cable visualization: {error}")
            return

        if self._curve is None:
            parent_path = str(Sdf.Path(self._prim_path).GetParentPath())
            UsdGeom.Xform.Define(self._stage, parent_path)
            self._curve = UsdGeom.BasisCurves.Define(self._stage, self._prim_path)
            self._curve.CreateTypeAttr(UsdGeom.Tokens.linear)
            self._curve.CreateWrapAttr(UsdGeom.Tokens.nonperiodic)
            self._curve.CreateWidthsAttr([self._width_m])

        self._curve.GetCurveVertexCountsAttr().Set([len(points)])
        self._curve.GetPointsAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*point) for point in points]))
        self._applied_version = version
