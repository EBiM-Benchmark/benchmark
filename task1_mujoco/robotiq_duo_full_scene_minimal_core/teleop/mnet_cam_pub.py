"""Standalone evidence-camera renderer + publisher process.

The sim process cannot render/publish the camera reliably from within:
three GL contexts (viewer thread, main-thread sync, offscreen) contend in
the driver (offscreen render measured 3.5 ms alone vs 20-24 ms in-process)
and mj_step holds the GIL. This process owns its own copy of the model, an
exclusive GL context, and an idle GIL: the sim only memcpy's its state
(qpos + body_pos + body_quat + geom_rgba, ~20 KB) into shared memory every
loop, and this process samples it at the target fps, runs the kinematics,
renders and publishes.

Syncing raw state arrays (instead of replaying logic) means the one-time
code plate (geom colors), client-driven fixture randomization (body_pos)
and the cable (qpos) all follow automatically.

Launched by teleop/mnet_bridge.py — not meant to be run by hand:

    python mnet_cam_pub.py <shm> <xml> <w> <h> <fps> <camera> <img_topic> <info_topic>

Exits by itself when the sim's heartbeat goes stale.
"""

from __future__ import annotations

import array
import math
import sys
import time
from multiprocessing import shared_memory

import numpy as np

import mujoco

import rclpy
from sensor_msgs.msg import CameraInfo, Image

HDR = 64  # [0..7] state seq (u64), [8..15] heartbeat unix time (f64)


def main() -> None:
    shm_name, xml_path, w, h, fps, cam_name, img_topic, info_topic = sys.argv[1:9]
    w, h, fps = int(w), int(h), float(fps)

    print("[mnet-cam] loading model...", flush=True)
    model = mujoco.MjModel.from_xml_path(xml_path)
    print("[mnet-cam] model loaded", flush=True)
    data = mujoco.MjData(model)
    nq, nbody, ngeom = model.nq, model.nbody, model.ngeom
    qpos_b = nq * 8
    bpos_b = nbody * 3 * 8
    bquat_b = nbody * 4 * 8
    rgba_b = ngeom * 4 * 4
    slot_b = qpos_b + bpos_b + bquat_b + rgba_b

    shm = shared_memory.SharedMemory(name=shm_name)
    seq_v = np.ndarray((1,), np.uint64, shm.buf, 0)
    hb_v = np.ndarray((1,), np.float64, shm.buf, 8)

    def slot_views(slot: int):
        off = HDR + slot * slot_b
        q = np.ndarray((nq,), np.float64, shm.buf, off)
        bp = np.ndarray((nbody, 3), np.float64, shm.buf, off + qpos_b)
        bq = np.ndarray((nbody, 4), np.float64, shm.buf, off + qpos_b + bpos_b)
        rg = np.ndarray((ngeom, 4), np.float32, shm.buf, off + qpos_b + bpos_b + bquat_b)
        return q, bp, bq, rg

    views = (slot_views(0), slot_views(1))

    renderer = mujoco.Renderer(model, height=h, width=w)
    print("[mnet-cam] renderer up", flush=True)
    scene_opt = mujoco.MjvOption()
    scene_opt.geomgroup[:] = 0
    for g in (0, 1, 2, 5):
        scene_opt.geomgroup[g] = 1
    camid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, cam_name)
    fovy = math.radians(float(model.cam_fovy[camid]) if camid >= 0 else float(model.vis.global_.fovy))

    rclpy.init()
    print("[mnet-cam] rclpy up", flush=True)
    node = rclpy.create_node("mujoco_mnet_camera")
    img_pub = node.create_publisher(Image, img_topic, 10)
    info_pub = node.create_publisher(CameraInfo, info_topic, 10)

    fy = 0.5 * h / math.tan(0.5 * fovy)
    info = CameraInfo()
    info.header.frame_id = "mnet_sim_camera"
    info.width, info.height = w, h
    info.distortion_model = "plumb_bob"
    info.d = [0.0] * 5
    info.k = [fy, 0.0, w / 2.0, 0.0, fy, h / 2.0, 0.0, 0.0, 1.0]
    info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    info.p = [fy, 0.0, w / 2.0, 0.0, 0.0, fy, h / 2.0, 0.0, 0.0, 0.0, 1.0, 0.0]

    msg = Image()
    msg.header.frame_id = "mnet_sim_camera"
    msg.height, msg.width = h, w
    msg.encoding = "rgb8"  # the client converts via cv_bridge to bgr8
    msg.is_bigendian = 0
    msg.step = w * 3

    print(f"[mnet-cam] renderer process ready ({w}x{h}@{fps:g}fps)", flush=True)
    period = 1.0 / fps
    next_t = time.perf_counter()
    last_seq = 0
    t0 = None
    sent = 0
    warned = False
    try:
        while True:
            now = time.perf_counter()
            if now < next_t:
                time.sleep(min(next_t - now, 0.005))
                continue
            next_t = max(next_t + period, now - period)
            s = int(seq_v[0])
            if s == 0:
                continue  # sim has not produced the first state yet
            if time.time() - float(hb_v[0]) > 5.0:
                break  # sim heartbeat stale: parent gone
            if s != last_seq:
                q, bp, bq, rg = views[s & 1]
                data.qpos[:] = q
                model.body_pos[:] = bp
                model.body_quat[:] = bq
                model.geom_rgba[:] = rg
                if int(seq_v[0]) == s:
                    last_seq = s
                    mujoco.mj_kinematics(model, data)
                    mujoco.mj_camlight(model, data)
            # publish at the target fps even when no new state arrived
            # (frame-hold, like any constant-frame-rate video pipeline):
            # the evidence stream must satisfy the client's 25 fps gate
            # even on machines whose physics loop runs slower
            renderer.update_scene(data, camera=cam_name if camid >= 0 else 0, scene_option=scene_opt)
            rgb = renderer.render()
            stamp = node.get_clock().now().to_msg()
            msg.header.stamp = stamp
            # array.array: rclpy's uint8[] fast path (bytes would be
            # validated per element, ~75 ms per frame)
            msg.data = array.array("B", rgb.tobytes())
            img_pub.publish(msg)
            info.header.stamp = stamp
            info_pub.publish(info)
            sent += 1
            if t0 is None:
                t0 = now
            elif not warned and now - t0 >= 15.0:
                rate = (sent - 1) / (now - t0)
                if rate < 25.0:
                    print(
                        f"[mnet] WARNING: evidence camera publishing at {rate:.1f} fps "
                        "(the client refuses < 25).",
                        flush=True,
                    )
                warned = True
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        del seq_v, hb_v, views
        shm.close()


if __name__ == "__main__":
    main()
