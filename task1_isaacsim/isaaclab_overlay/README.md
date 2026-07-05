# Isaac Lab `ros2_jazzy` Overlay (Task 1)

Task 1 runs the Newton/MJWarp bridge **inside an Isaac Lab container**. The
required physics (`isaaclab_newton`, MJWarp solver) ships with Isaac Lab
`release/3.0.0-beta2`, but the pipeline also needs a small `ros2_jazzy` Docker
service layered on top of Isaac Lab's base image (ROS 2 Jazzy + a bind mount of
this repository). These files reproduce that setup.

> The competition repo's other `docker/` profiles (`isaac-sim-*`,
> `isaac-lab-2.3.2`) are **not** used by Task 1 — they do not include the Newton
> physics or this ROS 2 overlay.

## Pinned Isaac Lab version

- Upstream: <https://github.com/isaac-sim/IsaacLab>
- Branch: `release/3.0.0-beta2`
- Commit: `0916ea3c0f126821ef1783c7119d248834fc8d0b`

## Contents

| File | Purpose |
| --- | --- |
| `Dockerfile.ros2_jazzy` | Adds ROS 2 Jazzy (rclpy, FastDDS/CycloneDDS, colcon) on top of `isaac-lab-base`. |
| `.env.ros2_jazzy` | ROS 2 middleware settings for the `ros2_jazzy` profile. |
| `ros2_jazzy_overlay.patch` | Adds the `isaac-lab-ros2_jazzy` compose service, the repo bind mount, its X11 config, an X11 robustness fix, and pins `ISAACSIM_VERSION`. The repo path is a `__EBIM_CHALLENGE_ROOT__` placeholder. |
| `apply_overlay.sh` | Copies the two files and applies the patch, substituting the repo path. |

## One-time setup

```bash
# 1. Clone Isaac Lab next to this repository, at the pinned commit.
cd ..                                   # parent of the benchmark checkout
git clone https://github.com/isaac-sim/IsaacLab.git
git -C IsaacLab checkout 0916ea3c0f126821ef1783c7119d248834fc8d0b

# 2. Apply this overlay (from the benchmark repo root).
task1_isaacsim/isaaclab_overlay/apply_overlay.sh            # auto-detects ../IsaacLab and this repo
# or explicitly:
task1_isaacsim/isaaclab_overlay/apply_overlay.sh /path/to/IsaacLab /path/to/benchmark

# 3. Build + start the ROS 2 Jazzy Isaac Lab container.
cd ../IsaacLab
./docker/container.py start ros2_jazzy
```

After this, `docker ps` should list `isaac-lab-ros2_jazzy`, and inside it
`/workspace/EBiM_Challenge/task1_isaacsim` is the mounted repo. The Task 1 launcher
(`task1_isaacsim/scripts/run_isaaclab_newton_teleop.sh`) drives this container.

## What the patch changes

- `docker/docker-compose.yaml` — adds the `isaac-lab-ros2_jazzy` service (profile
  `ros2_jazzy`) and a bind mount `__EBIM_CHALLENGE_ROOT__ -> /workspace/EBiM_Challenge`.
- `docker/x11.yaml` — X11 environment/volumes for the new service (GUI window).
- `docker/utils/x11_utils.py` — treats a stale/inaccessible cached `.xauth` path
  as missing instead of crashing (multi-user hosts).
- `docker/.env.base` — pins `ISAACSIM_VERSION=6.0.0`.

If `git apply` fails because your Isaac Lab checkout is not at the pinned commit,
check it out (`git -C ../IsaacLab checkout 0916ea3c0f...`) and re-run, or apply
the four changes by hand using the patch as a reference.

## Overriding paths

`run_isaaclab_newton_teleop.sh` and `apply_overlay.sh` both honour these:

- `ISAACLAB_ROOT` — Isaac Lab checkout (default: sibling `../IsaacLab`).
- `CONTAINER_REPO` — mount point inside the container (default `/workspace/<repo-name>`;
  must match the patch target `/workspace/EBiM_Challenge`).
