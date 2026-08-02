# Task 2 — AWS + local setup

## AWS instance

```bash
# one-time: Isaac Sim 5.1.0 container
# Kit caches must be writable by the container user (HOST_UID=1001 in docker/.env.base)
mkdir -p "$HOME/docker/ebim-challenge/isaac-sim-5.1.0"/{cache/main,cache/computecache,logs,config,data/Kit,data/documents,pkg}
sudo chown -R 1001:1001 "$HOME/docker/ebim-challenge/isaac-sim-5.1.0"
docker compose --env-file docker/.env.base -f docker/docker-compose.yaml \
  --profile isaac-sim-5.1.0 up -d

# every run — sim + remote 3D view + browser arms
# stop Task 1 helpers first if they are up (same port 8090 / ROS topics):
#   docker compose -f task1_isaacsim/docker-compose.yml down
PUBLIC_IP=13.217.2.25 CONTAINER_REPO=/workspace/EBiM_Challenge \
bash task2_isaacsim/scripts/run_isaacsim_teleop.sh \
  --scene room \
  --with-keyboard-teleop \
  --livestream
```

EC2 security group — open livestream ports from your IP (UDP is required):

1. AWS Console → **EC2** → **Instances**
2. Select this instance → **Security** tab → click the security group name
3. **Edit inbound rules** → **Add rule** (add two rules if either is missing)
4. Set:
   - **Type:** Custom TCP · **Port range:** `49100` · **Source:** My IP
   - **Type:** Custom UDP · **Port range:** `47998` · **Source:** My IP
     (pick **Custom UDP**, not Custom TCP — Type dropdown)
5. **Save rules**

Check the inbound list: you should see both `49100/tcp` and `47998/udp`.

## Local machine

```bash
# 3D view — Isaac WebRTC Streaming Client → 13.217.2.25
# https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/manual_livestream_clients.html

# arms/grippers — browser UI (SSH tunnel)
ssh -i /path/to/your-key.pem -L 8090:localhost:8090 ubuntu@13.217.2.25
# then open http://localhost:8090
```

## Giving teleop commands (ROS, on the EC2 instance)

The Isaac Sim bridge subscribes directly to `/isaac/*` command topics, so you can
drive the robot entirely over ROS from the EC2 instance — no browser, no
WebRTC, and no `EBiM-Benchmark/teleoperation` repo required. Run everything
through a helper container that is already up (`ros:jazzy`, host network, matching
RMW).

For pure ROS control, launch the sim with `--no-browser` so the browser
controller does not continuously stream its slider pose over the arm/gripper
topics. Run this in **SSH session #1** (it stays running):

```bash
PUBLIC_IP=13.217.2.25 CONTAINER_REPO=/workspace/EBiM_Challenge \
bash task2_isaacsim/scripts/run_isaacsim_teleop.sh --scene room --livestream --no-browser
```

Wait for `Isaac Sim ROS bridge listening on /isaac command topics`. Everything
below runs in a **second SSH session** into the same EC2 instance.

### Live keyboard driving (this is "where you type")

Open a **second SSH session** and start the interactive keyboard teleop. The
terminal you run this in becomes the place you press keys — focus it and drive:

```bash
docker exec -it task2_teleop_adapters bash -lc \
  "source /opt/ros/jazzy/setup.bash && python3 /workspace/scripts/keyboard_base_teleop.py"
```

```
w / s    base forward / back        o / c   open / close grippers
a / d    strafe left / right        space   stop base
q / e    rotate left / right        Ctrl+C  quit
```

No X display, no WebRTC, no teleoperation repo — it publishes `/pedal/state`
and the gripper topics directly. (The vertical spine is not covered here; ask if
you want it added.)

### One-shot commands (scripting / arms)

For scripted moves or to set arm joint targets, publish topics directly. Define
a helper in the second SSH session:

```bash
rospub() { docker exec -i task2_teleop_adapters bash -lc \
  "source /opt/ros/jazzy/setup.bash && $*"; }
```

**Mobile base** — `/pedal/state` swerve tokens (1 s watchdog, so publish at a
rate; `Ctrl+C` to stop):

```bash
# tokens: FWD, BACK, A (left), B (right), A+C (rotate left), B+C (rotate right)
rospub "ros2 topic pub -r 10 /pedal/state std_msgs/msg/String '{data: FWD}'"
```

**Arms** — `sensor_msgs/JointState` with the 7 FR3 joints (radians):

```bash
rospub "ros2 topic pub -r 20 /isaac/left_joint_commands sensor_msgs/msg/JointState \
'{name: [left_fr3v2_joint1,left_fr3v2_joint2,left_fr3v2_joint3,left_fr3v2_joint4,left_fr3v2_joint5,left_fr3v2_joint6,left_fr3v2_joint7], \
  position: [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]}'"
# right arm: same message on /isaac/right_joint_commands with right_fr3v2_joint*
```

**Grippers** — publish the primary knuckle joint (0.0 = open, ~0.8 = closed);
the bridge handles the Robotiq coupling:

```bash
rospub "ros2 topic pub -r 20 /isaac/left_robotiq_joint_commands sensor_msgs/msg/JointState \
'{name: [left_robotiq_85_left_knuckle_joint], position: [0.8]}'"
# right gripper: /isaac/right_robotiq_joint_commands, right_robotiq_85_left_knuckle_joint
```

**Confirm the robot responds without a viewer** — echo the joint states and
watch the numbers change while a command runs:

```bash
rospub "ros2 topic echo --once /isaac/left_joint_states"
```

Notes:
- Once `EBiM-Benchmark/teleoperation` is public you can instead run its device
  publishers (keyboard / GELLO / foot pedal); the commands above are the
  no-hardware equivalent.
- The vertical spine (`franka_spine_vertical_joint`) currently has no ROS
  command topic — it is driven by `Up`/`Down` keys in the WebRTC window. Ask if
  you want it exposed as a ROS topic too.
- Browser UI alternative (if you ever want it): drop `--no-browser` and
  SSH-tunnel `http://localhost:8090` (see above).
