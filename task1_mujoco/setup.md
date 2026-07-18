# Task 1 (MuJoCo) — AWS + local setup

MuJoCo has no Isaac-style WebRTC livestream. On a headless GPU box we
mirror that flow with a virtual display + **noVNC** (browser view of the
real MuJoCo viewer). Keyboard teleop works inside the VNC session.

## AWS instance

```bash
# every run — sim + remote 3D view (browser)
PUBLIC_IP=<ec2-public-ip> \
bash task1_mujoco/scripts/run_remote_teleop.sh
```

Later close by running:
```bash
bash task1_mujoco/docker-run.sh down
```

EC2 security group — open noVNC (TCP `6080`) from your IP:

1. AWS Console → **EC2** → **Instances**
2. Select this instance → **Security** tab → click the security group name
3. **Edit inbound rules** → **Add rule**
4. Set:
   - **Type:** Custom TCP
   - **Port range:** `6080`
   - **Source:** My IP (or your `/32` CIDR)
5. **Save rules**

## Local machine

```bash
# 3D view — browser (no client install)
# open http://<ec2-public-ip>:6080/vnc.html?autoconnect=1&resize=remote
# click the desktop once, then use keyboard teleop in that window
```

Keyboard cheat-sheet (same as the sim prints on start): `7/8/9` base /
left / right mode, arrows + PageUp/PageDown to move, `G` close gripper,
`V` / Space open, `-`/`=` speed.
