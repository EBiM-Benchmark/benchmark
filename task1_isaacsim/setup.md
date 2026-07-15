# Task 1 — AWS + local setup

## AWS instance

```bash
# every run — sim + remote 3D view
PUBLIC_IP=<ec2-public-ip> CONTAINER_REPO=/workspace/EBiM_Challenge EMBODIMENT=fr3duo_mobile bash task1_isaacsim/scripts/run_isaaclab_newton_teleop.sh \
  --usd-path assets/Robotiq_2f_85_with_d405_mobile_fr3_duo_v0_2.usd \
  --controller-mode position \
  -- --livestream 1 --no-spine-keyboard-control
```

EC2 security group (AWS console): inbound TCP `49100` + UDP `47998` from your IP.

## Local machine

```bash
# 3D view — install Isaac WebRTC client, connect to <ec2-public-ip>
# https://docs.isaacsim.omniverse.nvidia.com/latest/installation/manual_livestream_clients.html

# arms/grippers — browser UI
# open http://localhost:8090
```
