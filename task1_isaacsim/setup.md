# Task 1 — AWS + local setup

## AWS instance

```bash
# every run — sim + remote 3D view
PUBLIC_IP=54.237.209.65 CONTAINER_REPO=/workspace/EBiM_Challenge EMBODIMENT=fr3duo_mobile \
bash task1_isaacsim/scripts/run_isaaclab_newton_teleop.sh \
  --usd-path assets/Robotiq_2f_85_with_d405_mobile_fr3_duo_v0_2.usd \
  --controller-mode position --with-keyboard-teleop --with-cable \
  -- --spine-keyboard-control --spine-keyboard-step 0.02 \
     --spine-keyboard-min 0.0 --spine-keyboard-max 0.5 --livestream 1 \
     --cable-config-path cable_world/configs/table_board_fixture_cable.yaml
```

EC2 security group (AWS console): inbound TCP `49100` + UDP `47998` from your IP.

## Local machine

```bash
# 3D view — install Isaac WebRTC client, connect to <ec2-public-ip>
# https://docs.isaacsim.omniverse.nvidia.com/latest/installation/manual_livestream_clients.html

# arms/grippers — browser UI
# open http://localhost:8090
```
