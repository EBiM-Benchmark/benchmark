#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash

export PYTHONPATH="/workspace/cable:${PYTHONPATH:-}"

viewer="${CABLE_VIEWER:-null}"
device="${CABLE_DEVICE:-cuda:0}"
config_path="${CABLE_CONFIG_PATH:-/workspace/cable/configs/board_cable.yaml}"
gripper_config_path="${CABLE_GRIPPER_CONFIG_PATH:-/workspace/cable/configs/gripper_board_cable.yaml}"

args=(
  /workspace/cable/run_board_cable_ros.py
  --viewer "${viewer}"
  --device "${device}"
  --config-path "${config_path}"
  --gripper-config-path "${gripper_config_path}"
)

gripper_mode="${CABLE_GRIPPER_MODE:-proxy}"
if [[ "${CABLE_ENABLE_GRIPPER:-true}" != "true" ]]; then
  args+=(--no-gripper)
elif [[ "${gripper_mode}" == "proxy" ]]; then
  args+=(--no-gripper --proxy-gripper)
elif [[ "${gripper_mode}" == "sra" ]]; then
  :
elif [[ "${gripper_mode}" == "none" ]]; then
  args+=(--no-gripper)
else
  echo "Unsupported CABLE_GRIPPER_MODE='${gripper_mode}' (expected proxy, sra, or none)" >&2
  exit 2
fi

if [[ -n "${CABLE_POINT_TOPIC:-}" ]]; then
  args+=(--cable-point-topic "${CABLE_POINT_TOPIC}")
fi

if [[ -n "${CABLE_GRIPPER_POSE_TOPIC:-}" ]]; then
  args+=(--gripper-pose-topic "${CABLE_GRIPPER_POSE_TOPIC}")
fi

if [[ -n "${CABLE_GRIPPER_GAP_TOPIC:-}" ]]; then
  args+=(--gripper-gap-topic "${CABLE_GRIPPER_GAP_TOPIC}")
fi

if [[ -n "${CABLE_EXTRA_ARGS:-}" ]]; then
  # shellcheck disable=SC2206
  extra_args=(${CABLE_EXTRA_ARGS})
  args+=("${extra_args[@]}")
fi

exec python3 "${args[@]}"
