#!/usr/bin/env bash
# Copyright (c) 2026 The EBiM Benchmark Contributors
# SPDX-License-Identifier: Apache-2.0
#
# Start the Task 1 sim-side teleop adapters that connect the EBiM
# `teleoperation` device publishers to the Isaac Lab Newton bridge topics.
#
#   keyboard_to_base.py : /keyboard/state  -> /pedal/state          (mobile base)
#   gello_to_bridge.py  : /*/gello/joint_states -> /bridge/*        (arms + grippers)
#
# Both adapters are pure topic remappers: each is idle until its input topic is
# published, so running them together is safe regardless of which input device
# (keyboard or GELLO) a participant uses. Select a subset with TELEOP_ADAPTERS,
# e.g. `TELEOP_ADAPTERS=keyboard` or `TELEOP_ADAPTERS=gello`.
set -eo pipefail

source /opt/ros/jazzy/setup.bash

ADAPTERS_DIR=/workspace/scripts/adapters
SELECTED="${TELEOP_ADAPTERS:-keyboard gello}"

pids=()
for adapter in ${SELECTED}; do
  case "${adapter}" in
    keyboard)
      python3 "${ADAPTERS_DIR}/keyboard_to_base.py" &
      pids+=("$!")
      ;;
    gello)
      python3 "${ADAPTERS_DIR}/gello_to_bridge.py" &
      pids+=("$!")
      ;;
    *)
      echo "Unknown adapter: ${adapter} (expected 'keyboard' or 'gello')" >&2
      ;;
  esac
done

echo "teleop_adapters running: ${SELECTED}"

cleanup() {
  kill "${pids[@]}" 2>/dev/null || true
  wait "${pids[@]}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

wait -n "${pids[@]}"
