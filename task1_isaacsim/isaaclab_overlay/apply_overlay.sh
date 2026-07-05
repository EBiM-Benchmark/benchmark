#!/usr/bin/env bash
# Copyright (c) 2026 The EBiM Benchmark Contributors
# SPDX-License-Identifier: Apache-2.0
#
# Apply the ros2_jazzy docker overlay to a Newton-enabled Isaac Lab checkout so
# it can run the Task 1 teleoperation bridge. This:
#   * copies Dockerfile.ros2_jazzy and .env.ros2_jazzy into <isaaclab>/docker/
#   * patches docker-compose.yaml / x11.yaml / x11_utils.py / .env.base to add the
#     `isaac-lab-ros2_jazzy` service and bind-mount this repo at
#     /workspace/EBiM_Challenge inside the container.
#
# Usage:
#   task1_isaacsim/isaaclab_overlay/apply_overlay.sh [ISAACLAB_ROOT] [EBIM_CHALLENGE_ROOT]
#
# Defaults: ISAACLAB_ROOT      = sibling ../IsaacLab of this repo
#           EBIM_CHALLENGE_ROOT = this repository root
set -euo pipefail

OVERLAY_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${OVERLAY_DIR}/../.." && pwd)"

ISAACLAB_ROOT="${1:-$(cd "${REPO_ROOT}/.." && pwd)/IsaacLab}"
EBIM_CHALLENGE_ROOT="${2:-${REPO_ROOT}}"
PINNED_COMMIT="0916ea3c0f126821ef1783c7119d248834fc8d0b"  # release/3.0.0-beta2

if [[ ! -d "${ISAACLAB_ROOT}/docker" ]]; then
  echo "Isaac Lab checkout not found at: ${ISAACLAB_ROOT}" >&2
  echo "Pass the checkout path as the first argument." >&2
  exit 1
fi

echo "Isaac Lab checkout: ${ISAACLAB_ROOT}"
echo "EBiM_Challenge root: ${EBIM_CHALLENGE_ROOT}"

# Warn (do not fail) if not on the pinned commit.
if git -C "${ISAACLAB_ROOT}" rev-parse HEAD >/dev/null 2>&1; then
  head_commit="$(git -C "${ISAACLAB_ROOT}" rev-parse HEAD)"
  if [[ "${head_commit}" != "${PINNED_COMMIT}" ]]; then
    echo "WARNING: Isaac Lab HEAD is ${head_commit}," >&2
    echo "         overlay was captured at ${PINNED_COMMIT} (release/3.0.0-beta2)." >&2
    echo "         If 'git apply' fails, check out the pinned commit first." >&2
  fi
fi

# 1) Copy the new overlay files.
cp "${OVERLAY_DIR}/Dockerfile.ros2_jazzy" "${ISAACLAB_ROOT}/docker/Dockerfile.ros2_jazzy"
cp "${OVERLAY_DIR}/.env.ros2_jazzy" "${ISAACLAB_ROOT}/docker/.env.ros2_jazzy"
echo "Copied Dockerfile.ros2_jazzy and .env.ros2_jazzy into ${ISAACLAB_ROOT}/docker/"

# 2) Substitute the repo path into the patch and apply it.
tmp_patch="$(mktemp)"
trap 'rm -f "${tmp_patch}"' EXIT
sed "s#__EBIM_CHALLENGE_ROOT__#${EBIM_CHALLENGE_ROOT}#g" \
  "${OVERLAY_DIR}/ros2_jazzy_overlay.patch" > "${tmp_patch}"

if git -C "${ISAACLAB_ROOT}" apply --check "${tmp_patch}" 2>/dev/null; then
  git -C "${ISAACLAB_ROOT}" apply "${tmp_patch}"
  echo "Applied ros2_jazzy_overlay.patch."
elif git -C "${ISAACLAB_ROOT}" apply --reverse --check "${tmp_patch}" 2>/dev/null; then
  echo "Overlay already applied (patch reverses cleanly); nothing to do."
else
  cat >&2 <<EOF
Could not apply ros2_jazzy_overlay.patch to ${ISAACLAB_ROOT}.
This usually means the checkout is not at the pinned commit ${PINNED_COMMIT}.
Fix with:
  git -C "${ISAACLAB_ROOT}" checkout ${PINNED_COMMIT}
then re-run this script, or apply the changes manually (see task1_isaacsim/isaaclab_overlay/README.md).
EOF
  exit 1
fi

echo ""
echo "Done. Next:"
echo "  cd ${ISAACLAB_ROOT} && ./docker/container.py start ros2_jazzy"
