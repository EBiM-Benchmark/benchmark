#!/usr/bin/env bash
# Copyright (c) 2026 The EBiM Benchmark Contributors
# SPDX-License-Identifier: Apache-2.0
#
# Download the large Task 1 USD assets that are NOT stored in git (too large for
# the repo / Git LFS). They are hosted on OneDrive as a single zip that unpacks
# into task1_isaacsim/ with the correct relative layout:
#
#   assets/Robotiq_2f_85_with_d405_mobile_fr3_duo_v0_2.usd
#   cable_world/assets/table_board_fixture/Assets/board_segment.usd
#   cable_world/assets/table_board_fixture/Assets/board_segment_upper_right.usd
#
# Usage:
#   task1_isaacsim/scripts/download_large_assets.sh
#   LARGE_ASSETS_URL="<direct-download-url>" task1_isaacsim/scripts/download_large_assets.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TASK1_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

# OneDrive share link for the large-asset zip (override with LARGE_ASSETS_URL).
DEFAULT_URL="https://1drv.ms/u/c/392ac0752d520bef/IQDt8KPG0OVdSKDd6i1Ysxg3AS_LJuxk-UDQfjzcrhGveFw?e=w9rIB6"
URL="${LARGE_ASSETS_URL:-${DEFAULT_URL}}"

REQUIRED=(
  "assets/Robotiq_2f_85_with_d405_mobile_fr3_duo_v0_2.usd"
  "cable_world/assets/table_board_fixture/Assets/board_segment.usd"
  "cable_world/assets/table_board_fixture/Assets/board_segment_upper_right.usd"
)

all_present() {
  for rel in "${REQUIRED[@]}"; do
    [[ -f "${TASK1_ROOT}/${rel}" ]] || return 1
  done
  return 0
}

if all_present; then
  echo "Large assets already present under ${TASK1_ROOT}. Nothing to do."
  exit 0
fi

if [[ "${URL}" == "__ONEDRIVE_URL__" || -z "${URL}" ]]; then
  cat >&2 <<EOF
No download URL configured.
Set the OneDrive direct-download link and re-run:
  LARGE_ASSETS_URL="https://…" task1_isaacsim/scripts/download_large_assets.sh

The zip must unpack into task1_isaacsim/ with this layout:
$(printf '  %s\n' "${REQUIRED[@]}")
EOF
  exit 1
fi

tmp_zip="$(mktemp --suffix=.zip)"
trap 'rm -f "${tmp_zip}"' EXIT

echo "Downloading large assets from OneDrive..."
# OneDrive share links usually need to be fetched with redirects followed.
curl -fL --retry 3 -o "${tmp_zip}" "${URL}"

echo "Extracting into ${TASK1_ROOT} ..."
unzip -o -q "${tmp_zip}" -d "${TASK1_ROOT}"

if all_present; then
  echo "Done. All large assets are in place:"
  for rel in "${REQUIRED[@]}"; do
    printf '  %s (%s)\n' "${rel}" "$(du -h "${TASK1_ROOT}/${rel}" | cut -f1)"
  done
else
  echo "ERROR: some assets are still missing after extraction:" >&2
  for rel in "${REQUIRED[@]}"; do
    [[ -f "${TASK1_ROOT}/${rel}" ]] || echo "  MISSING: ${rel}" >&2
  done
  echo "Check that the zip's internal paths match the layout above." >&2
  exit 1
fi
