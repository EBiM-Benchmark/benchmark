#!/usr/bin/env bash
# Copyright (c) 2026 The EBiM Benchmark Contributors
# SPDX-License-Identifier: Apache-2.0
#
# Download the large Task 3 MuJoCo visual assets that are NOT stored in git.
# Every scene-referenced asset above the repository's 2 MB limit lives here
# (~169 MB raw, ~37 MB zipped). They are hosted on OneDrive as a single zip
# that unpacks into task3_mujoco/ with the correct relative layout:
#
#   assets/robot/<visual meshes>.obj
#   assets/scene_v2/<room meshes>.obj
#   assets/scene_v2/textures/<large textures>.png
#
# Without these files MuJoCo aborts model compilation with
# "Error opening file 'assets/robot/...obj'".
#
# Usage:
#   task3_mujoco/scripts/download_large_assets.sh
#   LARGE_ASSETS_URL="<direct-download-url>" task3_mujoco/scripts/download_large_assets.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TASK3_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

# OneDrive share link for the large-asset zip (override with LARGE_ASSETS_URL).
# Same caveat as task1_isaacsim: OneDrive serves its JavaScript viewer page to
# non-browser clients, so this curl can return 403 even though the link opens
# fine in a browser. That is expected — fall back to the manual download
# documented in ../README.md, or pass LARGE_ASSETS_URL.
DEFAULT_URL="https://1drv.ms/u/c/392ac0752d520bef/IQAwzSbp67qjTbxTc0InA3RFAayQJ7ylctn0G35UtsDAp2c?e=4ANjfe"
URL="${LARGE_ASSETS_URL:-${DEFAULT_URL}}"

# The file list is kept in one place only. run_simulation.py's preflight guard
# reads the same manifest, so the two cannot drift apart.
MANIFEST="${SCRIPT_DIR}/large_assets.txt"
if [[ ! -f "${MANIFEST}" ]]; then
  echo "Missing asset manifest: ${MANIFEST}" >&2
  exit 1
fi

REQUIRED=()
while IFS= read -r line; do
  [[ -n "${line}" ]] && REQUIRED+=("${line}")
done < "${MANIFEST}"

missing() {
  local rel
  for rel in "${REQUIRED[@]}"; do
    [[ -f "${TASK3_ROOT}/${rel}" ]] || printf '%s\n' "${rel}"
  done
}

# --check prints what is missing and exits non-zero; it downloads nothing.
# (No mapfile/readarray here: macOS still ships bash 3.2.)
if [[ "${1:-}" == "--check" ]]; then
  gaps="$(missing)"
  if [[ -z "${gaps}" ]]; then
    echo "All ${#REQUIRED[@]} large Task 3 assets are present."
    exit 0
  fi
  echo "Missing $(printf '%s\n' "${gaps}" | wc -l | tr -d ' ') of ${#REQUIRED[@]} large Task 3 assets:" >&2
  printf '  %s\n' ${gaps} >&2
  echo "Run: task3_mujoco/scripts/download_large_assets.sh" >&2
  exit 1
fi

if [[ -z "$(missing)" ]]; then
  echo "Large assets already present under ${TASK3_ROOT}. Nothing to do."
  exit 0
fi

if [[ "${URL}" == "__ASSET_ZIP_URL__" || -z "${URL}" ]]; then
  cat >&2 <<EOF
No download URL configured.
Set the direct-download link and re-run:
  LARGE_ASSETS_URL="https://…" task3_mujoco/scripts/download_large_assets.sh

The zip must unpack into task3_mujoco/ with this layout:
$(printf '  %s\n' "${REQUIRED[@]}")
EOF
  exit 1
fi

tmp_zip="$(mktemp -t task3_assets.XXXXXX)"
trap 'rm -f "${tmp_zip}"' EXIT

manual_fallback() {
  cat >&2 <<EOF

Automatic download failed. OneDrive share links often cannot be fetched by
curl: the link renders a JavaScript viewer page in a browser, and non-browser
clients get 403 or that HTML page instead of the zip.

Fetch it manually instead — open this link in a browser, download the zip, then
unpack it into task3_mujoco/ (it already has the right internal layout):

  ${URL}

  unzip -o ~/Downloads/task3_mujoco_large_assets.zip -d "${TASK3_ROOT}"

Then confirm with:
  task3_mujoco/scripts/download_large_assets.sh --check

Or point the script at a direct-download URL:
  LARGE_ASSETS_URL="https://…" task3_mujoco/scripts/download_large_assets.sh
EOF
  exit 1
}

echo "Downloading large Task 3 assets from OneDrive..."
# OneDrive share links usually need to be fetched with redirects followed.
curl -fL --retry 3 -o "${tmp_zip}" "${URL}" || manual_fallback

# A OneDrive viewer page returns HTTP 200 with HTML, so a successful curl is
# not proof we got the zip. Verify before unpacking.
if ! unzip -tqq "${tmp_zip}" >/dev/null 2>&1; then
  echo "Downloaded file is not a valid zip (likely a OneDrive HTML page)." >&2
  manual_fallback
fi

echo "Unpacking into ${TASK3_ROOT}..."
unzip -o -q "${tmp_zip}" -d "${TASK3_ROOT}"

gaps="$(missing)"
if [[ -n "${gaps}" ]]; then
  echo "Download finished but these files are still missing:" >&2
  printf '  %s\n' ${gaps} >&2
  exit 1
fi

echo "All ${#REQUIRED[@]} large Task 3 assets are present."
