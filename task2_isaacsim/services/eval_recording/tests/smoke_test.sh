#!/usr/bin/env bash
# Copyright (c) 2026 The EBiM Benchmark Contributors
# SPDX-License-Identifier: Apache-2.0
#
# Isaac-free end-to-end smoke test for the eval audit recorder: runs
# tests/mock_sim.py and a scripted recorder session in the compose
# service, then asserts on the recorded episode artifacts. Requires the
# recorder image (run_eval_recorder.sh build) plus jq and ffprobe on the
# host. Runtime ~90 s.
#
#   task2_isaacsim/services/eval_recording/tests/smoke_test.sh [--keep]
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TASK2_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
RUNNER="${TASK2_ROOT}/scripts/run_eval_recorder.sh"
EVAL_NAME="smoke_$$"
OUT="${TASK2_ROOT}/services/eval_recording/eval_raw_out/${EVAL_NAME}"
CAMERAS="head,wrist_left,wrist_right,eval_camera"
MOCK_NAME="task2_eval_recording_mock_$$"
KEEP=0
[[ "${1:-}" == "--keep" ]] && KEEP=1

fail() { echo "[FAIL] $*" >&2; exit 1; }
pass() { echo "[PASS] $*"; }

command -v jq >/dev/null || fail "jq not found on the host"
command -v ffprobe >/dev/null || fail "ffprobe not found on the host"

export HOST_UID="${HOST_UID:-$(id -u)}"
export HOST_GID="${HOST_GID:-$(id -g)}"

cleanup() {
  docker rm -f "${MOCK_NAME}" >/dev/null 2>&1 || true
  if [[ "${KEEP}" -eq 0 ]]; then
    rm -rf "${OUT}"
  else
    echo "kept: ${OUT}"
  fi
}
trap cleanup EXIT

echo "=== mock sim up (auto-reset every 20 s) ==="
(cd "${TASK2_ROOT}" && docker compose --profile eval_recording run --rm -T \
  --name "${MOCK_NAME}" eval_recording bash -lc \
  "source /opt/ros/jazzy/setup.bash && exec python3 \
   services/eval_recording/tests/mock_sim.py \
   --cameras ${CAMERAS} --reset-every 20" </dev/null) &
sleep 8

echo "=== recorder session (~55 s, piped stdin => line-mode console) ==="
# [e] fires an evaluator call mid-episode; the outcome must be recorded
# either way (evaluator down: transport-failure record; evaluator up:
# real Trigger response + artifact archive — note this also makes the
# live evaluator write a fresh artifact set into its output dir).
{ sleep 30; echo e; sleep 25; echo q; } | \
  "${RUNNER}" record --eval-name "${EVAL_NAME}"

echo "=== assertions ==="
[[ -f "${OUT}/.session_provenance.json" ]] \
  || fail "missing .session_provenance.json"
[[ "$(jq -r .bench_head "${OUT}/.session_provenance.json" | wc -c)" -gt 10 ]] \
  || fail "provenance bench_head empty"
pass "session provenance captured"

[[ -f "${OUT}/.status.json" ]] || fail "missing .status.json"
jq -e '.state' "${OUT}/.status.json" >/dev/null || fail ".status.json bad"
pass ".status.json written"

ep_count=$(find "${OUT}" -maxdepth 1 -type d -name 'ep_*' | wc -l)
[[ "${ep_count}" -ge 2 ]] || fail "expected >=2 episodes, got ${ep_count}"
pass "${ep_count} episodes recorded"

manifest="${OUT}/ep_001/manifest.json"
[[ -f "${manifest}" ]] || fail "missing ${manifest}"
jq -e '.clean_shutdown == true' "${manifest}" >/dev/null \
  || fail "ep_001 not cleanly closed"
jq -e --arg n "${EVAL_NAME}" '.eval_name == $n' "${manifest}" >/dev/null \
  || fail "manifest eval_name mismatch"
jq -e '.started_by == "scene_reset"' "${manifest}" >/dev/null \
  || fail "ep_001 not started by scene_reset"
for cam in head wrist_left wrist_right eval_camera; do
  jq -e --arg c "${cam}" \
    '.cameras[$c].status == "recorded" and .cameras[$c].encoded > 0' \
    "${manifest}" >/dev/null || fail "ep_001 camera ${cam} not recorded"
done
pass "ep_001 manifest complete (4 cameras recorded)"

[[ -f "${OUT}/ep_001/provenance.json" ]] \
  || fail "ep_001 missing provenance.json"
pass "provenance copied into ep_001"

for cam in head wrist_left wrist_right eval_camera; do
  mp4="${OUT}/ep_001/videos/${cam}.mp4"
  codec=$(ffprobe -v error -select_streams v:0 -show_entries \
    stream=codec_name -of default=nw=1:nk=1 "${mp4}")
  [[ "${codec}" == "h264" ]] || fail "${cam}.mp4 codec ${codec} != h264"
  frames=$(ffprobe -v error -count_frames -select_streams v:0 \
    -show_entries stream=nb_read_frames -of default=nw=1:nk=1 "${mp4}")
  [[ "${frames}" -gt 0 ]] || fail "${cam}.mp4 has no decodable frames"
done
pass "all 4 MP4s decode (h264)"

jq -s 'map(.pts_ms) as $p
       | (($p == ($p | sort)) and (($p | unique | length) == ($p | length)))
       | if . then empty else error("pts not strictly monotonic") end' \
  "${OUT}/ep_001/frames/head.jsonl" || fail "head.jsonl PTS not monotonic"
frame_lines=$(jq -s 'length' "${OUT}/ep_001/frames/head.jsonl")
encoded=$(jq -r '.cameras.head.encoded' "${manifest}")
[[ "${frame_lines}" == "${encoded}" ]] \
  || fail "head frames JSONL ${frame_lines} != manifest encoded ${encoded}"
pass "frame index monotonic and consistent (${encoded} frames)"

for dump in object_poses joint_states; do
  [[ -s "${OUT}/ep_001/dumps/${dump}.jsonl" ]] \
    || fail "ep_001 dump ${dump}.jsonl missing/empty"
done
pass "ground-truth dumps present"

calls=$(find "${OUT}" -maxdepth 3 -name calls.jsonl | head -1)
[[ -n "${calls}" ]] || fail "no evaluator calls.jsonl (the [e] key path)"
jq -e 'has("success")' "${calls}" >/dev/null \
  || fail "evaluator call record malformed"
result="$(dirname "${calls}")/call_001/evaluator_result.json"
[[ -f "${result}" ]] || fail "missing ${result}"
pass "evaluator call recorded (success=$(jq -c .success "${calls}"))"

echo
pass "smoke test complete"
