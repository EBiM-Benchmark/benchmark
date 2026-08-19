# Copyright (c) 2026 The EBiM Benchmark Contributors
# SPDX-License-Identifier: Apache-2.0
#
# Eval audit recorder image. Code is NOT baked in — the container runs
# record_eval_task2.py from the /repo bind mount (same convention as
# services/recording), so a rebuild is only needed for dependency changes.
FROM ros:jazzy-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
        ffmpeg \
        python3-pip \
        python3-numpy \
        python3-yaml \
        ros-jazzy-rosgraph-msgs \
    && rm -rf /var/lib/apt/lists/*

# PyAV bundles its own FFmpeg; assert at build time that it ships libx264
# (fallbacks if this ever fails: vcodec libopenh264/libsvtav1, or pipe
# rawvideo to the apt ffmpeg binary).
RUN pip3 install --no-cache-dir --break-system-packages av \
    && python3 -c "import av; assert 'libx264' in av.codecs_available, sorted(av.codecs_available)"

CMD ["bash"]
