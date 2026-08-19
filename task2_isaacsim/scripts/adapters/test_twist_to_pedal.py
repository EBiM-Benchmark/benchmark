# Copyright (c) 2026 The EBiM Benchmark Contributors
# SPDX-License-Identifier: Apache-2.0
"""ROS-free unit tests for the twist -> pedal-token quantizer.

Run from this directory (pytest adds it to sys.path):

    pytest test_twist_to_pedal.py

The module under test must stay importable without rclpy so these tests
run on a host without ROS 2.
"""

import pytest
from twist_to_pedal import pedal_token


class TestPedalToken:
    THRESHOLD = 0.25

    @pytest.mark.parametrize(
        ("twist", "token"),
        [
            ((0.5, 0.0, 0.0), "FWD"),
            ((-0.5, 0.0, 0.0), "BACK"),
            ((0.0, 0.5, 0.0), "A"),
            ((0.0, -0.5, 0.0), "B"),
            ((0.0, 0.0, 1.2), "A+C"),
            ((0.0, 0.0, -1.2), "B+C"),
            ((0.0, 0.0, 0.0), "NONE"),
            ((0.1, -0.05, 0.2), "NONE"),  # all under dead-band
            ((0.5, 0.3, 0.0), "FWD"),  # dominant axis wins
            ((0.3, 0.0, 1.2), "A+C"),  # full-rate yaw beats partial strafe
            # Exact strafe/yaw tie: earlier axis (strafe) wins.
            ((0.0, 0.5, 1.2), "A"),
        ],
    )
    def test_cases(self, twist, token):
        assert pedal_token(*twist, self.THRESHOLD) == token

    def test_threshold_is_exclusive(self):
        # A magnitude exactly at the dead-band still counts as motion.
        assert pedal_token(0.25, 0.0, 0.0, 0.25) == "FWD"
        assert pedal_token(0.2499, 0.0, 0.0, 0.25) == "NONE"
