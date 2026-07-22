"""All model names and tunable constants, grouped by topic.

Every magic number of the teleop lives here so behavior tuning never requires
touching control logic. A few values are OVERWRITTEN at runtime (marked
"mutable"): access those as ``config.NAME`` at call time — importing the bare
name would freeze the import-time value.
"""

from __future__ import annotations

import math

import numpy as np

# --------------------------------------------------------------------------
# model names (must match duo_full_scene_grasp.xml)
# --------------------------------------------------------------------------
ARM_SPECS = {
    "left": {
        "joints": [f"left_fr3v2_1_joint{i}" for i in range(1, 8)],
        "tcp": "left_fr3v2_1_robotiq_arg85_tcp",
        "target": "left_tcp_mocap_target",
        "gripper": "left_fr3v2_1_robotiq_85_left_knuckle_joint",
        "pad_left": "left_pad_left_geom",
        "pad_right": "left_pad_right_geom",
        "pad_left_prefix": "left_pad_left",
        "pad_right_prefix": "left_pad_right",
        # When operating from robot front, lateral input feels mirrored on left arm.
        "lateral_sign": -1.0,
    },
    "right": {
        "joints": [f"right_fr3v2_1_joint{i}" for i in range(1, 8)],
        "tcp": "right_fr3v2_1_robotiq_arg85_tcp",
        "target": "right_tcp_mocap_target",
        "gripper": "right_fr3v2_1_robotiq_85_left_knuckle_joint",
        "pad_left": "pad_left_geom",
        "pad_right": "pad_right_geom",
        "pad_left_prefix": "pad_left",
        "pad_right_prefix": "pad_right",
        "lateral_sign": 1.0,
    },
}
# body-name prefix of each gripper subtree ("robotiq" bodies under this arm);
# used to collect every collidable gripper geom for haptic feedback
GRIPPER_BODY_PREFIX = {"left": "left_fr3v2_1", "right": "right_fr3v2_1"}

BASE_BODY = "base_link"
BASE_X_JOINT = "base_planar_x"
BASE_Y_JOINT = "base_planar_y"
BASE_YAW_JOINT = "base_yaw"
SPINE_ACT = "franka_spine_vertical_joint"
BASE_STEER_FRONT_ACT = "steer_front"
BASE_STEER_REAR_ACT = "steer_rear"
BASE_DRIVE_FRONT_ACT = "drive_front_roll"
BASE_DRIVE_REAR_ACT = "drive_rear_roll"
BASE_CASTER_FRONT_STEER_ACT = "caster_front_left_steer"
BASE_CASTER_FRONT_ROLL_ACT = "caster_front_left_roll"
BASE_CASTER_REAR_STEER_ACT = "caster_rear_right_steer"
BASE_CASTER_REAR_ROLL_ACT = "caster_rear_right_roll"
BASE_VEL_X_ACT = "base_planar_x_velocity"
BASE_VEL_Y_ACT = "base_planar_y_velocity"
BASE_VEL_YAW_ACT = "base_yaw_velocity"
CLIP_BODY = "cclip_0"

# startup pose: both arms reaching forward-and-down at table height (gripper
# vertical) instead of the folded transport pose. The left values are the
# scene's own home keyframe; the right arm is its mirror (odd joints negated).
ARM_READY_QPOS = {
    "left": np.array([1.208, -0.152, 0.591, -2.447, 1.099, 3.07, -0.573]),
    "right": np.array([-1.208, -0.152, -0.591, -2.447, -1.099, 3.07, 0.573]),
}

# --------------------------------------------------------------------------
# loop / speed
# --------------------------------------------------------------------------
LOOP_HZ = 500.0
RENDER_HZ = 60.0
MOVE_SPEED = 4.0  # mutable: --move-speed
ROT_SPEED = math.radians(540.0)  # mutable: --rot-speed-deg
# while an arm holds the cable its commanded translation runs at this
# fraction of normal speed (release restores 1x automatically).
# 0.25 was tried on operator request 2026-07-14 and rolled back the same
# day (handling felt off); back at the long-tested 0.45.
# NOTE: in VR this multiplies a position-servo twist, so it acts as a gain
# cut (more tracking lag), not a clean speed scale. A "full gain + 0.30 m/s
# speed cap" replacement was tried 2026-07-14 and REJECTED by measurement:
# the stiff servo tore the cable off the pads in 4/4 grasped-drag trials
# (0.15-0.30 m/s hand speed, all grasp depths) while the gain cut held 4/4 -
# the gain cut IS the retention mechanism. Do not retry without new data.
GRASPED_SPEED_SCALE = 0.45
GRASPED_TWIST_FILTER_TAU = 0.055
GRASPED_BASE_SPEED_SCALE = 0.18
GRASPED_BASE_FILTER_TAU = 0.10
GAMEPAD_DEAD = 0.14
TRIGGER_DEAD = 0.08
TWIST_DEAD = 1e-5
# The passive viewer often never delivers key-release events. On Windows the
# OS key state is polled instead; elsewhere a held key expires this many
# seconds after its last press/repeat event so opposite keys cannot get stuck
# pressed forever and cancel each other out.
KEY_HOLD_TIMEOUT = 0.40

# --------------------------------------------------------------------------
# IK
# --------------------------------------------------------------------------
IK_POS_GAIN = 18.0
IK_ROT_GAIN = 10.0
IK_DAMPING = 0.07
JOINT_VEL_LIMIT = 120.0  # mutable: --joint-vel-limit
ORIENTATION_LOCK_GAIN = 0.0  # mutable: --ori-lock-gain
ORIENTATION_LOCK_MAX = math.radians(180.0)
CTRL_LEAD_LIMIT = 2.20

# --------------------------------------------------------------------------
# gripper / grasp
# --------------------------------------------------------------------------
GRIPPER_OPEN = 0.0
GRIPPER_CLOSE = 0.8
GRIPPER_CLOSE_RATE = 0.38
GRIPPER_OVERDRIVE = 0.002
GRIPPER_CONTACT_SIDE_FORCE = 0.03
# after both pads touch, keep squeezing (at half rate) until GRIPPER_FORCE_STOP:
# a diagonal cable can graze both pads while the fingers are still wide open,
# so stopping at first touch leaves the fingers visually agape and the grip weak
GRIPPER_FORCE_STOP = 14.0
SPINE_SPEED = 0.32
CABLE_BOARD_Z = 0.0075

# TCP motion clamps, expressed as max travel per PHYSICS step so cable
# contacts survive; wall-clock speed = step limit x loop rate.
FREE_MAX_TCP_STEP = 0.0040  # mutable: VR raises it (see run_vr)
GRASPED_MAX_TCP_STEP = 0.0013  # mutable: VR speed levels rescale it
MAX_ROT_STEP = math.radians(9.0)
Z_LIFT_XY_HOLD_GAIN = 45.0
Z_LIFT_XY_HOLD_MAX_SPEED = 0.08

GRASP_ASSIST_KP = 900.0
GRASP_ASSIST_KD = 18.0
# keep the assist weaker than a firm snag so a stuck cable slips out of the
# grasp (real-cable behavior) instead of storing energy and catapulting, but
# strong enough to drag the cable across rubbing pegs without shedding the
# grasp; genuine escapes are caught by the pad no-contact release below, so
# the distance threshold can stay loose
GRASP_ASSIST_MAX_FORCE = 26.0
GRASP_ASSIST_RELEASE_DIST = 0.08
# drop the assist once the pads have lost the cable for this long — otherwise
# an escaped cable keeps being dragged along below the gripper
# (2026-07-12: a "time AND distance" variant plus a speed-gated transport
# grip boost were both tried and user-rejected — combined feel was WORSE
# than this baseline; do not retry without measurements first)
GRASP_NOCONTACT_RELEASE_TIME = 0.3
# ...but drop instantly when the cable has ESCAPED at speed: an attached
# assist spring whipping a free segment through the grace period is where
# most of the catapult violence comes from (median 25.8 -> 9.2 m/s across
# wedge depths with this alone). Threshold sits well above the measured
# ~1.2 m/s standing oscillation of a held cable (so transport flicker
# cannot mis-trigger) and well below ejection speeds (3.6+ m/s)
GRASP_ESCAPE_SPEED = 2.5
# ...AND only once the segment is genuinely outside the gripper: seating a
# fresh grasp and dragging over pegs produce brief contact-flicker + speed
# transients right AT the slot, and releasing on those made grasping feel
# unreliable (user-reported). A real ejection blows past this radius within
# ~10 ms, so the whip is still cut ~30x sooner than the 0.3 s grace period.
GRASP_ESCAPE_DIST = 0.04

# NOTE a "post-grasp secure lift" (auto-raising the gripper a few mm after
# the squeeze) was implemented three ways and REMOVED after 20-trial
# statistics: it sometimes helped deep board-pinches but made shallow ones
# WORSE (its own motion injects energy) - net effect indistinguishable from
# noise. The instant escape release above is the change that actually
# collapsed the catapult median (25.8 -> 9.2 m/s across wedge depths).
GRASP_ASSIST_START_DELAY = 0.30
GRASP_ASSIST_RAMP_TIME = 0.55
GRASP_ASSIST_SLOT_VEL_LIMIT = 0.65

# --------------------------------------------------------------------------
# Cable ballistic safety valve: whip-crack dynamics multiply the pushed
# section's speed several-fold at the free end — measured 25-30 m/s tips
# from an 8 m/s fling — and at those speeds a segment crosses several cm
# per step, deep enough into the table plates that the contact normal
# flips sideways and the chain threads through the solid. Scale the whole
# cable's qvel down when any segment's LINEAR speed passes this cap.
# NOT the reverted angular limiter (50 rad/s bit into normal kink motion
# at ~38 rad/s and felt sticky): normal manipulation moves the cable at
# well under 1 m/s, ~12x below this trigger, so feel is untouched — the
# valve only fires during blow-ups.
# --------------------------------------------------------------------------
CABLE_LINVEL_MAX = 12.0

# --------------------------------------------------------------------------
# C-clip retention: segments inside the pocket get a capped spring pull
# toward the slot centerline (y/z only, never along the slot axis), so the
# cable slides freely along/through the clip but resists popping out of the
# open mouth; pulling harder than the cap simply extracts it (slip, no
# stored-energy catapult). The hook/backwall geometry blocks up and +y.
# --------------------------------------------------------------------------
CLIP_HOLD_FORCE = 8.0
# stiff enough to reach the force cap well within the pocket half-width
# (~6mm), otherwise the effective retention is far below CLIP_HOLD_FORCE
CLIP_GUIDE_KP = 2500.0
CLIP_GUIDE_KD = 12.0
CLIP_ZONE_LO = np.array([-0.0085, 0.0084, -0.0019])
CLIP_ZONE_HI = np.array([0.0215, 0.0213, 0.0091])
CLIP_SEAT_LOCAL = np.array([0.0149, 0.0031])  # y, z of the pocket centerline

# --------------------------------------------------------------------------
# haptics (gamepad rumble + VR controller vibration)
# --------------------------------------------------------------------------
HAPTIC_MIN_FORCE = 0.2  # ignore contact chatter below this (N)
HAPTIC_FULL_FORCE = 8.0  # gripper contact force (N) that maps to full rumble
# feedback is one short pulse when a gripper FIRST touches something, not a
# continuous buzz while the contact persists; re-armed only after the gripper
# has been contact-free for HAPTIC_REARM_S (debounces chattering contacts)
HAPTIC_PULSE_MS = 150  # length of the new-contact pulse
HAPTIC_PULSE_MIN_AMP = 0.4  # amplitude floor so light touches are still felt
HAPTIC_REARM_S = 0.25  # contact-free time before the next touch pulses again

# --------------------------------------------------------------------------
# VR
# --------------------------------------------------------------------------
VR_POS_GAIN = 14.0
VR_ROT_GAIN = 8.0
# hysteresis: engage above, release below — a half-squeezed grip hovering at
# a single threshold makes the clutch flutter mid-motion
VR_GRIP_ENGAGE = 0.6
VR_GRIP_RELEASE = 0.35
VR_TRIGGER_CLOSE = 0.6
VR_STICK_DEAD = 0.15
HMD_VIEW_SIZE = (960, 540)
# hand-tracking speed: the default free clamp feels glacial against a real
# hand, so VR mode loosens it (still gentle while grasping — faster tears the
# cable off the pads or catapults it off the pegs)
VR_FREE_MAX_TCP_STEP = 0.010
VR_GRASPED_MAX_TCP_STEP = 0.0013
# on-the-fly speed setting: click LEFT stick = faster, RIGHT stick = slower
VR_SPEED_LEVELS = (0.4, 0.6, 0.8, 1.0, 1.25, 1.6, 2.0)
# grasped motion stays capped: beyond ~1.5x it rips the cable loose
VR_GRASPED_SPEED_LEVEL_CAP = 1.5

# --------------------------------------------------------------------------
# GELLO (Franka GELLO Duo, official EBiM competition input device)
#
# ROS 2 topic contract from the reference franka_gello_state_publisher
# (github.com/EBiM-Benchmark/teleoperation, per-arm namespace from its
# launch config, e.g. "left"/"right"):
#   <ns>/gello/joint_states                                sensor_msgs/JointState
#     .position: 7 floats, fr3_joint1..7, radians, ALREADY assembly-offset-
#     corrected, sign-corrected and clamped to the real FR3 joint limits by
#     the publisher — usable directly as position targets, no IK needed
#   <ns>/gripper/gripper_client/target_gripper_width_percent   std_msgs/Float32
#     0.0..1.0 fraction despite the "percent" name (GelloHardware.
#     process_gripper_position clips to [0, 1]); assumed 1.0 = fully open,
#     0.0 = fully closed by the "width" naming — NOT verified against real
#     hardware yet.
# --------------------------------------------------------------------------
GELLO_NAMESPACES = {"left": "left", "right": "right"}
GELLO_JOINT_STATES_TOPIC = "gello/joint_states"
GELLO_GRIPPER_TOPIC = "gripper/gripper_client/target_gripper_width_percent"
# per-joint P-controller driving the velocity actuators toward GELLO's
# target angle (GELLO gives absolute joint space, so no Jacobian/IK step)
GELLO_JOINT_KP = 8.0
# gripper fraction thresholds to trigger the same force-servo close_ramp
# used by every other input mode (keeps grasp/cable-retention physics
# identical regardless of input device); hysteresis mirrors VR_GRIP_*
GELLO_GRIPPER_CLOSE_BELOW = 0.5
GELLO_GRIPPER_OPEN_ABOVE = 0.6
# treat GELLO data older than this as stale (publisher stopped / disconnected)
GELLO_DATA_TIMEOUT = 0.5

# USB foot pedal (reference repo's pedal_state_publisher): publishes the
# combined pressed state as a plain string on /pedal/state - one of
# "A", "B", "C", "A+C", "B+C", "NONE" (A/B are mutually exclusive at the
# hardware level, C only ever arrives combined with whichever of A/B was
# already held, so those six are the only reachable states). In the GELLO
# workflow the pedal drives the MOBILE BASE (GELLO occupies both hands).
# The state->motion mapping below matches the reference repo's own
# pedal_state_subscriber.py example (STATE_TO_ACTION: A=forward,
# B=turn left, A+C=backward, B+C=turn right; C alone and NONE are left
# unmapped there, so both go to no-motion here); tuples are
# (local_x, local_y, spine, yaw) in the same convention BaseDriver.drive
# takes. +yaw = turn right, matching the keyboard End key.
PEDAL_STATE_TOPIC = "/pedal/state"
PEDAL_BASE_COMMANDS = {
    "A": (1.0, 0.0, 0.0, 0.0),  # forward
    "B": (0.0, 0.0, 0.0, -1.0),  # turn left
    "A+C": (-1.0, 0.0, 0.0, 0.0),  # backward
    "B+C": (0.0, 0.0, 0.0, +1.0),  # turn right
    "C": (0.0, 0.0, 0.0, 0.0),  # unmapped in the reference example
    "NONE": (0.0, 0.0, 0.0, 0.0),
}
PEDAL_DATA_TIMEOUT = 0.5

# --------------------------------------------------------------------------
# Unified ROS 2 teleop input (--input ros_teleop): a standalone "Teleop
# Node" (keyboard/gamepad/VR, run separately from the sim - see
# teleop_state_publisher, meant for contribution to
# github.com/EBiM-Benchmark/teleoperation) publishes device-agnostic
# commands; this sim only ever consumes them, so its feel is identical to
# driving keyboard/gamepad/VR locally - IK/grasp/base-drive code is IDENTICAL
# either way, only the source of twist_cmd/base_cmd/gripper_cmd changes.
#
# GELLO deliberately stays OUT of this contract (see GELLO_* above): it is
# joint-space and IK-free by nature, this contract is Cartesian and always
# goes through the same apply_twist_ik/BaseDriver.drive path real devices do.
#
# /cmd_vel                      geometry_msgs/Twist   base, REP-103 base_link
#                                frame (x=forward, y=left, angular.z=yaw);
#                                linear.z is repurposed for the spine
#                                (vertical lift) rate - no standard field for
#                                it, but Twist has the unused DOF and this
#                                matches the existing EBiM_Challenge test
#                                commands for /cmd_vel exactly otherwise
# <side>/teleop_cmd              geometry_msgs/Twist   per-arm TCP twist
#                                (linear/angular m/s, rad/s) - same units and
#                                frame our internal twist_cmd already uses
# <side>/gripper_cmd             std_msgs/Float32      0=open request,
#                                1=close request (edge-triggered, matches the
#                                GELLO gripper contract's style but is a
#                                simple binary intent here, not a continuous
#                                width)
# --------------------------------------------------------------------------
ROS_TELEOP_NAMESPACES = {"left": "left", "right": "right"}
ROS_TELEOP_CMD_VEL_TOPIC = "/cmd_vel"
ROS_TELEOP_ARM_TOPIC = "teleop_cmd"
ROS_TELEOP_GRIPPER_TOPIC = "gripper_cmd"
# treat a topic as inactive (hold position / stop base) after this long
# without a new message - protects against a crashed/disconnected publisher
# leaving the last nonzero command latched forever
ROS_TELEOP_DATA_TIMEOUT = 0.3
ROS_TELEOP_GRIPPER_CLOSE_ABOVE = 0.5
# frame feedback published BY the sim FOR screen-relative device mapping:
# std_msgs/Float32MultiArray [viewer_cam_azimuth_deg, robot_yaw_rad].
# Publishers that map device axes to what the operator SEES (keyboard
# arrows, VR mirror teleop) subscribe to this; publishers that are
# frame-free (gamepad robot-frame sticks, pedal) ignore it. Carries NO
# control authority - all IK/physics stay in the sim.
ROS_TELEOP_FEEDBACK_TOPIC = "/mujoco/teleop_feedback"
ROS_TELEOP_FEEDBACK_HZ = 30.0
# <side>/vr_hand                 std_msgs/Float32MultiArray, 15 floats:
#   [0] valid  [1:4] pos xyz (m, VR standing space)  [4:8] quat wxyz
#   [8] grip  [9] trigger  [10] a  [11] b  [12:14] stick xy  [14] stick_click
# RAW controller state on purpose: the clutch anchor, VR->screen mapping,
# hand->arm mirroring (--facing), servo gains and gripper edge logic all run
# in the sim (run_ros_teleop), through the exact same code as local VR mode
# (run_vr) - so the feel is identical and the publisher stays a dumb device
# reader like the keyboard/gamepad ones. Namespaces name the HAND here (the
# sim decides which arm a hand drives), unlike teleop_cmd's arm namespaces.
ROS_TELEOP_VR_HAND_TOPIC = "vr_hand"
ROS_TELEOP_VR_HAND_LEN = 15
