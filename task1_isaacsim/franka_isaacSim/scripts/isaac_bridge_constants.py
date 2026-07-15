"""Shared constants for Isaac joint bridge."""

import os

# Default configuration
CONFIG = {
    "headless": False,
    "width": 1280,
    "height": 720,
    "sync_loads": True,
    # Keep an initial stage alive during startup so UI/collection watchers
    # don't attach against an invalid stage id (-1) in streaming mode.
    "create_new_stage": False,
}

LEFT_JOINTS = [
    "left_fr3v2_joint1",
    "left_fr3v2_joint2",
    "left_fr3v2_joint3",
    "left_fr3v2_joint4",
    "left_fr3v2_joint5",
    "left_fr3v2_joint6",
    "left_fr3v2_joint7",
]

RIGHT_JOINTS = [
    "right_fr3v2_joint1",
    "right_fr3v2_joint2",
    "right_fr3v2_joint3",
    "right_fr3v2_joint4",
    "right_fr3v2_joint5",
    "right_fr3v2_joint6",
    "right_fr3v2_joint7",
]

LEFT_GRIPPER_JOINTS = [
    "left_robotiq_85_left_knuckle_joint",
    "left_robotiq_85_right_knuckle_joint",
    "left_robotiq_85_left_inner_knuckle_joint",
    "left_robotiq_85_right_inner_knuckle_joint",
    "left_robotiq_85_left_finger_tip_joint",
    "left_robotiq_85_right_finger_tip_joint",
]

RIGHT_GRIPPER_JOINTS = [
    "right_robotiq_85_left_knuckle_joint",
    "right_robotiq_85_right_knuckle_joint",
    "right_robotiq_85_left_inner_knuckle_joint",
    "right_robotiq_85_right_inner_knuckle_joint",
    "right_robotiq_85_left_finger_tip_joint",
    "right_robotiq_85_right_finger_tip_joint",
]

LEFT_GRIPPER_DRIVER_JOINT = "left_right_finger_joint"
RIGHT_GRIPPER_DRIVER_JOINT = "right_right_finger_joint"

LEFT_GRIPPER_COUPLED_JOINT_MULTIPLIERS = {
    "left_robotiq_85_left_knuckle_joint": 1.0,
    "left_robotiq_85_right_knuckle_joint": -1.0,
    "left_robotiq_85_left_inner_knuckle_joint": 1.0,
    "left_robotiq_85_right_inner_knuckle_joint": -1.0,
    "left_robotiq_85_left_finger_tip_joint": -1.0,
    "left_robotiq_85_right_finger_tip_joint": 1.0,
}

RIGHT_GRIPPER_COUPLED_JOINT_MULTIPLIERS = {
    "right_robotiq_85_left_knuckle_joint": 1.0,
    "right_robotiq_85_right_knuckle_joint": -1.0,
    "right_robotiq_85_left_inner_knuckle_joint": 1.0,
    "right_robotiq_85_right_inner_knuckle_joint": -1.0,
    "right_robotiq_85_left_finger_tip_joint": -1.0,
    "right_robotiq_85_right_finger_tip_joint": 1.0,
}

DEFAULT_PRIMARY_CONTROLLER_NAME = ""


def build_joint_groups(primary_controller_name=DEFAULT_PRIMARY_CONTROLLER_NAME):
    """Return JOINT_GROUPS with the given primary controller name.

    Pass *None* or an empty string to disable primary-controller gating for
    arm groups (position-passthrough / position-controller mode).  Gripper
    groups never require a primary controller.
    """
    arm_controller = primary_controller_name or None
    return [
        {
            "label": "Left Arm",
            "state_topic": "/isaac/left_joint_states",
            "command_topic": "/isaac/left_joint_commands",
            "browser_command_topic": "/isaac/browser/left_joint_commands",
            "required_primary_controller": arm_controller,
            "default_joints": LEFT_JOINTS,
            "wrench_topic": "/isaac/left_ee_wrench",
        },
        {
            "label": "Right Arm",
            "state_topic": "/isaac/right_joint_states",
            "command_topic": "/isaac/right_joint_commands",
            "browser_command_topic": "/isaac/browser/right_joint_commands",
            "required_primary_controller": arm_controller,
            "default_joints": RIGHT_JOINTS,
            "wrench_topic": "/isaac/right_ee_wrench",
        },
        {
            "label": "Left Robotiq",
            "state_topic": "/isaac/left_robotiq_joint_states",
            "command_topic": "/isaac/left_robotiq_joint_commands",
            "browser_command_topic": "/isaac/browser/left_robotiq_joint_commands",
            "default_joints": LEFT_GRIPPER_JOINTS,
            "driver_joint": LEFT_GRIPPER_DRIVER_JOINT,
            "coupled_joint_multipliers": LEFT_GRIPPER_COUPLED_JOINT_MULTIPLIERS,
        },
        {
            "label": "Right Robotiq",
            "state_topic": "/isaac/right_robotiq_joint_states",
            "command_topic": "/isaac/right_robotiq_joint_commands",
            "browser_command_topic": "/isaac/browser/right_robotiq_joint_commands",
            "default_joints": RIGHT_GRIPPER_JOINTS,
            "driver_joint": RIGHT_GRIPPER_DRIVER_JOINT,
            "coupled_joint_multipliers": RIGHT_GRIPPER_COUPLED_JOINT_MULTIPLIERS,
        },
    ]


JOINT_GROUPS = build_joint_groups()

MODEL_DISPLAY_ALIAS = "fr3duo_m+v"
DEFAULT_LAYOUT_PATH = os.environ.get("ISAAC_SIM_STREAM_LAYOUT_PATH", "")
DEFAULT_PORTABLE_ROOT = os.environ.get("ISAACSIM_PORTABLE_ROOT", "/tmp/isaac_portable")
DEFAULT_PHYSICS_HZ = 240.0
DEFAULT_RENDER_HZ = 60.0
DEFAULT_PHYSICS_SUBSTEPS = 2
DEFAULT_CONTROLLER_ACTIVITY_TOPIC = "/isaac_controller_manager/activity"
DEFAULT_PRIMARY_EFFORT_STALE_AFTER_S = 0.25
DEFAULT_COMMAND_SMOOTHING_ALPHA = 0.08
DEFAULT_MAX_POSITION_STEP_RAD = 0.008
DEFAULT_POSITION_DEADBAND_RAD = 0.006
DEFAULT_SETTLE_POSITION_WINDOW_RAD = 0.015
DEFAULT_SETTLE_VELOCITY_THRESHOLD_RAD_S = 0.12
