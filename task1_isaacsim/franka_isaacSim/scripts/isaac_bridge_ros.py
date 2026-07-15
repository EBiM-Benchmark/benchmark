"""ROS bridge implementation for Isaac articulation state/command topics."""

import re
import threading
import time

from isaac_bridge_constants import (
    DEFAULT_COMMAND_SMOOTHING_ALPHA,
    DEFAULT_CONTROLLER_ACTIVITY_TOPIC,
    DEFAULT_MAX_POSITION_STEP_RAD,
    DEFAULT_PRIMARY_EFFORT_STALE_AFTER_S,
    DEFAULT_POSITION_DEADBAND_RAD,
    DEFAULT_SETTLE_POSITION_WINDOW_RAD,
    DEFAULT_SETTLE_VELOCITY_THRESHOLD_RAD_S,
)


PRIMARY_COMMAND_SOURCE = "primary"
BROWSER_COMMAND_SOURCE = "browser"
HOLD_COMMAND_SOURCE = "hold"
ACTIVE_CONTROLLER_STATE = "active"


class SimulationRosBridge:
    """ROS2 bridge implemented directly in Python, without OmniGraph ROS2 nodes."""

    def __init__(
        self,
        node,
        joint_state_type,
        robot,
        joint_groups,
        publish_rate_hz=60.0,
        browser_override_window_s=0.25,
        controller_activity_topic=DEFAULT_CONTROLLER_ACTIVITY_TOPIC,
        primary_effort_stale_after_s=DEFAULT_PRIMARY_EFFORT_STALE_AFTER_S,
        command_smoothing_alpha=DEFAULT_COMMAND_SMOOTHING_ALPHA,
        max_position_step_rad=DEFAULT_MAX_POSITION_STEP_RAD,
        position_deadband_rad=DEFAULT_POSITION_DEADBAND_RAD,
        settle_position_window_rad=DEFAULT_SETTLE_POSITION_WINDOW_RAD,
        settle_velocity_threshold_rad_s=DEFAULT_SETTLE_VELOCITY_THRESHOLD_RAD_S,
        wrench_state_type=None,
    ):
        self._node = node
        self._joint_state_type = joint_state_type
        self._wrench_state_type = wrench_state_type
        self._robot = robot
        self._lock = threading.Lock()
        self._pending_commands = {}
        self._publishers = {}
        self._wrench_publishers = {}
        self._subscriptions = []
        self._groups = {}
        self._articulation_action_cls = None
        self._publish_period = 1.0 / max(float(publish_rate_hz), 1.0)
        self._next_publish_time = 0.0
        self._apply_warning_emitted = False
        self._last_positions = {}
        self._last_efforts = {}
        self._active_targets = {}
        self._active_efforts = {}
        self._active_target_sources = {}
        self._active_effort_sources = {}
        self._active_effort_last_values = {}
        self._active_effort_last_change_time = {}
        self._suppressed_primary_efforts = {}
        self._browser_override_window_s = max(float(browser_override_window_s), 0.0)
        self._browser_override_until = {}
        self._controller_activity_topic = str(controller_activity_topic or "").strip()
        self._required_primary_controllers = set()
        self._controller_states = {}
        self._primary_effort_stale_after_s = max(float(primary_effort_stale_after_s), 0.0)
        self._command_smoothing_alpha = min(
            max(float(command_smoothing_alpha), 0.0),
            1.0,
        )
        self._max_position_step_rad = max(float(max_position_step_rad), 0.0)
        self._position_deadband_rad = max(float(position_deadband_rad), 0.0)
        self._settle_position_window_rad = max(float(settle_position_window_rad), 0.0)
        self._settle_velocity_threshold_rad_s = max(
            float(settle_velocity_threshold_rad_s),
            0.0,
        )

        self._joint_names = self._resolve_joint_names(joint_groups)
        self._name_to_index = {name: index for index, name in enumerate(self._joint_names)}

        for group in joint_groups:
            command_topic = group["command_topic"]
            browser_command_topic = group.get("browser_command_topic")
            required_primary_controller = str(group.get("required_primary_controller", "")).strip()
            state_topic = group["state_topic"]
            desired_names = list(group["default_joints"])
            valid_names = []
            missing_names = []
            joint_aliases = {}

            for desired_name in desired_names:
                resolved_name = self._resolve_joint_alias(desired_name, used_names=valid_names)
                if resolved_name is None:
                    missing_names.append(desired_name)
                    continue
                valid_names.append(resolved_name)
                joint_aliases[desired_name] = resolved_name
                joint_aliases[resolved_name] = resolved_name

            if missing_names:
                print(
                    f"Warning: '{group['label']}' has joints missing in articulation and will skip them: "
                    + ", ".join(missing_names)
                )
            if not valid_names:
                valid_names = desired_names
                print(
                    f"Warning: '{group['label']}' could not map joints to articulation indices. "
                    "Commanding this group may not work."
                )

            command_joint_names = list(valid_names)
            command_aliases = dict(joint_aliases)
            driver_joint_actual = None
            coupled_joint_multipliers = {}

            configured_driver_joint = group.get("driver_joint")
            configured_coupled_joint_multipliers = group.get("coupled_joint_multipliers")
            if configured_driver_joint and configured_coupled_joint_multipliers:
                driver_joint_actual = joint_aliases.get(configured_driver_joint)
                if driver_joint_actual is None:
                    driver_joint_actual = self._resolve_joint_alias(configured_driver_joint, used_names=[])

                if driver_joint_actual is None:
                    print(
                        f"Warning: '{group['label']}' could not resolve driver joint "
                        f"'{configured_driver_joint}'. Falling back to per-joint command handling."
                    )
                else:
                    command_joint_names = [driver_joint_actual]
                    command_aliases = {
                        configured_driver_joint: driver_joint_actual,
                        driver_joint_actual: driver_joint_actual,
                    }
                    for alias_name, resolved_name in joint_aliases.items():
                        if resolved_name == driver_joint_actual:
                            command_aliases[alias_name] = resolved_name

                    missing_coupled = []
                    for desired_coupled_name, multiplier in configured_coupled_joint_multipliers.items():
                        resolved_coupled_name = joint_aliases.get(desired_coupled_name)
                        if resolved_coupled_name is None:
                            resolved_coupled_name = self._resolve_joint_alias(
                                desired_coupled_name, used_names=[]
                            )
                        if resolved_coupled_name is None:
                            missing_coupled.append(desired_coupled_name)
                            continue
                        try:
                            coupled_joint_multipliers[resolved_coupled_name] = float(multiplier)
                        except (TypeError, ValueError):
                            continue

                    if missing_coupled:
                        print(
                            f"Warning: '{group['label']}' has coupled joints missing in articulation "
                            "and will skip them: " + ", ".join(missing_coupled)
                        )

                    if driver_joint_actual not in coupled_joint_multipliers:
                        coupled_joint_multipliers[driver_joint_actual] = 1.0

            self._groups[command_topic] = {
                "label": group["label"],
                "state_topic": state_topic,
                "command_topic": command_topic,
                "browser_command_topic": browser_command_topic,
                "required_primary_controller": required_primary_controller,
                "joint_names": valid_names,
                "joint_aliases": joint_aliases,
                "command_joint_names": command_joint_names,
                "command_joint_set": set(command_joint_names),
                "command_aliases": command_aliases,
                "driver_joint_actual": driver_joint_actual,
                "coupled_joint_multipliers": coupled_joint_multipliers,
                "joint_indices": [self._name_to_index[name] for name in valid_names if name in self._name_to_index],
            }
            self._publishers[command_topic] = self._node.create_publisher(
                self._joint_state_type, state_topic, 10
            )
            wrench_topic = group.get("wrench_topic")
            if wrench_topic and self._wrench_state_type is not None:
                self._wrench_publishers[command_topic] = self._node.create_publisher(
                    self._wrench_state_type, wrench_topic, 10
                )
                self._groups[command_topic]["wrench_topic"] = wrench_topic
            self._subscriptions.append(
                self._node.create_subscription(
                    self._joint_state_type,
                    command_topic,
                    lambda message, key=command_topic: self._on_command(
                        message, group_key=key, source="primary"
                    ),
                    10,
                )
            )
            if browser_command_topic and browser_command_topic != command_topic:
                self._subscriptions.append(
                    self._node.create_subscription(
                        self._joint_state_type,
                        browser_command_topic,
                        lambda message, key=command_topic: self._on_command(
                            message, group_key=key, source="browser"
                        ),
                        10,
                    )
                )

            if required_primary_controller:
                self._required_primary_controllers.add(required_primary_controller)
                self._controller_states.setdefault(required_primary_controller, "inactive")

        self._subscribe_controller_activity()

    def log_configuration(self):
        print(f"Discovered {len(self._joint_names)} articulation joints on robot.")
        print(f"  Articulation DOF names: {self._joint_names}")
        for group in self._groups.values():
            aliases = group.get("joint_aliases", {})
            desired = group.get("joint_names", [])
            resolved_map = {d: aliases.get(d, "UNRESOLVED") for d in desired}
            unresolved = [d for d, r in resolved_map.items() if r == "UNRESOLVED"]
            if unresolved:
                print(f"  WARNING '{group['label']}': unresolved joints → {unresolved}")
            else:
                print(f"  '{group['label']}' joint map: {resolved_map}")
            state_joint_count = len(group["joint_names"])
            command_joint_count = len(group["command_joint_names"])
            if state_joint_count == command_joint_count:
                group_size_text = f"({state_joint_count} joints)"
            else:
                group_size_text = (
                    f"({state_joint_count} state joints, {command_joint_count} command joints)"
                )
            browser_command_topic = group.get("browser_command_topic")
            if browser_command_topic and browser_command_topic != group["command_topic"]:
                command_description = (
                    f"{group['command_topic']} "
                    f"(browser override: {browser_command_topic})"
                )
            else:
                command_description = group["command_topic"]
            print(
                f"ROS group '{group['label']}': {group['state_topic']} <- {command_description} "
                f"{group_size_text}"
            )
            required_primary_controller = group.get("required_primary_controller")
            if required_primary_controller:
                print(
                    f"  Primary control gated by '{required_primary_controller}' on "
                    f"{self._controller_activity_topic or DEFAULT_CONTROLLER_ACTIVITY_TOPIC}"
                )
        print(
            "Command shaping: "
            f"alpha={self._command_smoothing_alpha:.3f}, "
            f"max_step={self._max_position_step_rad:.4f} rad/step, "
            f"deadband={self._position_deadband_rad:.4f} rad, "
            f"settle_window={self._settle_position_window_rad:.4f} rad @ "
            f"{self._settle_velocity_threshold_rad_s:.4f} rad/s, "
            f"primary_effort_stale_after={self._primary_effort_stale_after_s:.3f}s"
        )

    def clear_command_state(self):
        with self._lock:
            self._pending_commands = {}
            self._active_targets.clear()
            self._active_efforts.clear()
            self._active_target_sources.clear()
            self._active_effort_sources.clear()
            self._active_effort_last_values.clear()
            self._active_effort_last_change_time.clear()
            self._suppressed_primary_efforts.clear()
            self._browser_override_until.clear()
            self._last_positions.clear()
            self._last_efforts.clear()
            self._next_publish_time = 0.0

    def configure_shaping(
        self,
        command_smoothing_alpha=None,
        max_position_step_rad=None,
        position_deadband_rad=None,
        settle_position_window_rad=None,
        settle_velocity_threshold_rad_s=None,
    ):
        """Update command-shaping parameters at runtime.

        Pass *None* for any parameter to leave it unchanged.
        """
        with self._lock:
            if command_smoothing_alpha is not None:
                self._command_smoothing_alpha = min(max(float(command_smoothing_alpha), 0.0), 1.0)
            if max_position_step_rad is not None:
                self._max_position_step_rad = max(float(max_position_step_rad), 0.0)
            if position_deadband_rad is not None:
                self._position_deadband_rad = max(float(position_deadband_rad), 0.0)
            if settle_position_window_rad is not None:
                self._settle_position_window_rad = max(float(settle_position_window_rad), 0.0)
            if settle_velocity_threshold_rad_s is not None:
                self._settle_velocity_threshold_rad_s = max(float(settle_velocity_threshold_rad_s), 0.0)

    def hold_current_positions(self):
        """Latch current articulation positions as hold targets for all joints.

        Called after world.play() following a reset so that joints are held at
        their reset pose instead of falling to the USD default (0 rad) while
        waiting for the first controller command to arrive.
        """
        current_positions = self._get_joint_positions()
        with self._lock:
            for group_key, group in self._groups.items():
                for joint_index in group["joint_indices"]:
                    position = (
                        float(current_positions[joint_index])
                        if joint_index < len(current_positions)
                        else 0.0
                    )
                    self._set_active_target(
                        joint_index,
                        position,
                        source=HOLD_COMMAND_SOURCE,
                        group_key=group_key,
                    )

    def teleport_to_home_pose(self, left_positions: list, right_positions: list):
        """Teleport both arms to the given home joint positions (rad) and zero all velocities.

        Uses set_joint_positions / set_joint_velocities directly so the robot is
        at the home pose immediately after world.reset() without any controller
        transient.  Called before hold_current_positions() so the hold latches
        the correct pose.

        Args:
            left_positions:  7 joint angles for the left arm (fr3v2_joint1..7).
            right_positions: 7 joint angles for the right arm (fr3v2_joint1..7).
        """
        import numpy as np

        # Collect arm joint indices from the configured groups.
        left_indices = []
        right_indices = []
        for group in self._groups.values():
            label = group.get("label", "").lower()
            if "left arm" in label:
                left_indices = list(group.get("joint_indices", []))
            elif "right arm" in label:
                right_indices = list(group.get("joint_indices", []))

        if not left_indices or not right_indices:
            return  # joint mapping not ready yet

        # Read current full position array to build the patched version.
        current = self._get_joint_positions()
        if current is None or len(current) == 0:
            return
        pos_arr = np.array(current, dtype=float)

        for idx, val in zip(left_indices, left_positions):
            if idx < len(pos_arr):
                pos_arr[idx] = float(val)
        for idx, val in zip(right_indices, right_positions):
            if idx < len(pos_arr):
                pos_arr[idx] = float(val)

        # Teleport positions.
        setter = getattr(self._robot, "set_joint_positions", None)
        if callable(setter):
            try:
                setter(pos_arr.reshape(1, -1))
            except Exception:
                try:
                    setter(pos_arr)
                except Exception:
                    pass

        # Zero velocities so there's no residual motion after teleport.
        vel_setter = getattr(self._robot, "set_joint_velocities", None)
        if callable(vel_setter):
            try:
                vel_setter(np.zeros_like(pos_arr).reshape(1, -1))
            except Exception:
                try:
                    vel_setter(np.zeros_like(pos_arr))
                except Exception:
                    pass

    def _resolve_joint_names(self, joint_groups):
        fallback_names = []
        for group in joint_groups:
            for name in group["default_joints"]:
                if name not in fallback_names:
                    fallback_names.append(name)

        candidates = []
        for attr_name in ("dof_names", "joint_names"):
            attr = getattr(self._robot, attr_name, None)
            if attr:
                candidates.append(attr)

        for method_name in ("get_joint_names", "get_dof_names"):
            method = getattr(self._robot, method_name, None)
            if callable(method):
                try:
                    value = method()
                    if value:
                        candidates.append(value)
                except Exception:
                    pass

        for candidate in candidates:
            try:
                names = [str(name) for name in list(candidate)]
            except Exception:
                continue
            if names:
                return names

        print(
            "Warning: Could not introspect articulation joint names. "
            "Falling back to configured default joint list."
        )
        return fallback_names

    def _resolve_joint_alias(self, desired_name, used_names):
        if desired_name in self._name_to_index and desired_name not in used_names:
            return desired_name

        side_prefix = ""
        for prefix in ("left_", "right_"):
            if desired_name.startswith(prefix):
                side_prefix = prefix
                break

        desired_tail = desired_name[len(side_prefix) :] if side_prefix else desired_name
        desired_tokens = desired_name.split("_")
        normalized_desired = re.sub(r"_+", "_", desired_name).strip("_")

        candidates = []
        for actual_name in self._joint_names:
            if actual_name in used_names:
                continue

            normalized_actual = re.sub(r"_+", "_", actual_name).strip("_")
            match = False
            if normalized_actual == normalized_desired:
                match = True
            elif actual_name.endswith(desired_name):
                match = True
            elif side_prefix and actual_name.startswith(side_prefix) and actual_name.endswith(desired_tail):
                match = True
            elif "_robotiq_" in desired_name and "_robotiq_" in actual_name:
                robotiq_tail = desired_name.split("_robotiq_", 1)[1]
                if side_prefix:
                    if actual_name.startswith(side_prefix) and actual_name.endswith(robotiq_tail):
                        match = True
                elif actual_name.endswith(robotiq_tail):
                    match = True

            if match:
                candidates.append(actual_name)

        if not candidates:
            return None
        if len(candidates) == 1:
            return candidates[0]

        def score(actual_name):
            actual_tokens = actual_name.split("_")
            suffix_match_count = 0
            for index in range(1, min(len(desired_tokens), len(actual_tokens)) + 1):
                if desired_tokens[-index] != actual_tokens[-index]:
                    break
                suffix_match_count += 1
            side_score = 1 if (side_prefix and actual_name.startswith(side_prefix)) else 0
            length_score = -abs(len(actual_name) - len(desired_name))
            return (suffix_match_count, side_score, length_score)

        return max(candidates, key=score)

    def _subscribe_controller_activity(self):
        if not self._required_primary_controllers or not self._controller_activity_topic:
            return

        try:
            from controller_manager_msgs.msg import ControllerManagerActivity
        except Exception as error:
            print(
                "Warning: Could not import controller_manager_msgs.msg.ControllerManagerActivity. "
                f"Using a host-test fallback subscription type: {error}"
            )
            ControllerManagerActivity = object

        try:
            subscription = self._node.create_subscription(
                ControllerManagerActivity,
                self._controller_activity_topic,
                self._on_controller_activity,
                10,
            )
        except Exception as error:
            print(
                "Warning: Could not subscribe to the controller activity topic. "
                f"Primary controller gating is disabled: {error}"
            )
            self._required_primary_controllers.clear()
            return

        self._subscriptions.append(subscription)

    @staticmethod
    def _controller_state_label(controller):
        state = getattr(controller, "state", None)
        if hasattr(state, "label"):
            return str(state.label).strip().lower()
        if state is None:
            return ""
        return str(state).strip().lower()

    def _on_controller_activity(self, message):
        if not self._required_primary_controllers:
            return

        updated_states = {}
        for controller in getattr(message, "controllers", []):
            name = str(getattr(controller, "name", "")).strip()
            if not name or name not in self._required_primary_controllers:
                continue
            updated_states[name] = self._controller_state_label(controller)

        if not updated_states:
            return

        with self._lock:
            self._controller_states.update(updated_states)

    def _is_primary_controller_active(self, group):
        controller_name = str(group.get("required_primary_controller", "")).strip()
        if not controller_name:
            return True
        if controller_name not in self._required_primary_controllers:
            return True
        return self._controller_states.get(controller_name, "") == ACTIVE_CONTROLLER_STATE

    def _set_active_target(self, joint_index, target_position, *, source, group_key):
        self._active_targets[joint_index] = float(target_position)
        self._active_target_sources[joint_index] = (str(source), str(group_key))

    def _set_active_effort(self, joint_index, effort, *, source, group_key):
        effort_value = float(effort)
        if source == PRIMARY_COMMAND_SOURCE:
            suppressed_effort = self._suppressed_primary_efforts.get(joint_index)
            if suppressed_effort is not None and abs(suppressed_effort - effort_value) <= 1e-9:
                return False
            self._suppressed_primary_efforts.pop(joint_index, None)

        now = time.monotonic()
        last_effort = self._active_effort_last_values.get(joint_index)
        last_change_time = self._active_effort_last_change_time.get(joint_index, now)
        if last_effort is None or abs(last_effort - effort_value) > 1e-9:
            last_change_time = now
        self._active_efforts[joint_index] = effort_value
        self._active_effort_sources[joint_index] = (str(source), str(group_key))
        self._active_effort_last_values[joint_index] = effort_value
        self._active_effort_last_change_time[joint_index] = last_change_time
        return True

    def _clear_active_target(self, joint_index, *, source=None, group_key=None):
        metadata = self._active_target_sources.get(joint_index)
        if source is not None and (metadata is None or metadata[0] != source):
            return False
        if group_key is not None and (metadata is None or metadata[1] != group_key):
            return False
        removed = joint_index in self._active_targets
        self._active_targets.pop(joint_index, None)
        self._active_target_sources.pop(joint_index, None)
        return removed

    def _clear_active_effort(self, joint_index, *, source=None, group_key=None):
        metadata = self._active_effort_sources.get(joint_index)
        if source is not None and (metadata is None or metadata[0] != source):
            return False
        if group_key is not None and (metadata is None or metadata[1] != group_key):
            return False
        removed = joint_index in self._active_efforts
        self._active_efforts.pop(joint_index, None)
        self._active_effort_sources.pop(joint_index, None)
        self._active_effort_last_values.pop(joint_index, None)
        self._active_effort_last_change_time.pop(joint_index, None)
        return removed

    def _suppress_primary_effort(self, joint_index):
        suppressed_effort = self._active_effort_last_values.get(
            joint_index,
            self._active_efforts.get(joint_index),
        )
        if suppressed_effort is not None:
            self._suppressed_primary_efforts[joint_index] = float(suppressed_effort)

    def _enforce_inactive_group_holds(self, current_positions):
        for group_key, group in self._groups.items():
            if self._is_primary_controller_active(group):
                continue
            if not group.get("required_primary_controller"):
                continue

            for joint_index in group.get("joint_indices", []):
                self._suppress_primary_effort(joint_index)
                self._clear_active_effort(
                    joint_index,
                    source=PRIMARY_COMMAND_SOURCE,
                    group_key=group_key,
                )
                self._clear_active_target(
                    joint_index,
                    source=PRIMARY_COMMAND_SOURCE,
                    group_key=group_key,
                )
                if joint_index in self._active_efforts or joint_index in self._active_targets:
                    continue
                hold_position = (
                    float(current_positions[joint_index])
                    if joint_index < len(current_positions)
                    else 0.0
                )
                self._set_active_target(
                    joint_index,
                    hold_position,
                    source=HOLD_COMMAND_SOURCE,
                    group_key=group_key,
                )

    def _enforce_stale_primary_effort_holds(self, current_positions):
        now = time.monotonic()
        for joint_index in list(self._active_efforts.keys()):
            metadata = self._active_effort_sources.get(joint_index)
            if metadata is None:
                continue
            source, group_key = metadata
            if source != PRIMARY_COMMAND_SOURCE:
                continue

            last_change_time = self._active_effort_last_change_time.get(joint_index, now)
            if (now - last_change_time) < self._primary_effort_stale_after_s:
                continue

            self._suppress_primary_effort(joint_index)
            self._clear_active_effort(
                joint_index,
                source=PRIMARY_COMMAND_SOURCE,
                group_key=group_key,
            )
            if joint_index in self._active_efforts or joint_index in self._active_targets:
                continue
            hold_position = (
                float(current_positions[joint_index])
                if joint_index < len(current_positions)
                else 0.0
            )
            self._set_active_target(
                joint_index,
                hold_position,
                source=HOLD_COMMAND_SOURCE,
                group_key=group_key,
            )

    def _on_command(self, message, group_key, source):
        group = self._groups.get(group_key)
        if group is None:
            return

        names = list(message.name)
        positions = list(message.position)
        efforts = list(message.effort)
        use_efforts = source != BROWSER_COMMAND_SOURCE and bool(efforts)
        if not use_efforts and not positions:
            return

        if not names:
            names = list(group["joint_names"] if use_efforts else group["command_joint_names"])

        now = time.monotonic()
        with self._lock:
            if source != BROWSER_COMMAND_SOURCE:
                if not self._is_primary_controller_active(group):
                    return
                override_until = self._browser_override_until.get(group_key, 0.0)
                if now < override_until:
                    return
            else:
                self._browser_override_until[group_key] = now + self._browser_override_window_s

            group_pending = self._pending_commands.setdefault(
                group_key,
                {
                    "positions": {},
                    "positions_source": None,
                    "efforts": {},
                    "efforts_source": None,
                },
            )
            command_joint_set = group["command_joint_set"]
            command_aliases = group["command_aliases"]
            aliases = group.get("joint_aliases", {})
            values = efforts if use_efforts else positions
            bucket_key = "efforts" if use_efforts else "positions"
            passive_key = "positions" if use_efforts else "efforts"
            active_bucket = group_pending[bucket_key]
            passive_bucket = group_pending[passive_key]
            group_pending[f"{bucket_key}_source"] = source
            passive_bucket.clear()
            group_pending[f"{passive_key}_source"] = None
            for index, name in enumerate(names):
                if index >= len(values):
                    break
                resolved_name = command_aliases.get(name)
                if resolved_name is None:
                    resolved_name = aliases.get(name)
                if resolved_name is None:
                    resolved_name = self._resolve_joint_alias(name, used_names=[])
                    if resolved_name is None:
                        continue
                    aliases[name] = resolved_name
                if resolved_name not in command_joint_set:
                    continue
                command_aliases[name] = resolved_name
                try:
                    active_bucket[resolved_name] = float(values[index])
                except (TypeError, ValueError):
                    continue

    def _shape_command_target(self, joint_index, raw_target, current_positions, current_velocities):
        try:
            target_position = float(raw_target)
        except (TypeError, ValueError):
            return None, None

        if joint_index is None or joint_index < 0 or joint_index >= len(current_positions):
            return target_position, target_position

        current_position = float(current_positions[joint_index])
        current_velocity = 0.0
        if joint_index < len(current_velocities):
            try:
                current_velocity = float(current_velocities[joint_index])
            except (TypeError, ValueError):
                current_velocity = 0.0

        position_error = target_position - current_position
        abs_error = abs(position_error)
        if self._position_deadband_rad > 0.0 and abs_error <= self._position_deadband_rad:
            return None, current_position
        if (
            self._settle_position_window_rad > 0.0
            and abs_error <= self._settle_position_window_rad
            and abs(current_velocity) <= self._settle_velocity_threshold_rad_s
        ):
            return None, current_position

        shaped_position = target_position

        if self._command_smoothing_alpha < 1.0:
            alpha = self._command_smoothing_alpha
            shaped_position = current_position + alpha * position_error

        if self._max_position_step_rad > 0.0:
            delta = shaped_position - current_position
            step_limit = self._max_position_step_rad
            if delta > step_limit:
                shaped_position = current_position + step_limit
            elif delta < -step_limit:
                shaped_position = current_position - step_limit

        if abs(shaped_position - current_position) <= 1e-9:
            return None, target_position

        return shaped_position, target_position

    def apply_pending_commands(self):
        with self._lock:
            pending = self._pending_commands
            self._pending_commands = {}

        current_positions = self._get_joint_positions()
        if len(current_positions) < len(self._joint_names):
            current_positions.extend([0.0] * (len(self._joint_names) - len(current_positions)))
        current_velocities = self._get_joint_velocities()
        if len(current_velocities) < len(self._joint_names):
            current_velocities.extend([0.0] * (len(self._joint_names) - len(current_velocities)))

        for command_topic, by_name in pending.items():
            group = self._groups.get(command_topic)
            if group is None:
                continue
            by_name_positions = by_name.get("positions", {})
            by_name_positions_source = by_name.get("positions_source") or PRIMARY_COMMAND_SOURCE
            by_name_efforts = by_name.get("efforts", {})
            by_name_efforts_source = by_name.get("efforts_source") or PRIMARY_COMMAND_SOURCE

            driver_joint_actual = group.get("driver_joint_actual")
            coupled_joint_multipliers = group.get("coupled_joint_multipliers", {})
            if driver_joint_actual and coupled_joint_multipliers and by_name_positions:
                driver_position = by_name_positions.get(driver_joint_actual)
                if driver_position is None:
                    continue
                try:
                    driver_position_value = float(driver_position)
                except (TypeError, ValueError):
                    continue
                for coupled_name, multiplier in coupled_joint_multipliers.items():
                    joint_index = self._name_to_index.get(coupled_name)
                    if joint_index is not None:
                        self._clear_active_effort(joint_index)
                        self._set_active_target(
                            joint_index,
                            driver_position_value * multiplier,
                            source=by_name_positions_source,
                            group_key=command_topic,
                        )
                continue

            for name, effort in by_name_efforts.items():
                joint_index = self._name_to_index.get(name)
                if joint_index is not None:
                    try:
                        self._set_active_effort(
                            joint_index,
                            float(effort),
                            source=by_name_efforts_source,
                            group_key=command_topic,
                        )
                        self._clear_active_target(joint_index)
                    except (TypeError, ValueError):
                        continue

            for name, position in by_name_positions.items():
                joint_index = self._name_to_index.get(name)
                if joint_index is not None:
                    try:
                        self._clear_active_effort(joint_index)
                        self._set_active_target(
                            joint_index,
                            float(position),
                            source=by_name_positions_source,
                            group_key=command_topic,
                        )
                    except (TypeError, ValueError):
                        continue

        self._enforce_inactive_group_holds(current_positions)
        self._enforce_stale_primary_effort_holds(current_positions)

        if self._active_efforts and not self._apply_effort_updates(self._active_efforts):
            if not self._apply_warning_emitted:
                print(
                    "Warning: Could not apply joint effort commands through available articulation APIs. "
                    "Torque commands are received but robot motion update failed."
                )
                self._apply_warning_emitted = True

        if not self._active_targets:
            return

        updates_by_index = {}
        next_active_targets = {}
        next_active_target_sources = {}
        for joint_index, target_position in list(self._active_targets.items()):
            shaped_position, stabilized_target = self._shape_command_target(
                joint_index,
                target_position,
                current_positions,
                current_velocities,
            )
            if stabilized_target is not None:
                next_active_targets[joint_index] = stabilized_target
                metadata = self._active_target_sources.get(joint_index)
                if metadata is not None:
                    next_active_target_sources[joint_index] = metadata
            if shaped_position is None:
                continue
            updates_by_index[joint_index] = shaped_position
            joint_name = self._joint_names[joint_index] if joint_index < len(self._joint_names) else None
            if joint_name is not None:
                self._last_positions[joint_name] = shaped_position

        self._active_targets = next_active_targets
        self._active_target_sources = next_active_target_sources

        if not updates_by_index:
            return

        if not self._apply_joint_updates(updates_by_index) and not self._apply_warning_emitted:
            print(
                "Warning: Could not apply joint commands through available articulation APIs. "
                "Commands are received but robot motion update failed."
            )
            self._apply_warning_emitted = True

    def publish_joint_states(self, force=False):
        now = time.monotonic()
        if not force and now < self._next_publish_time:
            return
        self._next_publish_time = now + self._publish_period

        positions = self._get_joint_positions()
        velocities = self._get_joint_velocities()
        efforts = self._get_joint_efforts()
        stamp = self._node.get_clock().now().to_msg()

        for command_topic, group in self._groups.items():
            joint_names = list(group["joint_names"])
            joint_indices = list(group["joint_indices"])

            if joint_indices:
                joint_positions = [
                    positions[index] if index < len(positions) else 0.0 for index in joint_indices
                ]
                joint_velocities = [
                    velocities[index] if index < len(velocities) else 0.0 for index in joint_indices
                ]
                joint_efforts = [
                    efforts[index] if index < len(efforts) else 0.0 for index in joint_indices
                ]
            else:
                joint_positions = [self._last_positions.get(name, 0.0) for name in joint_names]
                joint_velocities = [0.0] * len(joint_names)
                joint_efforts = [self._last_efforts.get(name, 0.0) for name in joint_names]

            for name, position in zip(joint_names, joint_positions):
                self._last_positions[name] = position
            for name, effort in zip(joint_names, joint_efforts):
                self._last_efforts[name] = effort

            message = self._joint_state_type()
            message.header.stamp = stamp
            message.name = joint_names
            message.position = joint_positions
            message.velocity = joint_velocities
            message.effort = joint_efforts
            self._publishers[command_topic].publish(message)

            wrench_pub = self._wrench_publishers.get(command_topic)
            if wrench_pub is not None and self._wrench_state_type is not None and joint_indices:
                wrench_values = self._compute_ee_wrench(joint_indices)
                wrench_msg = self._wrench_state_type()
                try:
                    wrench_msg.header.stamp = stamp
                    wrench_msg.wrench.force.x = wrench_values[0]
                    wrench_msg.wrench.force.y = wrench_values[1]
                    wrench_msg.wrench.force.z = wrench_values[2]
                    wrench_msg.wrench.torque.x = wrench_values[3]
                    wrench_msg.wrench.torque.y = wrench_values[4]
                    wrench_msg.wrench.torque.z = wrench_values[5]
                except Exception:
                    pass
                wrench_pub.publish(wrench_msg)

    def _normalize_numeric_sequence(self, values):
        if values is None:
            return None
        if hasattr(values, "tolist"):
            values = values.tolist()
        try:
            sequence = list(values)
        except TypeError:
            return None
        # Unwrap a batch dimension of size 1: [[v1, v2, ...]] → [v1, v2, ...]
        # Isaac Sim ArticulationView APIs can return (1, N_dofs) shaped arrays.
        if (
            len(sequence) == 1
            and isinstance(sequence[0], (list, tuple))
            and sequence[0]
        ):
            sequence = list(sequence[0])
        normalized = []
        for value in sequence:
            try:
                normalized.append(float(value))
            except (TypeError, ValueError):
                normalized.append(0.0)
        return normalized

    def _get_joint_positions(self):
        getter = getattr(self._robot, "get_joint_positions", None)
        if callable(getter):
            for call in (lambda: getter(), lambda: getter(joint_indices=None)):
                try:
                    values = self._normalize_numeric_sequence(call())
                    if values is not None:
                        return values
                except Exception:
                    pass

        state_getter = getattr(self._robot, "get_joints_state", None)
        if callable(state_getter):
            try:
                state = state_getter()
                values = self._normalize_numeric_sequence(getattr(state, "positions", None))
                if values is not None:
                    return values
            except Exception:
                pass

        return [0.0] * len(self._joint_names)

    def _get_joint_velocities(self):
        getter = getattr(self._robot, "get_joint_velocities", None)
        if callable(getter):
            for call in (lambda: getter(), lambda: getter(joint_indices=None)):
                try:
                    values = self._normalize_numeric_sequence(call())
                    if values is not None:
                        return values
                except Exception:
                    pass

        state_getter = getattr(self._robot, "get_joints_state", None)
        if callable(state_getter):
            try:
                state = state_getter()
                values = self._normalize_numeric_sequence(getattr(state, "velocities", None))
                if values is not None:
                    return values
            except Exception:
                pass

        return [0.0] * len(self._joint_names)

    def _get_joint_efforts(self):
        for method_name in (
            "get_joint_efforts",
            "get_measured_joint_efforts",
            "get_applied_joint_efforts",
        ):
            getter = getattr(self._robot, method_name, None)
            if callable(getter):
                for call in (lambda: getter(), lambda: getter(joint_indices=None)):
                    try:
                        values = self._normalize_numeric_sequence(call())
                        if values is not None:
                            return values
                    except Exception:
                        pass

        state_getter = getattr(self._robot, "get_joints_state", None)
        if callable(state_getter):
            try:
                state = state_getter()
                values = self._normalize_numeric_sequence(getattr(state, "efforts", None))
                if values is not None:
                    return values
            except Exception:
                pass

        return [self._last_efforts.get(name, 0.0) for name in self._joint_names]

    def _get_joint_efforts_measured(self):
        """Return measured joint efforts (drive + contact reaction forces from PhysX)."""
        for method_name in ("get_measured_joint_efforts", "get_joint_efforts"):
            getter = getattr(self._robot, method_name, None)
            if callable(getter):
                for call in (lambda: getter(), lambda: getter(joint_indices=None)):
                    try:
                        values = self._normalize_numeric_sequence(call())
                        if values is not None:
                            return values
                    except Exception:
                        pass
        return None

    def _get_joint_efforts_applied(self):
        """Return applied joint efforts (commanded drive torques)."""
        for method_name in ("get_applied_joint_efforts",):
            getter = getattr(self._robot, method_name, None)
            if callable(getter):
                for call in (lambda: getter(), lambda: getter(joint_indices=None)):
                    try:
                        values = self._normalize_numeric_sequence(call())
                        if values is not None:
                            return values
                    except Exception:
                        pass
        return None

    def _compute_ee_wrench(self, joint_indices):
        """Compute 6D end-effector wrench [fx, fy, fz, tx, ty, tz] from external joint torques.

        External torques = measured_efforts - applied_efforts.
        Wrench = pinv(J^T) @ external_torques, where J is the arm Jacobian (6 x len(joint_indices)).

        Returns a list of 6 floats, or [0.0]*6 if the required data is unavailable.
        """
        zero_wrench = [0.0] * 6

        measured = self._get_joint_efforts_measured()
        applied = self._get_joint_efforts_applied()
        if measured is None:
            return zero_wrench

        n = len(joint_indices)
        if applied is not None and len(applied) >= max(joint_indices, default=0) + 1:
            ext_torques = [
                measured[i] - applied[i] if i < len(measured) else 0.0
                for i in joint_indices
            ]
        else:
            ext_torques = [measured[i] if i < len(measured) else 0.0 for i in joint_indices]

        # Attempt Jacobian-based wrench computation.
        jacobian = self._get_arm_jacobian(joint_indices)
        if jacobian is None:
            return zero_wrench

        try:
            import numpy as np

            tau = np.array(ext_torques, dtype=float)  # (n,)
            J = np.array(jacobian, dtype=float)  # (6, n)
            # wrench = pinv(J^T) @ tau  ⟺  wrench = (J @ J^T)^{-1} @ J @ tau (least-squares)
            wrench = np.linalg.lstsq(J.T, tau, rcond=None)[0]
            return [float(v) for v in wrench[:6]]
        except Exception:
            return zero_wrench

    def _get_arm_jacobian(self, joint_indices):
        """Return the 6×n end-effector Jacobian for the given joint indices, or None if unavailable.

        Tries Isaac Sim ArticulationView.get_jacobians() which returns shape (1, num_links, 6, num_dofs).
        The Robot class wraps a _prim_view (Articulation) that holds get_jacobians(); tries both.
        The last link's Jacobian columns matching arm joint_indices are extracted.
        """
        try:
            # Robot (SingleArticulation) delegates to _prim_view (Articulation) for batch ops.
            getter = getattr(self._robot, "get_jacobians", None)
            if not callable(getter):
                prim_view = getattr(self._robot, "_prim_view", None)
                getter = getattr(prim_view, "get_jacobians", None) if prim_view is not None else None
            if not callable(getter):
                return None
            raw = getter()
            if raw is None:
                return None
            if hasattr(raw, "tolist"):
                raw = raw.tolist()
            raw = list(raw)

            # Shape (1, num_links, 6, num_dofs) → take batch 0
            if raw and isinstance(raw[0], list):
                raw = raw[0]  # drop batch dim → (num_links, 6, num_dofs)
            if not raw or not isinstance(raw[0], list):
                return None

            n_dofs = len(raw[0][0]) if raw[0] else 0
            valid_indices = [i for i in joint_indices if i < n_dofs]
            if len(valid_indices) < 6:
                return None

            # Find and cache the EE link index: last link with non-zero Jacobian columns
            # for these joints. In multi-body scenes raw[-1] is often not the arm EE.
            cache_key = tuple(valid_indices)
            if not hasattr(self, "_ee_link_cache"):
                self._ee_link_cache = {}
            ee_link_idx = self._ee_link_cache.get(cache_key)
            if ee_link_idx is None:
                for li, block in enumerate(reversed(raw)):
                    col_sq_sum = sum(block[r][ji] ** 2 for r in range(len(block)) for ji in valid_indices)
                    if col_sq_sum > 1e-10:
                        ee_link_idx = len(raw) - 1 - li
                        break
                if ee_link_idx is None:
                    return None
                self._ee_link_cache[cache_key] = ee_link_idx

            ee_block = raw[ee_link_idx]
            jacobian = [[row[i] for i in valid_indices] for row in ee_block]  # (6, n)
            return jacobian
        except Exception:
            return None

    def _apply_joint_updates(self, updates_by_index):
        ordered_indices = sorted(updates_by_index.keys())
        subset_positions = [updates_by_index[index] for index in ordered_indices]

        for method_name, kwarg_name in (
            ("set_joint_positions", "joint_indices"),
            ("set_joint_positions", "indices"),
            ("set_joint_position_targets", "joint_indices"),
            ("set_joint_position_targets", "indices"),
        ):
            method = getattr(self._robot, method_name, None)
            if not callable(method):
                continue
            try:
                method(subset_positions, **{kwarg_name: ordered_indices})
                return True
            except TypeError:
                pass
            except Exception:
                pass
            try:
                method(subset_positions, ordered_indices)
                return True
            except Exception:
                pass

        full_positions = self._get_joint_positions()
        if len(full_positions) < len(self._joint_names):
            full_positions.extend([0.0] * (len(self._joint_names) - len(full_positions)))
        for index, position in updates_by_index.items():
            if index < len(full_positions):
                full_positions[index] = position

        for method_name in ("set_joint_positions", "set_joint_position_targets"):
            method = getattr(self._robot, method_name, None)
            if not callable(method):
                continue
            try:
                method(full_positions)
                return True
            except Exception:
                pass

        action = self._build_articulation_action(subset_positions, ordered_indices)
        if action is not None:
            controller_factory = getattr(self._robot, "get_articulation_controller", None)
            controller = controller_factory() if callable(controller_factory) else None
            if controller is not None and hasattr(controller, "apply_action"):
                try:
                    controller.apply_action(action)
                    return True
                except Exception:
                    pass
            apply_action = getattr(self._robot, "apply_action", None)
            if callable(apply_action):
                try:
                    apply_action(action)
                    return True
                except Exception:
                    pass

        return False

    def _apply_effort_updates(self, updates_by_index):
        ordered_indices = sorted(updates_by_index.keys())
        subset_efforts = [updates_by_index[index] for index in ordered_indices]

        for method_name, kwarg_name in (
            ("set_joint_efforts", "joint_indices"),
            ("set_joint_efforts", "indices"),
        ):
            method = getattr(self._robot, method_name, None)
            if not callable(method):
                continue
            try:
                method(subset_efforts, **{kwarg_name: ordered_indices})
                return True
            except TypeError:
                pass
            except Exception:
                pass
            try:
                method(subset_efforts, ordered_indices)
                return True
            except Exception:
                pass

        full_efforts = [0.0] * len(self._joint_names)
        for index, effort in updates_by_index.items():
            if index < len(full_efforts):
                full_efforts[index] = effort

        method = getattr(self._robot, "set_joint_efforts", None)
        if callable(method):
            try:
                method(full_efforts)
                return True
            except Exception:
                pass

        action = self._build_articulation_action(indices=ordered_indices, joint_efforts=subset_efforts)
        if action is not None:
            controller_factory = getattr(self._robot, "get_articulation_controller", None)
            controller = controller_factory() if callable(controller_factory) else None
            if controller is not None and hasattr(controller, "apply_action"):
                try:
                    controller.apply_action(action)
                    return True
                except Exception:
                    pass
            apply_action = getattr(self._robot, "apply_action", None)
            if callable(apply_action):
                try:
                    apply_action(action)
                    return True
                except Exception:
                    pass

        return False

    def _build_articulation_action(self, positions=None, indices=None, joint_efforts=None):
        if self._articulation_action_cls is None:
            for module_name in ("isaacsim.core.utils.types", "omni.isaac.core.utils.types"):
                try:
                    module = __import__(module_name, fromlist=["ArticulationAction"])
                    self._articulation_action_cls = getattr(module, "ArticulationAction")
                    break
                except Exception:
                    continue
        if self._articulation_action_cls is None:
            return None
        try:
            return self._articulation_action_cls(
                joint_positions=positions,
                joint_efforts=joint_efforts,
                joint_indices=indices,
            )
        except Exception:
            return None
