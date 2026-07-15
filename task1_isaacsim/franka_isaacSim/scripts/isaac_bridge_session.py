"""Shared bridge session setup for standalone and native Kit launches."""

import os
import sys
import time

import yaml

# Add services to path for camera service import
_SERVICES_DIR = os.path.join(os.path.dirname(__file__), "..", "services")
if _SERVICES_DIR not in sys.path:
    sys.path.insert(0, _SERVICES_DIR)

from isaac_bridge_constants import (
    DEFAULT_PRIMARY_CONTROLLER_NAME,
    MODEL_DISPLAY_ALIAS,
    build_joint_groups,
)
from isaac_camera_service import IsaacCameraService
from isaac_joint_drive_config import apply_joint_drive_configuration
from isaac_bridge_ros import SimulationRosBridge
from isaac_cable_visualizer import IsaacCablePointCloudVisualizer
from isaac_gripper_pose_publisher import IsaacGripperPosePublisher
from isaac_bridge_runtime import (
    _apply_model_alias,
    _configure_world_timing,
    _get_current_stage,
    _import_isaac_world_types,
    _open_stage_context,
    _resolve_robot_prim_path_with_retry,
    _rclpy_is_ok,
    _set_startup_camera_view,
    _stage_has_valid_prim,
)


class BridgeSession:
    """Owns the ROS bridge objects once a stage has been loaded."""

    def __init__(
        self,
        *,
        world,
        ros_node,
        sim_bridge,
        rclpy_module,
        render_every_n_steps,
        reset_service_name="/isaac/reset_simulation",
        trigger_service_type=None,
        home_pose=None,
        stage=None,
        gripper_pose_publisher=None,
        cable_visualizer=None,
    ):
        self.world = world
        self.ros_node = ros_node
        self.sim_bridge = sim_bridge
        self.rclpy_module = rclpy_module
        self.render_every_n_steps = render_every_n_steps
        # home_pose: dict with keys "left_arm" and "right_arm", each a list of 7 floats (rad).
        self._home_pose = home_pose
        self._stage = stage
        self._gripper_pose_publisher = gripper_pose_publisher
        self._cable_visualizer = cable_visualizer
        self._reset_requested = False
        self._reset_service = None
        self._object_spawner = None  # ObjectSpawner instance (if enabled)
        if (
            trigger_service_type is not None
            and self.ros_node is not None
            and reset_service_name
        ):
            self._reset_service = self.ros_node.create_service(
                trigger_service_type,
                str(reset_service_name),
                self._on_reset_requested,
            )

    # Maximum ROS callbacks to drain per tick.  Each spin_once(timeout=0)
    # handles at most one callback; repeating drains the queue so command
    # messages from the bridge/republisher are not starved when the Kit
    # update rate (render Hz) is lower than the aggregate message rate.
    _MAX_SPINS_PER_TICK = 64

    def tick(self):
        _spin = self.rclpy_module.spin_once
        _node = self.ros_node
        for _ in range(self._MAX_SPINS_PER_TICK):
            try:
                _spin(_node, timeout_sec=0.0)
            except Exception:
                break
        if self._reset_requested:
            self._perform_reset()

        # Update object spawner if enabled
        if self._object_spawner is not None:
            self._object_spawner.update(time.time())
        
        self.sim_bridge.apply_pending_commands()
        self.sim_bridge.publish_joint_states()
        if self._gripper_pose_publisher is not None:
            self._gripper_pose_publisher.publish()
        if self._cable_visualizer is not None:
            self._cable_visualizer.update()

    def shutdown(self):
        self._reset_service = None
        if self.ros_node is not None:
            self.ros_node.destroy_node()
            self.ros_node = None
        if self.rclpy_module is not None and _rclpy_is_ok(self.rclpy_module):
            self.rclpy_module.shutdown()
        self.rclpy_module = None

    def _on_reset_requested(self, _request, response):
        self._reset_requested = True
        response.success = True
        response.message = "Simulation reset scheduled"
        return response

    def _perform_reset(self):
        self._reset_requested = False
        if self.sim_bridge is not None:
            self.sim_bridge.clear_command_state()
        if self.world is not None:
            stop = getattr(self.world, "stop", None)
            if callable(stop):
                stop()
            self.world.reset()
            play = getattr(self.world, "play", None)
            if callable(play):
                play()
        if self.sim_bridge is not None:
            if self._home_pose is not None:
                self.sim_bridge.teleport_to_home_pose(
                    left_positions=self._home_pose["left_arm"],
                    right_positions=self._home_pose["right_arm"],
                )
            self.sim_bridge.hold_current_positions()
            self.sim_bridge.publish_joint_states(force=True)
        
        # Reset object spawner if enabled
        if self._object_spawner is not None:
            self._object_spawner.reset()
        
        print("Simulation state reset to home pose." if self._home_pose else "Simulation state reset to the initial stage state.")


def _stage_matches_path(stage, usd_path):
    if stage is None or not usd_path:
        return False
    try:
        root_layer = stage.GetRootLayer()
    except Exception:
        return False
    if root_layer is None:
        return False

    normalized_target = os.path.abspath(os.path.expanduser(usd_path))
    candidates = [getattr(root_layer, "realPath", None), getattr(root_layer, "identifier", None)]
    for candidate in candidates:
        if not candidate:
            continue
        try:
            if os.path.abspath(candidate) == normalized_target:
                return True
        except Exception:
            continue
    return False


_DEFAULT_HOME_POSE_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "assets", "isaac_assets", "config", "data_collection_home_pose.yaml",
)


def _load_home_pose(config_path=None):
    """Load home pose YAML.  Returns dict with 'left_arm' and 'right_arm' lists, or None."""
    path = config_path or _DEFAULT_HOME_POSE_PATH
    try:
        with open(path, "r", encoding="utf-8") as fh:
            cfg = yaml.safe_load(fh) or {}
        left = [float(v) for v in cfg.get("left_arm", {}).get("positions", [])]
        right = [float(v) for v in cfg.get("right_arm", {}).get("positions", [])]
        if len(left) == 7 and len(right) == 7:
            return {"left_arm": left, "right_arm": right}
    except Exception as exc:
        print(f"Warning: could not load home pose from {path}: {exc}")
    return None


def _kit_update(updater, count=1):
    update = getattr(updater, "update", None)
    if callable(update):
        for _ in range(max(0, int(count))):
            try:
                update()
            except Exception:
                break


def _enable_kit_extension(extension_name, updater):
    try:
        import omni.kit.app

        ext_manager = omni.kit.app.get_app().get_extension_manager()
        ext_manager.set_extension_enabled_immediate(extension_name, True)
        _kit_update(updater, 2)
        return True
    except Exception as exc:
        print(f"Warning: Could not enable Isaac extension {extension_name}: {exc}")
        return False


def _configure_newton_physics_backend(args, updater):
    backend = str(getattr(args, "physics_backend", "physx") or "physx").lower()
    if backend == "physx":
        print("Physics backend requested: physx")
        return True
    if backend != "newton":
        print(f"Error: Unsupported physics backend: {backend}")
        return False

    print("Physics backend requested: newton")
    for extension_name in ("isaacsim.physics.newton", "isaacsim.physics.newton.tensors"):
        _enable_kit_extension(extension_name, updater)

    try:
        from isaacsim.core.simulation_manager import SimulationManager
    except Exception as exc:
        print(f"Error: Could not import Isaac SimulationManager for Newton backend: {exc}")
        return False

    try:
        available = SimulationManager.get_available_physics_engines(verbose=True)
        print(f"Available Isaac physics engines: {available}")
    except Exception as exc:
        print(f"Warning: Could not query available Isaac physics engines: {exc}")

    try:
        switch_result = SimulationManager.switch_physics_engine("newton")
    except Exception as exc:
        print(f"Error: Failed to switch Isaac physics backend to Newton: {exc}")
        return False
    if switch_result is False:
        print("Error: Isaac refused to switch physics backend to Newton.")
        return False
    print("Isaac physics backend switched to Newton.")

    try:
        from isaacsim.core.simulation_manager.impl.mjc_scene import NewtonMjcScene
    except Exception as exc:
        print(f"Warning: NewtonMjcScene is not available; using Isaac's default Newton scene settings: {exc}")
        return True

    try:
        scene = NewtonMjcScene("/World/PhysicsScene")
        for method_name, value in (
            ("set_dt", 1.0 / float(getattr(args, "physics_hz", 240.0))),
            ("set_gravity", (0.0, 0.0, -9.81)),
        ):
            method = getattr(scene, method_name, None)
            if callable(method):
                try:
                    method(value)
                except TypeError:
                    if isinstance(value, tuple):
                        method(*value)
        for method_name, value in (
            ("set_integrator", os.getenv("NEWTON_INTEGRATOR", "implicit")),
            ("set_solver", os.getenv("NEWTON_SOLVER", "newton")),
            ("set_iterations", int(os.getenv("NEWTON_SOLVER_ITERATIONS", "100"))),
            ("set_nconmax", int(os.getenv("NEWTON_CONTACT_LIMIT", "50000"))),
        ):
            method = getattr(scene, method_name, None)
            if callable(method):
                try:
                    method(value)
                except Exception as exc:
                    print(f"Warning: Could not apply Newton scene setting {method_name}={value}: {exc}")
        print("Newton MJC scene configured at /World/PhysicsScene.")
    except Exception as exc:
        print(f"Warning: Could not configure Newton MJC scene explicitly: {exc}")
    return True


def initialize_bridge_session(args, updater, render_enabled):
    """Load the selected USD, bind the articulation, and start the ROS bridge."""
    world_type, robot_type, _ = _import_isaac_world_types()

    if not os.path.exists(args.usd_path):
        print(f"Error: Asset not found at {args.usd_path}")
        print("Please import/generate the URDF first.")
        return None

    stage = _get_current_stage()
    if not _stage_matches_path(stage, args.usd_path):
        stage = _open_stage_context(args.usd_path)
    if stage is None:
        print(
            "Error: Failed to open selected USD stage directly. "
            f"Path: {args.usd_path}"
        )
        return None

    if not _configure_newton_physics_backend(args, updater):
        return None

    _apply_model_alias(stage, MODEL_DISPLAY_ALIAS)
    robot_prim_path = _resolve_robot_prim_path_with_retry(
        stage,
        preferred_path=args.robot_prim_path,
        simulation_app=updater,
    )
    if not robot_prim_path:
        print("Error: Could not resolve articulation root in selected USD stage.")
        print("Hint: Use --robot-prim-path to point at the articulation root in your USD.")
        return None
    print(
        "Opened selected USD stage directly and preserved world setup. "
        f"Robot prim: {robot_prim_path}"
    )

    drive_summary = apply_joint_drive_configuration(
        stage,
        config_path=getattr(args, "joint_drive_config", None),
        stiffness_scale=getattr(args, "joint_drive_stiffness_scale", 1.0),
        damping_scale=getattr(args, "joint_drive_damping_scale", 1.0),
        max_force_scale=getattr(args, "joint_drive_max_force_scale", 1.0),
        gripper_stiffness_scale=getattr(args, "joint_drive_gripper_stiffness_scale", 1.0),
        gripper_damping_scale=getattr(args, "joint_drive_gripper_damping_scale", 1.0),
        gripper_max_force_scale=getattr(args, "joint_drive_gripper_max_force_scale", 1.0),
    )
    if drive_summary["config_path"]:
        print(
            "Applied Isaac joint drive settings: "
            f"{drive_summary['applied_count']} joints from {drive_summary['config_path']} "
            f"(arm: k={drive_summary['scales']['stiffness_scale']:.3f}, "
            f"d={drive_summary['scales']['damping_scale']:.3f}, "
            f"f={drive_summary['scales']['max_force_scale']:.3f}; "
            f"gripper: k={drive_summary['scales']['gripper_stiffness_scale']:.3f}, "
            f"d={drive_summary['scales']['gripper_damping_scale']:.3f}, "
            f"f={drive_summary['scales']['gripper_max_force_scale']:.3f})"
        )
        if drive_summary["missing_joints"]:
            print(
                "Warning: Missing joint-drive prims in stage: "
                + ", ".join(drive_summary["missing_joints"])
            )

    stage = _get_current_stage() or stage
    if not _stage_has_valid_prim(stage, robot_prim_path):
        print(
            "Error: Resolved robot prim path is not valid in the loaded stage: "
            f"{robot_prim_path}"
        )
        print("Hint: Use --robot-prim-path to point at the articulation root in your USD.")
        return None

    try:
        world = world_type(stage_units_in_meters=1.0, set_defaults=False)
    except TypeError:
        world = world_type(stage_units_in_meters=1.0)

    robot = robot_type(prim_path=robot_prim_path, name="bridge_robot")
    world.scene.add(robot)
    render_every_n_steps = _configure_world_timing(
        world,
        physics_hz=args.physics_hz,
        render_hz=args.render_hz,
        physics_substeps=args.physics_substeps,
    )
    camera_service = IsaacCameraService()
    camera_summary = camera_service.attach_configured_cameras(
        stage,
        config_path=getattr(args, "camera_config", None),
        render_hz=args.render_hz,
    )
    if camera_summary["config_path"]:
        print(
            "Configured Isaac cameras: "
            f"{camera_summary['attached_count']} attached from {camera_summary['config_path']}"
        )
        if camera_summary["missing_frames"]:
            print(
                "Warning: Missing configured camera attachment frames: "
                + ", ".join(camera_summary["missing_frames"])
            )
        if not camera_summary["contract_validation"]["compliant"]:
            print(
                "Warning: Camera config does not fully align with the data contract: "
                + "; ".join(camera_summary["contract_validation"]["errors"])
            )
    world.reset()
    world.play()

    import rclpy
    from geometry_msgs.msg import PoseStamped
    from geometry_msgs.msg import WrenchStamped
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from sensor_msgs.msg import PointCloud
    from std_srvs.srv import Trigger

    if not _rclpy_is_ok(rclpy):
        rclpy.init(args=None)

    ros_node = Node("isaac_sim_joint_bridge")
    primary_controller = getattr(args, "primary_controller", DEFAULT_PRIMARY_CONTROLLER_NAME)
    joint_groups = build_joint_groups(primary_controller)
    sim_bridge = SimulationRosBridge(
        ros_node,
        JointState,
        robot,
        joint_groups,
        publish_rate_hz=args.ros_publish_rate,
        wrench_state_type=WrenchStamped,
        browser_override_window_s=args.browser_command_hold_seconds,
        controller_activity_topic=args.controller_activity_topic,
        primary_effort_stale_after_s=args.primary_effort_stale_after_s,
        command_smoothing_alpha=args.command_smoothing_alpha,
        max_position_step_rad=args.max_position_step_rad,
        position_deadband_rad=args.position_deadband_rad,
        settle_position_window_rad=args.settle_position_window_rad,
        settle_velocity_threshold_rad_s=args.settle_velocity_threshold_rad_s,
    )
    sim_bridge.log_configuration()
    gripper_pose_publisher = IsaacGripperPosePublisher(
        ros_node,
        PoseStamped,
        stage,
        publish_rate_hz=args.ros_publish_rate,
    )
    cable_visualizer = None
    if os.getenv("CABLE_VISUALIZER_ENABLED", "true").lower() == "true":
        cable_visualizer = IsaacCablePointCloudVisualizer(
            ros_node,
            PointCloud,
            stage,
            topic=os.getenv("CABLE_POINT_TOPIC", "/cable/body_centers"),
            prim_path=os.getenv("CABLE_VISUALIZER_PRIM_PATH", "/World/NewtonCable/curve_0"),
            width_m=float(os.getenv("CABLE_VISUALIZER_WIDTH_M", "0.004")),
        )
    if render_enabled:
        _set_startup_camera_view(stage, robot_prim_path)

    home_pose = _load_home_pose(getattr(args, "home_pose_config", None))
    if home_pose is not None:
        sim_bridge.teleport_to_home_pose(
            left_positions=home_pose["left_arm"],
            right_positions=home_pose["right_arm"],
        )
    sim_bridge.hold_current_positions()
    sim_bridge.publish_joint_states(force=True)
    gripper_pose_publisher.publish(force=True)
    print("Simulation started. Press Ctrl+C to exit.")

    session = BridgeSession(
        world=world,
        ros_node=ros_node,
        sim_bridge=sim_bridge,
        rclpy_module=rclpy,
        render_every_n_steps=render_every_n_steps,
        trigger_service_type=Trigger,
        home_pose=home_pose,
        stage=stage,
        gripper_pose_publisher=gripper_pose_publisher,
        cable_visualizer=cable_visualizer,
    )

    # Initialize object spawner if enabled
    if os.getenv("OBJECT_SPAWNER_ENABLED", "false").lower() == "true":
        try:
            import sys
            spawner_dir = os.path.join(
                os.path.dirname(__file__), "..", "services", "object_spawner"
            )
            if spawner_dir not in sys.path:
                sys.path.insert(0, spawner_dir)
            
            from object_spawner import ObjectSpawner
            
            objects_config = os.path.join(spawner_dir, "objects_config.yaml")
            spawn_config = os.path.join(spawner_dir, "spawn_config.yaml")
            
            session._object_spawner = ObjectSpawner(
                objects_config_path=objects_config,
                spawn_config_path=spawn_config,
                stage=stage,
                world=world,
            )
            print("Object spawner enabled and initialized")
        except Exception as exc:
            print(f"Warning: Failed to initialize object spawner: {exc}")
            session._object_spawner = None
    else:
        print("Object spawner disabled (set OBJECT_SPAWNER_ENABLED=true to enable)")

    # Create UI controls for object spawner if enabled
    if session._object_spawner is not None:
        try:
            import omni.ui as ui
            
            # Create a simple UI window for spawner control
            spawner_window = ui.Window("Object Spawner Control", width=250, height=100)
            
            with spawner_window.frame:
                with ui.VStack(spacing=10):
                    ui.Label("Control object spawning", alignment=ui.Alignment.CENTER)
                    
                    def toggle_spawner():
                        new_state = session._object_spawner.toggle_enabled()
                        status_text = "ENABLED" if new_state else "DISABLED"
                        toggle_btn.text = f"Spawner: {status_text}"
                    
                    # Create toggle button with initial state
                    initial_state = "ENABLED" if session._object_spawner.is_enabled() else "DISABLED"
                    toggle_btn = ui.Button(f"Spawner: {initial_state}", clicked_fn=toggle_spawner, height=30)
            
            print("Object spawner UI control created")
        except Exception as exc:
            print(f"Warning: Failed to create spawner UI: {exc}")

    return session
