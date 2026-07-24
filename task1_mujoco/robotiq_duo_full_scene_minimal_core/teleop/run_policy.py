"""Policy rollout mode (train branch): --input policy.

Drives the sim from a trained policy instead of a human input device. No
IK / mode-switching here - state/action follow task1_isaacsim's own
fr3_duo_mobile_data_contract (data_record.py's build_contract_state /
apply_contract_action), not a MuJoCo-specific raw qpos/ctrl dump, so a
policy trained on Task 1 MuJoCo recordings and Task 1 Isaac Sim recordings
see the same structure. apply_contract_action only inverts the base
targets for --base-control=actuator (the default).

Two ways to supply the policy:

1. A LeRobot checkpoint (trained via lerobot-train, any --policy.type):

    python main.py --input policy --policy-path CKPT_DIR \
        --policy-dataset-root DATASET_DIR

2. --policy-adapter 'path/to/file.py:ClassName' - bring your own model
   trained however you like (not LeRobot at all, if you don't want it to
   be). The only requirement is implementing this contract:

    class MyAdapter:
        def __init__(self): ...  # no-arg constructor
        def predict(self, state: np.ndarray, images: dict[str, np.ndarray]) -> np.ndarray:
            \"\"\"state: (50,) float32, build_contract_state() layout (base
            x/y/theta/vx/vy/omega, then per arm position/velocity/effort
            x7, then left/right gripper open-fraction). images:
            {camera_name: (H,W,3) uint8} - one entry per --policy-camera.
            Returns: (19,) float32, build_contract_action() layout (base
            vx/vy/omega, left/right arm target angles x7, left/right
            gripper open-fraction) - contract order, not raw data.ctrl.\"\"\"

    python main.py --input policy --policy-adapter my_policy.py:MyAdapter

Either way, --policy-camera must match what the policy actually expects
(same value used for --record-camera when the training data was made) -
this module has no way to check that for you.
"""

from __future__ import annotations

import importlib.util
import sys
import time
from pathlib import Path

import numpy as np

import mujoco
import mujoco.viewer

from . import config, log
from .data_record import apply_contract_action, build_contract_state, make_camera_renderers
from .session import TeleopSession


def load_adapter(spec: str):
    """spec = 'path/to/file.py:ClassName' (or a module-level factory
    function/instance - anything callable-or-class works).
    rpartition, not partition: Windows paths have their own ':' in the
    drive letter (C:\\...), so splitting on the FIRST ':' breaks - the
    class name is always after the LAST ':'."""
    path_str, sep, name = spec.rpartition(":")
    if not sep or not name:
        raise ValueError(f"--policy-adapter must look like 'path/to/file.py:ClassName', got {spec!r}")
    path = Path(path_str).resolve()
    if not path.is_file():
        raise FileNotFoundError(f"--policy-adapter file not found: {path}")
    module_spec = importlib.util.spec_from_file_location(f"policy_adapter_{path.stem}", path)
    if module_spec is None or module_spec.loader is None:
        raise ImportError(f"could not load {path} as a Python module")
    module = importlib.util.module_from_spec(module_spec)
    module_spec.loader.exec_module(module)
    obj = getattr(module, name, None)
    if obj is None:
        raise AttributeError(f"{name!r} not found in {path}")
    instance = obj() if isinstance(obj, type) else obj
    if not hasattr(instance, "predict"):
        raise TypeError(f"{spec} has no .predict(state, images) method - see run_policy.py's docstring")
    return instance


def main(args) -> None:
    if sys.platform == "win32":
        try:
            import ctypes

            ctypes.windll.winmm.timeBeginPeriod(1)
        except Exception:
            pass

    adapter_spec = getattr(args, "policy_adapter", None)
    if adapter_spec:
        if args.policy_path or args.policy_dataset_root:
            raise SystemExit("--policy-adapter and --policy-path/--policy-dataset-root are mutually exclusive")
        adapter = load_adapter(adapter_spec)
        log(f"[policy] custom adapter loaded: {adapter_spec}")

        def infer(state: np.ndarray, images: dict[str, np.ndarray]) -> np.ndarray:
            return np.asarray(adapter.predict(state, images), dtype=np.float64)

    else:
        if not args.policy_path or not args.policy_dataset_root:
            raise SystemExit(
                "--input policy needs either --policy-adapter, or both "
                "--policy-path and --policy-dataset-root"
            )
        import torch  # noqa: F401 - imported for its side effect of registering torch dtypes
        from lerobot.configs.policies import PreTrainedConfig
        from lerobot.datasets.lerobot_dataset import LeRobotDatasetMetadata
        from lerobot.policies.factory import make_policy, make_pre_post_processors
        from lerobot.utils.control_utils import predict_action
        from lerobot.utils.utils import get_safe_torch_device

        log(f"[policy] loading checkpoint from {args.policy_path}")
        ds_meta = LeRobotDatasetMetadata(repo_id=args.policy_dataset_repo_id, root=args.policy_dataset_root)
        policy_cfg = PreTrainedConfig.from_pretrained(args.policy_path)
        policy = make_policy(policy_cfg, ds_meta=ds_meta)
        preprocessor, postprocessor = make_pre_post_processors(policy_cfg, pretrained_path=args.policy_path)
        device = get_safe_torch_device(policy_cfg.device)
        policy.reset()
        log(f"[policy] {policy_cfg.type} loaded on {device}")

        def infer(state: np.ndarray, images: dict[str, np.ndarray]) -> np.ndarray:
            obs = {"observation.state": state}
            obs.update({f"observation.images.{k}": v for k, v in images.items()})
            action = predict_action(
                obs, policy, device, preprocessor, postprocessor, use_amp=False, task=args.policy_task
            )
            # predict_action's docstring claims the batch dim is removed; it
            # isn't (verified empirically) - squeeze it before writing ctrl
            return action.squeeze(0).numpy()

    session = TeleopSession(args)
    model, data = session.model, session.data

    cam_renderers, camera_names, scene_option = make_camera_renderers(
        model, args.policy_camera, args.policy_width, args.policy_height
    )
    if not camera_names:
        log(f"[policy] no valid cameras in {args.policy_camera!r} - running on state only")

    loop_dt = 1.0 / config.LOOP_HZ
    last = time.perf_counter()

    with mujoco.viewer.launch_passive(model, data) as viewer:
        session.setup_viewer_cam(viewer)
        while viewer.is_running():
            start = time.perf_counter()
            dt = min(max(start - last, model.opt.timestep), 1.0 / 60.0)
            last = start

            state = build_contract_state(model, data, session)
            images: dict[str, np.ndarray] = {}
            for cam in camera_names:
                renderer, _, _ = cam_renderers[cam]
                renderer.update_scene(data, camera=cam, scene_option=scene_option)
                images[cam] = renderer.render()

            action = infer(state, images)
            apply_contract_action(model, data, session, action, float(model.opt.timestep))

            session.step_once(dt)
            viewer.sync()

            sleep = loop_dt - (time.perf_counter() - start)
            if sleep > 0:
                time.sleep(sleep)
