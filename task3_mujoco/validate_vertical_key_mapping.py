#!/usr/bin/env python3
from pathlib import Path
import importlib.util, sys
import mujoco
import numpy as np
ROOT=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location("teleop_mapping", ROOT/"teleop_keyboard.py")
t=importlib.util.module_from_spec(spec); sys.modules[spec.name]=t; spec.loader.exec_module(t)
model=mujoco.MjModel.from_xml_path(str(ROOT/"benchmark_task3_mesh_table.xml"))
data=mujoco.MjData(model); t.apply_ready_pose(model,data); mujoco.mj_forward(model,data)
arm=t.create_arm(model,data,"right")
base_body=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,"mobile_base")
operator=np.array([0.,0.,0.45,0.,0.,0.])
base_world=t.map_arm_operator_twist_to_world(None,data,arm.tcp_body,base_body,operator,"base","tcp","+x")
tcp_world=t.map_arm_operator_twist_to_world(None,data,arm.tcp_body,base_body,operator,"tcp","tcp","+x")
print("Default base-frame U/PageUp world velocity:",base_world[:3])
print("Legacy TCP-frame U/PageUp world velocity:",tcp_world[:3])
ok=base_world[2] > 0.449 and tcp_world[2] < -0.449
print("VERTICAL KEY MAPPING:","PASS" if ok else "FAIL")
raise SystemExit(0 if ok else 1)
