#!/usr/bin/env python3
"""Static regression check for the key-7 ArmState name-shadowing bug."""
from __future__ import annotations
import ast
from pathlib import Path

path = Path(__file__).with_name("teleop_keyboard.py")
tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))

function_defs = [n for n in ast.walk(tree) if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef)) and n.name == "selected_arm"]
stores = [n for n in ast.walk(tree) if isinstance(n, ast.Name) and isinstance(n.ctx, ast.Store) and n.id == "selected_arm"]

if len(function_defs) != 1:
    raise SystemExit(f"FAIL: expected one selected_arm() function, found {len(function_defs)}")
if stores:
    lines = ", ".join(str(getattr(n, "lineno", "?")) for n in stores)
    raise SystemExit(f"FAIL: selected_arm is overwritten at line(s): {lines}")

source = path.read_text(encoding="utf-8")
required = [
    'elif key in (glfw.KEY_7, glfw.KEY_KP_7):',
    'set_mode("base")',
    'active_arm = arms[mode[0]]',
]
for item in required:
    if item not in source:
        raise SystemExit(f"FAIL: missing expected code: {item}")

print("PASS: selected_arm() remains callable after physics steps and key 7 selects base mode.")
