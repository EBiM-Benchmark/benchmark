#!/usr/bin/env python3
# Copyright (c) 2026 The EBiM Benchmark Contributors
# SPDX-License-Identifier: Apache-2.0

"""Inspect the structure of a USD file."""

import sys

from pxr import Usd

if len(sys.argv) < 2:
    print("用法: python inspect_usd.py <usd文件路径>")
    sys.exit(1)

usd_path = sys.argv[1]
stage = Usd.Stage.Open(usd_path)

if not stage:
    print(f"无法打开文件: {usd_path}")
    sys.exit(1)

print(f"\n场景文件: {usd_path}")
print("=" * 80)
print("Prim 层级结构:\n")

for prim in stage.Traverse():
    depth = len(str(prim.GetPath()).split("/")) - 2
    indent = "  " * depth
    prim_type = prim.GetTypeName()
    print(f"{indent}{prim.GetPath()} ({prim_type})")

print("\n" + "=" * 80)
