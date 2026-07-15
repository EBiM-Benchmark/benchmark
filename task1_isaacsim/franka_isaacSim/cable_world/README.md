# Board Cable Newton Parser

Minimal entry point for parsing the authored `board_cable.usda` BasisCurves cable
with Newton. The script reuses the package cable importer in
`usd_cable_curve_import.py`.

This directory is a self-contained Board Cable package with an A1-style layout:

- `assets/` stores the cable USD and board scene USD.
- `configs/` stores scene and gripper YAML files.
- `run_board_cable.py`, `usd_cable_curve_import.py`, and `sra_gripper.py` are the
  runtime scripts needed by this demo.

The demo follows the A1-style config split:

- `configs/board_cable.yaml` stores the cable USD, board scene USD, solver, contact,
  shape, cable, board, and ground parameters.
- `configs/gripper_board_cable.yaml` stores the Franka gripper pose, finger friction,
  drive stiffness, gap limits, gravity compensation, and teleop settings.

`run_board_cable.py` loads those YAML files by default. Command-line arguments
still work as temporary overrides:

```bash
python run_board_cable.py --viewer gl --device cuda:0 \
  --config-path configs/board_cable.yaml \
  --gripper-config-path configs/gripper_board_cable.yaml
```

By default it also loads the board scene USD at `assets/board/board.usd` with
`ModelBuilder.add_usd(..., root_path="/World")`. This scene references the
board and its components; their `Collisions` hierarchies already author
`PhysicsCollisionAPI` meshes with `physics:approximation = convexHull`, so
Newton imports those meshes as convex hull colliders. Visual meshes are not
forced to collide.

Run a quick collision smoke test from the package directory. This skips the
large visual mesh but still loads the `Collisions` convex hulls:

```bash
python run_board_cable.py --viewer null --num-frames 1 --device cpu \
  --no-gripper --no-board-load-visual-shapes --require-board-convex-collision
```

Run with the GL viewer:

```bash
python run_board_cable.py --viewer gl --device cuda:0 --require-board-convex-collision
```

The demo defaults to a stability-first VBD setup. The cable has 576 tiny capsule
segments, so over-hard contact or excessive damping can inject large impulses and
make the cable fly away. The script softens body contact, overrides the authored
`newton:bend_damping = 1000` with a small value, disables hard contact/history by
default, and filters self-collision between nearby cable segments. The baseline
values live in `configs/board_cable.yaml` under `solver`, `contact`, `shape`, and
`cable`.

By default the board and components use only the authored convex collisions from
`board.usd`. If long-running cable-board contact leaks through the thin board
surface, add an invisible Newton support plane at the board top height with
`--board-support-plane`.

Use this as the baseline GL run:

```bash
python run_board_cable.py --viewer gl --device cuda:0 \
  --shape-ke 5000 \
  --shape-kd 0.01 \
  --bend-damping 0.05 \
  --contact-margin 0.0005 \
  --rigid-gap 0 \
  --no-rigid-contact-hard \
  --no-rigid-contact-history
```

The demo still defaults to higher cable-board friction than the source USD so
the cable is less likely to slide off the board. Tune both sides of the contact
like this:

```bash
python run_board_cable.py --viewer gl --device cuda:0 \
  --friction 3.0 \
  --board-friction 3.0
```

If the cable still explodes or twitches badly, reduce contact strength first:

```bash
python run_board_cable.py --viewer gl --device cuda:0 \
  --shape-ke 2500 \
  --shape-kd 0.005 \
  --friction 1.0 \
  --board-friction 1.0 \
  --contact-margin 0 \
  --rigid-gap 0 \
  --cable-self-collision-filter-neighbor-hops 4
```

Add and show the optional invisible support plane while debugging with:

```bash
python run_board_cable.py --viewer gl --device cuda:0 \
  --board-support-plane \
  --board-support-plane-visible
```

The older finite box-style top support collider is still available for
comparison, but the plane is the recommended anti-penetration support:

```bash
python run_board_cable.py --viewer gl --device cuda:0 \
  --board-top-support-collision \
  --board-top-support-visible
```

Teleop is enabled by default with a Franka gripper placed near the middle of the
cable. Hold `Ctrl` while using `W/S/A/D/Q/E` for translation, `C/V`, `Z/X`, and
`T/G` for rotation, and `N/M` to close/open the finger gap. The gripper defaults
to a stronger but still force-limited close for cable grasping. The baseline
values live in `configs/gripper_board_cable.yaml`; override them temporarily like this:

```bash
python run_board_cable.py --viewer gl --device cuda:0 \
  --gripper-finger-friction 20 \
  --gripper-drive-force 200 \
  --gripper-stiffness 5000 \
  --gripper-damping 20
```

If the cable still slips, increase `--gripper-finger-friction` or close the gap
with `N`. If the cable jumps on contact, reduce `--gripper-drive-force` first.

The default gripper pose is computed from the cable points. Override it when
tuning a grasp:

```bash
python run_board_cable.py --viewer gl --device cuda:0 \
  --gripper-position 0.49 0.735 0.12 \
  --gripper-rotation-euler-xyz-deg 0 180 0 \
  --gripper-target-gap 0.04
```

Require the board USD to produce at least one colliding shape:

```bash
python run_board_cable.py --viewer null --num-frames 1 --require-board-collision
```

Require convex hull collision specifically:

```bash
python run_board_cable.py --viewer null --num-frames 1 --require-board-convex-collision
```
