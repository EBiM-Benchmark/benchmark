# Task 3 Keyboard Dual-Arm Teleoperation Design

## Objective

Add usable keyboard control for the Task 3 mobile FR3 Duo while keeping the
control stack independent of its input device. The first iteration supports
the existing keyboard-driven mobile base plus keyboard Cartesian control of
both arms, grippers, and spine. ROS 2, GELLO, and foot-pedal adapters are
explicitly deferred, but their future addition must not require redesigning
the command or robot-control layers.

## Scope

This iteration will:

- preserve the existing WASD/QE mobile-base behavior;
- represent operator intent with simulator-independent command types;
- convert held keyboard keys into time-scaled Cartesian pose increments;
- support independent selection and motion of the left and right arm;
- solve each arm target with Isaac Sim Lula inverse kinematics;
- control both grippers and the vertical spine joint;
- compose joint targets without resetting unrelated joints every frame;
- reject stale commands and retain bounded targets;
- provide pure unit tests for all simulator-independent behavior; and
- document the extension boundary for future `teleoperation` ROS 2 adapters.

This iteration will not:

- add ROS 2 nodes, topics, or package dependencies;
- connect physical GELLO, pedal, or robot hardware;
- extract the binary Action Graph from `Tabletop_DEMO.usd`;
- add RMPFlow or obstacle avoidance; or
- change Task 3 grading behavior.

## Selected Approach

Use small Python components around the current Task 3 control loop. Importing
the archived demo extension directly was rejected because it is coupled to its
UI, target cubes, and robot-description paths. Recreating its USD Action Graph
was rejected because the embedded scripts are difficult to inspect, test, and
maintain.

The selected approach recreates the useful behavior as ordinary Python while
using the archived `Robotiq_DEMO` branch as the reference for dual Lula solver
configuration and action merging.

## Architecture

The runtime data flow is:

```text
Keyboard state
    -> keyboard command mapper
    -> unified teleoperation command
    -> command freshness/arbitration
    -> Cartesian target tracker
    -> dual Lula IK adapter
    -> joint-target composer
    -> Isaac Lab articulation API
    -> Isaac Sim
```

Future integrations insert a ROS 2/GELLO adapter before the unified command
without changing the target tracker, IK adapter, target composer, or simulation
loop.

## Components

### Unified command model

A simulator-independent module will define immutable command values for:

- body-frame mobile velocity `(vx, vy, wz)`;
- left and right Cartesian pose deltas;
- left and right gripper deltas;
- spine delta;
- monotonic timestamp;
- input source identifier; and
- an active/deadman state.

The command model must not import Isaac Sim, Isaac Lab, ROS, or PyTorch. A
freshness helper will turn stale or inactive commands into a safe stop command.
This supplies the contract that a future ROS 2 adapter will implement.

### Keyboard mapper

The existing held-key set remains the keyboard input. Base mappings stay
unchanged: W/S translate longitudinally, A/D laterally, and Q/E or arrow keys
rotate.

To prevent collisions with base keys, arm motion uses an explicit control
mode. The mapper supports base, left-arm, and right-arm modes, with visible
mode changes. In either arm mode, the same ergonomic translation/rotation keys
produce Cartesian deltas for only the selected arm. Gripper and spine commands
use dedicated keys. All pose changes are rates multiplied by simulation `dt`,
not increments tied to keyboard-repeat frequency.

Key constants and help text live with the mapper so tests and runtime output
cannot silently diverge.

### Cartesian target tracker

At activation, each target initializes from the corresponding simulated end
effector pose. It applies bounded translation and rotation deltas in a clearly
defined frame and stores normalized quaternions. Translation, rotation, spine,
and gripper limits are enforced before producing controller targets.

The first implementation uses the robot/base frame for Cartesian increments,
so behavior stays intuitive while the mobile base moves. World/base pose
conversion belongs in the simulator adapter rather than the input mapper.

### Dual Lula IK adapter

The Isaac Sim adapter owns two independent Lula solvers, one for each seven-DOF
FR3 arm. It resolves robot-description and URDF paths relative to the project,
never through hardcoded home-directory paths. Each step receives left and right
target poses and returns arm joint targets indexed by articulation joint name.

An IK failure retains the last valid target for that arm and emits a
rate-limited warning; failure in one arm does not suppress a valid solution for
the other. Solver initialization and runtime imports stay inside the Isaac Sim
adapter so pure tests can run outside Kit.

### Joint-target composition

The composer discovers joint IDs by exact names/patterns and partitions them
into steering, drive, left arm, right arm, grippers, and spine. It begins from
the last commanded or measured position target, updates only the groups owned
by active commands, and sends wheel speed through the existing velocity-target
path.

This replaces the current per-frame cloning of `default_joint_pos`, which
would otherwise overwrite every IK, gripper, and spine command.

### Task 3 runtime integration

`scene_robot_room_keyboard.py` remains the executable entry point. Its setup
will create the keyboard mapper, target tracker, Lula adapter, and joint-target
composer after the articulation is initialized. The main loop will:

1. snapshot held keys;
2. map them to one unified command using the current monotonic time and `dt`;
3. apply command freshness and target bounds;
4. solve requested arm targets;
5. compose position and velocity targets by joint group; and
6. step Isaac Sim through the existing Isaac Lab scene adapter.

`--no-keyboard-control` remains an Isaac Sim viewer-only path. Existing Task 3
scene construction and grading data remain unchanged.

## Future ROS 2 and GELLO Compatibility

A future adapter may consume `/keyboard/state`, namespaced GELLO joint states,
gripper values, and pedal state from the `teleoperation` repository. It will
translate those messages into the same unified command contract.

Keyboard Cartesian control will continue through Lula IK. GELLO may provide
validated absolute joint targets directly, bypassing Cartesian IK while still
passing through freshness checks, joint limits, source arbitration, and the
same target composer. Only one source may own a joint group at a time. Source
priority and takeover policy are deferred until ROS 2 integration because this
iteration has only one command source.

## Safety and Error Handling

- Held-key commands are continuous and stop immediately on key release.
- Stale or inactive input produces zero velocities and no new pose delta.
- Cartesian, gripper, spine, joint-velocity, and joint-position targets remain
  within configured limits.
- Missing required joints or end-effector frames fail at initialization with a
  diagnostic listing the missing names.
- Per-arm IK failure retains that arm's last valid target.
- Existing wheel-speed limiting and heading compensation remain active.
- Escape and `Ctrl+C` retain their current shutdown behavior.

This is simulation teleoperation, not a certified physical-robot safety
controller. Physical integration will require hardware emergency-stop and
watchdog behavior outside this scope.

## Testing

Pure tests, runnable without Isaac Sim, will cover:

- every keyboard mode and key mapping;
- simultaneous and opposing key behavior;
- `dt`-scaled Cartesian increments;
- command freshness and inactive-command behavior;
- quaternion normalization and target bounds;
- joint discovery and partition validation;
- selective target composition without unrelated-joint reset;
- independent left/right IK-result merging using a fake solver boundary; and
- help text derived from the actual mappings.

Existing base-control, Task 3 scene, grading unit, and integration tests remain
part of regression verification. Isaac Sim integration verification will check
that the robot initializes, base commands still move the base, and arm targets
can be applied without being overwritten. Hardware and ROS 2 tests are deferred.

## Acceptance Criteria

- Task 3 starts with the existing keyboard-controlled mobile base intact.
- An operator can select either arm and command translational and rotational
  end-effector motion from the keyboard.
- Both arm targets can be maintained independently through Lula IK.
- Gripper and spine targets can be changed without disturbing the base or arms.
- Releasing keys stops incremental motion, and stale input causes a safe stop.
- No loop iteration resets all joints to their default pose.
- Simulator-independent unit tests run in the normal development environment.
- Existing relevant tests pass.
- No ROS 2 or GELLO runtime dependency is introduced.
- The unified command boundary is documented sufficiently for a future
  `teleoperation` adapter.
