# Task 3 Bean Spawn Height Design

## Goal

Spawn Task 3 coffee beans 5 cm higher than their current position so their
fall into the bowl is visible when simulation starts.

## Design

Change `BEAN_PHYSICS["spawn_height"]` from `0.02` m to `0.07` m. Keep the
existing spawn-position algorithm unchanged: every bean layer remains
relative to the live bowl bounds, and only the vertical base offset changes.

## Verification

Add a focused unit assertion that the first generated bean position is 0.07 m
above the container's minimum Z bound. Run the scene-builder test module, then
run the Task 3 launcher tests to catch integration regressions.

## Scope

No changes to bean size, spacing, count, density, collision, rigid-body
configuration, or bowl placement.
