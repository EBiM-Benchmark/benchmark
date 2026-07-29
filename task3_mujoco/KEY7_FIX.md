# Key 7 mode-switch fix

## Symptom

Pressing `7` after the simulation had started raised:

```text
TypeError: 'ArmState' object is not callable
```

## Cause

`selected_arm` was first defined as a helper function. Later in the main loop, the same name was reused for an `ArmState` instance during post-step wall-contact checking. Python therefore replaced the function reference with the object. The next discrete-key event called `selected_arm()`, causing the exception.

## Correction

The post-step temporary variable is now named `active_arm`. The `selected_arm()` helper remains callable for the entire simulation.

## Validation

Run:

```bash
python validate_mode_switch_fix.py
```
