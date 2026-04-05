# Blueprint Alignment

This note maps the current repository layout to the target blueprint without changing runtime behavior yet.

## Current To Target Mapping

- `ros2_ws/src/rasprover_base`
  Target: `rasprover_base`
  Status: aligned. Owns the serial and protocol boundary.

- `ros2_ws/src/rasprover_msgs`
  Target: `rasprover_msgs`
  Status: aligned. Owns ROS message contracts.

- `ros2_ws/src/rasprover_mux`
  Target: `rasprover_control`
  Status: functional match, naming still legacy. Owns mux and joystick teleop behavior today.

- `ros2_ws/src/rasprover_web`
  Target: `rasprover_ui`
  Status: functional match, naming still legacy. Owns web/operator bridge behavior.

- `ros2_ws/src/rasprover_localization`
  Target: split between `rasprover_base`, `rasprover_slam`, and `rasprover_utils`
  Status: temporary integration package. Currently hosts the sensor bridge and SLAM-facing configs.

- `ros2_ws/src/rasprover_bringup`
  Target: `rasprover_bringup`
  Status: aligned. Owns launch profiles and startup modes.

- `ros2_ws/src/rasprover_cv`
  Target: `rasprover_cv`
  Status: aligned in intent.

## Root-Level Organization

- `docs/`
  Design references and migration notes.

- `tools/`
  Diagnostics, calibration, and safe build helpers.

- root shell entrypoints
  Kept in place intentionally for operator convenience and backwards compatibility.

## Deferred Structural Refactors

These are intentionally not done yet because they would change package names, imports, or operational entrypoints:

- rename `rasprover_mux` to `rasprover_control`
- rename `rasprover_web` to `rasprover_ui`
- split `rasprover_localization` into blueprint-native packages such as `rasprover_slam`, `rasprover_sensors`, or `rasprover_utils`
- relocate legacy root Python modules like `base_ctrl.py` and `base_driver.py` into package-local adapter folders

## Runtime Rule Kept Intact

No runtime code path was changed by this organization pass. Existing commands still work through root wrappers and current package names.
