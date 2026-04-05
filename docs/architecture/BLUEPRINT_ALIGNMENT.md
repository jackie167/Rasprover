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
  Status: compatibility layer. The primary control node implementations and arbiters now live in `rasprover_control`, while `rasprover_mux` remains as a legacy wrapper package.

- `ros2_ws/src/rasprover_web`
  Target: `rasprover_ui`
  Status: compatibility layer. The primary web bridge implementation now lives in `rasprover_ui`, while `rasprover_web` remains as a legacy wrapper package.

- `ros2_ws/src/rasprover_localization`
  Target: split between `rasprover_base`, `rasprover_slam`, and `rasprover_utils`
  Status: compatibility layer. Legacy bridge entrypoint and legacy config paths remain here for backward compatibility.

- `ros2_ws/src/rasprover_slam`
  Target: `rasprover_slam`
  Status: aligned as the package for SLAM launch/config ownership.

- `ros2_ws/src/rasprover_sensors`
  Target: `rasprover_sensors`
  Status: aligned. Owns the runtime sensor bridge today and is the future home for LiDAR/camera/static-TF integration.

- `ros2_ws/src/rasprover_utils`
  Target: `rasprover_utils`
  Status: placeholder package added to reserve a ROS-side home for diagnostics and utility helpers.

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

## Bringup Status

- Existing compatibility launches remain:
  - `motion_stack.launch.py`
  - `localization_stack.launch.py`
  - `slam_stack.launch.py`

- New blueprint-facing bringup modes now exist in `rasprover_bringup/launch`:
  - `base_only.launch.py`
  - `teleop.launch.py`
  - `web_control.launch.py`
  - `slam.launch.py`
  - `full_system.launch.py`

These new launch files prefer blueprint-aligned package names such as `rasprover_control`, `rasprover_ui`, and `rasprover_slam`, while compatibility paths remain available through the legacy package names.

## Ownership Status

- `rasprover_control`
  Owns the runtime implementations for mux, joystick bridge, local joy input, and arbiter helpers.

- `rasprover_ui`
  Owns the runtime implementation for the web bridge.

- `rasprover_sensors`
  Owns the runtime implementation for the sensor bridge that converts raw ESP feedback into ROS-standard IMU and wheel odometry topics.

- `rasprover_mux`
  Kept only for compatibility entrypoints and import forwarding.

- `rasprover_web`
  Kept only for compatibility entrypoints and import forwarding.

- `rasprover_localization`
  Kept for compatibility entrypoints and legacy config paths while runtime ownership moves into `rasprover_sensors` and `rasprover_slam`.

## Deferred Structural Refactors

These are intentionally not done yet because they would change package names, imports, or operational entrypoints:

- rename `rasprover_mux` to `rasprover_control`
- rename `rasprover_web` to `rasprover_ui`
- relocate legacy root Python modules like `base_ctrl.py` and `base_driver.py` into package-local adapter folders

## Runtime Rule Kept Intact

No runtime code path was changed by this organization pass. Existing commands still work through root wrappers and current package names.
