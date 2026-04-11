# Blueprint Alignment

This note maps the current repository layout to the target blueprint without changing runtime behavior yet.

## Current To Target Mapping

- `ros2_ws/src/rasprover_base`
  Target: `rasprover_base`
  Status: aligned. Owns the serial and protocol boundary. ROS-facing code now imports legacy root modules only through package-local shims inside `rasprover_base`.

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

- `ops/`
  Owns operator shell entrypoints for start, stop, status, restart, and setup flows.

- root Python runtime wrappers
  The old root-level runtime wrappers have been retired. Runtime ownership now
  lives in ROS packages and archived fallback code has been moved out of the
  active root path.

- `tutorial_en/legacy_runtime_archive/`
  Holds archived fallback Flask-first app code for reference only. It is no
  longer part of the active runtime.

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
  - `slam_noekf.launch.py`
  - `slam_ekf.launch.py`

These launch files now share common node construction helpers under
`rasprover_bringup.launch_builders`, which reduces duplication between runtime
and test bringup modes while keeping compatibility paths available through the
legacy package names.

## Ownership Status

- `rasprover_base`
  Owns the ROS hardware boundary and the package-local shims that isolate legacy `base_driver`, `base_ctrl`, and `state_store` access.

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

- `tutorial_en/legacy_runtime_archive/`
  Holds retired fallback app support modules such as the old in-process mux and
  command service logic for code-reading only.

## Deferred Structural Refactors

These are intentionally not done yet because they would change package names, imports, or operational entrypoints:

- rename `rasprover_mux` to `rasprover_control`
- rename `rasprover_web` to `rasprover_ui`
- continue internal cleanup inside `rasprover_base` and `rasprover_cv` so legacy implementation details become easier to replace later

## Runtime Rule Kept Intact

The active runtime remains ROS-first. Legacy fallback runtime code has been
retired from operation and archived for reference only.

## Retirement Direction

The fallback app path has been retired. Future cleanup should focus on
simplifying package-internal legacy implementation details rather than adding
new top-level entrypoints.
