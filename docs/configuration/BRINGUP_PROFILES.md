# Bringup Profiles

This note classifies the launch profiles under `rasprover_bringup/launch` so
it is obvious which ones are normal runtime stacks and which ones are only for
focused testing or diagnostics.

## Main Runtime Profiles

- [motion_stack.launch.py](/home/ws/ugv_rpi/ros2_ws/src/rasprover_bringup/launch/motion_stack.launch.py)
  Base motion + sensor bridge + mux + joystick bridge + web bridge.
- [navigation_stack.launch.py](/home/ws/ugv_rpi/ros2_ws/src/rasprover_bringup/launch/navigation_stack.launch.py)
  Nav2 localization/navigation layer and nav-to-CV velocity bridge.
- [localization_stack.launch.py](/home/ws/ugv_rpi/ros2_ws/src/rasprover_bringup/launch/localization_stack.launch.py)
  Sensor bridge and EKF only.
- [slam_stack.launch.py](/home/ws/ugv_rpi/ros2_ws/src/rasprover_bringup/launch/slam_stack.launch.py)
  EKF + slam_toolbox only.

## Focused Manual Profiles

- [base_only.launch.py](/home/ws/ugv_rpi/ros2_ws/src/rasprover_bringup/launch/base_only.launch.py)
  Minimal base + sensor bridge bringup for low-level checks.
- [teleop.launch.py](/home/ws/ugv_rpi/ros2_ws/src/rasprover_bringup/launch/teleop.launch.py)
  Manual driving stack without web bridge.
- [web_control.launch.py](/home/ws/ugv_rpi/ros2_ws/src/rasprover_bringup/launch/web_control.launch.py)
  Manual web-driven stack without local joystick.
- [slam.launch.py](/home/ws/ugv_rpi/ros2_ws/src/rasprover_bringup/launch/slam.launch.py)
  Full manual SLAM-focused stack with lidar and static TF in one launch.
- [slam_noekf.launch.py](/home/ws/ugv_rpi/ros2_ws/src/rasprover_bringup/launch/slam_noekf.launch.py)
  Manual mapping stack with control path, lidar, and `slam_toolbox`, but no EKF.
- [slam_ekf.launch.py](/home/ws/ugv_rpi/ros2_ws/src/rasprover_bringup/launch/slam_ekf.launch.py)
  Manual mapping stack with the same control path and lidar setup as `slam_noekf`, plus EKF.

## Test / Diagnostic Profiles

- [path_test.launch.py](/home/ws/ugv_rpi/ros2_ws/src/rasprover_bringup/launch/path_test.launch.py)
  Comparison stack for odometry path validation. This is test-only and should
  not be treated as a normal operator bringup mode.

## Standardization Notes

- Shared base/sensor/mux/joy/web node construction now lives in
  `rasprover_bringup.launch_builders`.
- Hardware-sensitive runtime parameters still come from
  [robot_runtime.yaml](/home/ws/ugv_rpi/ros2_ws/src/rasprover_bringup/config/robot_runtime.yaml).
- `path_test` remains available, but it is intentionally documented as
  diagnostic-only so it is not confused with the main runtime paths.
