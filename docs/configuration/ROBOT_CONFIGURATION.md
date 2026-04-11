# Robot Configuration Ledger

This project now keeps robot-sensitive runtime parameters in a small number of
authoritative files so the active configuration can be audited after hardware
changes or calibration work.

## Canonical Sources

- App/UI defaults: [config.yaml](/home/ws/ugv_rpi/config.yaml)
- Reference baseline config: [robot_baseline.yaml](/home/ws/ugv_rpi/ros2_ws/src/rasprover_bringup/config/robot_baseline.yaml)
- Active hardware-sensitive runtime config for ROS bringup: [robot_runtime.yaml](/home/ws/ugv_rpi/ros2_ws/src/rasprover_bringup/config/robot_runtime.yaml)
- Audit ledger with defaults/current/provenance: [robot_parameter_ledger.yaml](/home/ws/ugv_rpi/docs/configuration/robot_parameter_ledger.yaml)

## Runtime Rule

For hardware-sensitive ROS runtime parameters, the active source is a single
file:

- [robot_runtime.yaml](/home/ws/ugv_rpi/ros2_ws/src/rasprover_bringup/config/robot_runtime.yaml)

The untouched design/reference layer lives here:

- [robot_baseline.yaml](/home/ws/ugv_rpi/ros2_ws/src/rasprover_bringup/config/robot_baseline.yaml)

Launch files now read from that file through `rasprover_bringup.runtime_config`.
If the robot is behaving differently from expected, this is the first file to
inspect.

Calibration CSV files are evidence and measurement history, not the active
runtime source.

The newest machine-readable recommendation is written to:

- `tools/calibration/latest_recommendation.yaml`

That file is not the active runtime source either. It is only the pending
change set waiting to be reviewed and optionally applied into
`robot_runtime.yaml`.

Historical recommendation files are stored under:

- `tools/calibration/captures/*_recommendation.yaml`

## Three Layers

- `robot_baseline.yaml`
  The original reference/default values. Do not auto-write to this file during calibration.
- `robot_runtime.yaml`
  The only active runtime file used by the system.
- `captures/*.csv` and `*_recommendation.yaml`
  Measurement history and pending/applicable recommendations.

## Why Two Sensor Profiles In One File

The repo was already using different odometry trims for different operating
modes:

- motion/web stacks were effectively neutral
- SLAM/path stacks were using calibrated odometry scaling

That split is now explicit instead of being hidden inside hardcoded launch
files, but both profiles still live in the same runtime config file.

## What To Check When Changing Robot Hardware

Review these fields first:

- `serial_port`
- `left_drive_scale`
- `right_drive_scale`
- `feedback_wheel_separation_m`
- `feedback_wheel_yaw_scale`
- `wheel_yaw_scale`
- `linear_odom_scale`
- `left_odom_scale`
- `right_odom_scale`

If any of these no longer match the robot, update the bringup config YAML and
the ledger in the same change.

## Mapping Table

| App / Node / Tool | Main parameters it affects or depends on | Active file to update after accepting calibration |
| --- | --- | --- |
| `tools/calibration/yaw_calibration.py` | `feedback_wheel_yaw_scale`, `wheel_yaw_scale` | [robot_runtime.yaml](/home/ws/ugv_rpi/ros2_ws/src/rasprover_bringup/config/robot_runtime.yaml) |
| `tools/calibration/linear_calibration.py` | `linear_odom_scale`, `left_odom_scale`, `right_odom_scale`, validates `wheel_separation_m` | [robot_runtime.yaml](/home/ws/ugv_rpi/ros2_ws/src/rasprover_bringup/config/robot_runtime.yaml) |
| `tools/calibration/straight_controller_tune.py` | `straight_controller_*` gains | [robot_runtime.yaml](/home/ws/ugv_rpi/ros2_ws/src/rasprover_bringup/config/robot_runtime.yaml) |
| `robot_base_node` | drive command scales, wheel separation, yaw scale, straight controller | [robot_runtime.yaml](/home/ws/ugv_rpi/ros2_ws/src/rasprover_bringup/config/robot_runtime.yaml) |
| `slam_sensor_bridge_node` | odom scales, wheel yaw scale, wheel separation | [robot_runtime.yaml](/home/ws/ugv_rpi/ros2_ws/src/rasprover_bringup/config/robot_runtime.yaml) |
| `nav_cmd_vel_bridge_node` | navigation velocity clamps | [robot_runtime.yaml](/home/ws/ugv_rpi/ros2_ws/src/rasprover_bringup/config/robot_runtime.yaml) |

## Calibration Workflow

1. Run the relevant calibration tool in [tools/calibration](/home/ws/ugv_rpi/tools/calibration).
2. Save the generated CSV under `tools/calibration/captures/`.
3. Inspect `tools/calibration/latest_recommendation.yaml`.
4. Apply it with `python3 tools/calibration/apply_calibration_recommendation.py` when the recommendation looks correct.
5. Confirm the new values landed in [robot_runtime.yaml](/home/ws/ugv_rpi/ros2_ws/src/rasprover_bringup/config/robot_runtime.yaml).
6. Update [robot_parameter_ledger.yaml](/home/ws/ugv_rpi/docs/configuration/robot_parameter_ledger.yaml) with:
   - old value
   - new value
   - capture file path
   - date/reason
7. Re-test the affected mode, for example motion stack or SLAM stack.

## Useful Commands

```bash
python3 tools/diagnostics/robot_config_report.py
python3 tools/calibration/apply_calibration_recommendation.py --dry-run
python3 tools/calibration/apply_calibration_recommendation.py
python3 tools/diagnostics/calibration_history.py
```

## Current Risk Reduced By This Layout

Previously, the most important hardware trims were scattered across launch
files. That made it easy to tune one mode and forget another. The runtime now
loads those values from one active config file, which makes review and robot
swaps much safer.
