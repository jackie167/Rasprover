# Bringup Modes

This runbook shows the supported runtime entrypoints after the ROS-first restructuring.

Related references:
- `/home/ws/ugv_rpi/docs/runbooks/SLAM_BRINGUP_DEBUG_SUMMARY.md`
- `/home/ws/ugv_rpi/docs/configuration/SLAM_LIDAR_KNOWN_GOOD.yaml`

## Primary Runtime Modes

### Base Only

Purpose:
- verify serial access to the base board
- verify `/robot/state/feedback_raw`
- verify the sensor bridge path to `/imu/data_raw` and `/wheel/odometry`

Commands:
```bash
cd /home/ws/ugv_rpi
source /home/ws/ugv_rpi/ros2_ws/install/setup.bash
export ROS_LOG_DIR=/home/ws/ugv_rpi/.roslog
ros2 launch rasprover_bringup base_only.launch.py
```

### Motion Stack

Purpose:
- base runtime
- sensor bridge
- command mux
- joystick bridge
- web bridge
- optional CV node

Commands:
```bash
cd /home/ws/ugv_rpi
bash ./ops/start_ros_motion_stack.sh
bash ./ops/status_ros_motion_stack.sh
bash ./ops/stop_ros_motion_stack.sh
```

To include CV in the operator shell flow:
```bash
cd /home/ws/ugv_rpi
WITH_CV=true bash ./ops/start_ros_motion_stack.sh
bash ./ops/status_ros_motion_stack.sh
bash ./ops/stop_ros_motion_stack.sh
```

### SLAM Stack

Purpose:
- base runtime
- sensor bridge
- EKF
- `slam_toolbox`

Commands:
```bash
cd /home/ws/ugv_rpi
bash ./ops/start_ros_slam_stack.sh
bash ./ops/status_ros_slam_stack.sh
bash ./ops/stop_ros_slam_stack.sh
```

This stack is structurally ready before LiDAR arrives, but mapping only becomes meaningful once `/scan` is available.

### SLAM No EKF

Purpose:
- drive the robot manually while mapping
- use the calibrated `slam_sensor_bridge_node` `slam` profile
- run LiDAR + static TF + `slam_toolbox`
- skip EKF entirely

Commands:
```bash
cd /home/ws/ugv_rpi
bash ./ops/start_ros_slam_noekf_stack.sh
bash ./ops/status_ros_slam_noekf_stack.sh
bash ./ops/stop_ros_slam_noekf_stack.sh
```

This is the simplest operator-facing mapping mode when you want manual driving plus SLAM without the EKF layer.

### SLAM EKF

Purpose:
- drive the robot manually while mapping
- use the calibrated `slam_sensor_bridge_node` `slam` profile
- run LiDAR + static TF + EKF + `slam_toolbox`

Commands:
```bash
cd /home/ws/ugv_rpi
bash ./ops/start_ros_slam_ekf_stack.sh
bash ./ops/status_ros_slam_ekf_stack.sh
bash ./ops/stop_ros_slam_ekf_stack.sh
```

Use this when you want the same operator-facing mapping flow as `slam_noekf`, but with fused odometry through EKF.

### Navigation Stack

Purpose:
- Nav2 localization and navigation services
- `nav_cmd_vel_bridge_node` to feed motion control through the shared runtime config

Commands:
```bash
cd /home/ws/ugv_rpi
bash ./ops/start_ros_nav_stack.sh
bash ./ops/status_ros_nav_stack.sh
bash ./ops/stop_ros_nav_stack.sh
```

Use this after the base/motion/localization layers are already healthy and a map or SLAM localization source is available.

### Path Test Without LiDAR

Purpose:
- drive the robot manually
- fuse wheel odom and IMU with EKF
- draw a live trajectory path without needing `/scan`

Commands:
```bash
cd /home/ws/ugv_rpi
bash ./ops/start_ros_path_test_stack.sh
bash ./ops/status_ros_path_test_stack.sh
bash ./ops/stop_ros_path_test_stack.sh
```

Topics to inspect in RViz:
- `/odometry/filtered`
- `/odom_path`
- `/tf`

Suggested RViz config:
- `/home/ws/ugv_rpi/ros2_ws/src/rasprover_slam/config/path_test.rviz`

On the Mac:
- source the same ROS 2 distro/environment
- set the same ROS domain/network settings as the Pi
- open RViz with fixed frame `odom`
- add `Path`, `Odometry`, and `TF` displays if not using the provided config

## Blueprint Launch Files

The package-facing launch entrypoints live in:
- `/home/ws/ugv_rpi/ros2_ws/src/rasprover_bringup/launch/base_only.launch.py`
- `/home/ws/ugv_rpi/ros2_ws/src/rasprover_bringup/launch/motion_stack.launch.py`
- `/home/ws/ugv_rpi/ros2_ws/src/rasprover_bringup/launch/localization_stack.launch.py`
- `/home/ws/ugv_rpi/ros2_ws/src/rasprover_bringup/launch/slam_stack.launch.py`
- `/home/ws/ugv_rpi/ros2_ws/src/rasprover_bringup/launch/navigation_stack.launch.py`
- `/home/ws/ugv_rpi/ros2_ws/src/rasprover_bringup/launch/teleop.launch.py`
- `/home/ws/ugv_rpi/ros2_ws/src/rasprover_bringup/launch/web_control.launch.py`
- `/home/ws/ugv_rpi/ros2_ws/src/rasprover_bringup/launch/slam.launch.py`
- `/home/ws/ugv_rpi/ros2_ws/src/rasprover_bringup/launch/slam_noekf.launch.py`
- `/home/ws/ugv_rpi/ros2_ws/src/rasprover_bringup/launch/slam_ekf.launch.py`
- `/home/ws/ugv_rpi/ros2_ws/src/rasprover_bringup/launch/path_test.launch.py`

Use these when you want package-level launch semantics rather than shell-script orchestration.
Treat `path_test.launch.py` as diagnostic-only, not as a normal operator bringup mode.

## Legacy Code Policy

Legacy runtime code is now archive-only. For new work:

- use ROS packages for runtime code
- use `tutorial_en/legacy_runtime_archive/` only to read archived legacy code
- use `tools/` for diagnostics and calibration scripts
