# Bringup Modes

This runbook shows the supported runtime entrypoints after the ROS-first restructuring.

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

Commands:
```bash
cd /home/ws/ugv_rpi
bash ./start_ros_motion_stack.sh
bash ./status_ros_motion_stack.sh
bash ./stop_ros_motion_stack.sh
```

### Full ROS Stack

Purpose:
- motion stack
- CV node

Commands:
```bash
cd /home/ws/ugv_rpi
bash ./start_ros_full_stack.sh
bash ./status_ros_full_stack.sh
bash ./stop_ros_full_stack.sh
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
bash ./start_ros_slam_stack.sh
bash ./status_ros_slam_stack.sh
bash ./stop_ros_slam_stack.sh
```

This stack is structurally ready before LiDAR arrives, but mapping only becomes meaningful once `/scan` is available.

### Path Test Without LiDAR

Purpose:
- drive the robot manually
- fuse wheel odom and IMU with EKF
- draw a live trajectory path without needing `/scan`

Commands:
```bash
cd /home/ws/ugv_rpi
bash ./start_ros_path_test_stack.sh
bash ./status_ros_path_test_stack.sh
bash ./stop_ros_path_test_stack.sh
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
- `/home/ws/ugv_rpi/ros2_ws/src/rasprover_bringup/launch/teleop.launch.py`
- `/home/ws/ugv_rpi/ros2_ws/src/rasprover_bringup/launch/web_control.launch.py`
- `/home/ws/ugv_rpi/ros2_ws/src/rasprover_bringup/launch/slam.launch.py`
- `/home/ws/ugv_rpi/ros2_ws/src/rasprover_bringup/launch/full_system.launch.py`

Use these when you want package-level launch semantics rather than shell-script orchestration.

## Legacy Code Policy

Legacy runtime code is now archive-only. For new work:

- use ROS packages for runtime code
- use `tutorial_en/legacy_runtime_archive/` only to read archived legacy code
- use `tools/` for diagnostics and calibration scripts
