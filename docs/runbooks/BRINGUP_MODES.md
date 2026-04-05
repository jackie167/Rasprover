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

## Blueprint Launch Files

The package-facing launch entrypoints live in:
- `/home/ws/ugv_rpi/ros2_ws/src/rasprover_bringup/launch/base_only.launch.py`
- `/home/ws/ugv_rpi/ros2_ws/src/rasprover_bringup/launch/teleop.launch.py`
- `/home/ws/ugv_rpi/ros2_ws/src/rasprover_bringup/launch/web_control.launch.py`
- `/home/ws/ugv_rpi/ros2_ws/src/rasprover_bringup/launch/slam.launch.py`
- `/home/ws/ugv_rpi/ros2_ws/src/rasprover_bringup/launch/full_system.launch.py`

Use these when you want package-level launch semantics rather than shell-script orchestration.

## Legacy Fallback Runtime

The old fallback app runtime still exists, but it is no longer the primary system path.

Commands:
```bash
cd /home/ws/ugv_rpi
bash ./start_legacy_app.sh
bash ./status_legacy_app.sh
bash ./stop_legacy_app.sh
```

Use this only when comparing behavior against the old app architecture or when debugging migration gaps.
