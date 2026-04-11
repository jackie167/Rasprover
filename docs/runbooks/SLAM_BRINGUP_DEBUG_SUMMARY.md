# SLAM Bringup Debug Summary

## Problem Summary

Symptoms observed during `slam_noekf` and `slam_ekf`:

- `rplidar_node` could connect, but RViz sometimes showed empty, sparse, or flickering scan data.
- `slam_toolbox` could be `active`, but `/map` looked frozen or did not visibly grow.
- Ubuntu VM RViz sometimes saw only part of the TF tree.
- The stack could look "up" while the actual mapping pipeline was still stalled.

## Root Causes Found

The final problem was a combination of issues:

1. `robot_base_node` was opening `/dev/ttyUSB*`
   - This could steal the lidar serial device from `rplidar_node`.

2. Duplicate ROS nodes polluted the graph
   - Stale `local_joy_node`, `simple_odom_filter_node`, and other stack processes could remain alive.
   - This caused duplicate `/tf` and odometry publishers.

3. Cross-machine ROS discovery was inconsistent
   - Pi and Ubuntu VM were not always using the same ROS domain, RMW, and peer settings.

4. Lidar launch settings were not stable for full-stack mapping
   - Scan shape was unstable before the final lidar launch fix.

5. `slam_toolbox` intake was too strict
   - Scan throttling and motion thresholds were too aggressive.
   - Result: `slam_toolbox` could be `active` but still not update the map.

## Fixes Applied

### Serial ownership

- Fixed [base_ctrl.py](/home/ws/ugv_rpi/ros2_ws/src/rasprover_base/rasprover_base/base_ctrl.py) so auxiliary serial devices are opened only when explicitly enabled.
- Base board stays on `/dev/ttyAMA0`.
- LiDAR stays on `/dev/ttyUSB0`.

### Clean startup and shutdown

- Updated SLAM start/stop scripts so stale processes are cleaned first.
- Extra cleanup now includes lingering `local_joy_node` and `simple_odom_filter_node`.

### Stable TF path

- `slam_noekf` now uses:
  - `slam_sensor_bridge_node` with `publish_tf: false`
  - `simple_odom_filter_node` for `odom -> base_link`

### Reliable `slam_toolbox` activation

- Start scripts now wait for `/slam_toolbox`, then retry:
  - `configure`
  - `activate`
- The script only reports `slam_toolbox: OK` once lifecycle state is truly `active`.

### Known-good ROS network settings

Pi side is pinned in [ops/_project_env.sh](/home/ws/ugv_rpi/ops/_project_env.sh):

```bash
ROS_LOCALHOST_ONLY=0
ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
ROS_DOMAIN_ID=0
RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ROS_STATIC_PEERS=192.168.2.11
```

Ubuntu VM should use:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_LOCALHOST_ONLY=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_STATIC_PEERS=192.168.2.123
ros2 daemon stop
```

Addresses:

- Pi IP: `192.168.2.123`
- Ubuntu VM IP: `192.168.2.11`

### Known-good lidar settings

Current working lidar values in [slam_noekf.launch.py](/home/ws/ugv_rpi/ros2_ws/src/rasprover_bringup/launch/slam_noekf.launch.py):

```text
serial_port=/dev/ttyUSB0
serial_baudrate=460800
frame_id=laser
angle_compensate=true
scan_mode=''
laser_x=0.04
laser_y=0.0
laser_z=0.0
laser_roll=0.0
laser_pitch=0.0
laser_yaw=3.141592653589793
```

Observed after fix:

- scan length stabilized to `720` points per frame

### Known-good `slam_toolbox` settings

Current working mapping values in [slam_toolbox_online_async.yaml](/home/ws/ugv_rpi/ros2_ws/src/rasprover_slam/config/slam_toolbox_online_async.yaml):

```text
throttle_scans: 1
minimum_time_interval: 0.0
minimum_travel_distance: 0.05
minimum_travel_heading: 0.05
scan_queue_size: 50
scan_buffer_size: 30
restamp_tf: true
transform_timeout: 1.0
map_update_interval: 2.0
scan_topic: /scan
odom_frame: odom
base_frame: base_link
map_frame: map
```

These changes were the key final fix for the "map active but not growing" failure.

## Validation Performed

Verified directly on the Pi:

- `/scan` publishes correctly
- `/wheel/odometry` changes when the robot moves
- `/tf` contains `odom -> base_link`
- `/tf_static` contains `base_link -> laser`
- `/slam_toolbox` reaches `active [3]`
- `/slam_toolbox/dynamic_map` returns a real `OccupancyGrid`
- map content changes after robot motion

Final direct proof:

- map hash before motion and after motion became different
- conclusion: backend mapping is updating correctly

## Known-Good Bringup

Start mapping:

```bash
cd ~/ugv_rpi
bash ./ops/start_ros_slam_noekf_stack.sh
```

Start RViz on Ubuntu VM:

```bash
source /opt/ros/jazzy/setup.bash && export ROS_LOCALHOST_ONLY=0 ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_STATIC_PEERS=192.168.2.123 && ros2 daemon stop && rviz2
```

Run a direct health check on the Pi:

```bash
cd ~/ugv_rpi
bash ./ops/check_slam_health.sh slam_noekf
```

## If This Returns Again

Check in this order:

1. No stale stack processes remain.
2. Pi and Ubuntu VM ROS env match the values above.
3. `/scan` is live and stable.
4. TF tree contains:
   - `map -> odom`
   - `odom -> base_link`
   - `base_link -> laser`
5. `ros2 lifecycle get /slam_toolbox` returns `active`.
6. `/slam_toolbox/dynamic_map` changes after motion.
