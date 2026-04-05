# Legacy Runtime

This directory contains the archived fallback application runtime.

It is no longer an operational robot runtime. The preferred runtime is the ROS 2
stack launched through:

- `start_ros_full_stack.sh`
- `start_ros_motion_stack.sh`
- `start_ros_slam_stack.sh`

What remains here:

- legacy Flask/Socket.IO app flow
- legacy command router/service/mux path
- fallback audio/system-info helpers used by that app

Archive policy:

- this folder is preserved only for reading old code paths
- root-level fallback wrappers and legacy operator scripts have been retired
- do not wire active runtime code back to this folder

This folder should be treated as compatibility code, not as the source of
truth for the ROS-first system.
