# Legacy Runtime

This directory contains the archived fallback application runtime.

It is no longer the primary robot runtime. The preferred runtime is the ROS 2
stack launched through:

- `start_ros_full_stack.sh`
- `start_ros_motion_stack.sh`
- `start_ros_slam_stack.sh`

What remains here:

- legacy Flask/Socket.IO app flow
- legacy command router/service/mux path
- fallback audio/system-info helpers used by that app

Compatibility policy:

- root-level files such as `app.py`, `web_ui.py`, `cmd_mux.py`, and
  `command_service.py` are thin wrappers only
- operational legacy scripts now target `python -m legacy_runtime.app_main`

This folder should be treated as compatibility code, not as the source of
truth for the ROS-first system.
