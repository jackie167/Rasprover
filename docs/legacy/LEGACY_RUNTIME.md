# Legacy Runtime Notes

The primary runtime is now the ROS full stack:

- `start_ros_full_stack.sh`
- `web_bridge_node`
- `command_mux_node`
- `robot_base_node`
- `cv_node`

The old fallback Flask-first runtime is retired.

Archived legacy code now lives in:

- `/home/ws/ugv_rpi/tutorial_en/legacy_runtime_archive/`

Archive contents include:

- old app bootstrap and Flask web runtime
- old in-process mux and command service path
- legacy audio and host-info helpers

What is still legacy-only today:

- old Flask/Socket.IO control path in `web_ui.py`
- `/offer` WebRTC path
- `/send_command`, `/json`, `/ctrl` legacy command flow
- old templates driven by `templates/index.html` and `templates/control.js`

What remains intentionally non-ROS:

- `base_driver.py`
- `base_ctrl.py`

These now forward to package-local implementations in `rasprover_base`, which
stay as the hardware/backend layer used by `robot_base_node`.

What remains transitional but still active in ROS:

- `cv_ctrl.py`
- `state_store.py`

These now forward to package-local implementations still consumed indirectly by
`cv_node` and `rasprover_base`, so they are not ready to move into
tutorial/archive storage yet.

Operational note:

- the legacy runtime is no longer an operational mode
- use the archive only to read old code and understand previous behavior
