# Legacy Runtime Notes

The primary runtime is now the ROS full stack:

- `start_ros_full_stack.sh`
- `web_bridge_node`
- `command_mux_node`
- `robot_base_node`
- `cv_node`

The following files are kept only as fallback/dev paths and should not be
treated as the main runtime anymore:

- `app.py`
- `web_ui.py`
- `cmd_mux.py`
- `command_router.py`
- `command_service.py`
- `start_legacy_app.sh`
- `stop_legacy_app.sh`
- `status_legacy_app.sh`

Implementation note:

- `app.py`, `web_ui.py`, `cmd_mux.py`, `command_router.py`, and `command_service.py` now forward to
  `/home/ws/ugv_rpi/legacy_runtime/`
- `audio_ctrl.py` and `os_info.py` root files now also forward to
  `/home/ws/ugv_rpi/legacy_runtime/`

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

- legacy operator scripts now start the fallback runtime through
  `python -m legacy_runtime.app_main`
- the root `app.py` file remains only as a compatibility shim
