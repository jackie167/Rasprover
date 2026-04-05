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

What is still legacy-only today:

- old Flask/Socket.IO control path in `web_ui.py`
- `/offer` WebRTC path
- `/send_command`, `/json`, `/ctrl` legacy command flow
- old templates driven by `templates/index.html` and `templates/control.js`

What remains intentionally non-ROS:

- `base_driver.py`
- `base_ctrl.py`

These stay as the hardware/backend layer used by `robot_base_node`.

What remains transitional but still active in ROS:

- `cv_ctrl.py`
- `state_store.py`

These are still consumed indirectly by `cv_node` and `rasprover_base`, so they are not ready to move into tutorial/archive storage yet.
