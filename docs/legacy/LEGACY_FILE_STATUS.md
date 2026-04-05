# Legacy File Status

This note answers one practical question:

Can the remaining root-level legacy Python files be moved into `tutorial/` or archived away?

## Short Answer

Not yet.

Some root-level legacy files are still used by the active ROS runtime, not just by the fallback app.

## Still Active In ROS Runtime

These components are still live dependencies of the ROS stack, but the root-level files are no longer the implementation owners in every case.

- `base_ctrl.py`
  Root file is now a compatibility wrapper.
  Runtime implementation now lives in `/home/ws/ugv_rpi/ros2_ws/src/rasprover_base/rasprover_base/base_ctrl.py`.

- `base_driver.py`
  Root file is now a compatibility wrapper.
  Runtime implementation now lives in `/home/ws/ugv_rpi/ros2_ws/src/rasprover_base/rasprover_base/base_driver.py`.

- `state_store.py`
  Root file is now a compatibility wrapper.
  Runtime implementation now lives in `/home/ws/ugv_rpi/ros2_ws/src/rasprover_base/rasprover_base/state_store.py`.

- `cv_ctrl.py`
  Root file is now a compatibility wrapper.
  Runtime implementation now lives in `/home/ws/ugv_rpi/ros2_ws/src/rasprover_cv/rasprover_cv/cv_ctrl.py`.
  The ROS CV node is still a wrapper around `OpencvFuncs`, not yet a full rewrite.

These implementations cannot be moved to `tutorial/` without breaking the current ROS stack.

## Still Active In Legacy Fallback Runtime Only

These are not part of the primary ROS runtime, but they are still needed if you want the old fallback app path to work.

- `app.py`
- `web_ui.py`
- `cmd_mux.py`
- `command_service.py`
- `command_router.py`
- `audio_ctrl.py`
- `os_info.py`

Current status:
- `app.py` and `web_ui.py` root files are now compatibility wrappers.
- `cmd_mux.py`, `command_service.py`, and `command_router.py` root files are now compatibility wrappers.
- `audio_ctrl.py` and `os_info.py` root files are now compatibility wrappers.
- their fallback implementation code now lives under `/home/ws/ugv_rpi/legacy_runtime/`

These are still candidates for future archival once the fallback runtime is intentionally retired.

## Operational Wrappers

These are legacy/fallback operator entrypoints rather than core runtime logic:

- `start_legacy_app.sh`
- `status_legacy_app.sh`
- `stop_legacy_app.sh`
- `restart_app.sh`
- `status_app.sh`
- `stop_app.sh`

They should stay until the team decides the legacy fallback app is no longer needed.

## What Can Be Moved Later

The following move is reasonable in a later cleanup phase, but not yet:

- move legacy fallback app files into a dedicated folder such as `legacy_runtime/`
- keep compatibility wrappers at the root if operator habits still depend on the old script names

The following move is not safe yet:

- moving `base_ctrl.py`, `base_driver.py`, `state_store.py`, or `cv_ctrl.py` into `tutorial/`

## Recommended Next Step

Do not move the active legacy-backed runtime files into `tutorial/`.

Instead:
- keep them where they are
- document that they are transitional runtime dependencies
- continue shrinking their surface area by moving ownership into ROS packages first
