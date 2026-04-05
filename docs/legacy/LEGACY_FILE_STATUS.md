# Legacy File Status

This note answers one practical question:

Can the remaining root-level legacy Python files be moved into `tutorial/` or archived away?

## Short Answer

Mostly no, but the situation is cleaner now.

The active ROS runtime no longer depends on most root-level legacy files. What
remains at the root is mostly a compatibility shim layer plus a few config/tool
entrypoints.

## Still Active In ROS Runtime

These components are still live dependencies of the ROS stack, but the root-level files are wrapper entrypoints only.

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

## Root Shim Retirement Matrix

- `base_ctrl.py`
  Keep for now.
  Still used by local diagnostic tools and any operator habits that import `base_ctrl` from repo root.

- `base_driver.py`
  Keep for now.
  Still used by local diagnostic tools and capture scripts that are designed to run without ROS package imports in the command line.

- `state_store.py`
  Can be retired later after confirming no non-ROS scripts import it from root.

- `cv_ctrl.py`
  Keep for now.
  Still useful as a compatibility import while the CV path remains transitional.

- `app.py`
  Can be retired once the team officially drops the fallback app runtime entrypoint.

- `web_ui.py`
  Can be retired with `app.py`.

- `cmd_mux.py`
  Can be retired with the fallback app runtime.

- `command_service.py`
  Can be retired with the fallback app runtime.

- `command_router.py`
  Can be retired with the fallback app runtime.

- `audio_ctrl.py`
  Now only a fallback/compatibility shim.
  ROS-side `rasprover_ui` no longer imports it from root.

- `os_info.py`
  Now only a fallback/compatibility shim for the legacy runtime path.

## What Can Be Moved Later

The following move is already done:

- legacy fallback app files now live under `legacy_runtime/`
- root files are compatibility wrappers where needed

The following move is not safe yet:

- moving `base_ctrl.py`, `base_driver.py`, `state_store.py`, or `cv_ctrl.py` into `tutorial/`

## Recommended Next Step

Do not move the active legacy-backed runtime files into `tutorial/`.

Instead:
- keep only the minimum root shims still needed for operator compatibility
- document exactly which ones can be retired next
- continue shrinking their surface area by moving ownership into ROS packages first
