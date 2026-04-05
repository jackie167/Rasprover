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

## Retired Legacy Fallback Runtime

The old fallback app path is no longer supported as a runtime mode.

Archived code now lives under:

- `/home/ws/ugv_rpi/tutorial_en/legacy_runtime_archive/`

This includes the old:

- `app.py` behavior
- `web_ui.py` behavior
- `cmd_mux.py`
- `command_service.py`
- `command_router.py`
- `audio_ctrl.py`
- `os_info.py`

## Operational Wrappers

The legacy operator scripts have been retired with the fallback runtime.

## Root Shim Retirement Matrix

- `base_ctrl.py`
  Keep for now.
  No active source package imports it from root now.
  Keep only for operator compatibility and ad hoc local habits.

- `base_driver.py`
  Keep for now.
  No active source package or maintained tool imports it from root now.
  Keep only for operator compatibility and ad hoc local habits.

- `state_store.py`
  Can be retired later after confirming no user-side ad hoc scripts import it from root.

- `cv_ctrl.py`
  Keep for now.
  Still useful as a compatibility import while the CV path remains transitional.

- `audio_ctrl.py`
  Removed from the active root/runtime path.
  ROS-side `rasprover_ui` now owns its own audio helper module.

- `os_info.py`
  Removed from the active root/runtime path.

## Current Source Dependency Status

Verified in repository source after refactor:

- no maintained source file imports `base_driver`, `base_ctrl`, `state_store`, or `cv_ctrl` from repo root
- ROS-side `rasprover_ui` imports audio support from its own package module
- remaining root shim usage is now primarily operator compatibility around package-owned modules

## What Can Be Moved Later

The following move is already done:

- legacy fallback app files now live under `tutorial_en/legacy_runtime_archive/`
- fallback root wrappers and scripts have been removed

The following move is not safe yet:

- moving `base_ctrl.py`, `base_driver.py`, `state_store.py`, or `cv_ctrl.py` into `tutorial/`

## Recommended Next Step

Do not move the active legacy-backed runtime files into `tutorial/`.

Instead:
- keep only the minimum root shims still needed for operator compatibility
- document exactly which ones can be retired next
- continue shrinking their surface area by moving ownership into ROS packages first
