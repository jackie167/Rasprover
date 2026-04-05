# Legacy File Status

This note answers one practical question:

Can the remaining root-level legacy Python files be moved into `tutorial/` or archived away?

## Short Answer

The answer is now split in two:

- fallback app code has been archived into `tutorial_en/legacy_runtime_archive/`
- active ROS-backed legacy implementation details now live inside ROS packages,
  not in root-level Python files

## Still Active In ROS Runtime

These components are still live dependencies of the ROS stack, but they no
longer exist as root-level Python files.

- `base_ctrl`
  Runtime implementation lives in `/home/ws/ugv_rpi/ros2_ws/src/rasprover_base/rasprover_base/base_ctrl.py`.

- `base_driver`
  Runtime implementation lives in `/home/ws/ugv_rpi/ros2_ws/src/rasprover_base/rasprover_base/base_driver.py`.

- `state_store`
  Runtime implementation lives in `/home/ws/ugv_rpi/ros2_ws/src/rasprover_base/rasprover_base/state_store.py`.

- `cv_ctrl`
  Runtime implementation lives in `/home/ws/ugv_rpi/ros2_ws/src/rasprover_cv/rasprover_cv/cv_ctrl.py`.
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

## Current Source Dependency Status

Verified in repository source after refactor:

- no maintained source file imports `base_driver`, `base_ctrl`, `state_store`, or `cv_ctrl` from repo root
- ROS-side `rasprover_ui` imports audio support from its own package module
- root shim usage for these names is fully retired

## What Can Be Moved Later

The following move is already done:

- legacy fallback app files now live under `tutorial_en/legacy_runtime_archive/`
- fallback root wrappers and scripts have been removed

The following move is not safe yet:

- moving package-owned runtime implementations such as `rasprover_base/base_ctrl.py`
  or `rasprover_cv/cv_ctrl.py` into `tutorial/`

## Recommended Next Step

Do not move the active legacy-backed runtime files into `tutorial/`.

Instead:
- keep runtime ownership inside ROS packages
- archive only truly retired code in `tutorial_en/legacy_runtime_archive/`
- continue shrinking package-internal legacy implementation details over time
