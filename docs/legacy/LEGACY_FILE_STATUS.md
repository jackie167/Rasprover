# Legacy File Status

This note answers one practical question:

Can the remaining root-level legacy Python files be moved into `tutorial/` or archived away?

## Short Answer

Not yet.

Some root-level legacy files are still used by the active ROS runtime, not just by the fallback app.

## Still Active In ROS Runtime

These files are still live dependencies of the ROS stack and must stay in the repository runtime path for now.

- `base_ctrl.py`
  Used by `base_driver.py`, which is still the low-level serial/protocol backend behind `rasprover_base`.

- `base_driver.py`
  Used by `rasprover_base` through `/home/ws/ugv_rpi/ros2_ws/src/rasprover_base/rasprover_base/legacy_imports.py`.

- `state_store.py`
  Used by `rasprover_base` through the same package-local shim and also used by the fallback legacy app runtime.

- `cv_ctrl.py`
  Still used by `rasprover_cv/cv_node.py`.
  The ROS CV node is a wrapper around `OpencvFuncs`, not yet a full rewrite.

These files cannot be moved to `tutorial/` without breaking the current ROS stack.

## Still Active In Legacy Fallback Runtime Only

These files are not part of the primary ROS runtime, but they are still needed if you want the old fallback app path to work.

- `app.py`
- `web_ui.py`
- `cmd_mux.py`
- `command_service.py`
- `command_router.py`

These are candidates for future archival once the fallback runtime is intentionally retired.

## Operational Wrappers

These are legacy/fallback operator entrypoints rather than core runtime logic:

- `start_legacy_app.sh`
- `status_legacy_app.sh`
- `stop_legacy_app.sh`

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
