# Root Shim Retirement Plan

This note defines the last cleanup path for removing root-level compatibility
wrappers without destabilizing the ROS-first system.

## Goal

Shrink the project root until it contains only:

- operator shell entrypoints
- configuration files
- docs and tools wrappers
- intentionally preserved compatibility shims

The root should no longer be a place where runtime ownership lives.

## Current Status

As of this snapshot:

- ROS source packages no longer import `base_driver`, `base_ctrl`, `state_store`,
  or `cv_ctrl` from the repository root.
- maintained tools no longer import `base_driver` from the repository root.
- `rasprover_ui` no longer imports `audio_ctrl` from the repository root.
- the fallback Flask runtime has been retired and archived under
  `tutorial_en/legacy_runtime_archive/`.

This means most root `.py` files are now compatibility-only.

## Final Root Shim Status

The remaining root Python shims have now been retired.

What remains active is package-owned code inside:

- `rasprover_base`
- `rasprover_cv`
- `rasprover_ui`

## Safe Retirement Order

### Phase 1

Already completed:

- move implementation ownership into ROS packages and an isolated legacy archive path
- stop maintained source packages from importing root shims
- stop maintained tools from importing root shims where practical

### Phase 2

Completed:

- fallback app shims retired from the active root/runtime path
- fallback operator scripts retired
- legacy app code archived under `tutorial_en/legacy_runtime_archive/`

### Phase 3

Completed:

- `base_driver.py`
- `base_ctrl.py`
- `state_store.py`
- `cv_ctrl.py`

were retired from the root after audit and package updates.

### Phase 4

Future work moves from root-shim retirement to package-internal cleanup.

## Acceptance Checks Before Deleting Any Shim

Before deleting a root shim, verify all of the following:

1. `grep` or audit tool shows no maintained source imports of the root shim.
2. shell entrypoints do not target the shim as their real runtime.
3. docs and runbooks no longer tell operators to use the shim.
4. build and smoke tests still pass without the shim.

## Recommended Audit Command

Use:

```bash
cd /home/ws/ugv_rpi
./ugv-env/bin/python tools/diagnostics/root_shim_audit.py
```

This provides a quick report of whether deleted legacy module names still
appear in the repository source tree.
