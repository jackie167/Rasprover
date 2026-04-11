# Root Layout

This note explains the current repository layout after the ROS-first cleanup
and the follow-up root normalization pass.

## What Stays At The Root

### Operator scripts

- `ops/start_ros_*.sh`
- `ops/stop_ros_*.sh`
- `ops/status_ros_*.sh`
- `ops/restart_ros_*.sh`
- `ops/start_jupyter.sh`
- `ops/watch_nav_build.sh`
- `ops/autorun.sh`
- `ops/setup.sh`

These live directly under `ops/`.

Notes:

- operator commands should now be run from `ops/` paths explicitly
- runtime logs from these scripts do not stay at the root; they go to `runtime_logs/`

### Compatibility shims

- `app_config.py`
- `config_loader.py`
- `repo_paths.py`
- `sitecustomize.py`

These remain at the root only as import-compatible shims.

Notes:

- the actual implementations live in `project_support/`
- existing imports such as `from app_config import AppConfig` keep working

### Project config and metadata

- `config.yaml`
- `asound.conf`
- `requirements.txt`
- `README.md`
- `LICENSE`
- `.gitignore`

### Tool wrappers

- `read_base_feedback.py`
- `slam_prep_capture.py`
- `slam_prep_analyze.py`
- `safe_colcon_build.sh`

These are convenience wrappers that forward into `tools/`.

Notes:

- the real implementations live under `tools/`
- the wrappers exist so common commands stay short and discoverable

One special case:

- `.codex`
  This file is not part of the robot runtime. It may appear because of the
  local coding environment and can be ignored for system architecture purposes.

## What No Longer Belongs At The Root

- runtime logs
  moved to `runtime_logs/`

- SLAM prep CSV captures
  moved to `tools/calibration/captures/`

- fallback Flask-first runtime
  retired and archived in `tutorial_en/legacy_runtime_archive/`

- retired binary/helper artifacts
  archived alongside legacy runtime code when they are no longer part of the active system

- support module implementations
  moved to `project_support/`

- root shell operator shortcuts
  removed in favor of direct `ops/` entrypoints

## Cleanup Rule

If a new file appears at the root, it should belong to one of these categories:

- compatibility shim
- project config/metadata
- metadata
- thin wrapper into `tools/`

Otherwise it should probably live in a package, `project_support/`, `ops/`,
`tools/`, `docs/`, or the
legacy archive.
