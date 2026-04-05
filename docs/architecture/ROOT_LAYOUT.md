# Root Layout

This note explains why the repository root still contains a small set of files
after the ROS-first cleanup.

## What Stays At The Root

### Operator entrypoints

- `start_ros_full_stack.sh`
- `stop_ros_full_stack.sh`
- `status_ros_full_stack.sh`
- `start_ros_motion_stack.sh`
- `stop_ros_motion_stack.sh`
- `status_ros_motion_stack.sh`
- `start_ros_slam_stack.sh`
- `stop_ros_slam_stack.sh`
- `status_ros_slam_stack.sh`
- `restart_ros_full_stack.sh`
- `start_jupyter.sh`
- `autorun.sh`
- `setup.sh`

These stay at the root because operators expect to run them directly.

Notes:

- `restart_ros_full_stack.sh` is intentionally a small helper around stop/start.
- runtime logs from these scripts do not stay at the root; they go to `runtime_logs/`.

### Core config and support

- `config.yaml`
- `app_config.py`
- `config_loader.py`
- `asound.conf`
- `requirements.txt`
- `sitecustomize.py`

These are project-level configuration or Python support files.

### Tool wrappers

- `read_base_feedback.py`
- `slam_prep_capture.py`
- `slam_prep_analyze.py`
- `safe_colcon_build.sh`

These are convenience wrappers that forward into `tools/`.

Notes:

- the real implementations live under `tools/`
- the wrappers exist so common commands stay short and discoverable

### Project metadata

- `README.md`
- `LICENSE`
- `.gitignore`

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

- legacy root Python shims
  removed after package ownership was established

## Cleanup Rule

If a new file appears at the root, it should belong to one of these categories:

- operator entrypoint
- project config/support
- metadata
- thin wrapper into `tools/`

Otherwise it should probably live in a package, `tools/`, `docs/`, or the
legacy archive.
