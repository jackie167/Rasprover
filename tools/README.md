# Tools Layout

Helpers that support bringup, diagnostics, calibration, and maintenance live here.

- `build/`: controlled build helpers for low-resource environments.
- `calibration/`: scripts used to capture and analyze sensor data.
- `diagnostics/`: scripts used to inspect low-level feedback and hardware state.

Root wrapper scripts are kept when they improve day-to-day operator ergonomics.

Current root wrappers:

- `/home/ws/ugv_rpi/read_base_feedback.py`
- `/home/ws/ugv_rpi/slam_prep_capture.py`
- `/home/ws/ugv_rpi/slam_prep_analyze.py`
- `/home/ws/ugv_rpi/safe_colcon_build.sh`

Rule of thumb:

- if a tool is primarily for engineering workflows, put the real implementation in `tools/`
- keep a root wrapper only when it is a command people are likely to type directly
