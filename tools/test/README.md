# Test Tools

These scripts are for running validation workflows, not for computing a permanent calibration value directly.

Examples:

- `straight_controller_tune.py`
  - inspect straight-line feedback behavior
- `slam_prep_capture.py`
  - capture data before a SLAM prep / evaluation pass
- `slam_prep_analyze.py`
  - analyze captured SLAM prep data

Rule:

- if a script helps evaluate behavior, put it in `tools/test/`
- if a script computes a robot parameter you intend to keep, put it in `tools/calibration/`
