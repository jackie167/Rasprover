#!/usr/bin/env python3
"""Root convenience wrapper for SLAM prep capture."""

import runpy
from pathlib import Path


if __name__ == "__main__":
    target = Path(__file__).resolve().parent / "tools" / "test" / "slam_prep_capture.py"
    runpy.run_path(str(target), run_name="__main__")
