#!/usr/bin/env python3
"""Root convenience wrapper for the diagnostics tool."""

import runpy
from pathlib import Path


if __name__ == "__main__":
    target = Path(__file__).resolve().parent / "tools" / "diagnostics" / "read_base_feedback.py"
    runpy.run_path(str(target), run_name="__main__")
