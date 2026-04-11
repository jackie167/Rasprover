#!/usr/bin/env python3

from __future__ import annotations

import argparse
from pathlib import Path

import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
CAPTURES_DIR = REPO_ROOT / "tools" / "calibration" / "captures"
RUNTIME_CONFIG_DIR = REPO_ROOT / "ros2_ws" / "src" / "rasprover_bringup" / "config"


def load_yaml(path: Path) -> dict:
    with open(path, "r", encoding="utf-8") as handle:
        return yaml.safe_load(handle) or {}


def iter_recommendations():
    if not CAPTURES_DIR.exists():
        return []
    return sorted(CAPTURES_DIR.glob("*_recommendation.yaml"), key=lambda path: path.stat().st_mtime, reverse=True)


def iter_runtime_backups():
    if not RUNTIME_CONFIG_DIR.exists():
        return []
    return sorted(
        RUNTIME_CONFIG_DIR.glob("robot_runtime.bak_*.yaml"),
        key=lambda path: path.stat().st_mtime,
        reverse=True,
    )


def print_recommendations(limit: int):
    print("== recommendation history ==")
    for path in iter_recommendations()[:limit]:
        data = load_yaml(path)
        print(f"{path.name}")
        print(f"  created_at: {data.get('created_at', 'unknown')}")
        print(f"  type: {data.get('recommendation_type', 'unknown')}")
        print(f"  status: {data.get('status', 'unknown')}")
        print(f"  source: {data.get('source_csv') or data.get('source_log') or 'unknown'}")
        for update in data.get("updates", [])[:5]:
            print(
                f"  update: {update.get('path')} {update.get('previous_value')} -> {update.get('recommended_value')}"
            )


def print_backups(limit: int):
    print("")
    print("== runtime config backups ==")
    for path in iter_runtime_backups()[:limit]:
        print(path.name)


def main():
    parser = argparse.ArgumentParser(description="Show calibration recommendation/apply history.")
    parser.add_argument("--limit", type=int, default=10, help="Number of items to show per section.")
    args = parser.parse_args()

    print_recommendations(args.limit)
    print_backups(args.limit)


if __name__ == "__main__":
    main()
