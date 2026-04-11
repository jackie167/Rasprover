#!/usr/bin/env python3

from __future__ import annotations

import argparse
import time
from copy import deepcopy
from pathlib import Path

from recommendation_io import LATEST_RECOMMENDATION_PATH
from recommendation_io import RUNTIME_CONFIG_PATH
from recommendation_io import load_runtime_config
from recommendation_io import load_yaml
from recommendation_io import now_timestamp
from recommendation_io import save_yaml
from recommendation_io import set_nested


def backup_path_for(runtime_config_path: Path) -> Path:
    stamp = time.strftime("%Y%m%d_%H%M%S")
    return runtime_config_path.with_name(f"{runtime_config_path.stem}.bak_{stamp}{runtime_config_path.suffix}")


def main():
    parser = argparse.ArgumentParser(
        description="Apply the latest calibration recommendation into robot_runtime.yaml."
    )
    parser.add_argument(
        "--recommendation",
        default=str(LATEST_RECOMMENDATION_PATH),
        help="Recommendation YAML to apply. Defaults to tools/calibration/latest_recommendation.yaml",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Show the updates without writing robot_runtime.yaml.",
    )
    args = parser.parse_args()

    recommendation_path = Path(args.recommendation).resolve()
    if not recommendation_path.exists():
        raise SystemExit(f"Recommendation file not found: {recommendation_path}")

    recommendation = load_yaml(recommendation_path)
    updates = recommendation.get("updates", [])
    if not updates:
        raise SystemExit("Recommendation file has no updates.")

    runtime_config = load_runtime_config()
    updated_runtime_config = deepcopy(runtime_config)

    print(f"[apply_calib] recommendation: {recommendation_path}")
    print(f"[apply_calib] runtime config: {RUNTIME_CONFIG_PATH}")
    for update in updates:
        path = update["path"]
        old_value = update.get("previous_value")
        new_value = update["recommended_value"]
        print(f"[apply_calib] {path}: {old_value} -> {new_value}")
        set_nested(updated_runtime_config, path, new_value)

    if args.dry_run:
        print("[apply_calib] dry-run only; no files were written.")
        return

    backup_path = backup_path_for(RUNTIME_CONFIG_PATH)
    save_yaml(backup_path, runtime_config)
    save_yaml(RUNTIME_CONFIG_PATH, updated_runtime_config)

    recommendation["status"] = "applied"
    recommendation["applied_at"] = now_timestamp()
    recommendation["applied_runtime_backup"] = str(backup_path)
    save_yaml(recommendation_path, recommendation)
    if recommendation_path != LATEST_RECOMMENDATION_PATH:
        save_yaml(LATEST_RECOMMENDATION_PATH, recommendation)

    print(f"[apply_calib] wrote runtime config: {RUNTIME_CONFIG_PATH}")
    print(f"[apply_calib] backup saved to: {backup_path}")


if __name__ == "__main__":
    main()
