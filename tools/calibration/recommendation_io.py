#!/usr/bin/env python3

from __future__ import annotations

import time
from copy import deepcopy
from pathlib import Path

import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
RUNTIME_CONFIG_PATH = REPO_ROOT / "ros2_ws" / "src" / "rasprover_bringup" / "config" / "robot_runtime.yaml"
LATEST_RECOMMENDATION_PATH = Path(__file__).resolve().parent / "latest_recommendation.yaml"
CAPTURES_DIR = Path(__file__).resolve().parent / "captures"


def load_yaml(path: Path) -> dict:
    with open(path, "r", encoding="utf-8") as handle:
        return yaml.safe_load(handle) or {}


def save_yaml(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8") as handle:
        yaml.safe_dump(payload, handle, sort_keys=False)


def load_runtime_config() -> dict:
    return load_yaml(RUNTIME_CONFIG_PATH)


def get_nested(mapping: dict, dotted_path: str):
    current = mapping
    for part in dotted_path.split("."):
        if not isinstance(current, dict) or part not in current:
            raise KeyError(dotted_path)
        current = current[part]
    return current


def set_nested(mapping: dict, dotted_path: str, value) -> None:
    parts = dotted_path.split(".")
    current = mapping
    for part in parts[:-1]:
        if part not in current or not isinstance(current[part], dict):
            current[part] = {}
        current = current[part]
    current[parts[-1]] = value


def now_timestamp() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%S")


def build_update(runtime_config: dict, dotted_path: str, value, reason: str) -> dict:
    return {
        "path": dotted_path,
        "previous_value": deepcopy(get_nested(runtime_config, dotted_path)),
        "recommended_value": value,
        "reason": reason,
    }


def recommendation_output_path(csv_output_path: str) -> Path:
    csv_path = Path(csv_output_path).resolve()
    return csv_path.with_name(f"{csv_path.stem}_recommendation.yaml")


def standalone_recommendation_output_path(prefix: str) -> Path:
    CAPTURES_DIR.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    return CAPTURES_DIR / f"{prefix}_{stamp}_recommendation.yaml"


def write_recommendation(payload: dict, csv_output_path: str) -> tuple[Path, Path]:
    payload = deepcopy(payload)
    payload["schema_version"] = 1
    payload["created_at"] = now_timestamp()
    payload["runtime_config_path"] = str(RUNTIME_CONFIG_PATH)
    payload["latest_recommendation_path"] = str(LATEST_RECOMMENDATION_PATH)

    recommendation_path = recommendation_output_path(csv_output_path)
    save_yaml(recommendation_path, payload)
    save_yaml(LATEST_RECOMMENDATION_PATH, payload)
    return recommendation_path, LATEST_RECOMMENDATION_PATH


def write_standalone_recommendation(payload: dict, prefix: str) -> tuple[Path, Path]:
    payload = deepcopy(payload)
    payload["schema_version"] = 1
    payload["created_at"] = now_timestamp()
    payload["runtime_config_path"] = str(RUNTIME_CONFIG_PATH)
    payload["latest_recommendation_path"] = str(LATEST_RECOMMENDATION_PATH)

    recommendation_path = standalone_recommendation_output_path(prefix)
    save_yaml(recommendation_path, payload)
    save_yaml(LATEST_RECOMMENDATION_PATH, payload)
    return recommendation_path, LATEST_RECOMMENDATION_PATH
