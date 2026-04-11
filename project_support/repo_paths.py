"""Shared helpers for resolving the project root at runtime."""

from __future__ import annotations

import os
import sys
from pathlib import Path


DEFAULT_ROOT = Path(__file__).resolve().parent.parent


def find_repo_root(
    anchor_file: str = "config.yaml",
    extra_required: tuple[str, ...] = (),
    start_path: Path | None = None,
    fallback_root: Path | None = None,
) -> Path:
    """Resolve the repository root from env, ancestors, or cwd."""
    fallback = (fallback_root or DEFAULT_ROOT).resolve()
    search_start = (start_path or __file__)
    current_file = Path(search_start).resolve()

    candidates: list[Path] = []
    env_root = os.environ.get("PROJECT_DIR")
    if env_root:
        candidates.append(Path(env_root).resolve())

    candidates.extend(current_file.parents)
    candidates.append(Path.cwd().resolve())

    for candidate in candidates:
        if not (candidate / anchor_file).exists():
            continue
        if any(not (candidate / item).exists() for item in extra_required):
            continue
        return candidate

    return fallback


def ensure_repo_on_path(repo_root: Path) -> Path:
    """Ensure the repository root is importable and return it."""
    resolved = repo_root.resolve()
    repo_root_str = str(resolved)
    if repo_root_str not in sys.path:
        sys.path.insert(0, repo_root_str)
    return resolved
