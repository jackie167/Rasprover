#!/usr/bin/env python3

from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[2]

TARGETS = {
    "app": "app.py",
    "web_ui": "web_ui.py",
    "cmd_mux": "cmd_mux.py",
    "command_service": "command_service.py",
    "command_router": "command_router.py",
    "audio_ctrl": "audio_ctrl.py",
    "os_info": "os_info.py",
    "base_driver": "base_driver.py",
    "base_ctrl": "base_ctrl.py",
    "state_store": "state_store.py",
    "cv_ctrl": "cv_ctrl.py",
}

INCLUDE_SUFFIXES = {".py", ".sh", ".md"}
EXCLUDE_PARTS = {".git", "ugv-env", "build", "install", "__pycache__"}


def should_scan(path: Path) -> bool:
    if path.suffix not in INCLUDE_SUFFIXES:
        return False
    return not any(part in EXCLUDE_PARTS for part in path.parts)


def pattern_for(module_name: str) -> re.Pattern[str]:
    return re.compile(
        rf"\b(import\s+{re.escape(module_name)}|from\s+{re.escape(module_name)}\s+import)\b"
    )


def scan():
    patterns = {name: pattern_for(name) for name in TARGETS}
    results = {name: [] for name in TARGETS}

    for path in ROOT.rglob("*"):
        if not path.is_file() or not should_scan(path):
            continue
        try:
            lines = path.read_text(encoding="utf-8").splitlines()
        except Exception:
            continue
        rel = path.relative_to(ROOT)
        for lineno, line in enumerate(lines, start=1):
            for name, pattern in patterns.items():
                if pattern.search(line):
                    results[name].append(f"{rel}:{lineno}: {line.strip()}")
    return results


def main():
    results = scan()
    print("Root shim audit")
    print(f"Repo: {ROOT}")
    print()
    for name, filename in TARGETS.items():
        hits = results[name]
        print(f"{filename}: {len(hits)} hit(s)")
        for hit in hits[:20]:
            print(f"  {hit}")
        if len(hits) > 20:
            print(f"  ... {len(hits) - 20} more")
        print()


if __name__ == "__main__":
    main()
