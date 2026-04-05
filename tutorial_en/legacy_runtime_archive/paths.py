from pathlib import Path
import os
import sys


def repo_root() -> Path:
    env_root = os.environ.get("PROJECT_DIR")
    if env_root:
        return Path(env_root).resolve()
    return Path(__file__).resolve().parents[1]


REPO_ROOT = repo_root()

if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))
