"""Project-local Python startup tweaks.

Keeps the app virtualenv self-consistent when system site-packages are also
visible on the import path.
"""

from pathlib import Path
import sys
import types


def _prefer_venv_mpl_toolkits():
    repo_root = Path(__file__).resolve().parent
    venv_pkg = repo_root / 'ugv-env' / 'lib' / 'python3.11' / 'site-packages' / 'mpl_toolkits'
    if not venv_pkg.exists():
        return

    module = sys.modules.get('mpl_toolkits')
    if module is None:
        module = types.ModuleType('mpl_toolkits')
        module.__path__ = [str(venv_pkg)]
        sys.modules['mpl_toolkits'] = module
        return

    module.__path__ = [str(venv_pkg)]


_prefer_venv_mpl_toolkits()
