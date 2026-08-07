"""Locate the MT4 control repo and put it on ``sys.path``.

The simulated arm's geometry, joint limits and gripper range are read from
``mt4_jog`` at import time. Restating them here would create a second copy of
constants the MT4 repo already keeps in three places (firmware
``kinematics.h``, ``mt4_jog/joints.py``, ``mt4_jog/kinematics.py``) and warns
must be edited together.

Override the location with ``MT4_REPO`` when the control repo is not the
sibling ``MT4`` directory.
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

DEFAULT_MT4_REPO = Path(__file__).resolve().parent.parent.parent / "MT4"


def mt4_repo_path() -> Path:
    env = os.environ.get("MT4_REPO")
    return Path(env).resolve() if env else DEFAULT_MT4_REPO


def ensure_on_path() -> Path:
    """Put the MT4 control repo on ``sys.path`` and return its location."""
    repo = mt4_repo_path()
    if not (repo / "mt4_jog" / "kinematics.py").is_file():
        raise RuntimeError(
            f"MT4 control repo not found at {repo} (no mt4_jog/kinematics.py). "
            "Clone https://github.com/enghoff/MT4 beside this project, or set "
            "MT4_REPO to where it lives."
        )
    if str(repo) not in sys.path:
        sys.path.insert(0, str(repo))
    return repo


ensure_on_path()
