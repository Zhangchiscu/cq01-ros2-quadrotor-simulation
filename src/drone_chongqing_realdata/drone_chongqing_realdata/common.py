from __future__ import annotations

import os
from pathlib import Path


PACKAGE_NAME = "drone_chongqing_realdata"


def workspace_root() -> Path:
    return (
        Path.home()
        / "drone_task"
        / "your_drone_sim_ws"
    )


def source_web_root() -> Path:
    return (
        workspace_root()
        / "src"
        / PACKAGE_NAME
        / "web"
    )


def resolve_web_root() -> Path:
    override = os.environ.get(
        "DRONE_CHONGQING_REALDATA_WEB"
    )
    if override:
        path = Path(override).expanduser().resolve()
        if path.exists():
            return path

    source = source_web_root()
    if source.exists():
        return source

    try:
        from ament_index_python.packages import (
            get_package_share_directory,
        )

        share = Path(
            get_package_share_directory(PACKAGE_NAME)
        )
        candidate = share / "web"
        if candidate.exists():
            return candidate
    except Exception:
        pass

    raise FileNotFoundError(
        "Unable to locate the real-data web directory."
    )
