#!/usr/bin/env python3
"""Print the packaged full-scene manifest."""

from __future__ import annotations

import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory


def main() -> None:
    manifest_path = (
        Path(get_package_share_directory("drone_chongqing_fullscene"))
        / "runtime_data"
        / "yuzhong_full_scene_manifest.json"
    )
    data = json.loads(manifest_path.read_text(encoding="utf-8"))
    print(json.dumps(data, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
