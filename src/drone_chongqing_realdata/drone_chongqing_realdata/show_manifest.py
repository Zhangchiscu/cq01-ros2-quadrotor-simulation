#!/usr/bin/env python3
"""Print the latest real-data manifest."""

from __future__ import annotations

import json

from .common import resolve_web_root


def main() -> None:
    path = (
        resolve_web_root()
        / "data"
        / "manifest.json"
    )

    if not path.exists():
        print(
            "No fetched real-data manifest found."
        )
        print(
            "Run: ros2 run drone_chongqing_realdata "
            "fetch_yuzhong_realdata"
        )
        return

    data = json.loads(
        path.read_text(
            encoding="utf-8"
        )
    )
    print(
        json.dumps(
            data,
            ensure_ascii=False,
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
