#!/usr/bin/env python3
"""Serve the local real-data 3D viewers."""

from __future__ import annotations

import argparse
import functools
import http.server
import socketserver
import subprocess
import threading
import time
from pathlib import Path

from .common import resolve_web_root


class QuietHandler(
    http.server.SimpleHTTPRequestHandler
):
    def end_headers(self) -> None:
        self.send_header(
            "Cache-Control",
            "no-store",
        )
        super().end_headers()

    def log_message(
        self,
        format_string: str,
        *arguments,
    ) -> None:
        print(
            "[http] "
            + format_string % arguments
        )


class ReusableThreadingServer(
    socketserver.ThreadingTCPServer
):
    allow_reuse_address = True


def open_browser(url: str) -> None:
    time.sleep(1.0)
    try:
        subprocess.Popen(
            [
                "xdg-open",
                url,
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    except Exception as error:
        print(
            f"Could not open browser automatically: {error}"
        )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--port",
        type=int,
        default=8765,
    )
    parser.add_argument(
        "--mode",
        choices=[
            "maplibre",
            "cesium",
        ],
        default="cesium",
    )
    parser.add_argument(
        "--no-open",
        action="store_true",
    )
    arguments = parser.parse_args()

    root = resolve_web_root()
    target = (
        "cesium.html"
        if arguments.mode == "cesium"
        else "index.html"
    )
    url = (
        f"http://127.0.0.1:"
        f"{arguments.port}/{target}"
    )

    handler = functools.partial(
        QuietHandler,
        directory=str(root),
    )

    with ReusableThreadingServer(
        ("127.0.0.1", arguments.port),
        handler,
    ) as server:
        print(
            "===== Yuzhong real-data viewer ====="
        )
        print(f"Web root: {root}")
        print(f"Mode: {arguments.mode}")
        print(f"URL: {url}")
        print(
            "Press Ctrl+C to stop the local server."
        )

        if not arguments.no_open:
            threading.Thread(
                target=open_browser,
                args=(url,),
                daemon=True,
            ).start()

        try:
            server.serve_forever()
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
