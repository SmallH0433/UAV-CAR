#!/usr/bin/env python3
"""Capture one complete JPEG frame from the OV9281 MJPEG preview."""

from __future__ import annotations

import argparse
import urllib.request
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--url", default="http://127.0.0.1:8765/stream.mjpg")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    data = bytearray()
    with urllib.request.urlopen(args.url, timeout=8) as response:
        while len(data) < 4_000_000:
            chunk = response.read(65536)
            if not chunk:
                break
            data.extend(chunk)
            start = data.find(b"\xff\xd8")
            end = data.find(b"\xff\xd9", start + 2) if start >= 0 else -1
            if start >= 0 and end >= 0:
                args.output.write_bytes(bytes(data[start : end + 2]))
                print(f"captured={args.output} bytes={end + 2 - start}")
                return 0
    raise RuntimeError("complete JPEG frame not received")


if __name__ == "__main__":
    raise SystemExit(main())
