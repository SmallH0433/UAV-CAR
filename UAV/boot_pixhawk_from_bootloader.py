#!/usr/bin/env python3
"""Ask a PX4-compatible bootloader to start its verified flight firmware."""

from __future__ import annotations

import argparse
import importlib.util
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True)
    parser.add_argument("--uploader", type=Path, required=True)
    args = parser.parse_args()

    spec = importlib.util.spec_from_file_location("ardupilot_uploader", args.uploader)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Unable to load uploader module: {args.uploader}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    device = module.uploader(args.port, 115200, [115200])
    try:
        device.identify()
        print(
            f"BOOTLOADER board={device.board_type},{device.board_rev} "
            f"revision={device.bl_rev}"
        )
        device._uploader__reboot()
        print("BOOT_COMMAND_SENT=1")
    finally:
        device.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
