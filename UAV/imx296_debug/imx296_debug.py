#!/usr/bin/env python3
"""IMX296 / Picamera2 basic diagnostic tool for Raspberry Pi.

Keys in the preview window:
  q or Esc  quit
  s         save the current frame as a JPEG
  m         switch to manual exposure (defaults: 3000 us, gain 2.0)
  a         switch back to auto exposure/white balance
"""

from __future__ import annotations

import argparse
import time
from datetime import datetime
from pathlib import Path
from pprint import pprint

import cv2
from picamera2 import Picamera2


def find_imx296() -> tuple[int, list[dict]]:
    """Return the IMX296 camera index and the complete camera list."""
    cameras = Picamera2.global_camera_info()
    for index, camera in enumerate(cameras):
        if "imx296" in str(camera.get("Model", "")).lower():
            return index, cameras
    raise RuntimeError(
        "No IMX296 was found. Check the CSI ribbon, connector orientation, "
        "and the output of: rpicam-hello --list-cameras"
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="IMX296 live diagnostic")
    parser.add_argument("--width", type=int, default=1456)
    parser.add_argument("--height", type=int, default=1088)
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--shutter-us", type=int, default=3000)
    parser.add_argument("--gain", type=float, default=2.0)
    parser.add_argument(
        "--headless",
        action="store_true",
        help="do not open a window; save one frame and exit",
    )
    parser.add_argument("--output", type=Path, default=Path.home() / "imx296_test")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    args.output.mkdir(parents=True, exist_ok=True)

    camera_index, cameras = find_imx296()
    print("\nDetected cameras:")
    pprint(cameras)
    print(f"\nUsing camera index {camera_index} (IMX296)")

    picam2 = Picamera2(camera_index)
    try:
        print("\nCamera properties:")
        pprint(picam2.camera_properties)
        print("\nSensor modes:")
        pprint(picam2.sensor_modes)

        frame_period_us = int(1_000_000 / args.fps)
        config = picam2.create_video_configuration(
            main={
                # Picamera2's RGB888 memory layout is directly suitable for OpenCV.
                "format": "RGB888",
                "size": (args.width, args.height),
            },
            controls={"FrameDurationLimits": (frame_period_us, frame_period_us)},
            buffer_count=4,
        )
        picam2.configure(config)
        picam2.start()

        # Allow automatic exposure and white balance to settle.
        time.sleep(2.0)

        print("\nApplied camera configuration:")
        pprint(picam2.camera_configuration())

        manual_mode = False
        frame_count = 0
        fps_value = 0.0
        fps_window_start = time.monotonic()

        while True:
            # Fetch the image and its metadata from the same completed request.
            (frame,), metadata = picam2.capture_arrays(["main"])
            frame_count += 1

            now = time.monotonic()
            elapsed = now - fps_window_start
            if elapsed >= 1.0:
                fps_value = frame_count / elapsed
                frame_count = 0
                fps_window_start = now

            exposure_us = metadata.get("ExposureTime", 0)
            gain = metadata.get("AnalogueGain", 0.0)
            mode_text = "MANUAL" if manual_mode else "AUTO"
            overlay = (
                f"IMX296  {args.width}x{args.height}  {fps_value:.1f} fps  "
                f"exp={exposure_us} us  gain={gain:.2f}  {mode_text}"
            )
            cv2.putText(
                frame,
                overlay,
                (20, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )

            height, width = frame.shape[:2]
            cv2.drawMarker(
                frame,
                (width // 2, height // 2),
                (0, 255, 0),
                markerType=cv2.MARKER_CROSS,
                markerSize=30,
                thickness=2,
            )

            if args.headless:
                filename = args.output / "imx296_headless.jpg"
                cv2.imwrite(str(filename), frame)
                print(f"\nSaved: {filename}")
                print("Frame metadata:")
                pprint(metadata)
                break

            cv2.imshow("IMX296 debug - q quit | s save | m manual | a auto", frame)
            key = cv2.waitKey(1) & 0xFF

            if key in (ord("q"), 27):
                break
            if key == ord("s"):
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                filename = args.output / f"imx296_{timestamp}.jpg"
                cv2.imwrite(str(filename), frame)
                print(f"Saved: {filename}")
            elif key == ord("m"):
                picam2.set_controls(
                    {
                        "AeEnable": False,
                        "AwbEnable": False,
                        "ExposureTime": args.shutter_us,
                        "AnalogueGain": args.gain,
                    }
                )
                manual_mode = True
                print(
                    f"Manual exposure: {args.shutter_us} us, "
                    f"analogue gain: {args.gain}"
                )
            elif key == ord("a"):
                picam2.set_controls({"AeEnable": True, "AwbEnable": True})
                manual_mode = False
                print("Auto exposure and auto white balance enabled")
    finally:
        picam2.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
