# OV9281 tools

This directory is independent from `imx296_debug`. The legacy IMX296 preview
and calibration files are intentionally retained.

The receive-only follow adapter is `ov9281_follow_preview.py`.  It reads the
unified service's `/api/status` endpoint instead of reopening Picamera2, so the
service remains the only camera owner and the only process bound to port 8765.
BODY_FRD and velocity proposals stay blocked until measured OV9281 installation
extrinsics are explicitly enabled; legacy IMX296 extrinsics are never loaded.

Start the OV9281 calibration console on the Raspberry Pi:

```bash
~/venvs/landing/bin/python ~/ov9281_debug/ov9281_preview.py \
  --collect-output ~/ov9281_calibration_run1_17mm \
  --target-count 20
```

The native capture is 1280x800 at 30 fps. Browser encoding runs at 12 fps and
9x6 chessboard detection runs at 2 fps to keep the preview responsive.

After calibration collection, image-space AprilTag verification can run with:

```bash
~/venvs/landing/bin/python ~/ov9281_debug/ov9281_apriltag_preview.py
```

This monitor does not connect to the flight controller. Metric pose remains
disabled until a low-error OV9281 calibration has been accepted.

`ov9281_unified_service.py` is the production preview architecture for this
camera. It uses a 1280x800 monochrome analysis stream and a separate 640x400
hardware-MJPEG preview stream. Calibration and AprilTag modes switch in one UI.

The default AprilTag mode now detects a concentric nested `tag36h11` target:
outer ID 0 has a 0.100 m black edge and inner ID 1 has a 0.020 m black edge.
Both poses are returned in `/api/status.detections`; the top-level fields expose
one range-selected primary tag with 0.05 m hysteresis.  Copy
`ov9281_dual_tag.py` and `ov9281_range_correction_dual_tag_20260817.json` with
the service when deploying it.  ID 0 uses the existing 0.655 m one-point
candidate correction.  ID 1 remains identity/pending close-range measurements,
so flight use is still not approved.
