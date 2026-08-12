# IMX296 basic diagnostic on Raspberry Pi 4B

This is a hardware and image-pipeline test. It does not yet perform landing-pad detection.

## 1. Install the supported packages on Raspberry Pi OS

```bash
sudo apt update
sudo apt install -y python3-picamera2 python3-opencv
```

Use the Raspberry Pi OS packages instead of installing Picamera2 with `pip`, so that
Picamera2, libcamera, and the system camera driver remain compatible.

## 2. Verify the camera before running Python

```bash
rpicam-hello --list-cameras
rpicam-hello -t 5000
rpicam-still -o ~/imx296_cli_test.jpg --width 1456 --height 1088
```

The camera list should contain `imx296`. If the preview changes with movement but remains
an unfocused colour field, fit and focus the CS lens before debugging software.

## 3. Run the Python test on the Pi desktop

```bash
python3 ~/imx296_debug.py
```

Preview keys:

- `q` or `Esc`: exit
- `s`: save the current frame under `~/imx296_test/`
- `m`: use fixed exposure (`3000 us`) and gain (`2.0`)
- `a`: return to auto exposure and auto white balance

Choose different manual values from the command line:

```bash
python3 ~/imx296_debug.py --shutter-us 1000 --gain 4
```

For an SSH session without a graphical preview:

```bash
python3 ~/imx296_debug.py --headless
```

This saves `~/imx296_test/imx296_headless.jpg` and prints the frame metadata.

## AprilTag landing observer

`landing_observer.py` is the first project-level vision module. It detects
`tag36h11`, writes measurements to CSV, and saves an annotated frame. Without a
calibration file it deliberately reports pixel detections only; it does not
invent metric pose or send flight-control messages.

```bash
python3 -m venv --system-site-packages ~/venvs/landing
~/venvs/landing/bin/python -m pip install pupil-apriltags
source ~/venvs/landing/bin/activate
python3 ~/imx296_debug/landing_observer.py --headless --duration-s 10
```

After calibrating the camera at 1456x1088:

The automatic collector saves only frames where the complete 9x6-inner-corner
board is detected. Keep the board in view and slowly vary its position and
angle during the collection window:

```bash
~/venvs/landing/bin/python ~/imx296_debug/collect_calibration.py \
  --output ~/imx296_calibration_images \
  --duration-s 180 --target-count 15
```

```bash
python3 ~/imx296_debug/calibrate_imx296.py \
  --images ~/imx296_calibration_images \
  --output ~/imx296_calibration.yaml \
  --corners-cols 9 --corners-rows 6 --square-size-m 0.025

python3 ~/imx296_debug/landing_observer.py \
  --headless --duration-s 30 \
  --calibration ~/imx296_calibration.yaml \
  --tag-id 0 --tag-size-m 0.200
```

For a safe MAVLink boundary test, add `--mavlink-dry-run`. This writes MAVLink 2
`LANDING_TARGET` frames to JSONL and never opens a serial, UDP, or flight-control
link. The current frame is explicitly `MAV_FRAME_CAMERA_OPTICAL`; it must not be
changed to `BODY_FRD` until the physical camera-to-aircraft transform is measured.

```bash
~/venvs/landing/bin/python ~/imx296_debug/landing_observer.py \
  --headless --duration-s 10 \
  --calibration ~/imx296_calibration_run2.yaml \
  --tag-id 0 --tag-size-m 0.200 \
  --mavlink-dry-run \
  --output ~/landing_verify
```

An existing CSV can also be replayed through the dry-run bridge:

```bash
~/venvs/landing/bin/python ~/imx296_debug/landing_target_bridge.py \
  --csv ~/landing_verify/observations.csv
```

The output is `~/landing_observer/observations.csv`. The pose is reported in
the camera optical frame until the physical camera-to-body transform has been
measured; no MAVLink or flight-controller command is sent by this module.

## 4. Copy this file from the Mac to the Pi

Replace `<pi-user>` if your Raspberry Pi account is not named `pi`:

```bash
scp /Users/lyton/Documents/Codex/imx296_debug/imx296_debug.py <pi-user>@uavpi.local:~/
```

If `.local` name resolution is unavailable, replace `uavpi.local` with the Raspberry Pi
IP address displayed by the Windows hotspot.
