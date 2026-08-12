#!/usr/bin/env bash
set -euo pipefail

# Disarmed, PLND-disabled bench runner. It never writes parameters, changes
# modes, arms the vehicle, or sends motor commands.

PYTHON_BIN="/home/PI/venvs/landing/bin/python"
PROJECT_DIR="/home/PI/imx296_debug"
LOG_DIR="/home/PI/uav/logs"
DURATION_S="${DURATION_S:-15}"
CAMERA_FPS="${CAMERA_FPS:-30}"
DETECTOR_THREADS="${DETECTOR_THREADS:-4}"
QUAD_DECIMATE="${QUAD_DECIMATE:-3.0}"
FRAME_PROFILE="${FRAME_PROFILE:-camera-optical}"
STAMP="$(date -u +%Y%m%dT%H%M%SZ)"

mkdir -p "$LOG_DIR"

STATE_OUTPUT="$($PYTHON_BIN /home/PI/pi_vehicle_state_check.py \
  --device /dev/serial0 --baud 57600 --count 3)"
printf '%s\n' "$STATE_OUTPUT"
grep -q 'RESULT=DISARMED' <<<"$STATE_OUTPUT" || {
  echo "SAFETY_STOP=VEHICLE_NOT_CONFIRMED_DISARMED"
  exit 20
}

PARAM_OUTPUT="$($PYTHON_BIN /home/PI/pi_read_parameters.py \
  PLND_ENABLED PLND_TYPE --device /dev/serial0 --baud 57600 --timeout 3)"
printf '%s\n' "$PARAM_OUTPUT"

case "$FRAME_PROFILE" in
  camera-optical|body-frd-nominal)
    EXPECTED_PLND_ENABLED="0.0"
    EXPECTED_PLND_TYPE="0.0"
    ;;
  body-frd-plnd-enabled)
    EXPECTED_PLND_ENABLED="1.0"
    EXPECTED_PLND_TYPE="1.0"
    ;;
  *)
    echo "SAFETY_STOP=UNKNOWN_FRAME_PROFILE:$FRAME_PROFILE"
    exit 24
    ;;
esac

grep -q "PARAM PLND_ENABLED=${EXPECTED_PLND_ENABLED}" <<<"$PARAM_OUTPUT" || {
  echo "SAFETY_STOP=PLND_ENABLED_PROFILE_MISMATCH"
  exit 21
}
grep -q "PARAM PLND_TYPE=${EXPECTED_PLND_TYPE}" <<<"$PARAM_OUTPUT" || {
  echo "SAFETY_STOP=PLND_TYPE_PROFILE_MISMATCH"
  exit 23
}

if pgrep -f '/home/PI/imx296_debug/camera_stream.py' >/dev/null; then
  echo "SAFETY_STOP=CAMERA_PREVIEW_OWNS_DEVICE"
  exit 22
fi

cd "$PROJECT_DIR"
BRIDGE_ARGS=(
  ./landing_target_serial_bridge.py
  --serial /dev/serial0
  --baud 57600
  --tag-id 0
  --tag-size-m 0.135
  --calibration ./imx296_calibration_run4_17mm.yaml
  --range-correction ./range_correction_20260806.json
  --fps "$CAMERA_FPS"
  --detector-threads "$DETECTOR_THREADS"
  --quad-decimate "$QUAD_DECIMATE"
  --duration-s "$DURATION_S"
  --output "$LOG_DIR/landing_target_${FRAME_PROFILE}_${STAMP}.jsonl"
  --annotated-output "$LOG_DIR/landing_target_${FRAME_PROFILE}_${STAMP}.jpg"
)

case "$FRAME_PROFILE" in
  camera-optical)
    BRIDGE_ARGS+=(--frame camera-optical)
    ;;
  body-frd-nominal)
    BRIDGE_ARGS+=(
      --frame body-frd
      --extrinsics ./imx296_body_extrinsics_20260806.json
      --plnd-profile disabled
    )
    ;;
  body-frd-plnd-enabled)
    BRIDGE_ARGS+=(
      --frame body-frd
      --extrinsics ./imx296_body_extrinsics_20260806.json
      --plnd-profile mavlink-enabled
    )
    ;;
  *)
    echo "SAFETY_STOP=UNKNOWN_FRAME_PROFILE:$FRAME_PROFILE"
    exit 24
    ;;
esac

exec "$PYTHON_BIN" "${BRIDGE_ARGS[@]}"
