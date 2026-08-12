#!/usr/bin/env bash
set -euo pipefail

output_dir="${1:-$HOME/uav/logs}"
output_file="$output_dir/imx219-test.jpg"
flip_method="${IMX219_FLIP_METHOD:-0}"

if [[ ! "$flip_method" =~ ^[0-7]$ ]]; then
  echo "IMX219_FLIP_METHOD must be an integer from 0 to 7" >&2
  exit 2
fi

mkdir -p "$output_dir"

gst-launch-1.0 -e \
  nvarguscamerasrc num-buffers=1 \
  ! 'video/x-raw(memory:NVMM),width=1920,height=1080,framerate=30/1' \
  ! nvvidconv flip-method="$flip_method" \
  ! 'video/x-raw,format=I420' \
  ! jpegenc \
  ! filesink location="$output_file"

test -s "$output_file"
printf 'Captured: %s (%s bytes)\n' "$output_file" "$(stat -c '%s' "$output_file")"
