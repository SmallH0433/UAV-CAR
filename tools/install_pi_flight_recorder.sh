#!/bin/bash
# Run as the same Pi user that owns the existing MAVROS container.
set -euo pipefail
base=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
podman image exists localhost/air-ground-landing-ros2:humble
mkdir -p "$HOME/flight_recorder" "$HOME/flight_logs" "$HOME/.config/systemd/user"
install -m 644 "$base/tools/pi_flight_recorder.py" "$HOME/flight_recorder/pi_flight_recorder.py"
install -m 644 "$base/config/systemd/pi-flight-recorder.service" "$HOME/.config/systemd/user/pi-flight-recorder.service"
systemctl --user daemon-reload
systemctl --user enable --now pi-flight-recorder.service
if [ "$(loginctl show-user "$USER" -p Linger --value)" != yes ]; then
    sudo loginctl enable-linger "$USER"
fi
systemctl --user --no-pager status pi-flight-recorder.service
echo "Logs: $HOME/flight_logs. Verify received topic counts before treating deployment as complete."
