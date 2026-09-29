# Pi Podman recovery after the no-prop bench reboot

At 22:46 CST on 2026-09-29, the Pi restarted while the props were removed for bench testing. Podman then failed because `overlay/l` had become a broken symlink and one layer's `link` file was empty. MAVROS, landing, range bridge, and flight recorder could not start; the native vision web service remained up.

With those four services stopped, the image/layer/libpod metadata and every overlay `link`/`lower` file were backed up locally. The broken symlink was quarantined, the empty layer link received a unique short name, and 48 short links were rebuilt. No image, container, flight log, or backing layer was deleted. `podman info` and the `localhost/air-ground-landing-ros2:humble` image were verified, then all four services returned to `active`.

The guarded repair script, pre-repair inspection and metadata archive remain in local maintenance storage. They are deliberately excluded from the public repository. This storage fault was caused by the reboot state and is separate from the CH6 GUIDED telemetry issue.
