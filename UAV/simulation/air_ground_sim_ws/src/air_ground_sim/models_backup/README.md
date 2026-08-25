# UAV model backups

`uav_camera_rig_legacy_20260821/` is the byte-for-byte backup of the previous
UAV model before the OV9281 / optical-flow / GNSS sensor redesign. It is kept
outside the Gazebo model search path to prevent a duplicate model name from
shadowing the active `models/uav_camera_rig/` directory.

To restore it, stop Gazebo and copy its `model.sdf` and `model.config` back to
`models/uav_camera_rig/`, then rebuild the `air_ground_sim` package.
