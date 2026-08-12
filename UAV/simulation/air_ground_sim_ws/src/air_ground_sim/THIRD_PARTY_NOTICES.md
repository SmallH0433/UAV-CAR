# Third-party notices

## AstraDroneOpen

- Project: https://github.com/yidrone/AstraDroneOpen
- Upstream revision inspected: `000b7254d0148f7022f26b92e9fbdac1c0600d70`
- License: MIT
- Copyright: AstraDroneOpen contributors

The following files are adaptations of upstream ideas and data:

- `worlds/astra_forest.sdf`: keeps the main object names and XY layout from
  `simulation/astra_gazebo_worlds/forest.world`.
- `worlds/astra_dynamic_avoidance.sdf`: keeps the static layout and five obstacle
  trajectory definitions from `dynamic_avoidance.world` and
  `dynamic_obstacle_controller/config/obstacle_params.yaml`.

The adaptations target Gazebo Harmonic / SDF 1.9. Upstream Gazebo Classic meshes,
materials, PX4 plugins and ROS 1 controller code are not redistributed here. Native
primitive collision models and Gazebo Harmonic's trajectory follower system are used
instead.

The upstream MIT license permits use, modification and distribution while retaining
the copyright and permission notice. Consult the upstream repository's `LICENSE`
file for the complete license text.

## ArduPilot Gazebo runway

- Project: https://github.com/ArduPilot/ardupilot_gazebo
- License: LGPL-3.0
- Copyright: ArduPilot contributors

`worlds/cmac_test_field.sdf` references the upstream `model://runway` resource
already supplied by the local `ardupilot_gazebo` checkout and follows the runway
placement and CMAC spherical coordinates used by upstream `iris_runway.sdf`.
The runway meshes and textures are not duplicated in this ROS package.
