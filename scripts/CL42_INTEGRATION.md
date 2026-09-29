# Integrated CL42 web control

Main page: http://192.168.1.21:8765/
BCM wiring: M1 STEP17 DIR27 EN22; M2 STEP23 DIR24 EN5. Negative inputs GND.
Direction: inward DIR1=0, DIR2=0; outward DIR1=1, DIR2=1.
EN: LOW run/hold, HIGH disable/reset, reset holds 1 second then restores LOW.
No ALM/encoder/limit inputs: reset is not verified alarm clearance; no automatic fault retry.
Position is relative emitted pulses since node start, not homed physical position.
Defaults: 1600 pulses/revolution, pitch 2 mm. Match driver DIP settings and actual screw.
Each jog is limited to 62 mm, not an absolute travel limit. Software pulses are not hard real-time.

Architecture: main car_sim web/index.html -> web_gateway -> /leadscrew/control
service -> car_nodes/leadscrew_driver.py -> leadscrew_motion.py.
Legacy /leadscrew/cmd and /leadscrew/status remain supported.
GPIO ownership lock blocks standalone manual scripts while ROS owns outputs.

Panel service (web + screw driver only):
  sudo systemctl restart car-rpi-motor-web.service
Full vehicle bringup instead (also starts other vehicle hardware):
  sudo systemctl stop car-rpi-motor-web.service
  bash /home/ubuntu/CAR_ws/scripts/start_car.sh
Do not run both launch files at once.

Build:
  source /opt/ros/humble/setup.bash
  cd /home/ubuntu/CAR_ws
  colcon build --packages-select car_interfaces car_nodes car_sim
  source install/setup.bash

Mock GPIO test (no hardware writes): python3 scripts/test_cl42_integration.py
Simulation: ROS_DOMAIN_ID=87 ros2 launch car_sim control_panel.launch.py simulate:=true web_port:=18765
Backup: camera_ui_backups/ros_motor_integration_20260917_150318
Validated: build, mock pulses/direction/lock/reset/stop, isolated simulated HTTP->ROS controls.
Real driver startup and browser status verified; no real motor motion test performed.
