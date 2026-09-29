# H100 landing trigger

Receiver: car 192.168.50.2 UDP 8889. Allowed source: 192.168.50.3.
Payload: UTF-8 text 已成功降落
Also accepted: {"event":"landed","landing_id":"flight-001"} or {"message":"已成功降落"}.
landing_id is informational; ALL landing events share a persistent one-shot latch.
Both motors run simultaneously inward 50 mm, DIR LOW, 60 rpm, ramp 1 second.
Calibration remains 1600 pulses/revolution and 2 mm pitch: 40000 pulses each.
No encoder or travel-limit feedback: distance is commanded, not measured.

UDP replies to the sender source port:
pending -> accepted (driver accepted, NOT completed) or rejected/unknown.
duplicate means latched; no new motion. unavailable means no request sent.
Latch is written BEFORE dispatch. A failure/stop/rejection does not auto-retry.
Latch survives reboot. After preparing the mechanism for a NEW landing, explicitly rearm:
  source /opt/ros/humble/setup.bash
  source /home/ubuntu/CAR_ws/install/setup.bash
  ros2 service call /uav_bridge/rearm_landing std_srvs/srv/Trigger '{}'
Rearm issues no movement. Never rearm while old landing packets are still being sent.

Nodes are included in control_panel.launch.py and real_bringup.launch.py.
Existing web Stop all screws stops the motion. No automatic restart after Stop.
User parameters: uav_ip, listen_port, landing_rpm, landing_latch_file.

Verification (isolated simulated service and mocked outputs, no physical motion):
  ROS_DOMAIN_ID=88 python3 scripts/test_landing_trigger.py
H100 eth0 was DOWN during deployment; no live drone-to-car trigger test performed.
Drone sender was not modified. Its landing-success publisher must send above payload.

## Sequential cycle update
已成功降落 / event=landed: motor1 then motor2 inward50mm.
充电完成 / event=charge_complete: motor1 then motor2 outward50mm.
Both actions fixed100rpm, ramp0.1s. Inward DIR0, outward DIR1.
Independent persistent latches prevent repeat motion; rearm_landing resets BOTH for the next cycle.
Do not send charge message until inward sequence finishes; busy driver rejects it, requiring explicit rearm after inspection.
