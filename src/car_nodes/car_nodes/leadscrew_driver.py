"""ROS owner of both CL42 controllers; relative jog + EN reset + legacy commands.

BCM M1 PUL17 DIR27 EN22; M2 PUL23 DIR24 EN5. Negative inputs share GND.
Run EN=LOW, reset/relax EN=HIGH by default; configurable enable_invert_N.
No limit/alarm/encoder feedback. Position is a relative pulse estimate only.
"""
import json
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from std_msgs.msg import String, Float64
from car_interfaces.msg import LeadscrewCommand, LeadscrewStatus
from car_interfaces.srv import LeadscrewControl
from .leadscrew_motion import Motion


class LeadscrewDriverNode(Node):
    def __init__(self):
        super().__init__('leadscrew_driver_node')
        defaults = dict(simulate=True, publish_sim_joints=True, status_period=.5,
                        travel_mm=63.5, leadscrew_pitch_mm=2., pulses_per_rev=1600,
                        ramp_seconds=1., default_speed=1600,
                        step_pin_1=17, dir_pin_1=27, enable_pin_1=22,
                        step_pin_2=23, dir_pin_2=24, enable_pin_2=5,
                        dir_invert_1=False, dir_invert_2=False,
                        enable_invert_1=False, enable_invert_2=False)
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        p = lambda key: self.get_parameter(key).value
        self.engine = Motion(simulate=p('simulate'),
                             pins=tuple(tuple(p(f'{kind}_pin_{i}') for kind in ('step','dir','enable')) for i in (1,2)),
                             inversions=(p('dir_invert_1'),p('dir_invert_2')),
                             enable_inversions=(p('enable_invert_1'),p('enable_invert_2')),
                             pulses_per_rev=p('pulses_per_rev'), pitch=p('leadscrew_pitch_mm'), travel=p('travel_mm'))
        self.default_speed = p('default_speed')
        self.ramp = p('ramp_seconds')
        self.pub_status = self.create_publisher(LeadscrewStatus, '/leadscrew/status', 10)
        self.pub_detail = self.create_publisher(String, '/leadscrew/detail', 10)
        self.create_subscription(LeadscrewCommand, '/leadscrew/cmd', self.cmd_cb, 10)
        self.create_service(LeadscrewControl, '/leadscrew/control', self.control_cb)
        self.sim_joint_pubs = {}
        if p('simulate') and p('publish_sim_joints'):
            for i, names in enumerate((('a','c'),('b','d'))):
                self.sim_joint_pubs[i] = [self.create_publisher(Float64, f'/leadscrew/sim/pusher_{n}/cmd_pos',10) for n in names]
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(max(.1,float(p('status_period'))), self.publish_status, clock=self.steady_clock)
        self.get_logger().info(f'CL42 ROS controller ready; simulate={p("simulate")}, pins={self.engine.pins}')

    def endpoint(self, group, inward, rpm, ramp):
        if not math.isfinite(rpm) or not 5 <= rpm*self.engine.ppr/60 <= self.engine.ppr*1000/60 or not math.isfinite(ramp) or not .05 <= ramp <= 5:
            raise ValueError('Invalid speed or ramp')
        idxs = self.engine.indices(group)
        target = round(self.engine.travel*self.engine.steps_mm) if inward else 0
        def task():
            # Common paired travel is simultaneous; unequal offsets are completed separately.
            with self.engine.lock:
                deltas = [target-self.engine.pos[i] for i in idxs]
            if any(abs(d) > round(self.engine.travel*self.engine.steps_mm) for d in deltas):
                raise ValueError('Relative endpoint outside travel range; use measured jog')
            if deltas and len(set(deltas)) == 1:
                delta = deltas[0]
                if delta:
                    self.engine.pulses(idxs, 0 if delta>0 else 1, abs(delta), rpm*self.engine.ppr/60, ramp)
                return
            # Legacy full-stroke commands use the same relative position estimate.
            for i in idxs:
                if self.engine.cancel.is_set():
                    break
                with self.engine.lock:
                    delta = target-self.engine.pos[i]
                if delta:
                    if abs(delta) > round(self.engine.travel*self.engine.steps_mm):
                        raise ValueError('估算位置超出整行程范围，请用单次行程控制')
                    self.engine.pulses((i,), 0 if delta>0 else 1, abs(delta), rpm*self.engine.ppr/60, ramp)
        self.engine.launch(task, False, '按相对脉冲估算执行端点指令；不具备限位检测')

    def command(self, group, action, direction=0, rpm=6., distance=1., ramp=1.):
        self.engine.indices(group)
        if action == 'stop':
            self.engine.stop()  # Always stop both motors.
        elif action == 'move':
            self.engine.move(group, direction, rpm, distance, ramp)
        elif action == 'landing_sequence':
            self.engine.landing_sequence()
        elif action == 'charge_sequence':
            self.engine.landing_sequence(direction=1)
        elif action == 'simulation_toggle':
            self.engine.simulation_toggle(rpm, ramp)
        elif action == 'reset':
            self.engine.reset(group)
        elif action in ('relax','lock'):
            self.engine.enable(group, action=='lock')
        elif action in ('in','out'):
            self.endpoint(group, action=='in', rpm, ramp)
        else:
            raise ValueError('未知丝杆指令')

    def control_cb(self, request, response):
        try:
            if not request.deadline_unix > time.time():
                raise ValueError('指令已过期，未执行')
            self.command(request.group, request.action, request.direction, request.rpm,
                         request.distance_mm, request.ramp_seconds)
            response.accepted, response.message = True, '指令已接收；状态见运行反馈'
        except (ValueError, RuntimeError, OSError) as exc:
            response.accepted, response.message = False, str(exc)
        self.publish_status()
        return response

    def cmd_cb(self, msg):
        actions = {0:'stop',1:'in',2:'out',3:'relax',4:'lock'}
        try:
            speed = msg.speed or self.default_speed
            if speed < 5 or speed > self.engine.ppr*1000/60:
                raise ValueError('脉冲频率超出允许范围')
            self.command(0 if msg.command==0 else msg.group, actions.get(msg.command,''),
                         rpm=speed*60/self.engine.ppr, ramp=self.ramp)
        except (ValueError, RuntimeError, OSError) as exc:
            self.get_logger().warning(str(exc))
        self.publish_status()

    def publish_status(self):
        state = self.engine.snapshot()
        msg = LeadscrewStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.state, msg.pos_mm, msg.enabled, msg.speed = state['state'],state['pos_mm'],state['enabled'],state['speed']
        self.pub_status.publish(msg)
        detail = String()
        detail.data = json.dumps(state, ensure_ascii=False)
        self.pub_detail.publish(detail)
        for i, pubs in self.sim_joint_pubs.items():
            value = Float64()
            value.data = -state['pos_mm'][i]/1000
            for pub in pubs:
                pub.publish(value)

    def destroy_node(self):
        self.engine.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LeadscrewDriverNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
