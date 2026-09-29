"""Receive-only bridge for the known downward sensor. Never sends MAVLink."""
import math
import struct
import time


class RangeGate:
    def __init__(self):
        self.last_boot = None
        self.reason = 'NO_DATA'
        self.recovery = None
        self.recovery_count = 0
        self.rebase_count = 0

    def cancel_recovery(self):
        self.recovery = None
        self.recovery_count = 0

    def decode(self, *, sysid, compid, msgid, framing, payload, age,
               recovery_allowed=False, now=None):
        now = time.monotonic() if now is None else now
        if not recovery_allowed:
            self.cancel_recovery()
        if (sysid, compid, msgid) != (200, 88, 132):
            return None
        if framing != 1 or len(payload) < 13 or not math.isfinite(age) or not 0 <= age <= .3:
            self.cancel_recovery()
            self.reason = 'INVALID_FRAME_OR_STALE'
            return (math.nan, .02, 12.)
        data = payload.ljust(39, b'\0')
        boot, minimum, maximum, distance, kind, ident, orient, covariance = struct.unpack('<IHHHBBBB', data[:14])
        if ident != 0 or orient != 25:
            return None
        measurement_valid = (kind == 0 and 0 < minimum < maximum
                             and minimum < distance < maximum and data[38] != 1)
        if self.last_boot is not None and boot <= self.last_boot:
            # Small reordering/duplicates cannot start a new sensor epoch.
            self.reason = 'DUPLICATE_OR_REVERSED_SENSOR_TIME'
            if (not recovery_allowed or not measurement_valid or not math.isfinite(now)
                    or self.last_boot-boot < 1000):
                self.cancel_recovery()
                return None
            previous = self.recovery
            if (previous is None or not 0 < now-previous[1] <= .3
                    or not 0 < boot-previous[2] <= 300):
                self.recovery = (now, now, boot)
                self.recovery_count = 1
            else:
                self.recovery = (previous[0], now, boot)
                self.recovery_count += 1
            self.reason = 'VERIFY_NEW_SENSOR_EPOCH_DISARMED'
            if now-self.recovery[0] < .5 or self.recovery_count < 5:
                return (math.nan, .02, 12.)
            self.rebase_count += 1
        self.cancel_recovery()
        self.last_boot = boot
        if not measurement_valid:
            self.reason = 'INVALID_OR_RANGE_LIMIT'
            return (math.nan, .02, 12.)
        self.reason = 'VALID'
        return distance*.01, minimum*.01, maximum*.01


def main():
    import json
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from mavros_msgs.msg import Mavlink, State
    from sensor_msgs.msg import Range
    from std_msgs.msg import String
    rclpy.init()
    class Bridge(Node):
        def __init__(self):
            super().__init__('sensor_range_bridge')
            self.gate = RangeGate()
            self.last_valid = None
            self.valid_count = 0
            self.last_distance = None
            self.vehicle_state = None
            self.pub = self.create_publisher(Range, '/landing/sensor_range', qos_profile_sensor_data)
            self.status = self.create_publisher(String, '/landing/sensor_range/status', 10)
            self.create_subscription(Mavlink, '/uas1/mavlink_source', self.receive, qos_profile_sensor_data)
            self.create_subscription(State, '/mavros/state', self.vehicle, qos_profile_sensor_data)
            self.create_timer(.1, self.tick)
        def vehicle(self, msg):
            self.vehicle_state = msg
            if not self.recovery_allowed():
                self.gate.cancel_recovery()
        def recovery_allowed(self):
            msg = self.vehicle_state
            if msg is None or not msg.connected or msg.armed:
                return False
            age = (self.get_clock().now().nanoseconds -
                   (msg.header.stamp.sec*10**9 + msg.header.stamp.nanosec))*1e-9
            return 0 <= age <= 1.5
        def emit(self, distance, minimum=.02, maximum=12.):
            msg = Range()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'downward_sensor_200_88'
            msg.radiation_type = Range.INFRARED
            msg.field_of_view = 0.0  # unknown; do not invent sensor optics
            msg.min_range, msg.max_range, msg.range = minimum, maximum, distance
            self.pub.publish(msg)
        def receive(self, msg):
            age = (self.get_clock().now().nanoseconds -
                   (msg.header.stamp.sec*1000000000 + msg.header.stamp.nanosec))*1e-9
            payload = b''.join(struct.pack('<Q', n) for n in msg.payload64)[:msg.len]
            result = self.gate.decode(sysid=msg.sysid, compid=msg.compid, msgid=msg.msgid,
                                      framing=msg.framing_status, payload=payload, age=age,
                                      recovery_allowed=self.recovery_allowed())
            if result is None:
                return
            if math.isfinite(result[0]):
                self.last_valid = time.monotonic()
                self.valid_count += 1
                self.last_distance = result[0]
            else:
                self.last_valid = None
                self.last_distance = None
            self.emit(*result)
        def tick(self):
            if not self.recovery_allowed():
                self.gate.cancel_recovery()
            age = None if self.last_valid is None else time.monotonic()-self.last_valid
            healthy = age is not None and age <= .3
            if not healthy:
                self.emit(math.nan)
            self.status.publish(String(data=json.dumps({'source':'200/88', 'id':0,
                'orientation':25, 'healthy':healthy, 'age_s':age,
                'range_m':self.last_distance if healthy else None,
                'valid_count':self.valid_count, 'reason':self.gate.reason,
                'sensor_time_ms':self.gate.last_boot, 'rebase_count':self.gate.rebase_count,
                'recovery_samples':self.gate.recovery_count})))
    node = Bridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
