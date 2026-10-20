#!/usr/bin/env python3
"""Gated Stamped sensor-yaw xy -> original Adam Twist input, without another rotation.

This node never opens a serial device. Only legacy_adam_transport may wire its
private candidate topic to my_serial_py. Original global /cmd_vel is untouched.
"""
import copy
import json
import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Bool, String


class LegacyAdamAdapter(Node):
    def __init__(self):
        super().__init__('legacy_adam_adapter')
        self.timeout = self.declare_parameter('timeout', .25).value
        self.require_serial = self.declare_parameter('require_serial_health', True).value
        if not .05 <= self.timeout <= .5:
            raise ValueError('Invalid legacy adapter timeout')
        self.flags = {name: (False, -math.inf) for name in ('state', 'permission', 'serial')}
        self.command, self.received, self.last_stamp = None, -math.inf, 0
        self.output = self.create_publisher(Twist, '/hardware/legacy_adam/cmd_vel', 1)
        self.status = self.create_publisher(String, '/hardware/legacy_adam/adapter_status', 1)
        for key, topic in (
                ('state', '/state/sensor_reference_healthy'),
                ('permission', '/nav/motion_allowed'),
                ('serial', '/hardware/legacy_adam/serial_healthy')):
            self.create_subscription(Bool, topic, lambda m, k=key: self.health(k, m), 1)
        self.create_subscription(TwistStamped, '/nav/sensor_cmd_vel_shadow', self.velocity, 1)
        self.create_timer(.02, self.tick)

    def health(self, key, msg):
        self.flags[key] = msg.data, time.monotonic()
        if not msg.data and (key != 'serial' or self.require_serial):
            self.command = None

    def ready(self):
        keys = ('state', 'permission', 'serial') if self.require_serial else ('state', 'permission')
        return all(self.flags[key][0] and time.monotonic()-self.flags[key][1] <= self.timeout for key in keys)

    def velocity(self, msg):
        stamp = msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds-stamp)*1e-9
        v = msg.twist
        if (not self.ready() or msg.header.frame_id != 'sensor_yaw_reference' or
                stamp <= self.last_stamp or not 0 <= age <= self.timeout or
                not all(map(math.isfinite, (v.linear.x, v.linear.y, v.linear.z,
                                           v.angular.x, v.angular.y, v.angular.z))) or
                max(abs(v.linear.z), abs(v.angular.x), abs(v.angular.y), abs(v.angular.z)) > 1e-6 or
                math.hypot(v.linear.x, v.linear.y) > .5+1e-6):
            self.command = None
            return
        self.last_stamp, self.received = stamp, time.monotonic()
        self.command = copy.deepcopy(msg)

    def tick(self):
        active = self.ready() and self.command is not None and time.monotonic()-self.received <= self.timeout
        if active:
            stamp = self.command.header.stamp.sec*10**9+self.command.header.stamp.nanosec
            active = 0 <= (self.get_clock().now().nanoseconds-stamp)*1e-9 <= self.timeout
        out = Twist()
        if active:
            # R(-sensor_yaw) was already applied upstream. Wire negation belongs
            # ONLY to protocol.encode_command, exactly once as in original Adam.
            out.linear.x, out.linear.y = self.command.twist.linear.x, self.command.twist.linear.y
        else:
            self.command = None
        self.output.publish(out)
        self.status.publish(String(data=json.dumps({
            'adapter': 'legacy_adam', 'active': bool(active),
            'serial_health_required': self.require_serial,
            'axes': 'horizontal_sensor_yaw_xy', 'yaw_owner': 'external_original_cmd_yaw_angle',
            'velocity_point_compensation': 'NOT_VERIFIED', 'full_chassis_stop': 'NOT_VERIFIED'})))


def main():
    rclpy.init()
    node = LegacyAdamAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.output.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
