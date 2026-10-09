#!/usr/bin/env python3
"""Headless gate acceptance: fresh heartbeats, no cached restart, invalid commands."""
import os
from pathlib import Path
import signal
import subprocess
import time
import math

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Bool


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def main():
    require(os.environ.get('ROS_DOMAIN_ID') not in (None, '', '0'), 'Use a dedicated ROS_DOMAIN_ID')
    rclpy.init()
    node = Node('motion_gate_acceptance')
    received = []
    node.create_subscription(TwistStamped, '/nav/cmd_vel_safe', received.append, 10)
    command = node.create_publisher(TwistStamped, '/nav/cmd_vel_checked', 10)
    health = node.create_publisher(Bool, '/localization/healthy', 10)
    enable = node.create_publisher(Bool, '/nav/motion_enable', 10)
    chassis = node.create_publisher(Bool, '/state/chassis_healthy', 10)
    process = None
    logfile = Path('log/motion_gate_smoke.log')
    logfile.parent.mkdir(exist_ok=True)

    def pump(seconds, healthy=True, enabled=True, send=True, mutate=None, chassis_healthy=True):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if chassis_healthy is not None: chassis.publish(Bool(data=chassis_healthy))
            if healthy is not None:
                health.publish(Bool(data=healthy))
            if enabled is not None:
                enable.publish(Bool(data=enabled))
            if send:
                msg = TwistStamped()
                msg.header.frame_id = 'base_link'
                msg.header.stamp = node.get_clock().now().to_msg()
                msg.twist.linear.x = 0.2
                if mutate:
                    mutate(msg)
                command.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.02)

    def stopped():
        v = received[-1].twist
        return max(abs(v.linear.x), abs(v.linear.y), abs(v.angular.z)) < 1e-6

    try:
        with logfile.open('w') as log:
            process = subprocess.Popen(['ros2', 'run', 'rm_nav_control', 'motion_gate'],
                                       stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            end = time.monotonic() + 10
            while time.monotonic() < end and not received:
                rclpy.spin_once(node, timeout_sec=0.02)
            require(received and stopped(), 'Gate not stopped at startup')
            pump(0.5, healthy=False)
            require(stopped(), 'Unhealthy motion permitted')
            pump(0.5)
            require(abs(received[-1].twist.linear.x - 0.2) < 1e-6, 'Healthy motion not permitted')
            pump(0.15, healthy=False)
            require(stopped(), 'Explicit unhealthy signal did not stop')
            pump(0.15, send=False)
            require(stopped(), 'Cached motion resumed after health recovery')
            pump(0.4)
            require(not stopped(), 'Fresh command failed to recover')
            pump(0.15,chassis_healthy=False)
            require(stopped(),'Chassis state loss did not stop')
            pump(0.15,send=False)
            require(stopped(),'Cached motion replayed after chassis health recovery')
            pump(0.4)
            pump(0.4,chassis_healthy=None)
            require(stopped(),'Chassis heartbeat timeout did not stop')
            pump(0.4)
            pump(0.4, healthy=None)
            require(stopped(), 'Health heartbeat timeout did not stop')
            pump(0.4)
            pump(0.4, enabled=None)
            require(stopped(), 'Motion-enable heartbeat timeout did not stop')
            pump(0.4)
            pump(0.15, enabled=False)
            require(stopped(), 'Explicit disable did not stop')
            pump(0.15, send=False)
            require(stopped(), 'Cached motion resumed after enable recovery')
            pump(0.4)
            pump(0.4, send=False)
            require(stopped(), 'Command receipt timeout did not stop')
            for description, mutate in [
                ('NaN', lambda m: setattr(m.twist.linear, 'x', float('nan'))),
                ('wrong frame', lambda m: setattr(m.header, 'frame_id', 'map')),
                ('stale stamp', lambda m: setattr(m.header, 'stamp', (node.get_clock().now() - Duration(seconds=5)).to_msg())),
                ('future stamp', lambda m: setattr(m.header, 'stamp', (node.get_clock().now() + Duration(seconds=5)).to_msg())),
                ('nonplanar', lambda m: setattr(m.twist.angular, 'x', 1.0)),
            ]:
                pump(0.4)
                pump(0.15, mutate=mutate)
                require(stopped(), description + ' command did not stop')
            def large(msg):
                msg.twist.linear.x = msg.twist.linear.y = 0.5
                msg.twist.angular.z = 2.0
            pump(0.4, mutate=large)
            out = received[-1]
            require(abs(math.hypot(out.twist.linear.x, out.twist.linear.y) - 0.5) < 1e-6, 'Vector norm not limited')
            require(out.twist.angular.z == 1.0 and out.header.frame_id == 'base_link', 'Yaw limit or output frame incorrect')
            print('PASS gate startup, unhealthy/stale health, disabled/stale permission, no cached recovery, '
                  'command timeout, NaN/frame/time/nonplanar rejection and vector/yaw limits', flush=True)
    finally:
        if process is not None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
