#!/usr/bin/env python3
"""Real ROS GICP/MapOdom acceptance using simulated deskewed odom clouds."""
import copy
import math
import os
from pathlib import Path
import random
import signal
import struct
import subprocess
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped
from rm_nav_interfaces.msg import RegistrationEstimate
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Bool, String
from tf2_msgs.msg import TFMessage


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def cloud(points, frame, stamp):
    msg = PointCloud2()
    msg.header.frame_id, msg.header.stamp = frame, stamp
    msg.height, msg.width = 1, len(points)
    msg.fields = [PointField(name=name, offset=i * 4, datatype=PointField.FLOAT32, count=1)
                  for i, name in enumerate(('x', 'y', 'z'))]
    msg.point_step, msg.row_step = 12, len(points) * 12
    msg.is_dense = True
    msg.data = b''.join(struct.pack('<fff', *point) for point in points)
    return msg


class Acceptance(Node):
    def __init__(self):
        super().__init__('frozen_map_acceptance')
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map = self.create_publisher(PointCloud2, '/localization/frozen_map', latched)
        self.submap = self.create_publisher(PointCloud2, '/localization/odom_submap', qos_profile_sensor_data)
        self.candidate = self.create_publisher(RegistrationEstimate, '/localization/estimate', 10)
        self.estimate = None
        self.estimate_count = 0
        self.transform = None
        self.tf = None
        self.healthy = None
        self.pending = None
        self.reason = ''
        self.create_subscription(RegistrationEstimate, '/localization/estimate', self.receive_estimate, 10)
        self.create_subscription(TransformStamped, '/localization/map_to_odom', self.receive_transform, latched)
        self.create_subscription(Bool, '/localization/healthy', lambda m: setattr(self, 'healthy', m.data), latched)
        self.create_subscription(Bool, '/localization/correction_pending', lambda m: setattr(self, 'pending', m.data), latched)
        self.create_subscription(String, '/localization/status_reason', lambda m: setattr(self, 'reason', m.data), latched)
        self.create_subscription(TFMessage, '/tf', self.receive_tf, 10)

    def receive_estimate(self, msg):
        self.estimate, self.estimate_count = msg, self.estimate_count + 1

    def receive_transform(self, msg):
        self.transform = msg

    def receive_tf(self, msg):
        for transform in msg.transforms:
            if transform.header.frame_id == 'map' and transform.child_frame_id == 'odom':
                self.tf = transform

    def wait(self, predicate, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)
            if predicate():
                return True
        return False

    def send_estimate(self, original, mutate=None):
        msg = copy.deepcopy(original)
        msg.header.stamp = self.get_clock().now().to_msg()
        if mutate:
            mutate(msg)
        self.candidate.publish(msg)
        self.wait(lambda: False, 0.15)
        return msg


def main():
    require(os.environ.get('ROS_DOMAIN_ID') not in (None, '', '0'), 'Use an unused nonzero ROS_DOMAIN_ID')
    rclpy.init()
    node = Acceptance()
    process = None
    report = []
    log_path = Path('log/frozen_map_localization_smoke.log')
    log_path.parent.mkdir(exist_ok=True)
    try:
        with log_path.open('w') as log:
            process = subprocess.Popen([
                'ros2', 'launch', 'rm_nav_bringup', 'frozen_map_localization.launch.py',
                'map_version:=acceptance-v1'], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            require(node.wait(lambda: node.healthy is False and node.map.get_subscription_count() > 0
                              and node.submap.get_subscription_count() > 0, 15), 'Nodes did not start')
            require(node.tf is None and node.transform is None, 'Unverified startup TF published')
            report.append('startup has no unverified map->odom TF')
            rng = random.Random(2027)
            points = [tuple(rng.uniform(-2, 2) for _ in range(3)) for _ in range(2000)]
            c, s = math.cos(0.04), math.sin(0.04)
            mapped = [(c*x - s*y + 0.12, s*x + c*y - 0.10, z + 0.03) for x, y, z in points]
            mapped += [(20 + i * 0.1, 20, 20) for i in range(100)]
            map_msg = cloud(mapped, 'map', node.get_clock().now().to_msg())
            node.map.publish(map_msg)
            node.wait(lambda: False, 0.2)
            for _ in range(10):
                node.submap.publish(cloud(points, 'odom', node.get_clock().now().to_msg()))
                if node.wait(lambda: node.healthy is True and node.tf is not None and node.transform is not None, 0.3):
                    break
            require(node.healthy is True and node.transform is not None, 'Real GICP result did not become healthy')
            good = copy.deepcopy(node.estimate)
            tx = node.transform.transform.translation
            error = math.sqrt((tx.x - 0.12)**2 + (tx.y + 0.10)**2 + (tx.z - 0.03)**2)
            require(error < 0.01, 'map_T_odom transform direction or position incorrect')
            require(good.target_point_count < len(mapped) and good.inlier_ratio > 0.9, 'Map crop or inlier metric failed')
            require(abs(node.tf.transform.translation.x - tx.x) < 1e-6, 'Published TF differs from accepted result')
            print(f'PASS real ROS GICP: position_error={error:.8f}m, rmse={good.residual:.6f}m', flush=True)
            report.append('real GICP, cropped frozen map, quality acceptance and map->odom TF')
            accepted_transform = copy.deepcopy(node.transform.transform)

            def unchanged():
                require(node.tf.transform == accepted_transform, 'Rejected result changed map->odom TF')

            def restore():
                node.send_estimate(good)
                require(node.wait(lambda: node.healthy is True and node.pending is False, 0.5), 'Recovery to valid result failed')

            # Duplicate observation cannot keep localization alive.
            restore()
            duplicate = copy.deepcopy(node.estimate)
            node.candidate.publish(duplicate)
            require(node.wait(lambda: node.healthy is False and 'timestamp invalid' in node.reason, 0.5), 'Duplicate accepted')
            unchanged()
            report.append('duplicate estimate rejected')
            restore()
            node.send_estimate(good, lambda m: setattr(m, 'map_version', 'other-map'))
            require(node.healthy is False, 'Map version mismatch accepted')
            unchanged()
            report.append('wrong map version rejected')
            restore()
            node.send_estimate(good, lambda m: setattr(m.header, 'stamp', (node.get_clock().now() - Duration(seconds=5)).to_msg()))
            require(node.healthy is False, 'Stale estimate accepted')
            unchanged()
            report.append('stale estimate rejected')
            restore()
            node.send_estimate(good, lambda m: setattr(m, 'residual', float('nan')))
            require(node.healthy is False, 'NaN quality accepted')
            unchanged()
            report.append('NaN quality rejected')
            restore()
            node.send_estimate(good, lambda m: setattr(m.target_to_source.rotation, 'w', 0.0))
            require(node.healthy is False, 'Invalid quaternion accepted')
            unchanged()
            report.append('invalid transform rejected')
            restore()
            # Forged seed delta=0 must not bypass the manager's TF-relative gate.
            def large(msg):
                msg.target_to_source.translation.x += 3.0
                msg.translation_delta = 0.0
            node.send_estimate(good, large)
            require(node.pending is True and node.healthy is False, 'Large correction not held for confirmation')
            unchanged()
            report.append('large correction held; matcher delta cannot bypass gate')
            restore()
            require(node.wait(lambda: node.healthy is False and 'timed out' in node.reason, 2), 'Quality timeout failed')
            unchanged()
            report.append('missing estimates disable health and preserve continuous TF')
            # Bad or stale source clouds must not produce new estimates.
            count = node.estimate_count
            bad = cloud(points, 'odom', node.get_clock().now().to_msg())
            bad.data = bad.data[:-1]
            node.submap.publish(bad)
            node.wait(lambda: False, 0.25)
            require(node.estimate_count == count, 'Malformed source cloud produced estimate')
            old = cloud(points, 'odom', (node.get_clock().now() - Duration(seconds=5)).to_msg())
            node.submap.publish(old)
            node.wait(lambda: False, 0.25)
            require(node.estimate_count == count, 'Stale source cloud produced estimate')
            report.append('malformed/stale source clouds rejected')
            # A changed map in a frozen session suspends matching until restart.
            changed = cloud(mapped[:-1], 'map', node.get_clock().now().to_msg())
            node.map.publish(changed)
            node.wait(lambda: False, 0.2)
            node.submap.publish(cloud(points, 'odom', node.get_clock().now().to_msg()))
            node.wait(lambda: False, 0.3)
            require(node.estimate_count == count, 'Changed frozen map was used without new session')
            report.append('frozen-map mutation suspends matching')
            print('PASS ' + '; '.join(report), flush=True)
    finally:
        if process is not None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        node.destroy_node()
        rclpy.shutdown()
        Path('log/frozen_map_localization_smoke_results.txt').write_text('\n'.join(report), encoding='utf-8')


if __name__ == '__main__':
    main()
