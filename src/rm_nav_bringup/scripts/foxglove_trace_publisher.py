#!/usr/bin/env python3
"""Bounded Odometry -> Path traces for Foxglove; visualization only."""
import argparse
from collections import deque
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from nav_msgs.msg import Odometry, Path


class TracePublisher(Node):
    def __init__(self, specs, max_points):
        super().__init__('foxglove_trace_publisher')
        self.max_points = max_points
        self.buffers = {}
        self.pubs = {}
        for source, topic, output in specs:
            self.buffers[output] = deque(maxlen=max_points)
            self.pubs[output] = self.create_publisher(Path, output, QoSProfile(
                depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.create_subscription(Odometry, topic,
                lambda msg, key=output, label=source: self.receive(msg, key, label), qos_profile_sensor_data)

    def receive(self, msg, output, label):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        values = [p.x,p.y,p.z,q.x,q.y,q.z,q.w]
        if not msg.header.frame_id or not all(math.isfinite(v) for v in values): return
        if abs(sum(v*v for v in [q.x,q.y,q.z,q.w])-1) > 1e-3: return
        buffer = self.buffers[output]
        source_ns = msg.header.stamp.sec*10**9 + msg.header.stamp.nanosec
        last_ns = (buffer[-1].header.stamp.sec*10**9 + buffer[-1].header.stamp.nanosec) if buffer else -1
        if buffer and (msg.header.frame_id != buffer[-1].header.frame_id or
                       source_ns <= last_ns): return
        pose = msg.pose.pose
        from geometry_msgs.msg import PoseStamped
        sample = PoseStamped(); sample.header = msg.header; sample.pose = pose
        buffer.append(sample)
        path = Path(); path.header = msg.header; path.poses = list(buffer)
        self.pubs[output].publish(path)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--max-points', type=int, default=5000)
    args, _ = parser.parse_known_args()
    if not 10 <= args.max_points <= 20000: raise ValueError('max-points must be 10..20000')
    rclpy.init(); node = TracePublisher([
        ('lio','/lio/sensor_odometry','/visualization/lio_path'),
        ('chassis','/state/chassis','/visualization/chassis_path'),
        ('nav','/odom','/visualization/nav_path'),
    ], args.max_points)
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__': main()
