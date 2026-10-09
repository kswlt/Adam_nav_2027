#!/usr/bin/env python3
"""Publish a bounded binary XYZ PCD as a latched PointCloud2 for Foxglove."""
import argparse
import struct
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


class PcdMapPublisher(Node):
    def __init__(self, path, frame, topic, rate):
        super().__init__('foxglove_pcd_map_publisher')
        self.pub = self.create_publisher(PointCloud2, topic, QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE))
        self.msg = self.load(path, frame)
        self.timer = self.create_timer(1.0 / rate, lambda: self.pub.publish(self.msg))
        self.get_logger().info(f'Publishing {self.msg.width} map points on {topic} in frame {frame}')

    def load(self, path, frame):
        path = Path(path).resolve(strict=True)
        if path.stat().st_size > 256 * 1024 * 1024:
            raise ValueError('PCD exceeds 256 MiB visualization limit')
        raw = path.read_bytes(); marker = raw.find(b'DATA binary\n')
        if marker < 0: raise ValueError('Only binary XYZ PCD is supported')
        header = raw[:marker].decode('ascii', errors='strict')
        fields = next((line.split()[1:] for line in header.splitlines() if line.startswith('FIELDS ')), [])
        sizes = next((line.split()[1:] for line in header.splitlines() if line.startswith('SIZE ')), [])
        types = next((line.split()[1:] for line in header.splitlines() if line.startswith('TYPE ')), [])
        points = int(next(line.split()[1] for line in header.splitlines() if line.startswith('POINTS ')))
        if fields != ['x','y','z'] or sizes != ['4','4','4'] or types != ['F','F','F'] or points < 1 or points > 1000000:
            raise ValueError('Expected bounded binary XYZ PCD with <=1M points')
        payload = raw[marker + len(b'DATA binary\n'):]
        if len(payload) != points * 12: raise ValueError('PCD payload length mismatch')
        msg = PointCloud2(); msg.header = Header(); msg.header.frame_id = frame
        msg.height = 1; msg.width = points; msg.is_bigendian = False; msg.is_dense = True
        msg.fields = [PointField(name=n, offset=i*4, datatype=PointField.FLOAT32, count=1)
                      for i,n in enumerate(['x','y','z'])]
        msg.point_step = 12; msg.row_step = points * 12; msg.data = payload
        return msg


def main():
    parser = argparse.ArgumentParser(); parser.add_argument('--pcd', required=True)
    parser.add_argument('--frame', default='map'); parser.add_argument('--topic', default='/visualization/map_cloud')
    parser.add_argument('--rate', type=float, default=1.0); args, _ = parser.parse_known_args()
    if not 0.1 <= args.rate <= 10: raise ValueError('rate must be 0.1..10 Hz')
    rclpy.init(); node = PcdMapPublisher(args.pcd, args.frame, args.topic, args.rate)
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__': main()
