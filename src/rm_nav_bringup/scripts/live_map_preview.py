#!/usr/bin/env python3
"""Bounded odom cloud accumulation for display, independent of optimized/frozen maps."""
import argparse
from collections import OrderedDict
import json
import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String


class LiveMapPreview(Node):
    def __init__(self, voxel, max_voxels, publish_hz):
        super().__init__('live_map_preview')
        self.voxel = voxel
        self.limit = max_voxels
        self.voxels = OrderedDict()
        self.header = None
        self.last_ns = -1
        self.received_at = 0.
        self.dirty = False
        self.accepted = 0
        self.evicted = 0
        self.reason = 'waiting for deskewed odom cloud'
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.cloud_pub = self.create_publisher(PointCloud2, '/visualization/live_map_preview', qos)
        self.status_pub = self.create_publisher(String, '/visualization/live_map_status', qos)
        self.create_subscription(PointCloud2, '/lio/deskewed_odom_cloud', self.receive, qos_profile_sensor_data)
        self.create_timer(1.0/publish_hz, self.publish)

    def receive(self, msg):
        try:
            n = msg.width*msg.height
            stamp_ns = msg.header.stamp.sec*10**9 + msg.header.stamp.nanosec
            if (msg.header.frame_id != 'odom' or stamp_ns <= self.last_ns or stamp_ns <= 0
                    or msg.height != 1 or not 1 <= n <= 200000 or msg.is_bigendian
                    or not 12 <= msg.point_step <= 256 or msg.row_step != n*msg.point_step
                    or len(msg.data) != msg.row_step):
                raise ValueError('invalid odom cloud frame/time/storage')
            offsets = []
            for name in ('x', 'y', 'z'):
                fields = [f for f in msg.fields if f.name == name]
                if (len(fields) != 1 or fields[0].datatype != PointField.FLOAT32
                        or fields[0].count != 1 or fields[0].offset+4 > msg.point_step):
                    raise ValueError('expected unique float32 XYZ fields')
                offsets.append(fields[0].offset)
            dtype = np.dtype({'names':['x','y','z'], 'formats':['<f4']*3,
                              'offsets':offsets, 'itemsize':msg.point_step})
            scan = np.frombuffer(msg.data, dtype=dtype, count=n)
            xyz = np.column_stack([scan[k] for k in ('x','y','z')])
            if not np.isfinite(xyz).all() or np.abs(xyz).max() > 10000:
                raise ValueError('nonfinite or out-of-bounds odom geometry')
            keys = np.floor(xyz/self.voxel).astype(np.int64)
            # No TF conversion: upstream has already placed these points in odom.
            for key, point in zip(keys, xyz):
                k = tuple(int(v) for v in key)
                self.voxels[k] = tuple(float(v) for v in point)
                self.voxels.move_to_end(k)
                if len(self.voxels) > self.limit:
                    self.voxels.popitem(last=False)
                    self.evicted += 1
            self.header = msg.header
            self.last_ns = stamp_ns
            self.received_at = time.monotonic()
            self.accepted += 1
            self.dirty = True
            self.reason = 'preview updated from native odom points'
        except (ValueError, OverflowError) as exc:
            self.reason = str(exc)

    def publish(self):
        if self.dirty:
            msg = PointCloud2()
            msg.header = self.header  # Latest actual source time; never stamp a retained map as fresh.
            msg.height = 1
            msg.width = len(self.voxels)
            msg.fields = [PointField(name=name, offset=i*4, datatype=PointField.FLOAT32, count=1)
                          for i, name in enumerate(('x','y','z'))]
            msg.point_step = 12
            msg.row_step = 12*msg.width
            msg.is_dense = True
            msg.data = np.asarray(list(self.voxels.values()), dtype='<f4').tobytes()
            self.cloud_pub.publish(msg)
            self.dirty = False
        status = String()
        status.data = json.dumps({'scope':'odom preview only; not optimized or frozen navigation map',
                                  'frame':'odom', 'voxel_m':self.voxel, 'points':len(self.voxels),
                                  'point_limit':self.limit, 'evicted_voxels':self.evicted,
                                  'accepted_scans':self.accepted, 'last_source_stamp_ns':self.last_ns,
                                  'input_stale':not self.header or time.monotonic()-self.received_at > .5,
                                  'reason':self.reason})
        self.status_pub.publish(status)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--voxel', type=float, default=.1)
    parser.add_argument('--max-voxels', type=int, default=200000)
    parser.add_argument('--publish-hz', type=float, default=2.0)
    args, _ = parser.parse_known_args()
    if (not math.isfinite(args.voxel) or not .05 <= args.voxel <= 1 or not 10 <= args.max_voxels <= 200000
            or not math.isfinite(args.publish_hz) or not .2 <= args.publish_hz <= 10):
        raise ValueError('voxel .05..1 m, cap 10..200000, publish-hz .2..10')
    rclpy.init()
    node = LiveMapPreview(args.voxel, args.max_voxels, args.publish_hz)
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__': main()
