#!/usr/bin/env python3
"""Read recorded poses/IMU only; no playback, ROS node, or control output.

Operator-reported stationary chassis is a test condition, not motion ground truth.
The configured lever arm is taken from the vehicle profile without fitting it again.
"""
import argparse
import json
import math
from pathlib import Path
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/rm_nav_bringup/scripts'))
from sensor_reference_math import sensor_pose


def summarize(rows):
    if not rows:
        return {'count': 0}
    stamps = np.array([r[0] for r in rows], dtype=np.int64)
    a = np.array([r[1:] for r in rows])
    qx, qy, qz, qw = a[:, 3:7].T
    yaw = np.unwrap(np.arctan2(2*(qw*qz+qx*qy), 1-2*(qy*qy+qz*qz)))
    return {
        'count': len(rows), 'duration_s': float((stamps[-1]-stamps[0])*1e-9),
        'source_nonmonotonic_count': int(np.sum(np.diff(stamps) <= 0)),
        'xy_max_from_start_m': float(np.linalg.norm(a[:, :2]-a[0, :2], axis=1).max()),
        'xy_end_from_start_m': float(np.linalg.norm(a[-1, :2]-a[0, :2])),
        'z_span_m': float(np.ptp(a[:, 2])),
        'yaw_span_deg': float(np.degrees(np.ptp(yaw))),
        'yaw_end_delta_deg': float(np.degrees(yaw[-1]-yaw[0])),
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--bag', type=Path, required=True)
    ap.add_argument('--output', type=Path, required=True)
    ap.add_argument('--focus-stamp-ns', type=int, help='Source stamp of a failed edge; inspect +/-1 second')
    args = ap.parse_args()
    if args.output.exists():
        raise ValueError('Refusing to overwrite existing evidence')
    profile = Path(__file__).resolve().parents[1] / 'src/rm_nav_bringup/config/vehicle_164_geometry.yaml'
    lever = yaml.safe_load(profile.read_text())['mount']['axis_to_lidar_at_reference_height']
    offset = [a-b for a, b in zip([-.011, -.02329, .04412], lever)]
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(args.bag), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    topics = ['/lio/sensor_odometry', '/sensor_reference/odom', '/livox/imu']
    reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
    poses = {name: [] for name in topics[:2]}
    axis = []
    imu = []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        msg = deserialize_message(data, Imu if topic == topics[2] else Odometry)
        stamp = msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        if topic == topics[2]:
            a, g = msg.linear_acceleration, msg.angular_velocity
            imu.append([stamp, math.sqrt(a.x*a.x+a.y*a.y+a.z*a.z),
                        math.sqrt(g.x*g.x+g.y*g.y+g.z*g.z)])
            continue
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        row = [stamp, p.x, p.y, p.z, q.x, q.y, q.z, q.w]
        poses[topic].append(row)
        if topic == topics[0]:
            point, _ = sensor_pose(row[1:4], row[4:8], offset)
            axis.append([stamp, *point, *row[4:8]])
    expected = {r[0]: r[1:3] for r in axis}
    errors = [math.dist(r[1:3], expected[r[0]]) for r in poses[topics[1]] if r[0] in expected]
    report = {'scope': 'recorded estimates only; no independent ground truth',
              'condition': 'operator reports chassis stationary, gimbal rotated',
              'control_output': False, 'bag': str(args.bag),
              'configured_axis_to_lidar_m': lever,
              'lio_imu_pose': summarize(poses[topics[0]]),
              'recorded_planar_axis_pose': summarize(poses[topics[1]]),
              'reconstructed_3d_axis_pose': summarize(axis),
              'matched_reference_stamps': len(errors),
              'max_recorded_vs_reconstructed_xy_m': max(errors) if errors else None,
              'imu_count': len(imu),
              'navigation_ready': False}
    if imu:
        values = np.array([r[1:] for r in imu])
        report['imu_acc_norm_g_p50_p99_max'] = np.percentile(values[:, 0], [50, 99, 100]).tolist()
        report['imu_gyro_norm_rad_s_p50_p99_max'] = np.percentile(values[:, 1], [50, 99, 100]).tolist()
        peak = int(np.argmax(values[:, 1]))
        report['peak_gyro_stamp_ns'] = imu[peak][0]
        report['peak_gyro_seconds_from_bag_imu_start'] = (imu[peak][0]-imu[0][0])*1e-9
        times = np.array([r[0] for r in imu], dtype=np.int64)
        dt = np.diff(times)*1e-9
        report['imu_source_nonmonotonic_count'] = int(np.sum(dt <= 0))
        report['high_rotation_sample_count_above_0_2_rad_s'] = int(np.sum(values[:, 1] > .2))
        report['high_rotation_observed_interval_s'] = float(np.sum(
            dt[(values[:-1, 1] > .2) & (dt > 0) & (dt < .1)]))
    if args.focus_stamp_ns is not None:
        near = lambda rows: [r for r in rows if abs(r[0]-args.focus_stamp_ns) <= 10**9]
        focus_imu = near(imu)
        report['failed_edge_window'] = {
            'source_stamp_ns': args.focus_stamp_ns, 'half_width_s': 1,
            'lio': summarize(near(poses[topics[0]])),
            'reference': summarize(near(poses[topics[1]])),
            'imu_gyro_peak_rad_s': max((r[2] for r in focus_imu), default=None),
        }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
