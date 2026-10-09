#!/usr/bin/env python3
"""Bounded driver-only hardware probe. Never launches serial, TF or motion nodes."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2, PointField


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--duration', type=float, default=20.)
    parser.add_argument('--external-driver', action='store_true',
                        help='Use an already running single driver; only record/probe topics')
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') in (None, '') or (os.environ.get('ROS_DOMAIN_ID') == '0' and not args.external_driver):
        raise RuntimeError('An isolated ROS_DOMAIN_ID is required')
    if not 5 <= args.duration <= 120:
        raise ValueError('duration must be 5..120 seconds')
    config = Path(args.config).resolve(strict=True)
    output = Path(args.output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    rclpy.init()
    node = Node('mid360_hardware_probe')
    clouds, imus, errors = [], [], []
    last = {'cloud': 0, 'imu': 0}
    fields = []

    def stamp(header):
        return header.stamp.sec * 10**9 + header.stamp.nanosec

    def cloud_cb(msg):
        nonlocal fields
        try:
            n = msg.width * msg.height
            if not n or n > 200000 or msg.height != 1 or msg.is_bigendian:
                raise ValueError('invalid cloud shape/endian')
            if len(msg.data) != msg.row_step or msg.row_step != n * msg.point_step:
                raise ValueError('invalid cloud stride/length')
            fields = [{'name': f.name, 'offset': f.offset, 'datatype': f.datatype,
                       'count': f.count} for f in msg.fields]
            by_name = {f.name: f for f in msg.fields}
            for name, dtype in [('x', PointField.FLOAT32), ('y', PointField.FLOAT32),
                                ('z', PointField.FLOAT32), ('tag', PointField.UINT8),
                                ('timestamp', PointField.FLOAT64)]:
                if name not in by_name or by_name[name].datatype != dtype or by_name[name].count != 1:
                    raise ValueError('unexpected field ' + name)
            dtype = np.dtype({'names': ['x', 'y', 'z', 'timestamp'],
                              'formats': ['<f4', '<f4', '<f4', '<f8'],
                              'offsets': [by_name[k].offset for k in ('x', 'y', 'z', 'timestamp')],
                              'itemsize': msg.point_step})
            points = np.frombuffer(msg.data, dtype=dtype, count=n)
            times = points['timestamp']
            source_ns = stamp(msg.header)
            if source_ns <= last['cloud'] or not all(np.isfinite(points[k]).all() for k in dtype.names):
                raise ValueError('non-monotonic header or non-finite points')
            # Pinned driver offset_time is actually absolute nanoseconds; prove on live data.
            if times.min() < source_ns - 1024 or times.max() > source_ns + 250000000:
                raise ValueError('point time is not absolute ns within the scan')
            last['cloud'] = source_ns
            clouds.append({'stamp_ns': source_ns, 'points': n,
                           'span_ms': float(times.max() - times.min()) / 1e6,
                           'frame': msg.header.frame_id, 'point_step': msg.point_step,
                           'wall_skew_s': time.time() - source_ns / 1e9})
        except Exception as exc:
            if len(errors) < 20: errors.append(str(exc))

    def imu_cb(msg):
        source_ns = stamp(msg.header)
        a = msg.linear_acceleration
        w = msg.angular_velocity
        values = [a.x, a.y, a.z, w.x, w.y, w.z]
        if source_ns <= last['imu'] or not all(math.isfinite(v) for v in values):
            if len(errors) < 20: errors.append('invalid/non-monotonic IMU')
            return
        last['imu'] = source_ns
        imus.append({'stamp_ns': source_ns, 'acceleration_norm': math.sqrt(sum(v*v for v in values[:3])),
                     'gyro_norm': math.sqrt(sum(v*v for v in values[3:])), 'frame': msg.header.frame_id})

    node.create_subscription(PointCloud2, '/livox/lidar', cloud_cb, qos_profile_sensor_data)
    node.create_subscription(Imu, '/livox/imu', imu_cb, qos_profile_sensor_data)
    processes, handles = [], []
    def launch(cmd, logfile):
        handle = (output / logfile).open('w')
        handles.append(handle)
        proc = subprocess.Popen(cmd, stdout=handle, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(proc)
        return proc

    try:
        driver = None
        if not args.external_driver:
            driver = launch(['ros2', 'run', 'livox_ros_driver2', 'livox_ros_driver2_node',
                             '--ros-args', '-p', 'xfer_format:=0', '-p', 'multi_topic:=0',
                             '-p', 'data_src:=0', '-p', 'publish_freq:=10.0',
                             '-p', 'output_data_type:=0', '-p', 'frame_id:=front_mid360',
                             '-p', f'user_config_path:={config}'], 'driver.log')
        recorder = launch(['ros2', 'bag', 'record', '-o', str(output / 'raw_bag'),
                           '/livox/lidar', '/livox/imu'], 'recorder.log')
        start = time.monotonic()
        while time.monotonic() - start < args.duration:
            if (driver is not None and driver.poll() is not None) or recorder.poll() is not None:
                raise RuntimeError('Driver/recorder exited; inspect logs')
            rclpy.spin_once(node, timeout_sec=.01)
    except Exception as exc:
        errors.append(str(exc))
    finally:
        for proc in reversed(processes):
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGINT)
                try: proc.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid, signal.SIGTERM)
                    try: proc.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        os.killpg(proc.pid, signal.SIGKILL)
                        proc.wait()
        for handle in handles: handle.close()
        node.destroy_node()
        rclpy.shutdown()
    def rate(samples):
        return (len(samples)-1)*1e9/(samples[-1]['stamp_ns']-samples[0]['stamp_ns']) if len(samples)>1 else 0
    # Exclude startup backlog when evaluating sustained rates and clock alignment.
    steady_clouds = [c for c in clouds if c['stamp_ns'] >= clouds[0]['stamp_ns'] + 4*10**9] if clouds else []
    steady_imus = [c for c in imus if c['stamp_ns'] >= imus[0]['stamp_ns'] + 4*10**9] if imus else []
    max_skew = max((abs(c['wall_skew_s']) for c in steady_clouds), default=float('inf'))
    passed = (len(steady_clouds) >= 30 and rate(steady_clouds) >= 9.
              and rate(steady_imus) >= 180. and max_skew <= .5 and not errors)
    report = {'passed': passed, 'scope': 'live MID360 driver only; no robot calibration/actuation',
              'cloud_count': len(clouds), 'imu_count': len(imus),
              'cloud_source_hz': rate(clouds), 'imu_source_hz': rate(imus),
              'steady_cloud_hz': rate(steady_clouds), 'steady_imu_hz': rate(steady_imus),
              'steady_max_wall_skew_s': max_skew if math.isfinite(max_skew) else None,
              'point_count_range': [min(c['points'] for c in clouds), max(c['points'] for c in clouds)] if clouds else [],
              'cloud_first': clouds[0] if clouds else None, 'cloud_last': clouds[-1] if clouds else None,
              'imu_acc_norm_mean': sum(m['acceleration_norm'] for m in imus)/len(imus) if imus else None,
              'imu_first': imus[0] if imus else None, 'fields': fields, 'errors': errors}
    (output / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
