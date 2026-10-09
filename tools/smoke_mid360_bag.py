#!/usr/bin/env python3
"""Replay a real raw MID360 bag through pinned LIO; sensor-only diagnostic."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage
import yaml


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--bag', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--deps', default='/home/asus/nav_deps')
    parser.add_argument('--with-visualization', action='store_true')
    parser.add_argument('--max-speed', type=float, default=5.0)
    parser.add_argument('--max-displacement', type=float, default=10.0)
    parser.add_argument('--max-rotation', type=float, default=1.0,
                        help='Maximum orientation change from first estimate, radians')
    args = parser.parse_args()
    if not all(math.isfinite(v) and v > 0 for v in (args.max_speed, args.max_displacement, args.max_rotation)):
        raise ValueError('Positive finite diagnostic motion limits required')
    if os.environ.get('ROS_DOMAIN_ID') in (None, '', '0'):
        raise RuntimeError('Use an isolated ROS domain')
    bag = Path(args.bag).resolve(strict=True)
    meta = yaml.safe_load((bag / 'metadata.yaml').read_text())['rosbag2_bagfile_information']
    if meta['duration']['nanoseconds'] > 60*10**9:
        raise ValueError('This smoke test accepts bags up to 60 seconds')
    types = {t['topic_metadata']['name']: t['topic_metadata']['type'] for t in meta['topics_with_message_count']}
    if types.get('/livox/lidar') != 'sensor_msgs/msg/PointCloud2' or types.get('/livox/imu') != 'sensor_msgs/msg/Imu':
        raise ValueError('Expected pinned Livox PointCloud2 and raw IMU topics')
    output = Path(args.output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    upstream = Path(args.deps) / 'src/small_point_lio/config/mid360.yaml'
    params = yaml.safe_load(upstream.read_text())
    params['small_point_lio']['ros__parameters'].update({
        'lidar_type': 'livox_pointcloud2', 'lidar_topic': '/livox/lidar', 'imu_topic': '/livox/imu',
        'lidar_frame': 'front_mid360', 'state_frame': 'front_mid360_imu', 'odom_frame': 'odom',
        'publish_tf': False, 'use_sim_time': True, 'save_pcd': False,
        'acc_norm': 1.0, 'max_distance': 30.0,
    })
    # Preserve upstream example internal IMU/LiDAR extrinsics. This is NOT a measured
    # robot CalibrationBundle and does not authorize local_state or navigation.
    config = output / 'lio_diagnostic.yaml'
    config.write_text(yaml.safe_dump(params))
    rclpy.init()
    node = Node('mid360_real_bag_lio_probe')
    odom, clouds, tf, errors = [], [], [], []
    visual = {'path_messages':0, 'path_poses':0, 'preview_messages':0, 'preview_points':0}
    if args.with_visualization:
        from nav_msgs.msg import Path as RosPath
        from rclpy.qos import QoSProfile, DurabilityPolicy
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        def path_cb(m):
            visual['path_messages'] += 1; visual['path_poses'] = len(m.poses)
        def preview_cb(m):
            visual['preview_messages'] += 1; visual['preview_points'] = m.width*m.height
            if m.header.frame_id != 'odom': errors.append('preview coordinate frame invalid')
        node.create_subscription(RosPath, '/visualization/lio_path', path_cb, qos)
        node.create_subscription(PointCloud2, '/visualization/live_map_preview', preview_cb, qos)
    input_last = 0
    first_input = None
    def stamp(m):
        return m.header.stamp.sec*10**9 + m.header.stamp.nanosec
    def input_cb(m):
        nonlocal input_last, first_input
        input_last = stamp(m)
        if first_input is None: first_input = input_last
    def odom_cb(m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        values = [p.x,p.y,p.z,q.x,q.y,q.z,q.w] + list(m.pose.covariance)
        if (m.header.frame_id != 'odom' or m.child_frame_id != 'front_mid360_imu'
                or not all(math.isfinite(v) for v in values)
                or abs(sum(v*v for v in [q.x,q.y,q.z,q.w])-1) > 1e-4
                or (odom and stamp(m) < odom[-1]['stamp_ns'])):
            if len(errors) < 20: errors.append('invalid raw odometry')
        odom.append({'stamp_ns': stamp(m), 'position': [p.x,p.y,p.z],
                     'quaternion_xyzw': [q.x,q.y,q.z,q.w]})
    def cloud_cb(m):
        if m.header.frame_id != 'odom' or not m.width*m.height:
            if len(errors) < 20: errors.append('invalid deskewed cloud')
        clouds.append({'stamp_ns': stamp(m), 'points': m.width*m.height})
    node.create_subscription(PointCloud2,'/livox/lidar',input_cb,qos_profile_sensor_data)
    node.create_subscription(Odometry,'/lio/sensor_odometry',odom_cb,100)
    node.create_subscription(PointCloud2,'/lio/deskewed_odom_cloud',cloud_cb,qos_profile_sensor_data)
    node.create_subscription(TFMessage,'/tf',lambda m: tf.extend(m.transforms),10)
    procs, files = [], []
    def launch(cmd, name):
        f = (output/name).open('w'); files.append(f)
        p = subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
        procs.append(p); return p
    try:
        visual_procs = []
        if args.with_visualization:
            for exe in ('foxglove_trace_publisher', 'live_map_preview'):
                visual_procs.append(launch(['ros2','run','rm_nav_bringup',exe],exe+'.log'))
        lio = launch(['ros2','run','small_point_lio','small_point_lio_node',
                      '--ros-args','--params-file',str(config),
                      '-r','/Odometry:=/lio/sensor_odometry',
                      '-r','/cloud_registered:=/lio/deskewed_odom_cloud'],'lio.log')
        start = time.monotonic()
        while time.monotonic()-start < 3:
            rclpy.spin_once(node,timeout_sec=.02)
        player = launch(['ros2','bag','play',str(bag),'--clock','--delay','2'], 'player.log')
        deadline = time.monotonic()+meta['duration']['nanoseconds']/1e9+20
        while player.poll() is None and time.monotonic()<deadline:
            if lio.poll() is not None: raise RuntimeError('LIO exited early')
            rclpy.spin_once(node,timeout_sec=.01)
        if player.poll() != 0: raise RuntimeError('Bag player failed or timed out')
        start = time.monotonic()
        while time.monotonic()-start < 3:
            rclpy.spin_once(node,timeout_sec=.01)
        if lio.poll() is not None: raise RuntimeError('LIO exited early')
        if args.with_visualization:
            if any(p.poll() is not None for p in visual_procs): raise RuntimeError('visualization process exited')
            if visual['path_poses'] < 30 or visual['preview_points'] < 100 or visual['preview_messages'] < 3:
                raise RuntimeError('real LIO output did not generate live trace/map preview')
    except Exception as exc:
        errors.append(str(exc))
    finally:
        for p in reversed(procs):
            if p.poll() is None:
                os.killpg(p.pid,signal.SIGINT)
                try: p.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(p.pid,signal.SIGKILL); p.wait()
        for f in files: f.close()
        node.destroy_node(); rclpy.shutdown()
    lag_s = (input_last-odom[-1]['stamp_ns'])/1e9 if odom else None
    transport_passed = (len(odom)>=30 and len(clouds)>=30 and not tf and not errors
              and lag_s is not None and -.25<=lag_s<=.5
              and odom[0]['stamp_ns'] >= (first_input or 0)-250000000)
    def distance(a, b):
        return math.sqrt(sum((x-y)**2 for x,y in zip(a['position'],b['position'])))
    displacement = max((distance(odom[0],m) for m in odom),default=0)
    def rotation(a, b):
        qa, qb = a['quaternion_xyzw'], b['quaternion_xyzw']
        norm = math.sqrt(sum(v*v for v in qa) * sum(v*v for v in qb))
        if not math.isfinite(norm) or norm < 1e-12: return math.inf
        dot = abs(sum(x*y for x,y in zip(qa,qb))) / norm
        return 2 * math.acos(min(1.0,max(0.0,dot)))
    angle = max((rotation(odom[0],m) for m in odom),default=0)
    speed = max((distance(a,b)*1e9/(b['stamp_ns']-a['stamp_ns'])
                 for a,b in zip(odom,odom[1:]) if b['stamp_ns']-a['stamp_ns']>=1000000),default=0)
    motion_passed = (bool(odom) and displacement <= args.max_displacement
                     and speed <= args.max_speed and angle <= args.max_rotation)
    if not motion_passed: errors.append('estimated motion exceeds explicit diagnostic envelope')
    passed = transport_passed and motion_passed
    report = {'passed': passed, 'transport_passed':transport_passed,
              'visualization_enabled':args.with_visualization, 'visualization':visual,
              'motion_envelope_passed':motion_passed,
              'max_displacement_m':displacement,'max_step_speed_mps':speed,
              'max_rotation_rad':angle,
              'final_relative_displacement_m':distance(odom[0],odom[-1]) if odom else None,
              'output_span_s':(odom[-1]['stamp_ns']-odom[0]['stamp_ns'])/1e9 if odom else 0,
              'motion_limits':{'speed_mps':args.max_speed,'displacement_m':args.max_displacement,
                               'rotation_rad':args.max_rotation},
              'scope': 'real bag -> sensor LIO only; no truth accuracy or chassis/navigation acceptance',
              'bag': str(bag), 'internal_extrinsics': 'unverified upstream example; not measured robot calibration',
              'raw_acc_norm': 1.0, 'odometry_count': len(odom), 'deskewed_cloud_count':len(clouds),
              'tf_count':len(tf), 'last_input_to_output_lag_s':lag_s,
              'first_odometry':odom[0] if odom else None,'last_odometry':odom[-1] if odom else None,
              'errors':errors}
    (output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    (output/'odometry_trace.json').write_text(json.dumps(odom,indent=2)+'\n')
    print(json.dumps(report,indent=2))
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
