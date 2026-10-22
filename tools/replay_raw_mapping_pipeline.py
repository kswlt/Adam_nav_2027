#!/usr/bin/env python3
"""Raw-only MCAP -> actual vehicle state -> fresh keyframes -> offline optimizer.

Uses an explicitly isolated DDS domain. No device, serial or control launch.
An optimizer PASS is not an independent map accuracy or navigation acceptance.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--bag', type=Path, required=True)
    ap.add_argument('--params-file', type=Path, required=True)
    ap.add_argument('--output', type=Path, required=True)
    args = ap.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') in (None, '', '0', '97'):
        raise ValueError('Require explicit non-live DDS domain')
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    info = yaml.safe_load((args.bag/'metadata.yaml').read_text())['rosbag2_bagfile_information']
    duration = info['duration']['nanoseconds']*1e-9
    if not 0 < duration <= 600:
        raise ValueError('Raw replay must be bounded to 600 seconds')
    args.output.mkdir(parents=True, exist_ok=False)
    archive = args.output/'keyframes'
    processes, files = [], []
    rclpy.init()
    node = Node('raw_mapping_pipeline_audit')
    odometry, reference, clouds, health = [], [], [], []

    def pose_row(m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        return {'stamp_ns': m.header.stamp.sec*10**9+m.header.stamp.nanosec,
                'position': [p.x, p.y, p.z], 'quaternion_xyzw': [q.x, q.y, q.z, q.w]}
    node.create_subscription(Odometry, '/lio/sensor_odometry', lambda m: odometry.append(pose_row(m)), 100)
    node.create_subscription(Odometry, '/sensor_reference/odom', lambda m: reference.append(pose_row(m)), 100)
    node.create_subscription(PointCloud2, '/lio/deskewed_odom_cloud',
                             lambda m: clouds.append(m.width*m.height), qos_profile_sensor_data)
    node.create_subscription(Bool, '/state/sensor_reference_healthy', lambda m: health.append(m.data), 20)

    def start(name, command):
        log = (args.output/(name+'.log')).open('w')
        files.append(log)
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(process)
        return process

    def stop(process):
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
                raise RuntimeError('Managed replay process required forced termination')

    errors = []
    report = {'scope': 'real raw inputs through frontend/state/archive/optimizer; no independent truth',
              'bag': str(args.bag), 'output': str(args.output), 'navigation_accepted': False}
    try:
        state = start('state', ['ros2', 'launch', 'rm_nav_bringup',
            'vehicle_164_sensor_reference_state.launch.py', 'use_sim_time:=true',
            'lio_params_file:='+str(args.params_file)])
        recorder = start('recorder', ['ros2', 'launch', 'rm_nav_bringup', 'mapping_capture.launch.py',
            'archive_root:='+str(archive), 'calibration_id:=vehicle164_yaw_axis_fit153cm_debug_v3',
            'body_frame:=nav_reference', 'state_health_topic:=/state/sensor_reference_healthy',
            'rotation_trigger_source:=sensor', 'min_keyframe_interval_s:=0.5',
            'max_keyframe_interval_s:=2.0', 'use_sim_time:=true'])
        deadline = time.monotonic()+4
        while time.monotonic() < deadline:
            if state.poll() is not None or recorder.poll() is not None:
                raise RuntimeError('State/recorder failed at startup')
            rclpy.spin_once(node, timeout_sec=.02)
        player = start('player', ['ros2', 'bag', 'play', str(args.bag), '--clock', '100',
            '--delay', '2', '--disable-keyboard-controls', '--read-ahead-queue-size', '100',
            '--topics', '/livox/lidar', '/livox/imu'])
        deadline = time.monotonic()+duration+30
        while player.poll() is None and time.monotonic() < deadline:
            if state.poll() is not None or recorder.poll() is not None:
                raise RuntimeError('State/recorder exited during replay')
            rclpy.spin_once(node, timeout_sec=.01)
        if player.poll() != 0:
            raise RuntimeError('Player failed or exceeded duration budget')
        deadline = time.monotonic()+1
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
        report['physical_command_publishers'] = {
            name: node.count_publishers(name) for name in ('/cmd_vel', '/cmd_yaw_angle', '/hardware/cmd_vel')}
        if any(report['physical_command_publishers'].values()):
            raise RuntimeError('Unexpected control publisher in raw replay domain')
        stop(recorder)
        stop(state)
        sessions = list(archive.glob('session_*'))
        if len(sessions) != 1:
            raise RuntimeError('Expected exactly one new archive session')
        report['keyframes'] = len(list(sessions[0].glob('keyframe_*')))
        report['session'] = str(sessions[0])
        if min(len(odometry), len(reference), len(clouds)) < 100 or not any(health):
            raise RuntimeError('Insufficient real state/cloud output or real-input health')
        optimizer = start('optimizer', ['ros2', 'run', 'rm_nav_mapping', 'offline_graph_optimizer',
            str(sessions[0]), str(args.output/'optimized'), '--enable-loops'])
        report['optimizer_exit'] = optimizer.wait(timeout=360)
        if report['optimizer_exit'] != 0:
            raise RuntimeError('Offline optimizer refused; inspect optimizer.log and retain archives')
    except Exception as error:
        errors.append(str(error))
    finally:
        for process in reversed(processes):
            try:
                stop(process)
            except Exception as error:
                errors.append(str(error))
        for log in files:
            log.close()
        node.destroy_node()
        rclpy.shutdown()
    report.update({'passed': not errors, 'errors': errors, 'lio_samples': len(odometry),
                   'reference_samples': len(reference), 'cloud_samples': len(clouds),
                   'real_input_health_seen': any(health),
                   'published_cloud_point_count_min_max': [min(clouds), max(clouds)] if clouds else []})
    (args.output/'odometry_trace.json').write_text(json.dumps(odometry, indent=2)+'\n')
    (args.output/'reference_trace.json').write_text(json.dumps(reference, indent=2)+'\n')
    (args.output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
