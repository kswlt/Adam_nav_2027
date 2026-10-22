#!/usr/bin/env python3
"""Real raw-MCAP ZLIO/graph smoke in a private domain; no claim of pose accuracy."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String, Bool
from tf2_msgs.msg import TFMessage


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--bag', required=True, help='Derived raw-only bag with original bytes/times')
    parser.add_argument('--output', required=True)
    parser.add_argument('--rate', type=float, default=.5)
    parser.add_argument('--prior-map', help='Replay localization against an actual previously exported native Z map')
    parser.add_argument('--max-wall-seconds', type=float, default=240.)
    args = parser.parse_args()
    assert 200 <= int(os.environ['ROS_DOMAIN_ID']) <= 232, 'Private valid replay domain required'
    output = Path(args.output).absolute(); output.mkdir(parents=True, exist_ok=False)
    rclpy.init()
    node = Node('test_only_z_real_bag_observer', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
    counts = dict(odom=0, cloud=0, optimized_map=0, reference=0, backend_pose=0)
    positions = []; reference_positions = []; states = []; tf_parents = {}; health = []; reasons = {}
    resources = {}; peak_total_rss_kib = 0
    last_raw_stamp = last_lio_stamp = 0
    def odom(message):
        nonlocal last_lio_stamp
        counts['odom'] += 1
        last_lio_stamp = message.header.stamp.sec*10**9+message.header.stamp.nanosec
        p = message.pose.pose.position; positions.append([p.x, p.y, p.z])
    node.create_subscription(Odometry, '/lio/sensor_odometry', odom, qos_profile_sensor_data)
    def raw_cloud(message):
        nonlocal last_raw_stamp
        last_raw_stamp = message.header.stamp.sec*10**9+message.header.stamp.nanosec
    node.create_subscription(PointCloud2, '/livox/lidar', raw_cloud, qos_profile_sensor_data)
    def count(key): return lambda _: counts.update({key: counts[key]+1})
    node.create_subscription(PointCloud2, '/lio/deskewed_odom_cloud', count('cloud'), qos_profile_sensor_data)
    map_topic = '/backend_node/prior_map' if args.prior_map else '/backend_node/runtime_map'
    node.create_subscription(PointCloud2, map_topic, count('optimized_map'), QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    def reference(message):
        counts['reference'] += 1
        p = message.pose.pose.position; reference_positions.append([p.x,p.y,p.z])
    node.create_subscription(Odometry, '/sensor_reference/odom', reference, qos_profile_sensor_data)
    node.create_subscription(Odometry, '/localization/odometry', count('backend_pose'), qos_profile_sensor_data)
    node.create_subscription(Bool, '/localization/healthy', lambda m: health.append(m.data), 10)
    node.create_subscription(String, '/backend_node/localization_state', lambda m: states.append(m.data), QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    for topic in ('/lio/input_reason','/lio/estimator_reason','/state/sensor_reference_reason','/sensors/front_mid360/cloud_reason'):
        node.create_subscription(String, topic, lambda m,t=topic: reasons.update({t:m.data}), 10)
    def transforms(message):
        for t in message.transforms: tf_parents.setdefault(t.child_frame_id, set()).add(t.header.frame_id)
    node.create_subscription(TFMessage, '/tf', transforms, 100)
    processes = []; passed = False; error = ''
    def sample_resources():
        nonlocal peak_total_rss_kib
        pending = [p.pid for p in processes if p.poll() is None]
        seen = set(); total = 0
        while pending:
            pid = pending.pop()
            if pid in seen: continue
            seen.add(pid)
            try:
                root = Path('/proc')/str(pid)
                pending.extend(map(int,(root/f'task/{pid}/children').read_text().split()))
                status = dict(line.split(':',1) for line in (root/'status').read_text().splitlines() if ':' in line)
                rss = int(status.get('VmRSS','0 kB').split()[0]); total += rss
                hwm = int(status.get('VmHWM','0 kB').split()[0])
                record = resources.setdefault(str(pid), {'command': (root/'cmdline').read_bytes().replace(b'\0',b' ').decode(errors='replace')})
                record['peak_rss_kib'] = max(record.get('peak_rss_kib',0), hwm)
            except (OSError, ValueError): pass
        peak_total_rss_kib = max(peak_total_rss_kib, total)
    try:
        with (output/'launch.log').open('w') as launch_log, (output/'play.log').open('w') as play_log:
            command = ['ros2','launch','rm_nav_bringup','z_localization_mapping.launch.py',
                       'use_sim_time:=true','save_directory:='+str(output/'native_map')]
            if args.prior_map:
                command = ['ros2','launch','rm_nav_bringup','vehicle_164_sensor_reference_state.launch.py','use_sim_time:=true']
                processes.append(subprocess.Popen(['ros2','launch','rm_nav_bringup','z_localization_backend.launch.py',
                    'use_sim_time:=true','mode:=localization','prior_map:='+str(Path(args.prior_map).resolve(strict=True)),
                    'map_version:='+Path(args.prior_map).resolve().name], stdout=launch_log, stderr=launch_log, start_new_session=True))
            mapping = subprocess.Popen(command, stdout=launch_log, stderr=launch_log, start_new_session=True)
            processes.append(mapping)
            end = time.monotonic()+5.
            while time.monotonic()<end: rclpy.spin_once(node, timeout_sec=.05)
            play = subprocess.Popen(['ros2','bag','play',args.bag,'--clock','100','--rate',str(args.rate),
                                     '--delay','2','--read-ahead-queue-size','20000',
                                     '--topics','/livox/lidar','/livox/imu'], stdout=play_log, stderr=play_log, start_new_session=True)
            processes.append(play)
            deadline = time.monotonic()+args.max_wall_seconds
            sample_at = 0.
            while play.poll() is None and time.monotonic()<deadline:
                if mapping.poll() is not None: raise RuntimeError('Mapping launch exited')
                rclpy.spin_once(node, timeout_sec=.01)
                if time.monotonic()-sample_at >= 1.:
                    sample_resources(); sample_at = time.monotonic()
            if play.poll() != 0: raise RuntimeError('Playback failed/timed out')
            from z_save_map import call
            call(node, '/stop')
            if not args.prior_map: call(node, '/backend_node/save_map')
            if min(counts.values()) < 1 or counts['odom'] < 50: raise RuntimeError('Incomplete real data pipeline: '+str(counts))
            if not args.prior_map and any(health): raise RuntimeError('Mapping incorrectly enabled navigation health')
            if args.prior_map and not any(health): raise RuntimeError('Prior localization never admitted healthy output')
            if tf_parents.get('odom') != {'map'}: raise RuntimeError('Missing/conflicting map->odom')
            if not any('status=NORMAL;' in state for state in states): raise RuntimeError('Backend never NORMAL')
            if not states or not states[-1].startswith('status=NORMAL;'):
                raise RuntimeError('Backend did not finish NORMAL; partial counts cannot pass a complete replay')
            if last_raw_stamp-last_lio_stamp > 300000000:
                raise RuntimeError('LIO stopped before the end of raw input')
            if not args.prior_map and not (output/'native_map/keyframes.txt').is_file(): raise RuntimeError('No native saved map')
            passed = True
    except Exception as exc:
        error = str(exc)
    finally:
        for process in reversed(processes):
            if process.poll() is None: os.killpg(process.pid, signal.SIGINT)
        for process in processes:
            try: process.wait(timeout=10.)
            except subprocess.TimeoutExpired: os.killpg(process.pid, signal.SIGKILL)
        p = np.asarray(positions)
        ref = np.asarray(reference_positions)
        report = {'pipeline_pass': passed, 'error': error, 'counts': counts, 'bag': args.bag,
                  'rate': args.rate, 'backend_states_tail': states[-10:], 'reasons': reasons,
                  'prior_map': args.prior_map, 'map_topic': map_topic,
                  'last_raw_stamp_ns': last_raw_stamp, 'last_lio_stamp_ns': last_lio_stamp,
                  'tf_parent_sets': {k:sorted(v) for k,v in tf_parents.items()},
                  'max_estimated_displacement_m': float(np.linalg.norm(p-p[0],axis=1).max()) if len(p) else None,
                  'sensor_z_span_m': float(np.ptp(p[:,2])) if len(p) else None,
                  'max_reference_xy_displacement_m': float(np.linalg.norm(ref[:,:2]-ref[0,:2],axis=1).max()) if len(ref) else None,
                  'peak_sampled_process_tree_rss_kib': peak_total_rss_kib, 'process_resources': resources,
                  'accuracy': 'NOT VERIFIED: no ground truth, do not interpret as drift acceptance',
                  'scope': 'raw-only MCAP ZLIO/axis reference/online backend/map export; no serial/hardware motion'}
        (output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
        print(json.dumps(report, indent=2))
        node.destroy_node(); rclpy.shutdown()
    if not passed: raise SystemExit(1)


if __name__ == '__main__': main()
