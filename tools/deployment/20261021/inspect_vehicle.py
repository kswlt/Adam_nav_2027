#!/usr/bin/env python3
"""Read-only real sensor-reference commissioning evidence. No commands or TF sent."""
import argparse
from collections import defaultdict
import json
import math
from pathlib import Path
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, String
from tf2_msgs.msg import TFMessage


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--seconds', type=float, default=15.)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if not 5 <= args.seconds <= 60:
        raise ValueError('Probe duration must be 5..60 seconds')
    rclpy.init()
    node = Node('sensor_reference_vehicle_probe')
    counts, ages, parents = defaultdict(int), defaultdict(list), defaultdict(set)
    authorities = defaultdict(set)
    latest, health_at, positions, points = {}, {}, [], {}
    maximum_candidate_speed = 0.
    topics = [('/livox/lidar', PointCloud2), ('/lio/sensor_odometry', Odometry),
              ('/lio/deskewed_odom_cloud', PointCloud2), ('/sensor_reference/odom', Odometry),
              ('/localization/odom_submap', PointCloud2), ('/visualization/live_map_preview', PointCloud2)]
    def message(topic, m):
        counts[topic] += 1
        ages[topic].append((node.get_clock().now().nanoseconds-
                           (m.header.stamp.sec*10**9+m.header.stamp.nanosec))*1e-9)
        if isinstance(m, PointCloud2):
            points[topic] = m.width*m.height
        if topic == '/sensor_reference/odom':
            p = m.pose.pose.position
            positions.append((p.x,p.y,p.z))
    for topic, typ in topics:
        node.create_subscription(typ, topic, lambda m, t=topic: message(t,m), qos_profile_sensor_data)
    def health(topic, m):
        latest[topic], health_at[topic] = m.data, time.monotonic()
    for topic in ('/sensors/front_mid360/cloud_healthy', '/state/sensor_reference_healthy', '/localization/submap_healthy'):
        node.create_subscription(Bool, topic, lambda m, t=topic: health(t,m), 10)
    for topic in ('/state/sensor_reference_reason', '/sensors/front_mid360/cloud_reason', '/localization/submap_reason', '/mapping/online_status', '/mapping/recorder_status'):
        node.create_subscription(String, topic, lambda m, t=topic: latest.update({t:m.data}), 10)
    def transforms(message, info):
        for tf in message.transforms:
            parents[tf.child_frame_id].add(tf.header.frame_id)
            gid = info.get('publisher_gid') if isinstance(info, dict) else getattr(info, 'publisher_gid', None)
            if gid is not None:
                authorities[tf.child_frame_id].add(bytes(gid))
    node.create_subscription(TFMessage, '/tf', transforms, 50)
    node.create_subscription(TFMessage, '/tf_static', transforms, QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    def candidate(m):
        nonlocal maximum_candidate_speed
        maximum_candidate_speed = max(maximum_candidate_speed, math.hypot(m.linear.x,m.linear.y))
    node.create_subscription(Twist, '/hardware/legacy_adam/cmd_vel', candidate, 10)
    start = time.monotonic()
    while time.monotonic()-start < args.seconds:
        rclpy.spin_once(node, timeout_sec=.02)
    healthy = all(latest.get(t) is True and time.monotonic()-health_at.get(t,0) < .5 for t in (
        '/sensors/front_mid360/cloud_healthy', '/state/sensor_reference_healthy', '/localization/submap_healthy'))
    correct_tf = all(parents[key] == {parent} for key, parent in {
        'front_mid360_imu':'odom', 'front_mid360':'front_mid360_imu',
        'nav_reference':'odom', 'sensor_yaw_reference':'odom'}.items())
    correct_tf = correct_tf and all(len(v) == 1 for v in authorities.values())
    authority_names = {bytes(e.endpoint_gid): e.node_name for topic in ('/tf', '/tf_static') for e in node.get_publishers_info_by_topic(topic)}
    endpoints = {t:[e.node_name for e in node.get_publishers_info_by_topic(t)] for t,_ in topics}
    no_physical = node.count_publishers('/cmd_vel') == 0 and node.count_publishers('/cmd_yaw_angle') == 0
    report = {'scope':'REAL_SENSOR_READ_ONLY_NO_MOTION',
        'passed':bool(healthy and correct_tf and no_physical and maximum_candidate_speed == 0. and all(counts[t] >= 5 for t,_ in topics)),
        'counts':dict(counts), 'health_and_reasons':latest, 'points':points, 'publishers':endpoints,
        'tf_parents':{k:sorted(v) for k,v in parents.items()},
        'actual_tf_authorities':{child:[authority_names.get(gid,gid.hex()) for gid in ids] for child,ids in authorities.items()},
        'publisher_gid_available_in_python':bool(authorities),
        'tf_publishers':{t:[e.node_name for e in node.get_publishers_info_by_topic(t)] for t in ('/tf','/tf_static')}, 'maximum_candidate_speed':maximum_candidate_speed,
        'physical_publishers':{'cmd_vel':node.count_publishers('/cmd_vel'),'cmd_yaw_angle':node.count_publishers('/cmd_yaw_angle')},
        'source_age_s':{t:{'p50':float(np.median(v)),'p95':float(np.quantile(v,.95)),'max':max(v)} for t,v in ages.items()},
        'observed_position_span_m':float(np.max(np.linalg.norm(np.asarray(positions)-positions[0],axis=1))) if positions else None,
        'stationary_drift_or_ground_truth_verified':False}
    args.output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
    node.destroy_node()
    rclpy.shutdown()
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
