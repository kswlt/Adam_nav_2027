#!/usr/bin/env python3
"""TEST_ONLY adapter fault injection; synthetic messages never assert algorithm accuracy."""
import os
import signal
import subprocess
import tempfile
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import Bool, String, Header
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from localization_interfaces.msg import FrontendFrame, FrontendOdometry, FrontendSession


def main():
    assert 200 <= int(os.environ['ROS_DOMAIN_ID']) <= 232, 'Private valid TEST_ONLY ROS domain required'
    rclpy.init()
    node = Node('test_only_z_adapter_fixture')
    observed = {'cloud': None, 'odom': None, 'front': False, 'map': False}
    def subscribe(cls, topic, key, qos=10):
        node.create_subscription(cls, topic, lambda m: observed.update({key: m}), qos)
    subscribe(PointCloud2, '/lio/deskewed_odom_cloud', 'cloud', qos_profile_sensor_data)
    subscribe(Odometry, '/lio/sensor_odometry', 'odom', qos_profile_sensor_data)
    subscribe(Bool, '/lio/estimator_healthy', 'front')
    subscribe(Bool, '/localization/healthy', 'map')
    subscribe(String, '/localization/reason', 'reason')
    session = node.create_publisher(FrontendSession, '/frontend/session', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    frames = node.create_publisher(FrontendFrame, '/frontend/frame', 10)
    predictions = node.create_publisher(FrontendOdometry, '/frontend/odometry', 10)
    quality = node.create_publisher(Bool, '/lio/input_healthy', 10)
    states = node.create_publisher(String, '/backend_node/localization_state', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    map_odom = node.create_publisher(Odometry, '/localization/odometry', 10)
    processes = []
    with tempfile.TemporaryFile(mode='w+') as log:
        try:
            for role in ('frontend', 'backend'):
                processes.append(subprocess.Popen(['ros2', 'run', 'rm_nav_bringup', 'z_localization_bridge',
                    '--ros-args', '-p', 'role:='+role, '-p', 'mode:=localization', '-p', 'map_version:=TEST_ONLY_z'],
                    stdout=log, stderr=log, start_new_session=True))
            sequence = 0
            def pump(duration, *, failed=False, mismatch=False, backend_status='NORMAL', stale=False):
                nonlocal sequence
                end = time.monotonic()+duration
                while time.monotonic() < end:
                    assert all(p.poll() is None for p in processes), 'Adapter process exited'
                    sequence += 1
                    now = node.get_clock().now().to_msg()
                    if stale: now.sec -= 2
                    session.publish(FrontendSession(instance_id=101, epoch=3, failed=failed))
                    quality.publish(Bool(data=True))
                    odom = Odometry()
                    odom.header = Header(stamp=now, frame_id='odom'); odom.child_frame_id = 'front_mid360_imu'
                    odom.pose.pose.position.x = 2.; odom.pose.pose.orientation.w = 1.
                    odom.pose.covariance = (np.eye(6)*.01).ravel().tolist()
                    cloud = point_cloud2.create_cloud_xyz32(Header(stamp=now, frame_id='front_mid360_imu'), [[1., 0., .1]]*30)
                    if mismatch: cloud.header.stamp.sec -= 1
                    frames.publish(FrontendFrame(instance_id=101, epoch=3, sequence=sequence, odometry=odom,
                                                cloud=cloud, matches=30, lidar_updated=True))
                    predictions.publish(FrontendOdometry(instance_id=101, epoch=3, odometry=odom))
                    odom.header.frame_id = 'map'; map_odom.publish(odom)
                    ns = now.sec*10**9+now.nanosec
                    states.publish(String(data=f'status={backend_status};pose_trusted={str(backend_status=="NORMAL").lower()};tf_frozen=false;tf_published=true;epoch=3;stamp_ns={ns}'))
                    for _ in range(5): rclpy.spin_once(node, timeout_sec=.005)
                    time.sleep(.02)
            pump(3.)
            assert observed['front'].data and observed['map'].data, 'Fresh real-shaped adapter input not admitted'
            points = point_cloud2.read_points_numpy(observed['cloud'], field_names=('x','y','z'))
            assert np.allclose(points[0], [3.,0.,.1]), 'Cloud projected twice or wrong transform direction'
            assert observed['cloud'].header.frame_id == 'odom'
            pump(.5, backend_status='LOST')
            assert not observed['map'].data, 'LOST admitted'
            pump(.5, mismatch=True)
            assert not observed['front'].data, 'Mismatched atomic timestamps admitted'
            pump(.5, failed=True)
            assert not observed['front'].data and not observed['map'].data, 'Failed session admitted'
            pump(.5, stale=True)
            assert not observed['front'].data and not observed['map'].data, 'Expired ROS time admitted'
            pump(.5)
            end = time.monotonic()+.6
            while time.monotonic() < end: rclpy.spin_once(node, timeout_sec=.02)
            assert not observed['front'].data and not observed['map'].data, 'Stopped input heartbeat stayed healthy'
            assert observed['reason'].data.startswith('status=UNHEALTHY;'), 'Expired backend still advertised NORMAL'
            assert 'stale' in observed['reason'].data, 'Missing stale dependency diagnostics'
            print('TEST_ONLY Z adapter: transform, NORMAL, LOST, atomic mismatch, failed session, stale source, dropout PASS')
        except BaseException:
            log.seek(0); print(log.read()[-12000:]); raise
        finally:
            for process in processes:
                if process.poll() is None: os.killpg(process.pid, signal.SIGINT)
            for process in processes:
                try: process.wait(timeout=8.)
                except subprocess.TimeoutExpired: os.killpg(process.pid, signal.SIGKILL)
            node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__': main()
