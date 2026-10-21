#!/usr/bin/env python3
"""TEST_ONLY ideal sensor input in a private domain; never opens serial devices."""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from tf2_msgs.msg import TFMessage


def main():
    os.environ.setdefault('ROS_DOMAIN_ID', '201')
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    source = Path(__file__).parents[1]/'src/rm_nav_bringup/scripts/sensor_reference.py'
    axis_fixture = '--axis-fixture' in sys.argv
    params = ['--ros-args','-p','reference_point:=gimbal_axis',
              '-p','gimbal_axis_to_lidar:=[0.03,0.18,0.08]',
              '-p','reference_geometry_verified:=true','-p','calibration_id:=TEST_ONLY_axis'] if axis_fixture else []
    with tempfile.TemporaryFile(mode='w+') as log:
        process = subprocess.Popen([sys.executable, str(source), *params], stdout=log, stderr=log, start_new_session=True)
        rclpy.init()
        node = Node('test_only_sensor_reference_fixture')
        lio = node.create_publisher(Odometry, '/lio/sensor_odometry', 10)
        raw = node.create_publisher(Bool, '/sensors/front_mid360/cloud_healthy', 10)
        permission = node.create_publisher(Bool, '/nav/motion_allowed', 10)
        command = node.create_publisher(TwistStamped, '/nav/cmd_vel_safe', 1)
        observed = {'health': False, 'shadow': None, 'odom': None, 'tf': {}}
        node.create_subscription(Bool, '/state/sensor_reference_healthy', lambda m: observed.update(health=m.data), 10)
        node.create_subscription(String, '/state/sensor_reference_reason', lambda m: observed.update(reason=m.data), 10)
        node.create_subscription(TwistStamped, '/nav/sensor_cmd_vel_shadow', lambda m: observed.update(shadow=m), 10)
        node.create_subscription(Odometry, '/sensor_reference/odom', lambda m: observed.update(odom=m), 10)
        def tf_receive(message):
            for t in message.transforms:
                observed['tf'][t.child_frame_id] = t
        node.create_subscription(TFMessage, '/tf', tf_receive, 10)
        fixture_yaw, fixture_at = None, time.monotonic()
        last_command_stamp = 0
        def run(seconds, *, pose=True, quality=True, allow=True, send=False, yaw=math.pi/2, angular=0., bad_cov=False, old=False, position_x=0., command_stamp=None, drain=10):
            nonlocal fixture_yaw, fixture_at, last_command_stamp
            end = time.monotonic()+seconds
            while time.monotonic() < end:
                assert process.poll() is None, 'sensor process exited'
                raw.publish(Bool(data=quality))
                permission.publish(Bool(data=allow))
                if pose:
                    now = time.monotonic()
                    if fixture_yaw is None:
                        fixture_yaw = yaw
                    delta = math.atan2(math.sin(yaw-fixture_yaw),math.cos(yaw-fixture_yaw))
                    step = 8.*(now-fixture_at)
                    fixture_yaw += max(-step,min(step,delta))
                    fixture_at = now
                    m = Odometry()
                    m.header.frame_id, m.child_frame_id = 'odom', 'front_mid360_imu'
                    m.header.stamp = node.get_clock().now().to_msg()
                    m.pose.pose.position.x = position_x
                    if axis_fixture:
                        # Known TEST_ONLY geometry: fixed axis despite rotating sensor.
                        dx,dy,dz = (-.041,-.20329,-.03588)
                        c,s = math.cos(fixture_yaw),math.sin(fixture_yaw)
                        m.pose.pose.position.x -= c*dx-s*dy
                        m.pose.pose.position.y = -s*dx-c*dy
                        m.pose.pose.position.z = .2-dz
                    if old:
                        m.header.stamp.sec -= 2
                    m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(fixture_yaw/2), math.cos(fixture_yaw/2)
                    for i in range(6):
                        m.pose.covariance[i*7] = .01
                    if bad_cov:
                        m.pose.covariance[0] = -1.
                    lio.publish(m)
                if send:
                    m = TwistStamped()
                    m.header.frame_id, m.header.stamp = 'nav_reference', node.get_clock().now().to_msg()
                    if command_stamp is not None:
                        m.header.stamp.sec, m.header.stamp.nanosec = divmod(command_stamp,10**9)
                    last_command_stamp = m.header.stamp.sec*10**9+m.header.stamp.nanosec
                    m.twist.linear.x, m.twist.angular.z = .2, angular
                    command.publish(m)
                rclpy.spin_once(node, timeout_sec=.01)
                for _ in range(12):
                    rclpy.spin_once(node, timeout_sec=0.)
                time.sleep(.01)
            for _ in range(drain):
                rclpy.spin_once(node, timeout_sec=.01)
        def stopped():
            m = observed['shadow']
            return m is not None and math.hypot(m.twist.linear.x, m.twist.linear.y) < 1e-9 and m.twist.angular.z == 0
        try:
            run(2.)
            assert observed['health'], 'real-input fixture not healthy'
            assert observed['odom'].child_frame_id == 'nav_reference'
            assert set(observed['tf']) == {'front_mid360_imu', 'nav_reference', 'sensor_yaw_reference'}, observed['tf'].keys()
            assert observed['tf']['nav_reference'].transform.rotation.w == 1.
            run(.5, send=True)
            m = observed['shadow']
            assert abs(m.twist.linear.x) < 1e-6 and abs(m.twist.linear.y+.2) < 1e-6
            assert m.header.frame_id == 'sensor_yaw_reference' and m.twist.angular.z == 0
            replay_stamp = last_command_stamp-1
            run(.06, send=True, command_stamp=replay_stamp, drain=0)
            assert stopped(), 'out-of-order fresh command overrode newer command'
            run(.3, send=True)
            run(.45, pose=False, send=True)
            assert not observed['health'] and stopped(), 'LIO loss did not stop translation'
            run(.3)
            assert observed['health'] and stopped(), 'old command replayed after LIO recovery'
            # Allow the physically bounded turn and two output periods to settle.
            run(.6, send=True, yaw=0.)
            assert observed['shadow'].twist.linear.x > .19, str(observed)
            if axis_fixture:
                assert abs(observed['odom'].pose.pose.position.x)<1e-6
                assert abs(observed['odom'].pose.pose.position.y)<1e-6
            run(.35, send=True, quality=False)
            assert not observed['health'] and stopped(), 'quality failure not interlocked'
            run(.3)
            assert stopped()
            run(.3, send=True, angular=.2)
            assert stopped(), 'unexpected yaw command was accepted'
            run(.35, bad_cov=True)
            assert not observed['health'] and stopped(), 'invalid covariance accepted'
            run(.35, old=True)
            assert not observed['health'] and stopped(), 'stale source accepted'
            run(.3, send=True, allow=False)
            assert stopped(), 'revoked command accepted'
            run(.3)
            assert stopped(), 'permission restoration replayed an old command'
            assert node.count_publishers('/cmd_vel') == 0
            assert node.count_publishers('/cmd_yaw_angle') == 0
            jump_started = node.get_clock().now().nanoseconds
            run(.3, position_x=-4626.836, send=True)
            assert not observed['health'] and stopped(), 'kilometre LIO jump not stopped'
            stamp = observed['odom'].header.stamp
            assert stamp.sec*10**9+stamp.nanosec < jump_started, 'bad pose entered navigation odometry'
            run(.3, send=True)
            assert not observed['health'] and stopped(), 'continuity fault recovered without explicit restart'
            print(json.dumps({'result': 'PASS', 'mode': 'TEST_ONLY', 'yaw_rotation': 'PASS',
                'lio_timeout': 'PASS', 'raw_quality_failure': 'PASS', 'no_replay': 'PASS',
                'bad_covariance': 'PASS', 'stale_source': 'PASS', 'angular_rejection': 'PASS',
                'physical_publishers': 0, 'chassis_tf_publishers': 0, 'jump_latched_no_odom':True,
                'out_of_order_command': 'PASS', 'measured_axis_TEST_ONLY':axis_fixture}))
        except Exception:
            log.seek(0)
            print(log.read(), file=sys.stderr)
            raise
        finally:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
