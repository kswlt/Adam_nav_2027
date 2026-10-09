#!/usr/bin/env python3
"""Real bounded KISS/GICP ROS scheduling; candidate confirmation remains fail closed."""
import copy
import math
import os
from pathlib import Path
import random
import signal
import subprocess
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from rm_nav_interfaces.srv import RequestRecovery
from std_msgs.msg import Bool
from smoke_frozen_map_localization import Acceptance, cloud, require


def main():
    require(os.environ.get('ROS_DOMAIN_ID') not in (None, '', '0'), 'Use an isolated nonzero Domain')
    rclpy.init()
    node = Acceptance()
    node.confirmed = None
    node.safe = None
    node.create_subscription(Bool, '/localization/recovery_confirmed',
                             lambda m: setattr(node, 'confirmed', m.data), 10)
    node.create_subscription(TwistStamped, '/nav/cmd_vel_safe', lambda m: setattr(node, 'safe', m), 10)
    enable = node.create_publisher(Bool, '/nav/motion_enable', 10)
    chassis = node.create_publisher(Bool,'/state/chassis_healthy',10)
    raw = node.create_publisher(TwistStamped, '/nav/cmd_vel_checked', 10)
    client = node.create_client(RequestRecovery, '/localization/request_recovery')

    def ongoing_command():
        chassis.publish(Bool(data=True))  # Explicit commissioning health fixture.
        enable.publish(Bool(data=True))
        cmd = TwistStamped()
        cmd.header.frame_id = 'base_link'
        cmd.header.stamp = node.get_clock().now().to_msg()
        cmd.twist.linear.x = 0.3
        raw.publish(cmd)

    timer = node.create_timer(0.05, ongoing_command)
    processes = []
    log_path = Path('log/kiss_recovery_smoke.log')
    log_path.parent.mkdir(exist_ok=True)
    report = []
    try:
        with log_path.open('w') as log:
            commands = [
                ['ros2', 'launch', 'rm_nav_bringup', 'frozen_map_localization.launch.py',
                 'map_version:=kiss-test-v1', 'enable_recovery:=true',
                 'field_bounds:=[-12.0,12.0,-12.0,12.0]'],
                ['ros2', 'run', 'rm_nav_control', 'motion_gate'],
            ]
            for command in commands:
                processes.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                                  start_new_session=True))
            require(node.wait(lambda: client.service_is_ready() and node.healthy is False
                              and node.submap.get_subscription_count() > 0, 15), 'Recovery nodes not ready')
            rng = random.Random(42)
            points = [tuple(rng.uniform(-5, 5) for _ in range(3)) for _ in range(5000)]
            c, s = math.cos(1.2), math.sin(1.2)
            cp, sp = math.cos(0.3), math.sin(0.3)
            mapped = [(c*(cp*x+sp*z)-s*y+3, s*(cp*x+sp*z)+c*y-1, -sp*x+cp*z+0.4)
                      for x, y, z in points]
            mapped += [(30 + i * 0.1, 30, 0) for i in range(100)]
            node.map.publish(cloud(mapped, 'map', node.get_clock().now().to_msg()))
            node.wait(lambda: False, 0.3)

            def request(region_x=3.0, lost_time=0.5, version='kiss-test-v1'):
                req = RequestRecovery.Request()
                req.map_version = version
                req.center.x, req.center.y = float(region_x), -1.0
                req.lost_time = float(lost_time)
                future = client.call_async(req)
                require(node.wait(future.done, 2), 'Recovery request timed out')
                return future.result()

            require(not request(version='wrong').accepted, 'Wrong map request accepted')
            require(not request(region_x=float('nan')).accepted, 'NaN region accepted')
            require(not request(region_x=30).accepted, 'Region outside field accepted')
            require(not request(lost_time=100).accepted, 'Unbounded loss radius accepted')
            response = request()
            require(response.accepted and response.recovery_id != 0, 'Bounded recovery refused')
            report.append('map/finite/field/motion bounds validated by sole TF authority')
            node.submap.publish(cloud(points, 'odom', node.get_clock().now().to_msg()))
            require(node.wait(lambda: node.estimate is not None and
                              node.estimate.recovery_id == response.recovery_id, 3), 'No real KISS result')
            first = copy.deepcopy(node.estimate)
            require(first.method == first.KISS_GICP and first.converged and first.residual < 0.03,
                    'KISS/GICP quality failed')
            tx = first.target_to_source.translation
            error = math.sqrt((tx.x-3)**2 + (tx.y+1)**2 + (tx.z-0.4)**2)
            require(error < 0.02 and first.target_point_count < len(mapped), 'Transform or crop incorrect')
            require(node.wait(lambda: node.pending is True and node.confirmed is False, 1),
                    'Single candidate accepted without confirmation')
            report.append('real bounded KISS/GICP, far map points cropped, single result remains candidate')

            # A fresh timestamp on the SAME data is not a new independent submap.
            node.wait(lambda: False, 1.1)
            count = node.estimate_count
            node.submap.publish(cloud(points, 'odom', node.get_clock().now().to_msg()))
            require(node.wait(lambda: node.estimate_count > count, 2), 'Reused cloud not processed')
            require(node.wait(lambda: 'reused source' in node.reason, 1) and node.confirmed is False,
                    'Reused source independently confirmed')
            report.append('same cloud with a new timestamp cannot confirm recovery')
            # New observation has different points and a distinct measurement time.
            node.wait(lambda: False, 1.1)
            node.submap.publish(cloud(points[20:], 'odom', node.get_clock().now().to_msg()))
            require(node.wait(lambda: node.confirmed is True, 3), 'Independent result did not confirm')
            require(node.healthy is False and node.tf is None and node.transform is None,
                    'Confirmed candidate committed TF or resumed health before transaction')
            require(node.safe is not None and node.safe.twist.linear.x == 0 and
                    node.safe.twist.linear.y == 0 and node.safe.twist.angular.z == 0,
                    'Motion was allowed during recovery')
            report.append('two consistent fresh clouds confirm candidate; TF withheld and moving command gated')
            # Ordinary estimates cannot restore health while the session is active.
            ordinary = copy.deepcopy(first)
            ordinary.recovery_id = 0
            ordinary.method = ordinary.LOCAL_GICP
            node.send_estimate(ordinary)
            require(node.healthy is False and node.tf is None, 'Ordinary estimate escaped recovery hold')
            report.append('ordinary estimate cannot restore motion during recovery')
            # New bounded session invalidates the previous proof and its nonce.
            second = request()
            require(second.accepted and second.recovery_id != response.recovery_id, 'Session id reused')
            require(node.wait(lambda: node.confirmed is False, 1), 'Old candidate proof retained')
            node.send_estimate(first)
            require(node.confirmed is False and node.healthy is False, 'Old session result accepted')
            report.append('new session clears proof; old session results rejected')
            require(node.wait(lambda: 'timed out' in node.reason, 11), 'Timeout did not retain safe stop')
            require(node.healthy is False and node.confirmed is False and node.tf is None,
                    'Timeout resumed motion or committed candidate')
            report.append('expired session holds safe stop')
            print(f'PASS true KISS/GICP ROS position_error={error:.8f}m rmse={first.residual:.6f}m', flush=True)
            print('PASS ' + '; '.join(report), flush=True)
    finally:
        node.destroy_timer(timer)
        for process in processes:
            os.killpg(process.pid, signal.SIGINT)
        for process in processes:
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        node.destroy_node()
        rclpy.shutdown()
        Path('log/kiss_recovery_smoke_results.txt').write_text('\n'.join(report), encoding='utf-8')


if __name__ == '__main__':
    main()
