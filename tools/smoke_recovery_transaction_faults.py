#!/usr/bin/env python3
"""Fault fixtures for cancel rejection and clear timeout. These endpoints are test doubles."""
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from action_msgs.srv import CancelGoal
from nav2_msgs.srv import ClearEntireCostmap
from rm_nav_interfaces.msg import RecoveryState
from rm_nav_interfaces.srv import CommitRecovery, RequestRecovery
from smoke_frozen_map_localization import cloud, require
from smoke_recovery_transaction import Plant
import random
import math


def fixtures(mode):
    rclpy.init()
    node = Node('nav_reset_fault_fixture')
    group = ReentrantCallbackGroup()

    def cancel(request, response):
        response.return_code = response.ERROR_REJECTED if mode == 'cancel_rejected' else response.ERROR_NONE
        return response

    def clear(request, response):
        time.sleep(6.0)  # Deliberately exceed NavReset's 5 s transaction timeout.
        return response

    for name in ['/navigate_to_pose', '/navigate_through_poses', '/follow_path', '/spin', '/backup']:
        node.create_service(CancelGoal, name + '/_action/cancel_goal', cancel, callback_group=group)
    for name in ['/local_costmap/clear_entirely_local_costmap', '/global_costmap/clear_entirely_global_costmap']:
        node.create_service(ClearEntireCostmap, name, clear, callback_group=group)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try: executor.spin()
    except KeyboardInterrupt: pass
    finally:
        executor.shutdown(); node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


def acceptance(mode):
    require(os.environ.get('ROS_DOMAIN_ID') not in (None, '', '0'), 'Use isolated Domain')
    rclpy.init()
    node = Plant()
    node.feedback_mode = 'ideal'
    processes = []
    log_path = Path('log/transaction_fault_' + mode + '.log')
    log_path.parent.mkdir(exist_ok=True)
    try:
        with log_path.open('w') as log:
            for command in [
                [sys.executable, __file__, '--fixture', mode],
                ['ros2', 'launch', 'rm_nav_bringup', 'frozen_map_localization.launch.py',
                 'map_version:=transaction-v1', 'enable_recovery:=true', 'enable_recovery_transaction:=true',
                 'recovery_timeout:=20.0', 'field_bounds:=[-12.0,12.0,-12.0,12.0]'],
                ['ros2', 'run', 'rm_nav_control', 'motion_gate'],
            ]:
                processes.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
            request_client = node.create_client(RequestRecovery, '/localization/request_recovery')
            commit_client = node.create_client(CommitRecovery, '/localization/commit_recovery')
            require(node.wait(lambda: request_client.service_is_ready() and commit_client.service_is_ready()
                              and node.submap.get_subscription_count() > 0, 15), 'Fault acceptance nodes missing')
            rng = random.Random(42)
            source = [tuple(rng.uniform(-5, 5) for _ in range(3)) for _ in range(5000)]
            c, s = math.cos(1.2), math.sin(1.2)
            target = [(c*x-s*y+3, s*x+c*y-1, z+0.4) for x,y,z in source]
            node.frozen.publish(cloud(target, 'map', node.get_clock().now().to_msg()))
            node.wait(lambda: False, 0.3)
            request = RequestRecovery.Request()
            request.map_version = 'transaction-v1'
            request.center.x, request.center.y, request.lost_time = 3.0, -1.0, 0.5
            future = request_client.call_async(request)
            require(node.wait(future.done, 2) and future.result().accepted, 'Fault recovery refused')
            recovery_id = future.result().recovery_id
            node.submap.publish(cloud(source, 'odom', node.get_clock().now().to_msg()))
            require(node.wait(lambda: node.estimate is not None and node.estimate.recovery_id == recovery_id, 3),
                    'First true KISS result missing')
            node.wait(lambda: False, 1.05)
            node.submap.publish(cloud(source[20:], 'odom', node.get_clock().now().to_msg()))
            require(node.wait(lambda: node.state is not None and node.state.phase == RecoveryState.CONFIRMED, 3),
                    'Fault candidate not confirmed')
            commit = CommitRecovery.Request()
            commit.map_version, commit.recovery_id = 'transaction-v1', recovery_id
            if mode == 'map_changed':
                node.frozen.publish(cloud(target[:-1], 'map', node.get_clock().now().to_msg()))
                require(node.wait(lambda: node.state.phase == RecoveryState.FAULT, 2), 'Changed map not invalidated')
                future = commit_client.call_async(commit)
                require(node.wait(future.done, 2) and not future.result().accepted, 'Changed-map proof committed')
                require(node.transform is None and node.healthy is False, 'Changed map resumed motion or TF')
                print('PASS fault fixture map_changed: frozen-map mutation invalidates candidate and prevents commit', flush=True)
                return
            future = commit_client.call_async(commit)
            require(node.wait(future.done, 2) and future.result().accepted, 'Fault transaction refused to start')
            require(node.wait(lambda: node.state.phase == RecoveryState.FAULT, 7), 'Fault not reported')
            require(node.healthy is False and node.latest.twist.linear.x == 0 and node.latest.twist.angular.z == 0,
                    'Fault resumed motion')
            if mode == 'cancel_rejected':
                require(node.transform is None, 'Rejected cancellation committed TF')
            else:
                require(node.transform is not None and abs(node.transform.transform.translation.x-3) < 0.02,
                        'Clear timeout lost committed TF')
            print('PASS fault fixture ' + mode + ': unhealthy and stopped; TF commit policy preserved', flush=True)
    finally:
        for process in processes: os.killpg(process.pid, signal.SIGINT)
        for process in processes:
            try: process.wait(timeout=7)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL); process.wait()
        node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    if len(sys.argv) == 3 and sys.argv[1] == '--fixture': fixtures(sys.argv[2])
    else:
        mode = sys.argv[1] if len(sys.argv) > 1 else 'cancel_rejected'
        require(mode in ('cancel_rejected', 'clear_timeout', 'map_changed'), 'Unknown fixture')
        acceptance(mode)
