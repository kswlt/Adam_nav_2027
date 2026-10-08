#!/usr/bin/env python3
"""True KISS/GICP + real Nav2 cancellation and costmap reset with ideal feedback."""
import copy
import math
import os
from pathlib import Path
import random
import signal
import subprocess

import rclpy
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import TransformStamped, TwistStamped
from nav2_msgs.action import NavigateToPose
from rm_nav_interfaces.msg import RecoveryState, RegistrationEstimate
from rm_nav_interfaces.srv import RequestRecovery, CommitRecovery
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, String
from smoke_mppi_baseline import IdealPlant
from smoke_frozen_map_localization import cloud, require


class Plant(IdealPlant):
    def __init__(self, publish_motion_permission=True):
        super().__init__(publish_global_tf=False, publish_localization_health=False,
                         publish_motion_permission=publish_motion_permission)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.frozen = self.create_publisher(PointCloud2, '/localization/frozen_map', latched)
        self.submap = self.create_publisher(PointCloud2, '/localization/odom_submap', qos_profile_sensor_data)
        self.feedback = self.create_publisher(TwistStamped, '/hardware/measured_twist', 10)
        self.feedback_mode = 'missing'
        self.state = self.estimate = self.transform = None
        self.healthy = None
        self.reason = ''
        self.phases = []
        self.create_subscription(RecoveryState, '/localization/recovery_state', self.recovery_state, latched)
        self.create_subscription(RegistrationEstimate, '/localization/estimate', lambda m: setattr(self, 'estimate', m), 10)
        self.create_subscription(TransformStamped, '/localization/map_to_odom', lambda m: setattr(self, 'transform', m), latched)
        self.create_subscription(Bool, '/localization/healthy', lambda m: setattr(self, 'healthy', m.data), latched)
        self.create_subscription(String, '/localization/status_reason', lambda m: setattr(self, 'reason', m.data), latched)
        self.create_timer(0.02, self.feedback_tick)

    def recovery_state(self, msg):
        self.state = msg
        if not self.phases or self.phases[-1] != msg.phase:
            self.phases.append(msg.phase)

    def feedback_tick(self):
        # Explicit synthetic feedback for commissioning; never represented as hardware data.
        if self.feedback_mode == 'missing': return
        msg = TwistStamped()
        msg.header.stamp, msg.header.frame_id = self.get_clock().now().to_msg(), 'base_link'
        if self.feedback_mode == 'moving': msg.twist.angular.z = 0.2
        elif self.feedback_mode == 'stale': msg.header.stamp.sec -= 2
        elif self.feedback_mode == 'nan': msg.twist.linear.x = float('nan')
        elif self.feedback_mode == 'wrong_frame': msg.header.frame_id = 'map'
        else: msg.twist = copy.deepcopy(self.latest.twist)
        self.feedback.publish(msg)


def main():
    require(os.environ.get('ROS_DOMAIN_ID') not in (None, '', '0'), 'Use isolated nonzero Domain')
    rclpy.init()
    node = Plant()
    processes = []
    log_path = Path('log/recovery_transaction_smoke.log')
    log_path.parent.mkdir(exist_ok=True)
    report = []
    try:
        with log_path.open('w') as log:
            for command in [
                ['ros2', 'launch', 'rm_nav_bringup', 'frozen_map_localization.launch.py',
                 'map_version:=transaction-v1', 'enable_recovery:=true', 'enable_recovery_transaction:=true',
                 'recovery_timeout:=20.0', 'field_bounds:=[-12.0,12.0,-12.0,12.0]'],
                ['ros2', 'launch', 'rm_nav_bringup', 'mppi_baseline.launch.py', 'enable_task_supervisor:=false'],
            ]:
                processes.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
            rng = random.Random(42)
            points = [tuple(rng.uniform(-5, 5) for _ in range(3)) for _ in range(5000)]
            require(node.wait(lambda: node.submap.get_subscription_count() > 0, 15), 'Matcher not ready')
            node.frozen.publish(cloud(points, 'map', node.get_clock().now().to_msg()))
            node.wait(lambda: False, 0.3)
            # Real local GICP establishes the original map->odom for Nav2 startup.
            for _ in range(6):
                node.submap.publish(cloud(points, 'odom', node.get_clock().now().to_msg()))
                if node.wait(lambda: node.transform is not None and node.healthy is True, 0.4): break
            require(node.transform is not None, 'Initial real GICP failed')
            old_transform = copy.deepcopy(node.transform.transform)
            names = ['controller_server', 'planner_server', 'behavior_server', 'bt_navigator',
                     'velocity_smoother', 'collision_monitor']
            require(node.wait(lambda: all(node.active(name) for name in names), 35), 'Nav2 did not activate')
            action = ActionClient(node, NavigateToPose, '/navigate_to_pose')
            require(action.wait_for_server(timeout_sec=3), 'Navigation action missing')
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = 'map'
            goal.pose.header.stamp = node.get_clock().now().to_msg()
            goal.pose.pose.position.y = 4.0
            goal.pose.pose.orientation.w = 1.0
            report.append('real local GICP establishes prior; real Nav2 lifecycle active')

            request_client = node.create_client(RequestRecovery, '/localization/request_recovery')
            commit_client = node.create_client(CommitRecovery, '/localization/commit_recovery')
            require(request_client.wait_for_service(timeout_sec=2) and commit_client.wait_for_service(timeout_sec=2),
                    'Recovery services not ready')
            request = RequestRecovery.Request()
            request.map_version = 'transaction-v1'
            request.center.x, request.center.y = 3.0, -1.0
            request.source_origin.x, request.source_origin.y = node.x, node.y
            request.lost_time = 0.5
            future = request_client.call_async(request)
            require(node.wait(future.done, 2) and future.result().accepted, 'Recovery request refused')
            recovery_id = future.result().recovery_id
            # source = expected^{-1} * frozen map. Expected yaw=1.2 and translation=(3,-1,0.4).
            c, s = math.cos(1.2), math.sin(1.2)
            source = [(c*(x-3)+s*(y+1), -s*(x-3)+c*(y+1), z-0.4) for x, y, z in points]
            for offset in [0, 20]:
                node.submap.publish(cloud(source[offset:], 'odom', node.get_clock().now().to_msg()))
                require(node.wait(lambda: node.estimate is not None and node.estimate.recovery_id == recovery_id,
                                  3), 'Recovery estimate missing')
                node.wait(lambda: False, 1.05)
            require(node.wait(lambda: node.state is not None and node.state.phase == RecoveryState.CONFIRMED, 2),
                    'Real KISS candidate not confirmed')

            def commit(identifier=None):
                req = CommitRecovery.Request()
                req.map_version, req.recovery_id = 'transaction-v1', recovery_id if identifier is None else identifier
                result = commit_client.call_async(req)
                require(node.wait(result.done, 2), 'Commit service timed out')
                return result.result()

            require(not commit(recovery_id+1).accepted, 'Wrong session committed')
            for index, mode in enumerate(['missing', 'moving', 'stale', 'nan', 'wrong_frame']):
                node.feedback_mode = mode
                node.wait(lambda: False, 1.05)
                offset = 40 + 20 * index
                previous_fingerprint = node.estimate.source_fingerprint
                node.submap.publish(cloud(source[offset:], 'odom', node.get_clock().now().to_msg()))
                require(node.wait(lambda: node.estimate is not None and
                                  node.estimate.recovery_id == recovery_id and
                                  node.estimate.source_fingerprint != previous_fingerprint, 3),
                        'Fresh proof missing for feedback rejection: ' + mode)
                require(not commit().accepted, 'Invalid measured feedback committed: ' + mode)
                require(node.transform.transform == old_transform, 'Rejected commit changed TF')
            report.append('wrong session and missing/moving/stale/NaN/frame-invalid feedback refuse TF commit')
            node.feedback_mode = 'ideal'
            node.wait(lambda: False, 1.05)
            previous_fingerprint = node.estimate.source_fingerprint
            node.submap.publish(cloud(source[140:], 'odom', node.get_clock().now().to_msg()))
            require(node.wait(lambda: node.estimate is not None and node.estimate.recovery_id == recovery_id and
                              node.estimate.source_fingerprint != previous_fingerprint,
                              3), 'Fresh confirmation did not arrive')
            future = action.send_goal_async(goal)
            require(node.wait(future.done, 3), 'Old goal acceptance timed out')
            handle = future.result()
            require(handle.accepted, 'Old navigation goal rejected')
            old_result = handle.get_result_async()
            require(commit().accepted, 'Stopped, confirmed transaction refused')
            require(node.wait(lambda: node.state is not None and node.state.phase == RecoveryState.WAIT_REPLAN, 6),
                    'Cancel/commit/costmap transaction failed: ' + node.reason)
            require(node.wait(old_result.done, 2) and old_result.result().status == GoalStatus.STATUS_CANCELED,
                    'Old real Nav2 goal was not terminal canceled')
            tx = node.transform.transform.translation
            require(abs(tx.x-3) < 0.02 and abs(tx.y+1) < 0.02 and abs(tx.z-0.4) < 0.02,
                    'Committed map->odom has wrong direction or value')
            require(node.healthy is False and node.latest.twist.linear.x == 0 and node.latest.twist.linear.y == 0,
                    'Motion resumed before a new plan')
            report.append('real navigation goal terminal canceled; sole TF committed; both real costmaps cleared')
            node.submap.publish(cloud(source[60:], 'odom', node.get_clock().now().to_msg()))
            require(node.wait(lambda: 'fresh localization verified' in node.reason, 3), 'Post-commit local GICP not resumed')
            require(node.healthy is False, 'Fresh localization alone resumed motion')
            require(not commit().accepted, 'Committed session could be replayed')
            report.append('matcher returns to real local GICP; new planning/task permission remains required')
            print('PASS ' + '; '.join(report), flush=True)
    finally:
        for process in processes: os.killpg(process.pid, signal.SIGINT)
        for process in processes:
            try: process.wait(timeout=6)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL); process.wait()
        node.destroy_node(); rclpy.shutdown()
        Path('log/recovery_transaction_smoke_results.txt').write_text('\n'.join(report), encoding='utf-8')


if __name__ == '__main__': main()
