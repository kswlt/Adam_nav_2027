#!/usr/bin/env python3
"""Real Nav2 preflight planning, owned task permission and post-recovery resume."""
import copy
import math
import os
from pathlib import Path
import random
import signal
import subprocess

import rclpy
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rm_nav_interfaces.msg import RecoveryState
from rm_nav_interfaces.msg import RegistrationEstimate
from rm_nav_interfaces.srv import SubmitGoal, RequestRecovery, CommitRecovery, ResumeRecovery
from std_msgs.msg import Bool, String
from smoke_recovery_transaction import Plant
from smoke_frozen_map_localization import cloud, require


def main():
    require(os.environ.get('ROS_DOMAIN_ID') not in (None, '', '0'), 'Use isolated Domain')
    rclpy.init()
    node = Plant(publish_motion_permission=False)
    node.feedback_mode = 'ideal'
    node.task_status, node.permission = '', None
    diagnostic_history = []
    def diagnostic(msg):
        if not diagnostic_history or diagnostic_history[-1] != msg.data:
            diagnostic_history.append(msg.data)
    node.create_subscription(String, '/localization/status_reason', diagnostic, 10)
    estimates = []
    def estimate_trace(msg):
        estimates.append(f'{msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}: '
                         f'method={msg.method} converged={msg.converged} iterations={msg.iterations} '
                         f'residual={msg.residual} delta={msg.translation_delta}/{msg.rotation_delta} '
                         f'points={msg.source_point_count}/{msg.target_point_count}')
    node.create_subscription(RegistrationEstimate, '/localization/estimate', estimate_trace, 10)
    node.create_subscription(String, '/nav/task_status', lambda m: setattr(node, 'task_status', m.data), 10)
    node.create_subscription(Bool, '/nav/motion_enable', lambda m: setattr(node, 'permission', m.data), 10)
    rng = random.Random(42)
    points = [tuple(rng.uniform(-5,5) for _ in range(3)) for _ in range(5000)]
    feed = {'points': points, 'enabled': False}

    def observation():
        if feed['enabled']:
            node.submap.publish(cloud(feed['points'], 'odom', node.get_clock().now().to_msg()))

    timer = node.create_timer(0.15, observation)
    processes = []
    log_path = Path('log/task_resume_smoke.log')
    log_path.parent.mkdir(exist_ok=True)
    report = []
    try:
        with log_path.open('w') as log:
            for command in [
                ['ros2','launch','rm_nav_bringup','frozen_map_localization.launch.py',
                 'map_version:=transaction-v1','enable_recovery:=true','enable_recovery_transaction:=true',
                 'recovery_timeout:=20.0','field_bounds:=[-12.0,12.0,-12.0,12.0]'],
                ['ros2','launch','rm_nav_bringup','mppi_baseline.launch.py'],
            ]:
                processes.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
            require(node.wait(lambda: node.submap.get_subscription_count() > 0, 15), 'Matcher missing')
            node.frozen.publish(cloud(points,'map',node.get_clock().now().to_msg()))
            feed['enabled'] = True
            require(node.wait(lambda: node.healthy is True and node.transform is not None, 5), 'Initial GICP failed')
            names = ['controller_server','planner_server','behavior_server','bt_navigator','velocity_smoother','collision_monitor']
            require(node.wait(lambda: all(node.active(name) for name in names), 35), 'Nav2 inactive')
            require(node.wait(lambda: node.permission is False and ':IDLE:' in node.task_status, 3),
                    'Supervisor permits motion without an owned task')
            # A directly submitted external Nav2 goal cannot claim the supervisor's permission.
            action = ActionClient(node, NavigateToPose, '/navigate_to_pose')
            require(action.wait_for_server(timeout_sec=3), 'NavigateToPose missing')
            external = NavigateToPose.Goal()
            external.pose.header.frame_id, external.pose.pose.position.y = 'map', 4.0
            external.pose.pose.orientation.w = 1.0
            future = action.send_goal_async(external)
            require(node.wait(future.done, 3) and future.result().accepted, 'External fixture goal rejected')
            node.wait(lambda: False, 0.5)
            require(node.permission is False and math.hypot(node.x,node.y) < 0.03,
                    'External unowned goal moved ideal robot')
            report.append('healthy localization and external Nav2 goal do not grant owned-task permission')

            clients = {kind: node.create_client(service,name) for kind,service,name in [
                ('request',RequestRecovery,'/localization/request_recovery'),
                ('commit',CommitRecovery,'/localization/commit_recovery'),
                ('resume',ResumeRecovery,'/localization/resume_recovery'),
                ('submit',SubmitGoal,'/nav/submit_goal'),
            ]}
            def call(kind, request):
                result = clients[kind].call_async(request)
                require(node.wait(result.done, 3), kind + ' timed out')
                return result.result()

            feed['enabled'] = False
            request = RequestRecovery.Request()
            request.map_version, request.center.x, request.center.y, request.lost_time = 'transaction-v1',3.0,-1.0,0.5
            response = call('request',request)
            require(response.accepted, 'Recovery refused')
            recovery_id = response.recovery_id
            c,s = math.cos(1.2),math.sin(1.2)
            source = [(c*(x-3)+s*(y+1),-s*(x-3)+c*(y+1),z-0.4) for x,y,z in points]
            node.submap.publish(cloud(source,'odom',node.get_clock().now().to_msg()))
            require(node.wait(lambda: node.estimate is not None and node.estimate.recovery_id == recovery_id,3), 'KISS missing')
            node.wait(lambda: False,1.05)
            node.submap.publish(cloud(source[20:],'odom',node.get_clock().now().to_msg()))
            require(node.wait(lambda: node.state is not None and node.state.phase == RecoveryState.CONFIRMED,3), 'No confirmation')
            commit = CommitRecovery.Request()
            commit.map_version,commit.recovery_id = 'transaction-v1',recovery_id
            require(call('commit',commit).accepted,'Commit refused')
            require(node.wait(lambda: node.state.phase == RecoveryState.WAIT_REPLAN,6),'Commit did not finish')
            old_transform = copy.deepcopy(node.transform.transform)
            forged = ResumeRecovery.Request()
            forged.map_version, forged.recovery_id = 'transaction-v1',recovery_id
            require(not call('resume',forged).accepted, 'No-plan/no-goal resume accepted')
            feed['points'], feed['enabled'] = source, True
            node.wait(lambda: False,0.9)
            require(node.healthy is False and node.permission is False,'Stable localization alone released hold')

            goal = SubmitGoal.Request()
            goal.goal.header.frame_id = 'map'
            goal.goal.pose.position.x, goal.goal.pose.position.y = 3.0,0.0
            goal.goal.pose.orientation.w = 1.0
            # A real active goal UUID does not authorize an old or malformed plan.
            fixture_goal = NavigateToPose.Goal()
            fixture_goal.pose = copy.deepcopy(goal.goal)
            future = action.send_goal_async(fixture_goal)
            require(node.wait(future.done,3) and future.result().accepted,'Resume boundary fixture rejected')
            fixture_handle = future.result()
            node.wait(lambda:False,0.15)
            boundary = ResumeRecovery.Request()
            boundary.map_version,boundary.recovery_id = 'transaction-v1',recovery_id
            boundary.goal_id = fixture_handle.goal_id.uuid
            boundary.planned_path.header.frame_id = 'map'
            start = PoseStamped()
            start.header.frame_id = 'map'
            start.pose.position.x,start.pose.position.y = 3.0,-1.0
            start.pose.orientation.w = 1.0
            boundary.planned_path.poses = [start,copy.deepcopy(goal.goal)]
            require(not call('resume',boundary).accepted,'Pre-commit path released recovery hold')
            boundary.planned_path.header.stamp = node.get_clock().now().to_msg()
            boundary.goal_id = [0]*16
            require(not call('resume',boundary).accepted,'Wrong goal UUID released recovery hold')
            boundary.goal_id = fixture_handle.goal_id.uuid
            boundary.recovery_id += 1
            require(not call('resume',boundary).accepted,'Wrong session released recovery hold')
            cleanup = fixture_handle.cancel_goal_async()
            require(node.wait(cleanup.done,3),'Resume fixture cancel timed out')
            terminated = fixture_handle.get_result_async()
            require(node.wait(terminated.done,3),'Resume fixture did not terminate')
            node.wait(lambda:False,0.3)
            require(node.permission is False and node.healthy is False,'Resume rejection permitted motion')
            report.append('old plan, wrong active-goal UUID and wrong session cannot release recovery hold')
            invalid = copy.deepcopy(goal)
            invalid.goal.header.frame_id = 'odom'
            require(not call('submit',invalid).accepted,'Wrong-frame task accepted')
            invalid = copy.deepcopy(goal)
            invalid.goal.pose.position.x = float('nan')
            require(not call('submit',invalid).accepted,'NaN task accepted')
            response = call('submit',goal)
            require(response.accepted,'New task refused: '+response.reason)
            require(node.wait(lambda: node.permission is True and node.state.phase == RecoveryState.TRACKING,6),
                    'New plan/goal did not resume: '+node.task_status+' / '+node.reason)
            require(node.wait(lambda: abs(node.x)+abs(node.y) > 0.08 or ':FAULT:' in node.task_status,5) and
                    ':FAULT:' not in node.task_status,'Owned resumed goal did not move model: '+str(diagnostic_history[-12:]))
            require(node.wait(lambda: (':IDLE:' in node.task_status and node.permission is False) or ':FAULT:' in node.task_status,35) and
                    ':IDLE:' in node.task_status,
                    'Owned task did not finish: '+node.task_status)
            tx = node.transform.transform.translation
            map_x,map_y = c*node.x-s*node.y+tx.x,s*node.x+c*node.y+tx.y
            require(math.hypot(map_x-3,map_y) < 0.25,'Resumed navigation ended at wrong map position')
            current_tx = node.transform.transform.translation
            old_tx = old_transform.translation
            require(math.sqrt((current_tx.x-old_tx.x)**2+(current_tx.y-old_tx.y)**2+(current_tx.z-old_tx.z)**2) < 0.02,
                    'Local matching changed correction unexpectedly')
            report.append('true Nav2 preflight path and new goal UUID release hold; owned task reaches map goal')

            goal.goal.pose.position.y = 3.0
            require(call('submit',goal).accepted,'Local-state fault fixture task refused')
            require(node.wait(lambda:node.permission is True,5),'Local-state fixture task not active')
            node.chassis_healthy = False
            require(node.wait(lambda:node.permission is False and ':FAULT:' in node.task_status,3),
                    'Fresh global localization hid loss of chassis health')
            require(node.healthy is True,'Local-state failure fixture unexpectedly lost global localization')
            node.chassis_healthy = True
            node.wait(lambda:False,.6)
            require(node.permission is False,'Local health recovery replayed old owned task')
            report.append('local chassis loss cancels owned task despite fresh global localization; no replay on recovery')
            require(call('submit',goal).accepted,'Second owned task refused')
            require(node.wait(lambda: node.permission is True,5),'Second task not active')
            foreign = NavigateToPose.Goal()
            foreign.pose = copy.deepcopy(goal.goal)
            foreign.pose.pose.position.x = 4.0
            sent = action.send_goal_async(foreign)
            require(node.wait(sent.done,3) and sent.result().accepted,'Foreign preemption fixture rejected')
            foreign_handle = sent.result()
            require(node.wait(lambda: node.permission is False and ':FAULT:' in node.task_status,3),
                    'Foreign goal inherited owned-task permission')
            require(node.healthy is True,'Foreign goal test unexpectedly lost localization')
            canceled = foreign_handle.cancel_goal_async()
            require(node.wait(canceled.done,3),'Foreign cleanup cancellation timed out')
            ended = foreign_handle.get_result_async()
            require(node.wait(ended.done,3),'Foreign goal did not terminate')
            node.wait(lambda:False,0.3)
            report.append('foreign preemption cannot inherit owned-task motion permission')
            require(call('submit',goal).accepted,'Task after foreign cancellation refused')
            require(node.wait(lambda:node.permission is True,5),'Third owned task not active')
            feed['enabled'] = False
            require(node.wait(lambda: node.permission is False and ':FAULT:' in node.task_status,4),
                    'Lost localization did not revoke and cancel task')
            feed['enabled'] = True
            node.wait(lambda: False,1.2)
            require(node.permission is False and abs(node.latest.twist.linear.x)+abs(node.latest.twist.linear.y) < 1e-9,
                    'Old task replayed after health recovered')
            report.append('localization loss revokes owned task; health recovery does not replay old task')
            print('PASS '+ '; '.join(report),flush=True)
    finally:
        node.destroy_timer(timer)
        for process in processes: os.killpg(process.pid,signal.SIGINT)
        for process in processes:
            try: process.wait(timeout=6)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGKILL);process.wait()
        node.destroy_node();rclpy.shutdown()
        Path('log/task_resume_smoke_results.txt').write_text('\n'.join(report),encoding='utf-8')
        Path('log/task_resume_localization_history.txt').write_text('\n'.join(diagnostic_history),encoding='utf-8')
        Path('log/task_resume_estimates.txt').write_text('\n'.join(estimates),encoding='utf-8')


if __name__ == '__main__': main()
