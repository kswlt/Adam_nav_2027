#!/usr/bin/env python3
"""Own navigation tasks and publish fresh permission only for an owned live goal."""
import copy
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy
from action_msgs.msg import GoalStatus, GoalStatusArray
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from rm_nav_interfaces.msg import RecoveryState
from rm_nav_interfaces.srv import SubmitGoal, ResumeRecovery
from std_msgs.msg import Bool, String


class TaskSupervisor(Node):
    def __init__(self):
        super().__init__('task_supervisor')
        self.phase = 'IDLE'
        self.reason = 'no owned task'
        self.task_id = 0
        self.navigation_handle = self.planner_handle = None
        self.healthy = False
        self.health_received = 0.0
        self.require_chassis = self.declare_parameter('require_chassis_health',True).value
        self.chassis_healthy,self.chassis_received = False,0.0
        self.recovery = None
        self.recovery_received = 0.0
        self.started = time.monotonic()
        self.active_ids = set()
        self.planner = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')
        self.navigator = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.resume = self.create_client(ResumeRecovery, '/localization/resume_recovery')
        self.permission = self.create_publisher(Bool, '/nav/motion_enable', 10)
        self.status = self.create_publisher(String, '/nav/task_status', 10)
        self.create_subscription(Bool, '/localization/healthy', self.health, 10)
        self.create_subscription(Bool, '/state/chassis_healthy', self.chassis_health, 10)
        self.create_subscription(RecoveryState, '/localization/recovery_state', self.recovery_state,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', self.goal_status,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_service(SubmitGoal, '/nav/submit_goal', self.submit)
        self.create_timer(0.05, self.tick)

    def health(self, msg):
        self.healthy, self.health_received = msg.data, time.monotonic()

    def chassis_health(self,msg):
        self.chassis_healthy,self.chassis_received = msg.data,time.monotonic()

    def chassis_ready(self):
        return not self.require_chassis or (self.chassis_healthy and time.monotonic()-self.chassis_received<=.25)

    def recovery_state(self, msg):
        self.recovery, self.recovery_received = msg, time.monotonic()

    def goal_status(self, msg):
        self.active_ids = {bytes(s.goal_info.goal_id.uuid) for s in msg.status_list if s.status in (1,2,3)}

    def owns_active_goal(self):
        return self.navigation_handle is not None and self.active_ids == {bytes(self.navigation_handle.goal_id.uuid)}

    def mode(self):
        if not self.chassis_ready(): return 'STOP'
        if self.recovery is not None:
            if time.monotonic() - self.recovery_received > 0.25: return 'STOP'
            if self.recovery.phase == RecoveryState.WAIT_REPLAN: return 'REPLAN'
            if self.recovery.phase != RecoveryState.TRACKING: return 'STOP'
        return 'NORMAL' if self.healthy and time.monotonic() - self.health_received <= 0.25 else 'STOP'

    def fail(self, reason):
        self.get_logger().warning(reason)
        self.phase, self.reason = 'FAULT', reason
        self.permission.publish(Bool(data=False))
        if self.navigation_handle is not None: self.navigation_handle.cancel_goal_async()
        if self.planner_handle is not None: self.planner_handle.cancel_goal_async()

    def submit(self, request, response):
        p, q = request.goal.pose.position, request.goal.pose.orientation
        norm = q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w
        values = [p.x,p.y,p.z,q.x,q.y,q.z,q.w]
        mode = self.mode()
        if self.phase not in ('IDLE', 'FAULT') or self.navigation_handle is not None or self.planner_handle is not None or self.active_ids or mode == 'STOP':
            response.reason = 'task busy or localization/recovery not ready'; return response
        if request.goal.header.frame_id != 'map' or not all(math.isfinite(v) for v in values) or abs(norm-1) > 0.01:
            response.reason = 'goal frame or geometry invalid'; return response
        if not self.planner.server_is_ready() or not self.navigator.server_is_ready():
            response.reason = 'Nav2 actions unavailable'; return response
        if mode == 'REPLAN' and not self.resume.service_is_ready():
            response.reason = 'recovery resume service unavailable'; return response
        self.task_id += 1
        task = self.task_id
        self.phase, self.reason = 'PLANNING', 'waiting for a new Nav2 path'
        self.started = time.monotonic()
        self.goal = copy.deepcopy(request.goal)
        self.goal.header.stamp = self.get_clock().now().to_msg()
        self.recovery_context = copy.deepcopy(self.recovery) if mode == 'REPLAN' else None
        goal = ComputePathToPose.Goal()
        goal.goal, goal.planner_id, goal.use_start = self.goal, 'GridBased', False
        future = self.planner.send_goal_async(goal)
        future.add_done_callback(lambda f: self.planning_accepted(task, f))
        response.accepted, response.task_id, response.reason = True, task, self.reason
        return response

    def planning_accepted(self, task, future):
        try:
            handle = future.result()
            if task != self.task_id or self.phase != 'PLANNING':
                if handle.accepted: handle.cancel_goal_async()
                return
            if not handle.accepted: self.fail('planner rejected goal'); return
            self.planner_handle = handle
            handle.get_result_async().add_done_callback(lambda f: self.planned(task, f))
        except Exception as error:
            if task == self.task_id: self.fail('planner response failed: ' + str(error))

    def planned(self, task, future):
        if task != self.task_id: return
        self.planner_handle = None
        if self.phase != 'PLANNING': return
        try:
            result = future.result()
            path = result.result.path
            if result.status != GoalStatus.STATUS_SUCCEEDED or result.result.error_code != 0 or path.header.frame_id != 'map' or not 2 <= len(path.poses) <= 10000:
                self.fail('new planning failed or path invalid'); return
            for pose in path.poses:
                p, q = pose.pose.position, pose.pose.orientation
                if not all(math.isfinite(v) for v in (p.x,p.y,p.z,q.x,q.y,q.z,q.w)):
                    self.fail('planned path has nonfinite pose'); return
            if self.recovery_context is not None and (self.recovery is None or self.recovery.recovery_id != self.recovery_context.recovery_id or
                                            self.recovery.phase != RecoveryState.WAIT_REPLAN):
                self.fail('recovery changed while planning'); return
            self.path = path
            self.phase, self.reason = 'STARTING', 'waiting for new navigation goal acceptance'
            goal = NavigateToPose.Goal()
            goal.pose = self.goal
            self.navigator.send_goal_async(goal).add_done_callback(lambda f: self.navigation_accepted(task, f))
        except Exception as error: self.fail('planning failed: ' + str(error))

    def navigation_accepted(self, task, future):
        try:
            handle = future.result()
            if task != self.task_id or self.phase != 'STARTING':
                if handle.accepted: handle.cancel_goal_async()
                return
            if not handle.accepted: self.fail('navigator rejected goal'); return
            self.navigation_handle = handle
            handle.get_result_async().add_done_callback(lambda f: self.finished(task, f))
            self.started = time.monotonic()
            if self.recovery_context is None:
                self.phase, self.reason = 'NAVIGATING', 'owned navigation goal active'
            else:
                self.phase, self.reason = 'RESUMING', 'new goal accepted; checking localization resume'
                request = ResumeRecovery.Request()
                request.map_version, request.recovery_id = self.recovery_context.map_version, self.recovery_context.recovery_id
                request.goal_id, request.planned_path = handle.goal_id.uuid, self.path
                self.resume.call_async(request).add_done_callback(lambda f: self.resumed(task, f))
        except Exception as error:
            if task == self.task_id: self.fail('navigation acceptance failed: ' + str(error))

    def resumed(self, task, future):
        if task != self.task_id or self.phase != 'RESUMING': return
        try:
            response = future.result()
            if not response.accepted: self.fail('resume rejected: ' + response.reason); return
            self.phase, self.reason = 'NAVIGATING', 'new plan and localization resume accepted'
            self.started = time.monotonic()
        except Exception as error: self.fail('resume failed: ' + str(error))

    def finished(self, task, future):
        if task != self.task_id: return
        self.navigation_handle = None
        try:
            result = future.result()
            self.phase = 'IDLE' if result.status == GoalStatus.STATUS_SUCCEEDED else 'FAULT'
            self.reason = 'owned goal terminal status=' + str(result.status)
        except Exception as error: self.phase, self.reason = 'FAULT', str(error)
        self.permission.publish(Bool(data=False))

    def tick(self):
        mode = self.mode()
        if self.phase in ('PLANNING','STARTING','RESUMING') and not self.chassis_ready():
            self.fail('chassis state health lost during task startup')
        if self.phase == 'NAVIGATING' and (mode != 'NORMAL' or not self.owns_active_goal() or not self.navigator.server_is_ready()):
            if time.monotonic() - self.started > 0.3:
                self.fail(f'localization/recovery/action health lost: mode={mode}, healthy={self.healthy}, '
                          f'health_age={time.monotonic()-self.health_received:.3f}, '
                          f'recovery_age={time.monotonic()-self.recovery_received:.3f}, '
                          f'owned={self.owns_active_goal()}, active_goals={len(self.active_ids)}')
        elif self.phase in ('PLANNING', 'STARTING', 'RESUMING') and time.monotonic() - self.started > 5.0:
            self.fail('task planning/acceptance/resume timed out')
        allowed = self.phase == 'NAVIGATING' and self.owns_active_goal() and mode == 'NORMAL' and self.navigator.server_is_ready()
        self.permission.publish(Bool(data=allowed))
        self.status.publish(String(data=f'{self.task_id}:{self.phase}:{self.reason}'))


def main():
    rclpy.init()
    node = TaskSupervisor()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        if rclpy.ok(): node.fail('supervisor shutting down')
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__': main()
