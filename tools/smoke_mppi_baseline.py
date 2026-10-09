#!/usr/bin/env python3
"""Headless integration acceptance. Run in an unused ROS_DOMAIN_ID.

Starts only the navigation baseline and an ideal holonomic plant. No serial,
hardware driver, wheel feedback or real localization is involved.
"""
import json
import math
import os
from pathlib import Path
import subprocess
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import TransformStamped, TwistStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


class IdealPlant(Node):
    def __init__(self, publish_global_tf=True, publish_localization_health=True, publish_motion_permission=True):
        super().__init__('baseline_acceptance_plant')
        self.x = self.y = self.yaw = 0.0
        self.latest = TwistStamped()
        self.received_at = 0.0
        self.max_lateral = 0.0
        self.scan_mode = 'clear'
        self.output_count = 0
        self.state_clients = {}
        self.state_futures = {}
        self.localization_healthy = True
        self.publish_health = publish_localization_health
        self.motion_enabled = True
        self.publish_motion_permission = publish_motion_permission
        self.health = self.create_publisher(Bool, '/localization/healthy', 10) if publish_localization_health else None
        self.chassis_health = self.create_publisher(Bool,'/state/chassis_healthy',10)
        self.chassis_healthy = True
        self.enable = self.create_publisher(Bool, '/nav/motion_enable', 10)
        self.raw = self.create_publisher(TwistStamped, '/nav/cmd_vel_raw', 10)
        self.create_subscription(TwistStamped, '/nav/cmd_vel_safe', self.command, 10)
        self.odom = self.create_publisher(Odometry, '/odom', 10)
        self.scan = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.map = self.create_publisher(
            OccupancyGrid, '/map', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        t.transform.rotation.w = 1.0
        if publish_global_tf:
            self.static_tf.sendTransform(t)
        grid = OccupancyGrid()
        grid.header.frame_id = 'map'
        grid.info.resolution = 0.05
        grid.info.width = grid.info.height = 200
        grid.info.origin.position.x = grid.info.origin.position.y = -5.0
        grid.info.origin.orientation.w = 1.0
        grid.data = [0] * 40000
        self.map.publish(grid)
        self.previous_tick = time.monotonic()
        self.create_timer(0.02, self.tick)

    def command(self, msg):
        self.latest = msg
        self.received_at = time.monotonic()
        self.max_lateral = max(self.max_lateral, abs(msg.twist.linear.y))
        self.output_count += 1

    def tick(self):
        now = time.monotonic()
        dt = min(now - self.previous_tick, 0.1)
        self.previous_tick = now
        twist = self.latest.twist if now - self.received_at < 0.3 else TwistStamped().twist
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        self.x += (c * twist.linear.x - s * twist.linear.y) * dt
        self.y += (s * twist.linear.x + c * twist.linear.y) * dt
        self.yaw += twist.angular.z * dt
        stamp = self.get_clock().now().to_msg()
        # Ideal-plant commissioning inputs; no measured hardware health is implied.
        self.chassis_health.publish(Bool(data=self.chassis_healthy))
        if self.publish_health:
            self.health.publish(Bool(data=self.localization_healthy))
        if self.publish_motion_permission:
            self.enable.publish(Bool(data=self.motion_enabled))
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x, t.transform.translation.y = self.x, self.y
        t.transform.rotation.z = math.sin(self.yaw / 2.0)
        t.transform.rotation.w = math.cos(self.yaw / 2.0)
        self.tf.sendTransform(t)
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id, odom.child_frame_id = 'odom', 'base_link'
        odom.pose.pose.position.x, odom.pose.pose.position.y = self.x, self.y
        odom.pose.pose.orientation = t.transform.rotation
        odom.twist.twist = twist
        self.odom.publish(odom)
        if self.scan_mode != 'dropout':
            scan = LaserScan()
            scan.header.stamp, scan.header.frame_id = stamp, 'base_link'
            scan.angle_min, scan.angle_max = -math.pi, math.pi
            scan.angle_increment = 2.0 * math.pi / 359.0
            scan.range_min, scan.range_max = 0.05, 6.0
            scan.ranges = [0.25 if self.scan_mode == 'blocked' else float('inf')] * 360
            self.scan.publish(scan)

    def wait(self, predicate, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)
            if predicate():
                return True
        return False

    def active(self, name):
        if name not in self.state_clients:
            self.state_clients[name] = self.create_client(GetState, '/' + name + '/get_state')
        client = self.state_clients[name]
        if not client.wait_for_service(timeout_sec=0.1):
            return False
        if name not in self.state_futures:
            self.state_futures[name] = client.call_async(GetState.Request())
        future = self.state_futures[name]
        if not self.wait(future.done, 0.5):
            return False
        del self.state_futures[name]
        return future.result().current_state.id == 3

    def inject_velocity(self, duration):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            msg.twist.linear.y = 0.2
            self.raw.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.02)

    def stopped(self):
        v = self.latest.twist
        return max(abs(v.linear.x), abs(v.linear.y), abs(v.angular.z)) < 0.001


def require(condition, description):
    if not condition:
        raise RuntimeError(description)


def main():
    require(os.environ.get('ROS_DOMAIN_ID') not in (None, '', '0'),
            'Set a dedicated nonzero ROS_DOMAIN_ID before running acceptance')
    report = {'test': 'ideal_plant_mppi_omni', 'hardware_connected': False}
    log_path = Path('log/mppi_baseline_smoke.log')
    log_path.parent.mkdir(exist_ok=True)
    rclpy.init()
    plant = IdealPlant()
    process = None
    try:
        with log_path.open('w') as log:
            process = subprocess.Popen(
                ['ros2', 'launch', 'rm_nav_bringup', 'mppi_baseline.launch.py', 'enable_task_supervisor:=false'],
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            names = ['controller_server', 'planner_server', 'behavior_server',
                     'bt_navigator', 'velocity_smoother', 'collision_monitor']
            for name in names:
                require(plant.wait(lambda: plant.active(name), 35.0), name + ' not active')
                print('ACTIVE ' + name, flush=True)
            client = ActionClient(plant, NavigateToPose, '/navigate_to_pose')
            require(client.wait_for_server(timeout_sec=5.0), 'NavigateToPose unavailable')
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = 'map'
            goal.pose.header.stamp = plant.get_clock().now().to_msg()
            goal.pose.pose.position.y = 1.0
            goal.pose.pose.orientation.w = 1.0
            sent = client.send_goal_async(goal)
            require(plant.wait(sent.done, 5.0), 'Goal response timeout')
            handle = sent.result()
            require(handle.accepted, 'Navigation goal rejected')
            result = handle.get_result_async()
            require(plant.wait(result.done, 40.0), 'Navigation timeout')
            report['action_status'] = result.result().status
            report['position'] = [plant.x, plant.y, plant.yaw]
            report['max_lateral_velocity'] = plant.max_lateral
            require(result.result().status == GoalStatus.STATUS_SUCCEEDED, 'Navigation failed')
            require(math.hypot(plant.x, plant.y - 1.0) < 0.2, 'Final position inaccurate')
            require(plant.max_lateral > 0.05, 'No Omni lateral velocity observed')
            print('PASS lateral NavigateToPose', flush=True)
            plant.inject_velocity(1.0)
            require(abs(plant.latest.twist.linear.y) > 0.05, 'Clear scan blocks motion')
            plant.scan_mode = 'blocked'
            plant.inject_velocity(0.8)
            require(plant.stopped(), 'Collision monitor failed obstacle stop')
            report['obstacle_stop'] = True
            plant.scan_mode = 'clear'
            plant.inject_velocity(1.0)
            require(abs(plant.latest.twist.linear.y) > 0.05, 'Failed to recover after obstacle')
            plant.scan_mode = 'dropout'
            plant.inject_velocity(0.8)
            require(plant.stopped(), 'Collision monitor failed stale scan stop')
            report['scan_dropout_stop'] = True
            plant.scan_mode = 'clear'
            plant.inject_velocity(1.0)
            require(abs(plant.latest.twist.linear.y) > 0.05, 'Failed to recover after scan dropout')
            require(plant.wait(plant.stopped, 2.0), 'Velocity smoother command timeout failed')
            report['command_timeout_stop'] = True
            plant.inject_velocity(1.0)
            require(abs(plant.latest.twist.linear.y) > 0.05, 'Gate blocked healthy permitted motion')
            plant.localization_healthy = False
            plant.inject_velocity(0.6)
            require(plant.stopped(), 'Unhealthy localization failed to inhibit motion')
            report['localization_unhealthy_stop'] = True
            plant.localization_healthy = True
            plant.inject_velocity(1.0)
            require(abs(plant.latest.twist.linear.y) > 0.05, 'Localization recovery failed')
            plant.publish_health = False
            plant.inject_velocity(0.6)
            require(plant.stopped(), 'Lost health heartbeat failed to inhibit motion')
            report['localization_heartbeat_stop'] = True
            plant.publish_health = True
            plant.inject_velocity(1.0)
            plant.motion_enabled = False
            plant.inject_velocity(0.6)
            require(plant.stopped(), 'Disabled motion permission failed to inhibit motion')
            report['motion_permission_stop'] = True
            report['passed'] = True
    finally:
        if process is not None:
            import signal
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        plant.destroy_node()
        rclpy.shutdown()
        Path('log/mppi_baseline_smoke.json').write_text(json.dumps(report, indent=2))
        print(json.dumps(report, indent=2), flush=True)


if __name__ == '__main__':
    main()
