#!/usr/bin/env python3
"""Real LIO -> virtual planar sensor reference and shadow xy command adapter.

Never publishes chassis TF, /cmd_vel or /cmd_yaw_angle. No encoder is synthesized.
"""
import copy
import json
import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, TwistStamped
from std_msgs.msg import Bool, String
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from sensor_reference_math import sensor_pose, sensor_velocity
from lio_continuity import PoseContinuity


class SensorReference(Node):
    def __init__(self):
        super().__init__('sensor_reference')
        self.timeout = self.declare_parameter('timeout', .25).value
        self.offset = self.declare_parameter('imu_to_lidar', [-.011, -.02329, .04412]).value
        if not .05 <= self.timeout <= 1 or len(self.offset) != 3 or not all(map(math.isfinite, self.offset)):
            raise ValueError('Invalid factory geometry/timeout')
        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        fixed = TransformStamped()
        fixed.header.stamp = self.get_clock().now().to_msg()
        fixed.header.frame_id, fixed.child_frame_id = 'front_mid360_imu', 'front_mid360'
        fixed.transform.translation.x, fixed.transform.translation.y, fixed.transform.translation.z = self.offset
        fixed.transform.rotation.w = 1.
        self.static_tf.sendTransform(fixed)
        self.odom_pub = self.create_publisher(Odometry, '/sensor_reference/odom', 10)
        self.health_pub = self.create_publisher(Bool, '/state/sensor_reference_healthy', 10)
        self.reason_pub = self.create_publisher(String, '/state/sensor_reference_reason', 10)
        self.shadow = self.create_publisher(TwistStamped, '/nav/sensor_cmd_vel_shadow', 1)
        self.command_reason = self.create_publisher(String, '/nav/sensor_command_status', 10)
        self.raw_at = self.pose_at = self.allowed_at = self.command_at = -math.inf
        self.raw_ok = self.pose_ok = self.allowed = False
        self.last_stamp = 0
        self.yaw = 0.
        self.command = None
        self.previous = None
        self.detail = 'waiting for real LIO and raw-cloud quality'
        self.continuity = PoseContinuity(
            self.declare_parameter('max_pose_gap', 1.).value,
            self.declare_parameter('max_sensor_speed', 6.).value,
            self.declare_parameter('max_sensor_angular_speed', 12.).value)
        self.create_subscription(Bool, '/sensors/front_mid360/cloud_healthy', self.raw, 10)
        self.create_subscription(Odometry, '/lio/sensor_odometry', self.pose, qos_profile_sensor_data)
        self.create_subscription(Bool, '/nav/motion_allowed', self.permission, 10)
        self.create_subscription(TwistStamped, '/nav/cmd_vel_safe', self.velocity, 1)
        self.create_timer(.02, self.tick)

    def raw(self, msg):
        self.raw_ok, self.raw_at = msg.data, time.monotonic()
        if not msg.data:
            self.command = None

    def permission(self, msg):
        self.allowed, self.allowed_at = msg.data, time.monotonic()
        if not msg.data:
            self.command = None

    def healthy(self):
        return (self.pose_ok and self.raw_ok and
                time.monotonic()-self.pose_at <= self.timeout and
                time.monotonic()-self.raw_at <= self.timeout and
                -.05 <= (self.get_clock().now().nanoseconds-self.last_stamp)*1e-9 <= self.timeout)

    def pose(self, msg):
        try:
            stamp = msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
            age = (self.get_clock().now().nanoseconds-stamp)*1e-9
            if (msg.header.frame_id != 'odom' or msg.child_frame_id != 'front_mid360_imu' or
                    msg.header.stamp.sec < 0 or not 0 <= msg.header.stamp.nanosec < 10**9 or
                    stamp <= self.last_stamp or not -.05 <= age <= self.timeout):
                raise ValueError('LIO frame, source age or monotonic timestamp invalid')
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            covariance = np.array(msg.pose.covariance).reshape(6, 6)
            if (not np.isfinite(covariance).all() or
                    not np.allclose(covariance, covariance.T, rtol=0, atol=1e-8) or
                    np.linalg.eigvalsh(covariance).min() < -1e-9 or
                    np.diag(covariance).min() <= 0):
                raise ValueError('LIO covariance must be finite symmetric and positive')
            point, yaw = sensor_pose((p.x, p.y, p.z), (q.x, q.y, q.z, q.w), self.offset)
            self.continuity.check(stamp, point, (q.x,q.y,q.z,q.w))
            self.last_stamp, self.pose_at, self.pose_ok = stamp, time.monotonic(), True
            self.yaw, self.detail = yaw, 'real sensor-axis state; chassis orientation unobserved'
            # Never keep broadcasting TF after raw quality or source time expires.
            if not self.healthy():
                self.command = None
                self.previous = None
                return
            sensor = TransformStamped()
            sensor.header, sensor.child_frame_id = copy.deepcopy(msg.header), msg.child_frame_id
            sensor.transform.translation.x, sensor.transform.translation.y, sensor.transform.translation.z = p.x, p.y, p.z
            sensor.transform.rotation = copy.deepcopy(q)
            reference = TransformStamped()
            reference.header, reference.child_frame_id = copy.deepcopy(msg.header), 'nav_reference'
            reference.transform.translation.x, reference.transform.translation.y = point[:2]
            reference.transform.rotation.w = 1.
            yaw_reference = copy.deepcopy(reference)
            yaw_reference.child_frame_id = 'sensor_yaw_reference'
            yaw_reference.transform.rotation.z = math.sin(yaw/2)
            yaw_reference.transform.rotation.w = math.cos(yaw/2)
            self.tf.sendTransform([sensor, reference, yaw_reference])
            odom = Odometry()
            odom.header, odom.child_frame_id = copy.deepcopy(msg.header), 'nav_reference'
            odom.pose.pose.position.x, odom.pose.pose.position.y = point[:2]
            odom.pose.pose.orientation.w = 1.
            # Virtual heading is exactly odom-aligned, not an estimate of chassis yaw.
            # Propagate IMU pose covariance to the LiDAR point; ROS rotational
            # perturbations are expressed about the fixed parent axes.
            delta = np.array(point)-np.array((p.x,p.y,p.z))
            dx,dy,dz = delta
            jacobian = np.zeros((2,6))
            jacobian[:,:3] = np.eye(3)[:2]
            jacobian[:,3:] = -np.array([[0.,-dz,dy],[dz,0.,-dx]])
            xy_covariance = jacobian@covariance@jacobian.T
            odom.pose.covariance[0] = float(xy_covariance[0,0])
            odom.pose.covariance[1] = float(xy_covariance[0,1])
            odom.pose.covariance[6] = float(xy_covariance[1,0])
            odom.pose.covariance[7] = float(xy_covariance[1,1])
            # No measured wheel velocity: derivative is navigation-only, not drive feedback.
            for index in range(6):
                odom.twist.covariance[index*7] = 1e6
            if self.previous is not None:
                old_stamp, old_point = self.previous
                dt = (stamp-old_stamp)*1e-9
                if 0 < dt <= self.timeout:
                    odom.twist.twist.linear.x = (point[0]-old_point[0])/dt
                    odom.twist.twist.linear.y = (point[1]-old_point[1])/dt
            self.previous = stamp, point
            self.odom_pub.publish(odom)
        except (ValueError, np.linalg.LinAlgError) as error:
            self.pose_ok, self.detail, self.command, self.previous = False, str(error), None, None

    def velocity(self, msg):
        age = (self.get_clock().now().nanoseconds-
               (msg.header.stamp.sec*10**9+msg.header.stamp.nanosec))*1e-9
        v = msg.twist
        if (not self.healthy() or not self.allowed or time.monotonic()-self.allowed_at > self.timeout or
                msg.header.frame_id != 'nav_reference' or not 0 <= age <= self.timeout or
                not all(map(math.isfinite, (v.linear.x, v.linear.y, v.linear.z, v.angular.x, v.angular.y, v.angular.z))) or
                max(abs(v.linear.z),abs(v.angular.x),abs(v.angular.y)) > 1e-6 or
                abs(v.angular.z) > 1e-6 or math.hypot(v.linear.x, v.linear.y) > .5+1e-6):
            self.command = None
            return
        self.command, self.command_at = copy.deepcopy(msg), time.monotonic()

    def tick(self):
        healthy = self.healthy()
        self.health_pub.publish(Bool(data=healthy))
        self.reason_pub.publish(String(data=self.detail if healthy else 'unhealthy/stale: '+self.detail))
        active = (healthy and self.allowed and time.monotonic()-self.allowed_at <= self.timeout and
                  self.command is not None and time.monotonic()-self.command_at <= self.timeout)
        out = TwistStamped()
        out.header.stamp, out.header.frame_id = self.get_clock().now().to_msg(), 'sensor_yaw_reference'
        if active:
            age = (self.get_clock().now().nanoseconds-
                   (self.command.header.stamp.sec*10**9+self.command.header.stamp.nanosec))*1e-9
            active = 0 <= age <= self.timeout
        if active:
            out.twist.linear.x, out.twist.linear.y = sensor_velocity(
                self.command.twist.linear.x, self.command.twist.linear.y, self.yaw)
        else:
            self.command = None  # Fresh permissions cannot replay an earlier command.
        self.shadow.publish(out)
        self.command_reason.publish(String(data=json.dumps({
            'mode': 'sensor_reference_shadow', 'active': active, 'physical_output': False,
            'velocity_point': 'lidar_yaw_axis', 'axes': 'horizontal_sensor_yaw_xy',
            'chassis_yaw': 'UNOBSERVED', 'full_chassis_stop': 'NOT_VERIFIED'})))


def main():
    rclpy.init()
    node = SensorReference()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
