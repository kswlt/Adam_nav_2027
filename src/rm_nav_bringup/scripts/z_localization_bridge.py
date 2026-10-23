#!/usr/bin/env python3
"""Adapt atomic ZLIO observations and actual backend trust, without estimating poses/TF."""
import copy
import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from localization_interfaces.msg import FrontendFrame, FrontendOdometry, FrontendSession
from z_localization_policy import parse_state, trusted_for_navigation, rotation


def stamp_ns(stamp):
    return stamp.sec*10**9+stamp.nanosec


class ZLocalizationBridge(Node):
    def __init__(self):
        super().__init__('z_localization_bridge')
        self.role = self.declare_parameter('role', 'frontend').value
        self.mode = self.declare_parameter('mode', 'mapping').value
        self.sensor_frame = self.declare_parameter('sensor_frame', 'front_mid360_imu').value
        self.timeout = self.declare_parameter('timeout', .25).value
        if self.role not in ('frontend', 'backend') or not .05 <= self.timeout <= 1.:
            raise ValueError('Invalid bridge role/timeout')
        self.session = None
        self.quarantine_until = math.inf
        self.raw_ok = False
        self.raw_at = self.frame_at = self.pose_at = self.state_at = -math.inf
        self.last_frame = self.last_pose = self.last_sequence = 0
        self.state = {}
        self.reason = 'waiting for real ZLIO session and observations'
        self.create_subscription(FrontendSession, '/frontend/session', self.on_session,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Bool, '/lio/input_healthy', self.on_input, 10)
        if self.role == 'frontend':
            self.odom_pub = self.create_publisher(Odometry, '/lio/sensor_odometry', qos_profile_sensor_data)
            self.cloud_pub = self.create_publisher(PointCloud2, '/lio/deskewed_odom_cloud', qos_profile_sensor_data)
            self.health_pub = self.create_publisher(Bool, '/lio/estimator_healthy', 10)
            self.create_subscription(FrontendFrame, '/frontend/frame', self.on_frame, 2)
            self.create_subscription(FrontendOdometry, '/frontend/odometry', self.on_odometry, 10)
            self.reason_pub = self.create_publisher(String, '/lio/estimator_reason', 10)
            self.session_pub = self.create_publisher(String, '/lio/session_identity',
                                        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        else:
            self.health_pub = self.create_publisher(Bool, '/localization/healthy', 10)
            self.reason_pub = self.create_publisher(String, '/localization/reason', 10)
            self.create_subscription(String, '/backend_node/localization_state', self.on_state,
                                     QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.create_subscription(Odometry, '/localization/odometry', self.on_map_pose, qos_profile_sensor_data)
            version = self.declare_parameter('map_version', '').value
            if self.mode == 'localization' and not version:
                raise ValueError('Localization requires a map version')
            self.version_pub = self.create_publisher(String, '/localization/map_version',
                                        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.version_pub.publish(String(data=version))
        self.create_timer(.02, self.tick)

    def on_session(self, msg):
        if self.session is None or (msg.instance_id, msg.epoch) != (self.session.instance_id, self.session.epoch):
            self.last_frame = self.last_pose = self.last_sequence = 0
            self.frame_at = self.pose_at = self.state_at = -math.inf
            self.state = {}
            self.quarantine_until = time.monotonic()+self.timeout+.05
            if self.role == 'frontend':
                self.session_pub.publish(String(data=f'{msg.instance_id}:{msg.epoch}'))
        self.session = msg

    def on_input(self, msg):
        self.raw_ok, self.raw_at = msg.data, time.monotonic()

    def session_ok(self, message=None):
        return (self.session is not None and not self.session.failed and not self.session.reset_required and
                (message is None or (message.instance_id, message.epoch) == (self.session.instance_id, self.session.epoch)))

    def fresh(self, stamp):
        return 0 < stamp and -.05 <= (self.get_clock().now().nanoseconds-stamp)*1.e-9 <= self.timeout

    def valid_pose(self, msg, frame='odom'):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        covariance = np.asarray(msg.pose.covariance).reshape(6, 6)
        if (msg.header.frame_id != frame or msg.child_frame_id != self.sensor_frame or
                not self.fresh(stamp_ns(msg.header.stamp)) or
                not all(map(math.isfinite, (p.x, p.y, p.z))) or not np.isfinite(covariance).all() or
                not np.allclose(covariance, covariance.T, atol=1.e-8, rtol=0) or
                np.linalg.eigvalsh(covariance).min() <= 0):
            raise ValueError('Sensor pose frame/time/covariance invalid')
        return rotation((q.x, q.y, q.z, q.w)), np.array([p.x, p.y, p.z])

    def on_odometry(self, msg):
        try:
            stamp = stamp_ns(msg.odometry.header.stamp)
            if not self.session_ok(msg) or stamp <= self.last_pose:
                return
            self.valid_pose(msg.odometry)
            self.last_pose, self.pose_at = stamp, time.monotonic()
            if self.raw_ok and time.monotonic()-self.frame_at <= self.timeout:
                self.odom_pub.publish(msg.odometry)
        except (ValueError, np.linalg.LinAlgError) as error:
            self.reason = str(error)
            self.pose_at = -math.inf

    def on_frame(self, msg):
        try:
            stamp = stamp_ns(msg.odometry.header.stamp)
            if not self.session_ok(msg):
                return
            if (stamp <= self.last_frame or msg.sequence <= self.last_sequence or not msg.lidar_updated or
                    msg.cloud.header.frame_id != self.sensor_frame or
                    stamp_ns(msg.cloud.header.stamp) != stamp):
                raise ValueError('Atomic observation identity/time/geometry update invalid')
            transform, offset = self.valid_pose(msg.odometry)
            if msg.cloud.width*msg.cloud.height > 200000:
                raise ValueError('Oversized deskewed observation')
            points = point_cloud2.read_points_numpy(msg.cloud, field_names=('x', 'y', 'z'), skip_nans=False)
            if len(points) < 20 or not np.isfinite(points).all():
                raise ValueError('Invalid deskewed observation geometry')
            header = copy.deepcopy(msg.odometry.header)
            cloud = point_cloud2.create_cloud_xyz32(header, (points @ transform.T + offset).astype(np.float32))
            self.last_frame, self.last_sequence = stamp, msg.sequence
            self.frame_at = time.monotonic()
            if self.raw_ok and time.monotonic()-self.raw_at <= self.timeout:
                self.cloud_pub.publish(cloud)
            self.reason = 'ZLIO atomic IMU-frame deskewed cloud projected once into odom'
        except (ValueError, AssertionError, np.linalg.LinAlgError) as error:
            self.frame_at = -math.inf
            self.reason = str(error)

    def on_state(self, msg):
        try:
            state = parse_state(msg.data)
            # Diagnostic-only messages have stamp=0/tf_published=false; reject them immediately
            # when untrusted, but keep a fresh trusted projection until its normal timeout.
            if state['tf_published'] == 'true' or state['pose_trusted'] != 'true':
                self.state, self.state_at = state, time.monotonic()
            self.reason = msg.data
        except ValueError as error:
            self.state = {}
            self.reason = str(error)

    def on_map_pose(self, msg):
        try:
            self.valid_pose(msg, 'map')
            stamp = stamp_ns(msg.header.stamp)
            if stamp <= self.last_pose:
                raise ValueError('Backend odometry timestamp is not increasing')
            self.last_pose, self.pose_at = stamp, time.monotonic()
        except (ValueError, np.linalg.LinAlgError) as error:
            self.pose_at = -math.inf
            self.reason = str(error)

    def tick(self):
        now = time.monotonic()
        ok = now >= self.quarantine_until and self.session_ok() and self.raw_ok and now-self.raw_at <= self.timeout
        ok = ok and now-self.pose_at <= self.timeout and self.fresh(self.last_pose)
        if self.role == 'frontend':
            ok = ok and now-self.frame_at <= self.timeout and self.fresh(self.last_frame)
        else:
            ok = ok and now-self.state_at <= self.timeout and trusted_for_navigation(
                self.state, self.mode, self.session.epoch if self.session else -1,
                self.get_clock().now().nanoseconds, self.timeout)
        self.health_pub.publish(Bool(data=bool(ok)))
        # Do not repeatedly label stale backend text as a current NORMAL state.
        # The unmodified backend diagnostic remains on its native topic.
        if ok:
            detail = self.reason
        else:
            faults = []
            if not self.session_ok(): faults.append('session missing/failed/reset required')
            if now < self.quarantine_until: faults.append('session initialization quarantine')
            if not self.raw_ok: faults.append('LIO input health false')
            if now-self.raw_at > self.timeout: faults.append('LIO input heartbeat stale')
            if now-self.pose_at > self.timeout or not self.fresh(self.last_pose):
                faults.append('pose missing/stale/invalid')
            if self.role == 'frontend' and (now-self.frame_at > self.timeout or not self.fresh(self.last_frame)):
                faults.append('atomic frame missing/stale/invalid')
            if self.role == 'backend':
                if now-self.state_at > self.timeout: faults.append('backend state stale')
                elif not trusted_for_navigation(self.state, self.mode,
                        self.session.epoch if self.session else -1,
                        self.get_clock().now().nanoseconds, self.timeout):
                    faults.append('backend trust rejected')
            detail = 'status=UNHEALTHY;reason='+', '.join(faults)
        self.reason_pub.publish(String(data=detail))


def main():
    rclpy.init()
    node = ZLocalizationBridge()
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
