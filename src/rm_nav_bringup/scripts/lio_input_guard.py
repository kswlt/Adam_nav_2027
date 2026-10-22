#!/usr/bin/env python3
"""Pass original IMU/cloud messages only while both inputs remain continuous."""
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import Bool, String
from lio_continuity import InputContinuity


class LioInputGuard(Node):
    def __init__(self):
        super().__init__('lio_input_guard')
        self.guard = InputContinuity(
            self.declare_parameter('max_input_gap', .5).value,
            self.declare_parameter('max_source_age', .25).value,
            self.declare_parameter('max_sensor_skew', .25).value)
        # 200Hz IMU can arrive in short recorder/DDS bursts. Retain them without
        # changing source-age/gap checks; depth=5 silently loses only 25ms of history.
        imu_qos = rclpy.qos.QoSProfile(depth=4000,
                                      reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT)
        self.pubs = {
            'imu':self.create_publisher(Imu, '/lio/input/imu', imu_qos),
            'cloud':self.create_publisher(PointCloud2, '/lio/input/cloud', qos_profile_sensor_data)}
        self.health = self.create_publisher(Bool, '/lio/input_healthy', 10)
        self.reason = self.create_publisher(String, '/lio/input_reason', 10)
        self.create_subscription(Imu, '/livox/imu', lambda m:self.receive('imu',m), imu_qos)
        self.create_subscription(PointCloud2, '/sensors/front_mid360/guarded_points', lambda m:self.receive('cloud',m), qos_profile_sensor_data)
        self.create_timer(.02, self.tick)
        self.reported_fault = ''

    def receive(self, stream, msg):
        if stream == 'imu' and not all(math.isfinite(v) for v in (
                msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z,
                msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z)):
            self.guard.fail('Nonfinite IMU values')
        if msg.header.stamp.sec < 0 or not 0 <= msg.header.stamp.nanosec < 10**9:
            self.guard.fail('Invalid input ROS timestamp')
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if self.guard.observe(stream,stamp,self.get_clock().now().nanoseconds*1e-9,time.monotonic()):
            self.pubs[stream].publish(msg)

    def tick(self):
        ok = self.guard.tick(time.monotonic())
        self.health.publish(Bool(data=ok))
        reason = self.guard.fault or ('continuous original inputs' if ok else 'waiting for both live input streams')
        self.reason.publish(String(data=reason))
        if self.guard.fault and self.reported_fault != self.guard.fault:
            self.get_logger().error(self.guard.fault)
            self.reported_fault = self.guard.fault


def main():
    rclpy.init()
    node = LioInputGuard()
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
