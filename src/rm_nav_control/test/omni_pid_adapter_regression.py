"""TEST_ONLY: isolated DDS domain, all adapter topics remapped, no serial process."""
import math
import os
import subprocess
import sys
import time
import unittest

os.environ['ROS_DOMAIN_ID'] = '199'
os.environ.pop('ROS_LOCALHOST_ONLY', None)
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32

BINARY = sys.argv[1]
PREFIX = '/test_only/omni'


class AdapterRegression(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = rclpy.create_node('omni_adapter_fixture')
        self.cmd = self.node.create_publisher(TwistStamped, PREFIX+'/safe', 1)
        self.feedback = self.node.create_publisher(TwistStamped, PREFIX+'/feedback', 1)
        self.odom = self.node.create_publisher(Odometry, PREFIX+'/odom', 1)
        self.ready = self.node.create_publisher(Bool, PREFIX+'/ready', 1)
        self.outputs, self.angles = [], []
        self.sub = self.node.create_subscription(Twist, PREFIX+'/cmd', self.outputs.append, 10)
        self.yaw_sub = self.node.create_subscription(Float32, PREFIX+'/yaw', self.angles.append, 10)
        self.process = None

    def tearDown(self):
        if self.process:
            self.process.terminate()
            try:
                self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait()
        self.node.destroy_node()
        rclpy.shutdown()

    def start(self, feedback=False, enabled=True):
        args = [BINARY, '--ros-args', '-p', f'enable_physical_output:={str(enabled).lower()}',
                '-p', f'require_measured_feedback:={str(feedback).lower()}', '-p', 'vx.kp:=1.0']
        for source, target in [('/nav/cmd_vel_safe', 'safe'), ('/hardware/measured_twist', 'feedback'),
                               ('/odom', 'odom'), ('/hardware/drive_ready', 'ready'),
                               ('/cmd_vel', 'cmd'), ('/cmd_yaw_angle', 'yaw')]:
            args.extend(['-r', source+':='+PREFIX+'/'+target])
        self.process = subprocess.Popen(args)
        deadline = time.monotonic()+5
        while self.cmd.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.assertIsNone(self.process.poll())
        self.assertGreater(self.cmd.get_subscription_count(), 0)

    def exercise(self, duration=0.4, *, command=True, odom=True, ready=True,
                 feedback=False, velocity=0.1, mode='valid', stopped=False):
        self.outputs.clear()
        self.angles.clear()
        deadline = time.monotonic()+duration
        while time.monotonic() < deadline:
            stamp = self.node.get_clock().now().to_msg()
            if command:
                m = TwistStamped()
                m.header.stamp, m.header.frame_id = stamp, 'base_link'
                m.twist.linear.x = 0.0 if stopped else velocity
                m.twist.linear.y = 0.0 if stopped else velocity
                m.twist.angular.z = 0.0 if stopped else 0.3
                if mode == 'old_command': m.header.stamp.sec -= 2
                if mode == 'future_command': m.header.stamp.sec += 2
                if mode == 'negative_stamp': m.header.stamp.sec = -1
                if mode == 'invalid_nanosec': m.header.stamp.nanosec = 1000000000
                if mode == 'wrong_frame': m.header.frame_id = 'map'
                if mode == 'nan': m.twist.linear.x = math.nan
                self.cmd.publish(m)
            if odom:
                m = Odometry()
                m.header.stamp, m.header.frame_id, m.child_frame_id = stamp, 'odom', 'base_link'
                m.pose.pose.orientation.w = math.cos(0.25)
                m.pose.pose.orientation.z = math.sin(0.25)
                if mode == 'invalid_quaternion': m.pose.pose.orientation.w = 0.0
                if mode == 'old_odom': m.header.stamp.sec -= 2
                self.odom.publish(m)
            if ready: self.ready.publish(Bool(data=mode != 'not_ready'))
            if feedback:
                m = TwistStamped()
                m.header.stamp, m.header.frame_id = stamp, 'base_link'
                m.twist.linear.x = 0.4
                self.feedback.publish(m)
            rclpy.spin_once(self.node, timeout_sec=0.01)
            time.sleep(0.01)
        self.assertIsNone(self.process.poll(), 'adapter crashed')
        return list(self.outputs[-3:])

    def assert_stopped(self, outputs):
        self.assertTrue(outputs)
        for m in outputs:
            self.assertEqual((m.linear.x, m.linear.y, m.angular.z), (0.0, 0.0, 0.0))

    def test_no_feedback_timeouts_and_rejection(self):
        self.start()
        outputs = self.exercise(velocity=4.0)
        self.assertTrue(outputs)
        for m in outputs:
            self.assertAlmostEqual(math.hypot(m.linear.x, m.linear.y), 0.5)
        self.assertTrue(self.angles)
        for mode in ('old_command', 'future_command', 'negative_stamp', 'invalid_nanosec', 'wrong_frame', 'nan',
                     'invalid_quaternion', 'old_odom', 'not_ready'):
            with self.subTest(mode=mode):
                self.assert_stopped(self.exercise(mode=mode))
                self.assertGreater(self.exercise()[-1].linear.x, 0)
        for absent in ('command', 'odom', 'ready'):
            with self.subTest(absent=absent):
                self.assert_stopped(self.exercise(duration=0.55, **{absent: False}))
                self.assertGreater(self.exercise()[-1].linear.x, 0)
        # Recovered health alone must not replay a cached command.
        self.assert_stopped(self.exercise(mode='not_ready'))
        self.assert_stopped(self.exercise(command=False))
        self.assertGreater(self.exercise()[-1].linear.x, 0)
        # Safe zero holds the latest measured yaw instead of an old integrated target.
        self.assert_stopped(self.exercise(stopped=True))
        self.assertAlmostEqual(self.angles[-1].data, 0.5*180/math.pi, places=4)

    def test_feedback_required_and_zero_stop(self):
        self.start(feedback=True)
        self.assert_stopped(self.exercise())
        outputs = self.exercise(feedback=True, velocity=4.0)
        self.assertTrue(outputs)
        self.assertLessEqual(math.hypot(outputs[-1].linear.x, outputs[-1].linear.y), 0.500001)
        self.assert_stopped(self.exercise(feedback=True, stopped=True))
        self.exercise(feedback=True)
        self.assert_stopped(self.exercise(duration=0.55))

    def test_disabled_has_no_output(self):
        self.start(enabled=False)
        self.assertEqual(self.exercise(), [])
        self.assertEqual(self.angles, [])


if __name__ == '__main__':
    unittest.main(argv=[sys.argv[0]])
