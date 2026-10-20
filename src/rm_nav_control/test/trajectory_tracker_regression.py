"""TEST_ONLY ROS tracker regression: localhost Domain 198, no serial/hardware nodes."""
import os
import subprocess
import sys
import time
import unittest
os.environ['ROS_DOMAIN_ID']='198'
os.environ.pop('ROS_LOCALHOST_ONLY', None)
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE']='LOCALHOST'
import rclpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Bool,String
from rm_nav_interfaces.msg import TimedTrajectory, TrajectoryPoint, TrajectoryEnvironment

BINARY=sys.argv[1]
PREFIX='/test_only/tracker'

class TrackerRegression(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node=rclpy.create_node('trajectory_tracker_fixture')
        self.outputs=[]
        self.status=''
        self.pub={
            'trajectory': self.node.create_publisher(TimedTrajectory,PREFIX+'/trajectory',1),
            'environment': self.node.create_publisher(TrajectoryEnvironment,PREFIX+'/environment',1),
            'odom': self.node.create_publisher(Odometry,PREFIX+'/odom',1),
            'health': self.node.create_publisher(Bool,PREFIX+'/health',1),
            'permission': self.node.create_publisher(Bool,PREFIX+'/permission',1)}
        self.sub=self.node.create_subscription(TwistStamped,PREFIX+'/output',self.outputs.append,10)
        self.status_sub=self.node.create_subscription(String,PREFIX+'/status',lambda m:setattr(self,'status',m.data),10)
        self.process=None
        self.ident=0

    def tearDown(self):
        if self.process:
            self.process.terminate()
            try: self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.process.kill(); self.process.wait()
        self.node.destroy_node(); rclpy.shutdown()

    def start(self, enabled=True):
        args=[BINARY,'--ros-args','-p','enable_tracking:='+str(enabled).lower(),
              '-p','footprint_xy:=[-0.1,-0.1,0.1,-0.1,0.1,0.1,-0.1,0.1]']
        for source,target in [('/nav/timed_trajectory','trajectory'),('/nav/trajectory_environment','environment'),
            ('/odom','odom'),('/localization/healthy','health'),('/nav/motion_enable','permission'),
            ('/nav/trajectory_cmd_vel','output'),('/nav/trajectory_status','status')]:
            args+=['-r',source+':='+PREFIX+'/'+target]
        self.process=subprocess.Popen(args)
        deadline=time.monotonic()+5
        while (not all(p.get_subscription_count() for p in self.pub.values()) or
               self.sub.get_publisher_count()==0) and time.monotonic()<deadline:
            rclpy.spin_once(self.node,timeout_sec=.05)
        self.assertIsNone(self.process.poll())
        self.assertTrue(all(p.get_subscription_count() for p in self.pub.values()))
        # DDS discovery is per endpoint. Merely discovering the trajectory
        # subscriber does not prove odom/environment/permission reached the node.
        # Wait for the node's real ready-and-idle acknowledgement, retaining the
        # production 0.25 s watchdog and all rejection assertions unchanged.
        if enabled:
            deadline=time.monotonic()+5
            while self.status!='tracking_status=0' and time.monotonic()<deadline:self.exercise(.1)
            self.assertEqual(self.status,'tracking_status=0','Tracker did not acknowledge ready inputs')
        else:self.exercise(.3)

    def exercise(self,duration=.3,*,environment=True,version='e',obstacle=False,
                 permission=True,odom_frame='odom',bad_stamp=False):
        self.outputs.clear()
        end=time.monotonic()+duration
        while time.monotonic()<end:
            stamp=self.node.get_clock().now().to_msg()
            self.pub['health'].publish(Bool(data=True))
            self.pub['permission'].publish(Bool(data=permission))
            odom=Odometry(); odom.header.stamp=stamp; odom.header.frame_id=odom_frame
            odom.child_frame_id='base_footprint'; odom.pose.pose.position.x=2.; odom.pose.pose.position.y=2.
            odom.pose.pose.orientation.w=1.
            if bad_stamp: odom.header.stamp=type(stamp)(sec=0,nanosec=0)
            self.pub['odom'].publish(odom)
            if environment:
                env=TrajectoryEnvironment(); env.map_bundle_version='m'; env.environment_version=version
                env.grid.header.stamp=stamp; env.grid.header.frame_id='odom'
                env.grid.info.width=env.grid.info.height=100; env.grid.info.resolution=.05
                env.grid.info.origin.orientation.w=1.; env.grid.data=[0]*10000
                if obstacle: env.grid.data[40*100+40]=100
                self.pub['environment'].publish(env)
            rclpy.spin_once(self.node,timeout_sec=.02)
        self.assertIsNone(self.process.poll())

    def send_trajectory(self,*,ident=None,version='e',frame='odom'):
        if ident is None: self.ident+=1; ident=self.ident
        msg=TimedTrajectory(); msg.header.stamp=self.node.get_clock().now().to_msg()
        msg.header.frame_id=frame; msg.trajectory_id=ident
        msg.map_bundle_version='m'; msg.environment_version=version
        msg.valid_until.sec=msg.header.stamp.sec+3; msg.valid_until.nanosec=msg.header.stamp.nanosec
        first=TrajectoryPoint(); first.pose.x=first.pose.y=2.; first.velocity.x=.2
        final=TrajectoryPoint(); final.t_from_start_s=2.; final.pose.x=2.4; final.pose.y=2.
        msg.points=[first,final]
        self.pub['trajectory'].publish(msg)
        return msg

    def assert_moving(self):
        self.assertTrue(any(m.twist.linear.x>.05 for m in self.outputs), 'No tracking output')
        for msg in self.outputs:
            self.assertEqual(msg.header.frame_id,'base_footprint')
            self.assertLessEqual(abs(msg.twist.linear.x),.500001)
            self.assertLessEqual(abs(msg.twist.angular.z),1.000001)

    def assert_stopped(self):
        self.assertGreater(len(self.outputs),2)
        for m in self.outputs[-3:]:
            self.assertEqual(m.twist.linear.x,0.)
            self.assertEqual(m.twist.linear.y,0.)
            self.assertEqual(m.twist.angular.z,0.)

    def test_tracking_recovery_and_obstacles(self):
        self.start(); self.send_trajectory(); self.exercise(); self.assert_moving()
        self.exercise(version='changed'); self.assert_stopped()
        self.exercise(); self.assert_stopped() # old trajectory is not replayed
        self.send_trajectory(ident=1); self.exercise(); self.assert_stopped()
        self.send_trajectory(); self.exercise(); self.assert_moving()
        self.exercise(obstacle=True); self.assert_stopped()
        self.exercise(); self.assert_stopped()
        self.send_trajectory(); self.exercise(); self.assert_moving()
        self.exercise(permission=False); self.assert_stopped()
        delayed=self.send_trajectory()
        self.exercise(permission=False); self.assert_stopped()
        self.exercise(); self.assert_stopped()
        self.pub['trajectory'].publish(delayed)
        self.exercise(); self.assert_stopped()
        self.send_trajectory(); self.exercise(); self.assert_moving()
        self.exercise(.5,environment=False); self.assert_stopped()

    def test_disabled_and_invalid_state(self):
        self.start(enabled=False); self.send_trajectory(); self.exercise(); self.assert_stopped()

    def test_wrong_frame_stale_pose_and_invalid_replacement(self):
        self.start(); self.send_trajectory(); self.exercise(); self.assert_moving()
        self.exercise(odom_frame='map'); self.assert_stopped()
        self.exercise(); self.send_trajectory(); self.exercise(); self.assert_moving()
        self.exercise(bad_stamp=True); self.assert_stopped()
        self.exercise(); self.send_trajectory(); self.exercise(); self.assert_moving()
        self.send_trajectory(frame='map'); self.exercise(); self.assert_stopped()

if __name__=='__main__': unittest.main(argv=[sys.argv[0]],verbosity=2)
