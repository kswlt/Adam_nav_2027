#!/usr/bin/env python3
"""TEST_ONLY: actual GICP accepts a distant field origin only after two matches."""
import os
from pathlib import Path
import random
import signal
import subprocess
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from tf2_ros import StaticTransformBroadcaster
from smoke_frozen_map_localization import Acceptance, cloud, require

def main():
    require(os.environ.get('ROS_DOMAIN_ID') not in (None,'','0'),'Dedicated ROS domain required')
    rclpy.init();node=Acceptance();process=None
    logs=Path('log/initial_pose_smoke.log');logs.parent.mkdir(exist_ok=True)
    try:
        with logs.open('w') as log:
            process=subprocess.Popen(['ros2','launch','rm_nav_bringup','frozen_map_localization.launch.py',
                'map_version:=TEST_ONLY_initial_pose'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            require(node.wait(lambda: node.map.get_subscription_count()>0 and node.healthy is False,15),'Startup failed')
            require(node.tf is None,'Unverified startup TF')
            broadcaster=StaticTransformBroadcaster(node)
            tf=TransformStamped();tf.header.frame_id='odom';tf.child_frame_id='base_footprint';tf.transform.rotation.w=1.
            tf.transform.translation.x=2.;broadcaster.sendTransform(tf)
            pub=node.create_publisher(PoseWithCovarianceStamped,'/initialpose',1)
            require(node.wait(lambda:pub.get_subscription_count()>0,5),'Initial pose subscriber missing')
            node.wait(lambda:False,.3)
            hint=PoseWithCovarianceStamped();hint.header.frame_id='map';hint.header.stamp=node.get_clock().now().to_msg()
            hint.pose.pose.orientation.w=1.;hint.pose.pose.position.x=22.;hint.pose.pose.position.y=-10.
            pub.publish(hint);node.wait(lambda:False,.3)
            require(node.tf is None,'Operator hint incorrectly became verified TF')
            rng=random.Random(2027);points=[tuple(rng.uniform(-2,2) for _ in range(3)) for _ in range(2000)]
            node.map.publish(cloud([(x+20.,y-10.,z) for x,y,z in points],'map',node.get_clock().now().to_msg()))
            node.wait(lambda:False,.3)
            node.submap.publish(cloud(points,'odom',node.get_clock().now().to_msg()))
            require(node.wait(lambda:node.estimate is not None,3),'First real GICP output missing')
            node.wait(lambda:False,.1)
            require(node.tf is None and node.healthy is False,'Single hinted match published TF')
            node.wait(lambda:False,.2)
            node.submap.publish(cloud(points,'odom',node.get_clock().now().to_msg()))
            require(node.wait(lambda:node.healthy is True and node.tf is not None,3),'Two consistent registrations failed')
            t=node.tf.transform.translation
            require(abs(t.x-20)<.01 and abs(t.y+10)<.01,'Initial pose used wrong body/odom reference')
            print('PASS TEST_ONLY exact-time initialpose conversion, no hint TF, two real GICP matches, distant official origin')
    finally:
        if process:
            os.killpg(process.pid,signal.SIGINT)
            try:process.wait(timeout=5)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
        node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
