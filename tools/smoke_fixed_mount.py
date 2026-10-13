#!/usr/bin/env python3
"""TEST_ONLY synthetic raw poses, actual fixed/legacy TF, resolver and EKF."""
import argparse
import os
from pathlib import Path
import signal
import subprocess
import time
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from tf2_msgs.msg import TFMessage
from smoke_local_state import quaternion,rotation_yaw,require

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--legacy',action='store_true');args=parser.parse_args()
    require(os.environ.get('ROS_DOMAIN_ID') not in (None,'','0'),'Isolated test domain required')
    root=Path(__file__).resolve().parents[1]
    file=root/'src/rm_nav_frames/config'/('local_state_calibration.legacy_adam_static.yaml' if args.legacy else 'vehicle_164_fixed_mid360.yaml')
    if args.legacy:
        offset=np.array([.16,0.,.18]);rotation=np.diag([1.,-1.,-1.])
    else:
        bundle=yaml.safe_load(file.read_text());offset=np.array(bundle['transforms']['chassis_to_imu']['translation']);rotation=np.eye(3)
    rclpy.init();node=Node('test_only_fixed_mount');messages=[];dynamic=[];static=[];health=[False];encoder=[]
    node.create_subscription(Odometry,'/odom',messages.append,10)
    node.create_subscription(TFMessage,'/tf',dynamic.append,20)
    node.create_subscription(TFMessage,'/tf_static',static.append,QoSProfile(depth=20,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(Bool,'/state/chassis_healthy',lambda m:health.__setitem__(0,m.data),10)
    node.create_subscription(Bool,'/state/gimbal_healthy',encoder.append,10)
    pub=node.create_publisher(Odometry,'/lio/sensor_odometry',qos_profile_sensor_data)
    c,s=np.cos(.2),np.sin(.2);world_rotation=rotation_yaw(.4)@np.array([[c,0,s],[0,1,0],[-s,0,c]])
    position=np.array([1.,2.,.3]);enabled=[True]
    def tick():
        if not enabled[0]:return
        m=Odometry();m.header.frame_id='odom';m.child_frame_id='front_mid360_imu';m.header.stamp=node.get_clock().now().to_msg()
        p=position+world_rotation@offset;m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z=p.tolist()
        q=m.pose.pose.orientation;q.x,q.y,q.z,q.w=quaternion(world_rotation@rotation)
        for i in range(6):m.pose.covariance[i*7]=.001
        pub.publish(m)
    timer=node.create_timer(.02,tick)
    def wait(predicate,seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.01)
            if predicate():return True
        return False
    process=None;log=root/'log'/('legacy_projection_smoke.log' if args.legacy else 'fixed_mount_smoke.log');log.parent.mkdir(exist_ok=True)
    try:
        with log.open('w') as handle:
            process=subprocess.Popen(['ros2','launch','rm_nav_bringup','local_state.launch.py','calibration_file:='+str(file),
                'lio_params_file:=TEST_ONLY_unused','enable_lio:=false',
                'allow_legacy_static_calibration:='+str(args.legacy).lower(),'allow_user_mount:='+str(not args.legacy).lower()],
                stdout=handle,stderr=subprocess.STDOUT,start_new_session=True)
            require(wait(lambda:health[0] and len(messages)>30,12),'Fixed/legacy chain did not become healthy')
            p=messages[-1].pose.pose.position
            require(np.linalg.norm(np.array([p.x,p.y,p.z])-position)<.02,'Mount extrinsic applied twice or inverted')
            require(not encoder,'Fixed sensor fabricated encoder health')
            dyn={(t.header.frame_id,t.child_frame_id) for group in dynamic for t in group.transforms}
            stat={(t.header.frame_id,t.child_frame_id) for group in static for t in group.transforms}
            require(dyn=={('odom','base_footprint'),('base_footprint','chassis')},'Projection authority differs')
            require(not any(child=='chassis' for _,child in stat) and ('chassis','base_link') in stat,'Static/dynamic chassis conflict or wrong alias parent')
            enabled[0]=False
            require(wait(lambda:not health[0],.7),'Raw pose dropout hidden by EKF predictions')
            print('PASS TEST_ONLY '+('legacy' if args.legacy else 'fixed mount')+': actual SE3 resolver/EKF, one chassis authority, correct mount, no fake encoder, dropout hold')
    finally:
        node.destroy_timer(timer)
        if process:
            os.killpg(process.pid,signal.SIGINT)
            try:process.wait(timeout=5)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
        node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
