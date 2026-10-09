#!/usr/bin/env python3
"""Actual encoder TF, resolver and robot_localization with explicit synthetic pose fixtures."""
import copy
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Bool
from tf2_msgs.msg import TFMessage


def require(value,reason):
    if not value: raise RuntimeError(reason)


def rotation_yaw(angle):
    c,s=math.cos(angle),math.sin(angle)
    return np.array([[c,-s,0],[s,c,0],[0,0,1.]])


def quaternion(matrix):
    # Fixture rotations stay near identity, so trace branch is well conditioned.
    w=math.sqrt(1+np.trace(matrix))/2
    return [(matrix[2,1]-matrix[1,2])/(4*w),(matrix[0,2]-matrix[2,0])/(4*w),
            (matrix[1,0]-matrix[0,1])/(4*w),w]


def main():
    require(os.environ.get('ROS_DOMAIN_ID') not in (None,'','0'),'Use isolated Domain')
    rclpy.init()
    node=Node('local_state_fixture')
    encoder=node.create_publisher(JointState,'/hardware/gimbal_joint_states',qos_profile_sensor_data)
    pose=node.create_publisher(Odometry,'/lio/sensor_odometry',qos_profile_sensor_data)
    resolved,filtered,transforms,nav_odom=[],[],[],[]
    health={'lio':False,'encoder':False,'chassis':False}
    node.create_subscription(PoseWithCovarianceStamped,'/state/lio_pose',resolved.append,10)
    node.create_subscription(Odometry,'/state/chassis',filtered.append,10)
    node.create_subscription(Odometry,'/odom',nav_odom.append,10)
    node.create_subscription(TFMessage,'/tf',transforms.append,10)
    for kind,topic in [('lio','/state/lio_healthy'),('encoder','/state/gimbal_healthy'),('chassis','/state/chassis_healthy')]:
        node.create_subscription(Bool,topic,lambda msg,k=kind:health.update({k:msg.data}),10)
    Path('log').mkdir(exist_ok=True)
    identity=[0.,0.,0.,1.]
    def extrinsic(t):return {'translation':t,'quaternion_xyzw':identity}
    bundle={'bundle_id':'synthetic-state-fixture','status':'verified',
            'frames':{'base_footprint':'base_footprint','chassis':'chassis','big_gimbal_yaw':'big_gimbal_yaw',
                      'imu':'front_mid360_imu','lidar':'front_mid360'},
            'encoder':{'joint':'big_gimbal_yaw','sign':1.,'zero_offset':0.},
            'transforms':{'footprint_to_chassis':extrinsic([0.,0.,.1]),
                          'chassis_to_yaw_zero':extrinsic([.2,0.,.4]),
                          'yaw_to_imu':extrinsic([.6,.1,.2]),'imu_to_lidar':extrinsic([.01,0.,0.])}}
    bundle_path=Path('log/local_state_fixture_calibration.yaml')
    bundle_path.write_text(yaml.safe_dump(bundle))
    profile=Path('/home/asus/nav_deps/src/small_point_lio/config/mid360.yaml')
    c,s=math.cos(.2),math.sin(.2)
    world_R_body=rotation_yaw(.4)@np.array([[c,0,s],[0,1,0],[-s,0,c]])
    body_position=np.array([2.,-1.,.3])
    origin=time.monotonic()
    def yaw(t):return .7*math.sin(2*t)
    def stamp(message,ns):
        message.sec,message.nanosec=divmod(ns,10**9)
    enabled={'encoder':True,'pose':True,'mode':'normal'}
    def tick():
        current=node.get_clock().now().nanoseconds
        t=time.monotonic()-origin
        if enabled['encoder']:
            joint=JointState()
            stamp(joint.header.stamp,current)
            joint.header.frame_id='chassis'
            joint.name,joint.position=['big_gimbal_yaw'],[yaw(t)]
            encoder.publish(joint)
        if enabled['pose']:
            sample=current-30_000_000
            angle=yaw(t-.03)
            body_R_sensor=rotation_yaw(angle)
            body_T_sensor=np.array([.2,0.,.5])+body_R_sensor@np.array([.6,.1,.2])
            world_R_sensor=world_R_body@body_R_sensor
            world_position=body_position+world_R_body@body_T_sensor
            msg=Odometry()
            stamp(msg.header.stamp,sample)
            msg.header.frame_id,msg.child_frame_id='odom','front_mid360_imu'
            p,q=msg.pose.pose.position,msg.pose.pose.orientation
            p.x,p.y,p.z=world_position.tolist()
            q.x,q.y,q.z,q.w=quaternion(world_R_sensor)
            for i in range(6):msg.pose.covariance[i*7]=.001
            if enabled['mode']=='stale':msg.header.stamp.sec-=2
            elif enabled['mode']=='wrong_frame':msg.child_frame_id='base_link'
            elif enabled['mode']=='nan':p.x=float('nan')
            elif enabled['mode']=='zero_cov':msg.pose.covariance=[0.]*36
            elif enabled['mode']=='indefinite_cov':msg.pose.covariance[0]=-.1
            pose.publish(msg)
    timer=node.create_timer(.01,tick)
    process=None
    def wait(predicate,timeout):
        until=time.monotonic()+timeout
        while time.monotonic()<until:
            rclpy.spin_once(node,timeout_sec=.01)
            if predicate():return True
        return False
    try:
        rejected=subprocess.run(['ros2','launch','rm_nav_bringup','local_state.launch.py',
            'calibration_file:=src/rm_nav_frames/config/local_state_calibration.template.yaml',
            'lio_params_file:='+str(profile),'enable_lio:=false'],
            stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=10)
        Path('log/local_state_draft_rejection.log').write_text(rejected.stdout)
        require(rejected.returncode!=0 and 'Measured CalibrationBundle with verified status required' in rejected.stdout,
                'Draft calibration was not rejected before starting state nodes')
        with Path('log/local_state_smoke.log').open('w') as log:
            process=subprocess.Popen(['ros2','launch','rm_nav_bringup','local_state.launch.py',
                'calibration_file:='+str(bundle_path),'lio_params_file:='+str(profile),'enable_lio:=false'],
                stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            require(wait(lambda:len(resolved)>30 and len(filtered)>30 and health['lio'],12),'State chain did not become ready')
            require(wait(lambda:len(nav_odom)>10 and health['chassis'],3),'Nav2 state reference bridge did not become healthy')
            require(wait(lambda:False,1.0) is False,'Unexpected fixture wait')
            for out in resolved[-30:]:
                p=out.pose.pose.position
                require(np.linalg.norm(np.array([p.x,p.y,p.z])-body_position)<.002,
                        'Yaw rotation/lever arm falsely moved chassis; SE3 transform wrong')
                q=out.pose.pose.orientation
                expected=np.array(quaternion(world_R_body))
                require(np.linalg.norm(np.array([q.x,q.y,q.z,q.w])-expected)<.002,'Resolver lost pitch/yaw orientation')
            out=filtered[-1]
            p=out.pose.pose.position
            require(out.header.frame_id=='odom' and out.child_frame_id=='base_footprint','EKF frame ownership wrong')
            require(np.linalg.norm(np.array([p.x,p.y,p.z])-body_position)<.03,'Actual EKF did not track resolved pose')
            p=nav_odom[-1].pose.pose.position
            require(nav_odom[-1].child_frame_id=='base_link' and np.linalg.norm(
                    np.array([p.x,p.y,p.z])-(body_position+world_R_body@np.array([0.,0.,.1])))<.03,
                    'Nav2 odometry used footprint reference instead of chassis reference')
            covariance=np.array(resolved[-1].pose.covariance).reshape(6,6)
            require(np.linalg.eigvalsh(covariance).min()>0 and np.linalg.norm(covariance[:3,3:])>1e-5,
                    'Lever-arm pose covariance was not propagated')
            owners={tf.child_frame_id for group in transforms for tf in group.transforms}
            require(owners=={'big_gimbal_yaw','base_footprint'},'Unexpected dynamic TF authority '+str(owners))
            for mode in ['stale','wrong_frame','nan','zero_cov','indefinite_cov']:
                enabled['mode']=mode
                require(wait(lambda:not health['lio'],.5),'Illegal pose kept resolver healthy: '+mode)
                wait(lambda:False,.1)
                count=len(resolved)
                wait(lambda:False,.1)
                require(len(resolved)==count,'Illegal pose reached EKF: '+mode)
                enabled['mode']='normal'
                require(wait(lambda:health['lio'],.6),'Valid stream did not recover: '+mode)
            enabled['encoder']=False
            require(wait(lambda:not health['encoder'] and not health['lio'],.5),'Encoder loss did not invalidate resolver')
            require(wait(lambda:not health['chassis'],.3),'Encoder loss did not invalidate Nav2 state')
            wait(lambda:False,.1)
            count=len(resolved)
            wait(lambda:False,.2)
            require(len(resolved)==count,'Latest-time/static yaw fallback released pose')
            count=len(nav_odom)
            before_prediction=len(filtered)
            wait(lambda:False,.25)
            require(len(filtered)>before_prediction and len(nav_odom)==count,
                    'Fresh EKF predictions hid encoder loss or continued Nav2 odometry')
            enabled['encoder']=True
            require(wait(lambda:health['lio'],.8),'Encoder stream did not recover time coverage')
            enabled['pose']=False
            require(wait(lambda:not health['lio'],.5),'Raw pose loss did not invalidate resolver')
            require(wait(lambda:not health['chassis'],.3),'Raw pose loss did not invalidate Nav2 state')
            print('PASS real encoder TF/interpolation + SE3 resolver + robot_localization; '
                  'rotating-offset sensor preserves fixed chassis, full pose/covariance; '
                  'bad frame/age/NaN/covariance and encoder/pose loss refused; sole EKF odom TF',flush=True)
    finally:
        node.destroy_timer(timer)
        if process is not None:
            os.killpg(process.pid,signal.SIGINT)
            try:process.wait(timeout=5)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
