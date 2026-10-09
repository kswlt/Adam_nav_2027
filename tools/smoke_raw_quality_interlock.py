#!/usr/bin/env python3
"""Explicit synthetic pose/quality fixture tests resolver interlock, not hardware."""
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped,PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool


def main():
    if os.environ.get('ROS_DOMAIN_ID') in (None,'','0'):raise RuntimeError('Isolated domain required')
    rclpy.init();node=Node('synthetic_raw_quality_interlock_fixture')
    quality=node.create_publisher(Bool,'/sensors/front_mid360/cloud_healthy',10)
    poses=node.create_publisher(Odometry,'/lio/sensor_odometry',qos_profile_sensor_data)
    outputs=[];health=[];phase={'quality':None}
    node.create_subscription(PoseWithCovarianceStamped,'/state/lio_pose',outputs.append,10)
    node.create_subscription(Bool,'/state/lio_healthy',lambda m:health.append(m.data),10)
    broadcaster=StaticTransformBroadcaster(node)
    tf=TransformStamped();tf.header.stamp=node.get_clock().now().to_msg()
    tf.header.frame_id='base_footprint';tf.child_frame_id='front_mid360_imu';tf.transform.rotation.w=1.
    broadcaster.sendTransform(tf)
    def tick():
        if phase['quality'] is not None:
            h=Bool();h.data=phase['quality'];quality.publish(h)
        m=Odometry();m.header.stamp=node.get_clock().now().to_msg()
        m.header.frame_id='odom';m.child_frame_id='front_mid360_imu';m.pose.pose.orientation.w=1.
        for i in range(6):m.pose.covariance[7*i]=.001
        poses.publish(m)
    node.create_timer(.025,tick)
    def spin(seconds):
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.01)
    def require(test,reason):
        if not test:raise RuntimeError(reason)
    Path('log').mkdir(exist_ok=True);checks=[];proc=None
    try:
        with Path('log/raw_quality_interlock.log').open('w') as f:
            proc=subprocess.Popen(['ros2','run','rm_nav_localization','sensor_pose_resolver','--ros-args',
                '-p','calibration_id:=explicit-synthetic-interlock-fixture',
                '-p','body_frame:=base_footprint','-p','require_encoder_health:=false',
                '-p','require_raw_cloud_health:=true'],stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
            spin(2)
            require(not outputs and health and not any(health),'missing quality permitted raw pose')
            checks.append('missing quality rejects continuously fresh valid pose')
            phase['quality']=True;health.clear();spin(1)
            require(len(outputs)>10 and any(health),'fresh accepted quality did not permit pose')
            checks.append('fresh accepted quality permits pose')
            phase['quality']=False;spin(.3);n=len(outputs);health.clear();spin(.5)
            require(len(outputs)==n and health and not any(health),'quality rejection did not block pose')
            checks.append('rejected quality blocks fresh pose and health')
            phase['quality']=True;health.clear();n=len(outputs);spin(.7)
            require(len(outputs)>n+10 and any(health),'quality recovery did not permit fresh pose')
            checks.append('quality recovery requires and accepts new pose')
            phase['quality']=None;spin(.4);n=len(outputs);health.clear();spin(.5)
            require(len(outputs)==n and health and not any(health),'quality heartbeat dropout did not block pose')
            checks.append('quality heartbeat timeout blocks continuously fresh pose')
            require(proc.poll() is None,'resolver exited unexpectedly')
    finally:
        if proc and proc.poll() is None:
            os.killpg(proc.pid,signal.SIGINT)
            try:proc.wait(timeout=10)
            except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
        node.destroy_node();rclpy.shutdown()
    report={'passed':True,'scope':'explicit synthetic pose and quality heartbeat; no hardware calibration or actuation',
            'checks':checks,'accepted_pose_count':len(outputs)}
    Path('log/raw_quality_interlock_report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
