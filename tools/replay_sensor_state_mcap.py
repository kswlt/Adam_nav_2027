#!/usr/bin/env python3
"""Replay real raw cloud and recorded LIO through the repaired state launch.

Private DDS domain, simulated clock, no control commands, no physical devices.
Recorded TF/state/health are excluded; cloud guard derives health from real scans.
This validates the state adapter, not a fresh run of the small_point_lio frontend.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import yaml

import rclpy
import rosbag2_py
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--vehicle164-axis',action='store_true',help='Vehicle164 configured axis profile')
    args=parser.parse_args()
    lever=(0.,0.,0.)
    if args.vehicle164_axis:
        profile=Path(__file__).parents[1]/'src/rm_nav_bringup/config/vehicle_164_geometry.yaml'
        lever=yaml.safe_load(profile.read_text())['mount']['axis_to_lidar_at_reference_height']
    os.environ.setdefault('ROS_DOMAIN_ID','209')
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE']='LOCALHOST'
    reader=rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(args.bag),storage_id='mcap'),
                rosbag2_py.ConverterOptions('',''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/sensor_reference/odom','/lio/sensor_odometry']))
    expected={}
    rotations={}
    while reader.has_next():
        topic,data,_=reader.read_next()
        value=deserialize_message(data,Odometry)
        stamp=value.header.stamp.sec*10**9+value.header.stamp.nanosec
        if topic=='/lio/sensor_odometry':
            q=value.pose.pose.orientation
            a,b,c=lever
            rotations[stamp]=((1-2*(q.y*q.y+q.z*q.z))*a+2*(q.x*q.y-q.z*q.w)*b+2*(q.x*q.z+q.y*q.w)*c,
                              2*(q.x*q.y+q.z*q.w)*a+(1-2*(q.x*q.x+q.z*q.z))*b+2*(q.y*q.z-q.x*q.w)*c)
        else:
            expected[stamp]=(value.pose.pose.position.x,value.pose.pose.position.y)
    if args.vehicle164_axis:
        expected={t:(point[0]-rotations[t][0],point[1]-rotations[t][1])
                  for t,point in expected.items() if t in rotations}
    if not expected:raise ValueError('Bag contains no recorded reference for comparison')
    rclpy.init()
    node=Node('recorded_sensor_state_audit',parameter_overrides=[Parameter('use_sim_time',value=True)])
    measured={}
    health=[]
    def observe(value):
        stamp=value.header.stamp.sec*10**9+value.header.stamp.nanosec
        measured[stamp]=(value.pose.pose.position.x,value.pose.pose.position.y)
    node.create_subscription(Odometry,'/sensor_reference/odom',observe,100)
    node.create_subscription(Bool,'/state/sensor_reference_healthy',lambda m:health.append(m.data),10)
    args.output.mkdir(parents=True,exist_ok=True)
    processes,logs=[],[]
    def start(name,command):
        log=(args.output/(name+'.log')).open('w')
        logs.append(log)
        process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        processes.append(process)
        return process
    try:
        launch='vehicle_164_sensor_reference_state.launch.py' if args.vehicle164_axis else 'sensor_reference_state.launch.py'
        state=start('state',['ros2','launch','rm_nav_bringup',launch,
            'enable_lio:=false','use_sim_time:=true','lio_params_file:=/unused_lio_disabled.yaml'])
        until=time.monotonic()+3
        while time.monotonic()<until:
            assert state.poll() is None,'state launch failed'
            rclpy.spin_once(node,timeout_sec=.05)
        playback=start('playback',['ros2','bag','play',str(args.bag),'--clock','100','--rate','2',
            '--read-ahead-queue-size','100','--disable-keyboard-controls',
            '--topics','/livox/lidar','/livox/imu','/lio/sensor_odometry','/lio/deskewed_odom_cloud'])
        deadline=time.monotonic()+100
        while playback.poll() is None and time.monotonic()<deadline:
            assert state.poll() is None,'state launch exited during playback'
            rclpy.spin_once(node,timeout_sec=.01)
        assert playback.poll()==0,('playback failed/timed out',playback.poll())
        until=time.monotonic()+.5
        while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.01)
        common=set(expected)&set(measured)
        assert len(common)>.8*len(expected),('too few accepted real samples',len(common),len(expected))
        error=max(max(abs(a-b) for a,b in zip(expected[t],measured[t])) for t in common)
        assert error<1e-6,('default reference changed unexpectedly',error)
        assert any(health),'real pointcloud guard never allowed state'
        assert node.count_publishers('/cmd_vel')==node.count_publishers('/cmd_yaw_angle')==0
        result={'result':'PASS','scope':'real MCAP state-adapter replay, not frontend rerun',
                'reference_mode':'vehicle164_configured_axis' if args.vehicle164_axis else 'recorded_lidar_center',
                'axis_to_lidar_m':lever,
                'recorded_samples':len(expected),'matched_samples':len(common),
                'max_xy_difference_m':error,'real_cloud_health_observed':True,'physical_output':False}
        points=[measured[t] for t in sorted(common)]
        result['horizontal_max_from_first_matched_m']=max(math.dist(p,points[0]) for p in points)
        result['horizontal_end_from_first_matched_m']=math.dist(points[-1],points[0])
        (args.output/'result.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result),flush=True)
    finally:
        for process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid,signal.SIGINT)
                try:process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid,signal.SIGKILL)
                    process.wait()
        for log in logs:log.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':main()
