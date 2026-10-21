#!/usr/bin/env python3
"""Replay real raw cloud and recorded LIO through the repaired state launch.

Private DDS domain, simulated clock, no control commands, no physical devices.
Recorded TF/state/health are excluded; cloud guard derives health from real scans.
This validates the state adapter, not a fresh run of the small_point_lio frontend.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

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
    args=parser.parse_args()
    os.environ.setdefault('ROS_DOMAIN_ID','209')
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE']='LOCALHOST'
    reader=rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(args.bag),storage_id='mcap'),
                rosbag2_py.ConverterOptions('',''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/sensor_reference/odom']))
    expected={}
    while reader.has_next():
        _,data,_=reader.read_next()
        value=deserialize_message(data,Odometry)
        stamp=value.header.stamp.sec*10**9+value.header.stamp.nanosec
        expected[stamp]=(value.pose.pose.position.x,value.pose.pose.position.y)
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
        state=start('state',['ros2','launch','rm_nav_bringup','sensor_reference_state.launch.py',
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
                'recorded_samples':len(expected),'matched_samples':len(common),
                'max_xy_difference_m':error,'real_cloud_health_observed':True,'physical_output':False}
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
