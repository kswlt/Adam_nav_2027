#!/usr/bin/env python3
"""Read-only live timing capture; publishes no messages and changes no thresholds."""
import argparse
from collections import deque
import json
from pathlib import Path
import time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import Bool, String


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--seconds',type=float,default=180)
    args=parser.parse_args()
    if not 5 <= args.seconds <= 600: raise ValueError('duration must be 5..600 seconds')
    if args.output.exists(): raise ValueError('Choose a new evidence path')
    rclpy.init();node=rclpy.create_node('read_only_input_latency_capture')
    history=deque(maxlen=12000);stats={};anomalies=[];trigger=None;armed=False;previous={};subs=[]
    def receive(topic):
        def callback(message, info):
            mono=time.monotonic();now=node.get_clock().now().nanoseconds
            stamp=message.header.stamp.sec*10**9+message.header.stamp.nanosec
            row={'topic':topic,'mono_s':mono,'callback_ros_ns':now,'header_ns':stamp,
                 'dds_source_ns':info['source_timestamp'],'dds_received_ns':info['received_timestamp'],
                 'age_s':(now-stamp)*1e-9,'callback_gap_s':mono-previous.get(topic,mono)}
            previous[topic]=mono;history.append(row)
            st=stats.setdefault(topic,{'count':0,'max_age_s':-1.,'max_callback_gap_s':0.})
            st['count']+=1;st['max_age_s']=max(st['max_age_s'],row['age_s'])
            st['max_callback_gap_s']=max(st['max_callback_gap_s'],row['callback_gap_s'])
            if row['age_s']>.2 or row['callback_gap_s']>.2:
                if len(anomalies)<200:anomalies.append(row)
        return callback
    for topic,kind in [('/livox/imu',Imu),('/livox/lidar',PointCloud2),('/sensors/front_mid360/guarded_points',PointCloud2)]:
        subs.append(node.create_subscription(kind,topic,receive(topic),qos_profile_sensor_data))
    def health(m):
        nonlocal armed
        if m.data:armed=True
    def reason(m):
        nonlocal trigger
        if armed and 'restart state' in m.data and trigger is None:
            trigger={'reason':m.data,'mono_s':time.monotonic(),'history':list(history)}
            print('Input fault captured; preserving 5 seconds after trigger',flush=True)
    subs.append(node.create_subscription(Bool,'/lio/input_healthy',health,10))
    subs.append(node.create_subscription(String,'/lio/input_reason',reason,10))
    start=time.monotonic();last_print=start
    try:
        while time.monotonic()-start<args.seconds:
            rclpy.spin_once(node,timeout_sec=.02)
            now=time.monotonic()
            if trigger and now-trigger['mono_s']>=5:break
            if now-last_print>=25:
                print(json.dumps({'elapsed_s':now-start,'armed':armed,'stats':stats}),flush=True);last_print=now
    finally:
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps({'scope':'read_only_no_timestamp_rewrite','duration_s':time.monotonic()-start,
            'guard_healthy_observed':armed,'stats':stats,'anomalies':anomalies,'trigger':trigger,'tail':list(history)},indent=2))
        print('Evidence saved: '+str(args.output),flush=True)
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
