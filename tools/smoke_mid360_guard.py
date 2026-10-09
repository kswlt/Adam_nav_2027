#!/usr/bin/env python3
"""Real bag regression for native MID360 quality gate; no physical driver/motion."""
import argparse
from collections import Counter
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import serialize_message
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, String


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--bag',required=True)
    p.add_argument('--expect',choices=['accept','reject'],required=True)
    p.add_argument('--output',required=True)
    args=p.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') in (None,'','0'): raise RuntimeError('Isolated domain required')
    output=Path(args.output).resolve();output.mkdir(parents=True,exist_ok=False)
    bag=Path(args.bag).resolve(strict=True)
    import yaml
    duration=yaml.safe_load((bag/'metadata.yaml').read_text())['rosbag2_bagfile_information']['duration']['nanoseconds']/1e9
    if duration>60:raise ValueError('Bounded smoke bag <=60s required')
    rclpy.init();node=Node('mid360_guard_fixture')
    inputs,outputs=Counter(),Counter();health=[];reasons=Counter()
    def digest(m):
        # CDR serialization may be re-aligned by a ROS publisher; compare the
        # source stamp and raw PointCloud2 payload/field contract instead.
        h=hashlib.sha256();h.update(bytes(m.data));h.update(str(m.header.stamp.sec).encode());
        h.update(str(m.header.stamp.nanosec).encode());h.update(str(m.point_step).encode());
        return h.hexdigest()
    node.create_subscription(PointCloud2,'/livox/lidar',lambda m:inputs.update([digest(m)]),qos_profile_sensor_data)
    node.create_subscription(PointCloud2,'/sensors/front_mid360/guarded_points',lambda m:outputs.update([digest(m)]),qos_profile_sensor_data)
    node.create_subscription(Bool,'/sensors/front_mid360/cloud_healthy',lambda m:health.append(m.data),10)
    node.create_subscription(String,'/sensors/front_mid360/cloud_reason',lambda m:reasons.update([m.data]),10)
    processes=[];handles=[];errors=[]
    def start(cmd,filename):
        f=(output/filename).open('w');handles.append(f)
        proc=subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT,start_new_session=True);processes.append(proc);return proc
    def spin(seconds):
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.01)
    try:
        guard=start(['ros2','run','rm_nav_sensors','mid360_cloud_guard','--ros-args','-p','use_sim_time:=true'],'guard.log')
        spin(2)
        player=start(['ros2','bag','play',str(bag),'--clock','--delay','2'],'player.log')
        deadline=time.monotonic()+duration+20
        while player.poll() is None and time.monotonic()<deadline:
            if guard.poll() is not None:raise RuntimeError('guard exited')
            rclpy.spin_once(node,timeout_sec=.01)
        if player.poll()!=0:raise RuntimeError('player failure/timeout')
        spin(.5)
        stopped_count=sum(outputs.values());health.clear()
        spin(1)
        if sum(outputs.values())!=stopped_count:errors.append('cached scan replay after dropout')
        if not health or any(health):errors.append('health did not remain false after dropout')
        if not inputs:errors.append('no fixture input')
        if any(count>inputs[h] for h,count in outputs.items()):errors.append('output changed/duplicated bytes')
        if args.expect=='reject' and outputs:errors.append('degraded native scans were forwarded')
        if args.expect=='accept' and stopped_count<30:errors.append('healthy bag did not pass sufficient scans')
        if args.expect=='reject' and not any('Insufficient valid returns' in k for k in reasons):
            errors.append('expected geometric rejection diagnostic missing')
    except Exception as exc:errors.append(str(exc))
    finally:
        for proc in reversed(processes):
            if proc.poll() is None:
                os.killpg(proc.pid,signal.SIGINT)
                try:proc.wait(timeout=10)
                except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
        for f in handles:f.close()
        node.destroy_node();rclpy.shutdown()
    report={'passed':not errors,'scope':'real recorded native scans; gate only, no robot actuation',
            'bag':str(bag),'expected':args.expect,'input_count':sum(inputs.values()),
            'forwarded_count':sum(outputs.values()),'all_outputs_byte_identical_inputs':not any(c>inputs[h] for h,c in outputs.items()),
            'dropout_health_false':bool(health) and not any(health),'reason_counts':dict(reasons),'errors':errors}
    (output/'report.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
    return 0 if not errors else 1


if __name__=='__main__':raise SystemExit(main())
