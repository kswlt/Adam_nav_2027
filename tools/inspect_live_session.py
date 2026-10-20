#!/usr/bin/env python3
"""Read-only live sensor/state probe. Never publishes TF, health or commands."""
import argparse
from collections import defaultdict
import json
import math
from pathlib import Path
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
from sensor_msgs.msg import Imu,PointCloud2
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool,String
from tf2_msgs.msg import TFMessage

def main():
    p=argparse.ArgumentParser();p.add_argument('--duration',type=float,default=30);p.add_argument('--phase',choices=['raw','state'],default='state');p.add_argument('--output',required=True);a=p.parse_args()
    if not math.isfinite(a.duration) or not 5<=a.duration<=600:raise ValueError('Duration must be 5..600 s')
    rclpy.init();node=Node('nav20261020_live_probe');count=defaultdict(int);ages=defaultdict(list);health={};reasons={};parents=defaultdict(set)
    imu_norm=[];positions=[];layout={};points={};endpoints={};received_at={};health_at={}
    def sample(topic,m):
        count[topic]+=1
        received_at[topic]=time.monotonic()
        if hasattr(m,'header'):
            stamp=m.header.stamp.sec*10**9+m.header.stamp.nanosec
            ages[topic].append((node.get_clock().now().nanoseconds-stamp)/1e9)
        if isinstance(m,PointCloud2):
            points[topic]=m.width*m.height
            if topic=='/livox/lidar' and not layout:
                layout.update({'frame':m.header.frame_id,'point_step':m.point_step,'fields':[[f.name,f.offset,f.datatype,f.count] for f in m.fields]})
        elif isinstance(m,Imu):
            v=m.linear_acceleration;imu_norm.append(math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z))
        elif topic=='/odom':
            v=m.pose.pose.position;positions.append([v.x,v.y,v.z])
    topics=[('/livox/lidar',PointCloud2),('/livox/imu',Imu),('/lio/sensor_odometry',Odometry),('/lio/deskewed_odom_cloud',PointCloud2),
        ('/state/chassis',Odometry),('/odom',Odometry),('/localization/odom_submap',PointCloud2),('/visualization/live_map_preview',PointCloud2)]
    for name,kind in topics:node.create_subscription(kind,name,lambda m,t=name:sample(t,m),qos_profile_sensor_data)
    def health_message(topic,m):
        health[topic]=m.data;health_at[topic]=time.monotonic()
    for name in ['/sensors/front_mid360/cloud_healthy','/state/lio_healthy','/state/chassis_healthy','/sensors/localization_healthy']:
        node.create_subscription(Bool,name,lambda m,t=name:health_message(t,m),1)
    for name in ['/sensors/front_mid360/cloud_reason','/state/lio_reason','/state/chassis_reason','/localization/submap_reason']:
        node.create_subscription(String,name,lambda m,t=name:reasons.update({t:m.data}),1)
    def transforms(m):
        for t in m.transforms:parents[t.child_frame_id].add(t.header.frame_id)
    node.create_subscription(TFMessage,'/tf',transforms,30)
    node.create_subscription(TFMessage,'/tf_static',transforms,QoSProfile(depth=30,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    started=time.monotonic()
    while time.monotonic()-started<a.duration:rclpy.spin_once(node,timeout_sec=.02)
    for topic,_ in topics:
        endpoints[topic]=[{'node':e.node_name,'namespace':e.node_namespace} for e in node.get_publishers_info_by_topic(topic)]
    at=time.monotonic()
    raw=(count['/livox/lidar']>=10 and count['/livox/imu']>=100 and
         at-received_at.get('/livox/lidar',0)<=.5 and at-received_at.get('/livox/imu',0)<=.25)
    state=(count['/odom']>=10 and all(health.get(k,False) for k in ['/sensors/front_mid360/cloud_healthy','/state/lio_healthy','/state/chassis_healthy','/sensors/localization_healthy']) and
        at-received_at.get('/odom',0)<=.25 and all(at-t<=.25 for t in health_at.values()) and
        parents['base_footprint']=={'odom'} and parents['chassis']=={'base_footprint'} and all(len(v)==1 for v in parents.values()))
    report={'scope':'real live read-only '+a.phase+' probe; no motion commands or ground truth','passed':bool(raw and (a.phase=='raw' or state)),
        'duration_s':time.monotonic()-started,'counts':dict(count),'last_points':points,'last_health':health,'reasons':reasons,
        'tf_parents':{k:sorted(v) for k,v in parents.items()},'publishers':endpoints,'raw_cloud_layout':layout,
        'final_receive_age_s':{k:at-v for k,v in received_at.items()},
        'final_health_receive_age_s':{k:at-v for k,v in health_at.items()},
        'imu_acc_norm_median':float(np.median(imu_norm)) if imu_norm else None,
        'message_source_age_s':{k:{'min':min(v),'p50':float(np.median(v)),'p95':float(np.quantile(v,.95)),'max':max(v)} for k,v in ages.items()},
        'observed_odom_position_span_m':float(np.max(np.linalg.norm(np.asarray(positions)-positions[0],axis=1))) if positions else None,
        'position_span_is_drift_measurement':False}
    with Path(a.output).open('x') as f:json.dump(report,f,indent=2);f.write('\n')
    print(json.dumps(report,indent=2));node.destroy_node();rclpy.shutdown()
    return 0 if report['passed'] else 1
if __name__=='__main__':raise SystemExit(main())
