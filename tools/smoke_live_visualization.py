#!/usr/bin/env python3
"""Actual visualization nodes with explicit synthetic odom clouds/poses."""
import json
import os
from pathlib import Path
import signal
import struct
import subprocess
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from nav_msgs.msg import Odometry, Path as RosPath
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String


def main():
    if os.environ.get('ROS_DOMAIN_ID') in (None, '', '0'): raise RuntimeError('Use isolated domain')
    rclpy.init(); node = Node('synthetic_visualization_fixture')
    odom = node.create_publisher(Odometry, '/lio/sensor_odometry', qos_profile_sensor_data)
    cloud = node.create_publisher(PointCloud2, '/lio/deskewed_odom_cloud', qos_profile_sensor_data)
    paths, maps, states = [], [], []
    qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(RosPath, '/visualization/lio_path', paths.append, qos)
    node.create_subscription(PointCloud2, '/visualization/live_map_preview', maps.append, qos)
    node.create_subscription(String, '/visualization/live_map_status', lambda m:states.append(json.loads(m.data)), qos)
    procs, logs = [], []
    def launch(exe, args):
        Path('log').mkdir(exist_ok=True)
        f = Path('log/'+exe+'_fixture.log').open('w'); logs.append(f)
        p = subprocess.Popen(['ros2','run','rm_nav_bringup',exe,*args],stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
        procs.append(p)
    def spin(seconds):
        deadline = time.monotonic()+seconds
        while time.monotonic()<deadline: rclpy.spin_once(node,timeout_sec=.01)
    def require(value, reason):
        if not value: raise RuntimeError(reason)
    try:
        launch('foxglove_trace_publisher',['--max-points','10'])
        launch('live_map_preview',['--max-voxels','10'])
        spin(2)
        for i in range(30):
            m = Odometry(); m.header.frame_id='odom'; m.header.stamp=node.get_clock().now().to_msg()
            m.child_frame_id='front_mid360_imu'; m.pose.pose.position.x=i*.2; m.pose.pose.orientation.w=1.
            odom.publish(m)
            c = PointCloud2(); c.header=m.header; c.height=1; c.width=1; c.point_step=12; c.row_step=12
            c.fields=[PointField(name=n,offset=j*4,datatype=PointField.FLOAT32,count=1) for j,n in enumerate(('x','y','z'))]
            c.data=struct.pack('<fff',i*.2,1.,2.); cloud.publish(c)
            spin(.06)
        spin(.7)
        require(all(p.poll() is None for p in procs),'visualization callback crashed')
        require(paths and len(paths[-1].poses)==10,'bounded multi-message path not generated')
        require(maps and maps[-1].width==10,'bounded accumulation not generated')
        xyz=struct.unpack('<fff',maps[-1].data[-12:])
        require(abs(xyz[0]-5.8)<1e-4 and xyz[1:]==(1.,2.),'odom coordinates transformed twice')
        ns=lambda stamp:stamp.sec*10**9+stamp.nanosec
        require(ns(maps[-1].header.stamp)==ns(c.header.stamp),'preview source timestamp changed')
        old_path_count=len(paths); old_map_count=len(maps)
        odom.publish(m); cloud.publish(c);spin(.7)
        require(len(paths)==old_path_count and len(maps)==old_map_count,'duplicate source time accepted/cached map republished')
        require(states and states[-1]['input_stale'],'retained map marked as live after dropout')
        report={'passed':True,'scope':'explicit synthetic inputs, real ROS visualization nodes',
                'path_poses':len(paths[-1].poses),'preview_points':maps[-1].width,
                'evicted_voxels':states[-1]['evicted_voxels'],
                'checks':['multi-pose callback remains alive','trace/map bounded','native odom coordinates preserved',
                          'source timestamp preserved','duplicate time rejected','dropout stale; no cached restamping']}
        Path('log/live_visualization_report.json').write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(report,indent=2))
    finally:
        for p in reversed(procs):
            if p.poll() is None:
                os.killpg(p.pid,signal.SIGINT)
                try: p.wait(timeout=5)
                except subprocess.TimeoutExpired: os.killpg(p.pid,signal.SIGKILL);p.wait()
        for f in logs:f.close()
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
