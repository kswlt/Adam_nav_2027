#!/usr/bin/env python3
"""Run the actual pinned Point-LIO with explicit synthetic Livox-schema cloud/IMU inputs."""
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2, PointField
from tf2_msgs.msg import TFMessage
import yaml


def require(value,message):
    if not value: raise RuntimeError(message)


def main():
    require(os.environ.get('ROS_DOMAIN_ID') not in (None,'','0'),'Use isolated Domain')
    rclpy.init()
    node = Node('small_point_lio_fixture')
    imu_pub = node.create_publisher(Imu,'/fixture/imu',qos_profile_sensor_data)
    cloud_pub = node.create_publisher(PointCloud2,'/fixture/lidar',qos_profile_sensor_data)
    observations,clouds,transforms = [],[],[]
    node.create_subscription(Odometry,'/lio/sensor_odometry',observations.append,10)
    node.create_subscription(PointCloud2,'/lio/deskewed_odom_cloud',clouds.append,10)
    node.create_subscription(TFMessage,'/tf',transforms.append,10)
    params = yaml.safe_load(Path('/home/asus/nav_deps/src/small_point_lio/config/mid360.yaml').read_text())
    params['small_point_lio']['ros__parameters'].update({
        'lidar_topic':'/fixture/lidar','imu_topic':'/fixture/imu','lidar_type':'livox_pointcloud2',
        'lidar_frame':'front_mid360','state_frame':'front_mid360_imu','odom_frame':'odom','publish_tf':False,
        'extrinsic_T':[0.0,0.0,0.0],'extrinsic_R':[1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0],
        'acc_norm':9.81,'min_distance':0.5,'max_distance':20.0,'save_pcd':False,
    })
    Path('log').mkdir(exist_ok=True)
    config = Path('log/lio_fixture.yaml')
    config.write_text(yaml.safe_dump(params))
    points = []
    # Three mutually orthogonal surfaces give the genuine filter geometric constraints.
    for i in range(30):
        for j in range(30):
            a,b = -3.0+i*0.2,-3.0+j*0.2
            points.extend([(4.0,a,b),(a,4.0,b),(a,b,-2.0)])
    fields = [PointField(name=n,offset=o,datatype=d,count=1) for n,o,d in
              [('x',0,7),('y',4,7),('z',8,7),('tag',12,2),('timestamp',16,8)]]
    started = time.monotonic()
    def spin(duration):
        end = time.monotonic()+duration
        while time.monotonic()<end: rclpy.spin_once(node,timeout_sec=0.01)
    process = None
    try:
        with Path('log/small_point_lio_smoke.log').open('w') as log:
            process = subprocess.Popen(['ros2','run','small_point_lio','small_point_lio_node',
                '--ros-args','--params-file',str(config),'-r','/Odometry:=/lio/sensor_odometry',
                '-r','/cloud_registered:=/lio/deskewed_odom_cloud'],stdout=log,stderr=subprocess.STDOUT,
                start_new_session=True)
            while cloud_pub.get_subscription_count()==0 and time.monotonic()-started<10: spin(0.05)
            require(cloud_pub.get_subscription_count()>0,'Real LIO did not start')
            spin(0.3)
            require(not observations and not clouds,'LIO produced pose without sensor data')
            last_cloud = time.monotonic()
            end = time.monotonic()+7.0
            while time.monotonic()<end:
                imu = Imu()
                imu.header.stamp,imu.header.frame_id = node.get_clock().now().to_msg(),'front_mid360_imu'
                imu.linear_acceleration.z = 9.81
                imu.orientation_covariance[0] = -1.0
                imu_pub.publish(imu)
                if time.monotonic()-last_cloud >= 0.1:
                    last_cloud = time.monotonic()
                    stamp = node.get_clock().now()
                    msg = PointCloud2()
                    msg.header.stamp,msg.header.frame_id = stamp.to_msg(),'front_mid360'
                    msg.height,msg.width,msg.fields,msg.point_step = 1,len(points),fields,24
                    msg.row_step = 24*len(points)
                    msg.data = b''.join(struct.pack('<fffB3xd',x,y,z,
                        0,float(stamp.nanoseconds-100_000_000+i*100_000_000//len(points)))
                        for i,(x,y,z) in enumerate(points))
                    msg.is_dense = True
                    cloud_pub.publish(msg)
                spin(0.005)
                require(process.poll() is None,'Point-LIO crashed; inspect log')
            spin(0.1)
            require(len(observations)>10 and len(clouds)>5,'Actual filter did not produce enough outputs')
            latest = observations[-1]
            require(latest.header.frame_id=='odom' and latest.child_frame_id=='front_mid360_imu','Raw pose frame corrupted')
            p,q = latest.pose.pose.position,latest.pose.pose.orientation
            require(all(math.isfinite(x) for x in [p.x,p.y,p.z,q.x,q.y,q.z,q.w]),'Nonfinite filter pose')
            require(math.sqrt(p.x*p.x+p.y*p.y+p.z*p.z)<0.15,'Stationary synthetic filter drifted')
            require(abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1)<1e-5,'Nonunit raw quaternion')
            require(all(math.isfinite(x) for x in latest.pose.covariance) and
                    all(latest.pose.covariance[i*7]>0 for i in range(6)),'Filter covariance missing')
            require(all(c.header.frame_id=='odom' for c in clouds),'Odom cloud frame mismatch')
            require(not transforms,'LIO published TF despite publish_tf=false')
            # Core output is already odom XYZ: do not apply a second body extrinsic.
            last = clouds[-1]
            xyz = [struct.unpack_from('<fff',last.data,i*last.point_step) for i in range(last.width)]
            require(all(all(math.isfinite(v) for v in point) for point in xyz),'Nonfinite deskewed cloud')
            require(sum(min(abs(x-4),abs(y-4),abs(z+2))<0.1 for x,y,z in xyz)/len(xyz)>0.95,
                    'Core odom cloud was transformed a second time')
            print(f'PASS actual pinned Point-LIO: {len(observations)} poses, {len(clouds)} clouds; '
                  'raw IMU pose/covariance, stationary geometry, correct cloud frame and no TF',flush=True)
    finally:
        if process is not None:
            os.killpg(process.pid,signal.SIGINT)
            try: process.wait(timeout=5)
            except subprocess.TimeoutExpired: os.killpg(process.pid,signal.SIGKILL);process.wait()
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__': main()
