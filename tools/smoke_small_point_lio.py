#!/usr/bin/env python3
"""Run the actual pinned Point-LIO with explicit synthetic Livox-schema cloud/IMU inputs."""
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import time
import sys
import json

import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2, PointField, JointState
from std_msgs.msg import Bool,String
from rclpy.qos import QoSProfile,DurabilityPolicy
from rm_nav_interfaces.msg import ObservationBatch,ObservationFrame
from smoke_frozen_map_localization import cloud
from tf2_msgs.msg import TFMessage
import yaml


def require(value,message):
    if not value: raise RuntimeError(message)


def main():
    require(os.environ.get('ROS_DOMAIN_ID') not in (None,'','0'),'Use isolated Domain')
    pipeline='--with-state' in sys.argv
    mapping='--with-mapping' in sys.argv
    require(not mapping or pipeline,'--with-mapping requires --with-state')
    rclpy.init()
    node = Node('small_point_lio_fixture')
    imu_pub = node.create_publisher(Imu,'/fixture/imu',qos_profile_sensor_data)
    cloud_pub = node.create_publisher(PointCloud2,'/fixture/lidar',qos_profile_sensor_data)
    observations,clouds,transforms = [],[],[]
    node.create_subscription(Odometry,'/lio/sensor_odometry',observations.append,10)
    node.create_subscription(PointCloud2,'/lio/deskewed_odom_cloud',clouds.append,10)
    node.create_subscription(TFMessage,'/tf',transforms.append,10)
    encoder=node.create_publisher(JointState,'/hardware/gimbal_joint_states',qos_profile_sensor_data)
    submaps,batches,odom=[],[],[]
    health={'chassis':False,'global':False}
    diagnostics={}
    archives=[]
    if mapping:
        node.create_subscription(String,'/mapping/keyframe_archive',lambda m:archives.append(m.data),10)
        node.create_subscription(Bool,'/mapping/recorder_healthy',lambda m:health.update(mapping=m.data),10)
    if pipeline:
        for topic in ['/state/lio_reason','/state/chassis_reason','/sensors/localization_reason',
                      '/localization/submap_reason','/localization/status_reason']:
            node.create_subscription(String,topic,lambda msg,key=topic:diagnostics.update({key:msg.data}),10)
        node.create_subscription(PointCloud2,'/localization/odom_submap',submaps.append,qos_profile_sensor_data)
        node.create_subscription(ObservationBatch,'/sensors/localization_observations',batches.append,10)
        node.create_subscription(Odometry,'/odom',odom.append,10)
        node.create_subscription(Bool,'/state/chassis_healthy',lambda m:health.update(chassis=m.data),10)
        node.create_subscription(Bool,'/localization/healthy',lambda m:health.update({'global':m.data}),10)
    frozen=node.create_publisher(PointCloud2,'/localization/frozen_map',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
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
    bundle={'bundle_id':'synthetic-lio-pipeline','status':'verified',
            'frames':{'base_footprint':'base_footprint','chassis':'chassis','big_gimbal_yaw':'big_gimbal_yaw',
                      'imu':'front_mid360_imu','lidar':'front_mid360'},
            'encoder':{'joint':'big_gimbal_yaw','sign':1.,'zero_offset':0.},
            'transforms':{name:{'translation':[0.,0.,0.],'quaternion_xyzw':[0.,0.,0.,1.]} for name in
                          ['footprint_to_chassis','chassis_to_yaw_zero','yaw_to_imu','imu_to_lidar']}}
    calibration=Path('log/lio_pipeline_fixture.yaml')
    calibration.write_text(yaml.safe_dump(bundle))
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
    processes=[]
    try:
        with Path('log/small_point_lio_smoke.log').open('w') as log:
            command=['ros2','run','small_point_lio','small_point_lio_node',
                '--ros-args','--params-file',str(config),'-r','/Odometry:=/lio/sensor_odometry',
                '-r','/cloud_registered:=/lio/deskewed_odom_cloud']
            if pipeline:
                command=['ros2','launch','rm_nav_bringup','local_state.launch.py',
                         'calibration_file:='+str(calibration),'lio_params_file:='+str(config)]
                processes.append(subprocess.Popen(['ros2','launch','rm_nav_bringup','frozen_map_localization.launch.py',
                    'map_version:=synthetic-lio-pipeline'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
                if mapping:
                    processes.append(subprocess.Popen(['ros2','launch','rm_nav_bringup','mapping_capture.launch.py',
                        'archive_root:='+str(Path('log/lio_mapping_capture').resolve()),
                        'calibration_id:=synthetic-lio-pipeline'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
            process = subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            processes.append(process)
            while cloud_pub.get_subscription_count()==0 and time.monotonic()-started<10: spin(0.05)
            require(cloud_pub.get_subscription_count()>0,'Real LIO did not start')
            spin(0.3)
            require(not observations and not clouds,'LIO produced pose without sensor data')
            if pipeline: frozen.publish(cloud(points,'map',node.get_clock().now().to_msg()))
            last_cloud = time.monotonic()
            end = time.monotonic()+7.0
            while time.monotonic()<end:
                imu = Imu()
                imu.header.stamp,imu.header.frame_id = node.get_clock().now().to_msg(),'front_mid360_imu'
                imu.linear_acceleration.z = 9.81
                imu.orientation_covariance[0] = -1.0
                imu_pub.publish(imu)
                if pipeline:
                    joint=JointState()
                    joint.header=imu.header
                    joint.header.frame_id='chassis'
                    joint.name,joint.position=['big_gimbal_yaw'],[0.]
                    encoder.publish(joint)
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
            spin(0.02)
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
            if not pipeline: require(not transforms,'LIO published TF despite publish_tf=false')
            else:
                require(len(submaps)>5 and len(batches)>5 and len(odom)>10 and health['chassis'] and health['global'],
                        f'Real LIO/state/observation/submap/GICP chain did not become healthy: '
                        f'odom={len(odom)} batches={len(batches)} submaps={len(submaps)} health={health} reasons={diagnostics}')
                frame=batches[-1].frames[0]
                require(frame.source_id=='mid360_main' and frame.sensor_frame=='front_mid360' and
                        frame.calibration_id=='synthetic-lio-pipeline' and frame.roles==17 and
                        frame.reference_origin_only and not frame.per_point_time,'Observation provenance or raycast limits lost')
                require(all(s.header.frame_id=='odom' and 20<=s.width<=50000 for s in submaps),'Submap frame/point bound wrong')
                require(all(s.header.stamp.sec*10**9+s.header.stamp.nanosec <= observations[-1].header.stamp.sec*10**9+
                            observations[-1].header.stamp.nanosec for s in submaps),'Submap fabricated a future source stamp')
                if mapping:
                    require(archives and health.get('mapping',False),'Actual upstream stream did not reach mapping archive')
                    originals={f.sequence:f for b in batches for f in b.frames}
                    for path in archives:
                        directory=Path(path)
                        metadata=json.loads((directory/'metadata.json').read_text())
                        saved=deserialize_message((directory/'observation.cdr').read_bytes(),ObservationFrame)
                        require(saved.sequence in originals and saved==originals[saved.sequence],
                                'Actual upstream observation not archived unchanged')
                        require(metadata['calibration_id']=='synthetic-lio-pipeline' and
                                metadata['body_frame']=='base_footprint','Mapping archive mixed source calibration/body')
                spin(.7)
                require(not health['chassis'],'Input dropout did not invalidate chassis state')
                settled=len(submaps)
                archive_count=len(archives)
                spin(.5)
                require(not health['global'],'Input dropout did not expire global registration health')
                require(len(submaps)==settled,'Submap refreshed cached geometry without new sensor observations')
                if mapping:
                    require(not health.get('mapping',True) and len(archives)==archive_count,
                            'Mapping dropout stayed healthy or replayed cached observation')
                owners={tf.child_frame_id for group in transforms for tf in group.transforms}
                require(owners=={'base_footprint','big_gimbal_yaw','odom'},'Unexpected duplicate/raw LIO TF '+str(owners))
            # Core output is already odom XYZ: do not apply a second body extrinsic.
            last = clouds[-1]
            xyz = [struct.unpack_from('<fff',last.data,i*last.point_step) for i in range(last.width)]
            require(all(all(math.isfinite(v) for v in point) for point in xyz),'Nonfinite deskewed cloud')
            require(sum(min(abs(x-4),abs(y-4),abs(z+2))<0.1 for x,y,z in xyz)/len(xyz)>0.95,
                    'Core odom cloud was transformed a second time')
            print(f'PASS actual pinned Point-LIO: {len(observations)} poses, {len(clouds)} clouds; '
                  'raw IMU pose/covariance, stationary geometry, correct cloud frame and no LIO-owned TF',flush=True)
            if pipeline: print('PASS actual LIO -> time-aligned resolver -> EKF -> body odometry -> observation contract -> '
                               'bounded submap -> real GICP/sole map TF; dropout does not replay cached observations',flush=True)
            if mapping:print(f'PASS actual upstream mapping archive: {len(archives)} original keyframes, calibrated source and chassis pose; dropout hold',flush=True)
    finally:
        for item in processes: os.killpg(item.pid,signal.SIGINT)
        for item in processes:
            try: item.wait(timeout=5)
            except subprocess.TimeoutExpired: os.killpg(item.pid,signal.SIGKILL);item.wait()
        node.destroy_node();rclpy.shutdown()
        if pipeline: Path('log/lio_pipeline_diagnostics.txt').write_text(str(diagnostics))


if __name__=='__main__': main()
