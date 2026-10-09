#!/usr/bin/env python3
"""Real observation/submap nodes, explicit cloud/health/TF fixtures and rejection boundaries."""
import copy
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
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Bool
from sensor_msgs.msg import PointCloud2
from rm_nav_interfaces.msg import ObservationBatch
from tf2_ros import StaticTransformBroadcaster
from smoke_frozen_map_localization import cloud,require


def main():
    require(os.environ.get('ROS_DOMAIN_ID') not in (None,'','0'),'Use isolated Domain')
    rclpy.init()
    node=Node('observation_fixture')
    state=node.create_publisher(Bool,'/state/chassis_healthy',10)
    source=node.create_publisher(PointCloud2,'/lio/deskewed_odom_cloud',qos_profile_sensor_data)
    forged=node.create_publisher(ObservationBatch,'/sensors/localization_observations',10)
    batches,submaps=[],[]
    status={'source':False,'submap':False}
    node.create_subscription(ObservationBatch,'/sensors/localization_observations',batches.append,10)
    node.create_subscription(PointCloud2,'/localization/odom_submap',submaps.append,qos_profile_sensor_data)
    node.create_subscription(Bool,'/sensors/localization_healthy',lambda m:status.update(source=m.data),10)
    node.create_subscription(Bool,'/localization/submap_healthy',lambda m:status.update(submap=m.data),10)
    tf=StaticTransformBroadcaster(node)
    transform=TransformStamped()
    transform.header.frame_id,transform.child_frame_id='odom','front_mid360'
    transform.transform.translation.x,transform.transform.translation.y,transform.transform.translation.z=3.,-2.,.5
    transform.transform.rotation.w=1.
    tf.sendTransform(transform)
    points_a=[(-3.+i*.1,-1.+j*.1,.1) for i in range(10) for j in range(10)]
    points_b=[(3.+i*.1,1.+j*.1,.2) for i in range(10) for j in range(10)]
    feed={'on':True,'health':True,'points':points_a,'mode':'normal'}
    def tick():
        state.publish(Bool(data=feed['health']))
        if not feed['on']:return
        msg=cloud(feed['points'],'odom',node.get_clock().now().to_msg())
        if feed['mode']=='stale':msg.header.stamp.sec-=2
        elif feed['mode']=='wrong_frame':msg.header.frame_id='map'
        elif feed['mode']=='nan':msg.data=struct.pack('<f',float('nan'))+bytes(msg.data[4:])
        elif feed['mode']=='truncated':msg.data=bytes(msg.data[:-1])
        elif feed['mode']=='wrong_field':msg.fields[0].datatype=2
        source.publish(msg)
    timer=node.create_timer(.1,tick)
    processes=[]
    Path('log').mkdir(exist_ok=True)
    def wait(predicate,timeout):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.01)
            if predicate():return True
        return False
    def xyz(msg):return [struct.unpack_from('<fff',msg.data,i*msg.point_step) for i in range(msg.width)]
    try:
        with Path('log/observation_submap_smoke.log').open('w') as log:
            for package,executable in [('rm_nav_sensors','lio_observation_adapter'),('rm_nav_localization','observation_submap')]:
                processes.append(subprocess.Popen(['ros2','run',package,executable,'--ros-args','-p','calibration_id:=fixture-v1'],
                    stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
            require(wait(lambda:status['source'] and status['submap'] and len(submaps)>2,10),'Observation nodes missing')
            frame=batches[-1].frames[0]
            p=frame.sensor_pose.position
            require(frame.source_id=='mid360_main' and frame.calibration_id=='fixture-v1' and
                    (p.x,p.y,p.z)==(3.,-2.,.5) and frame.reference_origin_only and not frame.per_point_time,
                    'Source identity/origin or acquisition-time limits missing')
            require(all(-3.01<=x<=-2.0 for x,y,z in xyz(submaps[-1])),'Sensor origin was applied to already-odom points twice')
            feed['points']=points_b
            require(wait(lambda:submaps and any(x>2 for x,y,z in xyz(submaps[-1])) and
                         any(x<-2 for x,y,z in xyz(submaps[-1])),.8),'Rolling window did not retain both observations')
            wait(lambda:False,1.2)
            require(all(x>2 for x,y,z in xyz(submaps[-1])),'Old points were not evicted by source time')
            for mode in ['stale','wrong_frame','nan','truncated','wrong_field']:
                feed['mode']=mode
                require(wait(lambda:not status['source'] and not status['submap'],.7),'Bad cloud accepted: '+mode)
                wait(lambda:False,.25)
                count=len(submaps)
                wait(lambda:False,.25)
                require(len(submaps)==count,'Invalid cloud emitted cached submap: '+mode)
                feed['mode']='normal'
                require(wait(lambda:status['source'] and status['submap'],.8),'Valid source failed to recover')
            feed['on']=False
            original=copy.deepcopy(batches[-1])
            for variant in ['calibration','origin','multiple_primary','sequence']:
                fake=copy.deepcopy(original)
                fake.header.stamp=node.get_clock().now().to_msg()
                for frame in fake.frames:
                    frame.header=copy.deepcopy(fake.header);frame.point_cloud.header=copy.deepcopy(fake.header)
                    frame.sequence+=1
                if variant=='calibration':fake.frames[0].calibration_id='wrong-version'
                elif variant=='origin':fake.frames[0].sensor_pose.position.x=float('nan')
                elif variant=='multiple_primary':fake.frames.append(copy.deepcopy(fake.frames[0]))
                else:fake.frames[0].sequence=original.frames[0].sequence
                forged.publish(fake)
                require(wait(lambda:not status['submap'],.5),'Forged batch accepted: '+variant)
                feed['on']=True
                require(wait(lambda:status['source'] and status['submap'],.8),'Valid batch could not recover after rejection')
                feed['on']=False
                original=copy.deepcopy(batches[-1])
            feed['on']=True;feed['health']=False
            require(wait(lambda:not status['source'] and not status['submap'],.6),'State loss failed to invalidate observations')
            wait(lambda:False,.3)
            count=len(submaps)
            wait(lambda:False,.3)
            require(len(submaps)==count,'State loss emitted localization geometry')
            print('PASS actual observation/submap: per-source identity and reference origin, no second transform, '
                  'source-time window eviction, no cached replay; malformed cloud and calibration/origin/sequence/primary/state rejection',flush=True)
    finally:
        node.destroy_timer(timer)
        for item in processes:os.killpg(item.pid,signal.SIGINT)
        for item in processes:
            try:item.wait(timeout=5)
            except subprocess.TimeoutExpired:os.killpg(item.pid,signal.SIGKILL);item.wait()
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
