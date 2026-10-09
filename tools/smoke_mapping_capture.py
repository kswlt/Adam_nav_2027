#!/usr/bin/env python3
"""Actual ROS mapping recorder; explicit odom/observation fixtures, real durable files."""
import copy
import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import tempfile
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Bool,String
from rm_nav_interfaces.msg import ObservationBatch,ObservationFrame
from tf2_ros import TransformBroadcaster
from smoke_frozen_map_localization import cloud,require


def main():
    require(os.environ.get('ROS_DOMAIN_ID') not in (None,'','0'),'Use isolated Domain')
    rclpy.init();node=Node('mapping_capture_fixture')
    state=node.create_publisher(Bool,'/state/chassis_healthy',10)
    health=node.create_publisher(Bool,'/sensors/localization_healthy',10)
    publisher=node.create_publisher(ObservationBatch,'/sensors/localization_observations',10)
    broadcaster=TransformBroadcaster(node)
    archives,statuses=[],[];healthy={'value':False}
    node.create_subscription(String,'/mapping/keyframe_archive',lambda m:archives.append(m.data),10)
    node.create_subscription(String,'/mapping/recorder_status',lambda m:statuses.append(m.data),10)
    node.create_subscription(Bool,'/mapping/recorder_healthy',lambda m:healthy.update(value=m.data),10)
    feed={'x':0.,'yaw':0.,'origin_yaw':0.,'health':True,'source':True,'on':True,'mode':'normal'}
    sequence=0;originals={}
    points=[(1.+i*.1,2.+j*.1,.5) for i in range(10) for j in range(10)]
    def tick():
        nonlocal sequence
        state.publish(Bool(data=feed['health']));health.publish(Bool(data=feed['source']))
        stamp=node.get_clock().now().to_msg()
        transform=TransformStamped();transform.header.frame_id='odom';transform.child_frame_id='base_footprint'
        transform.header.stamp=stamp;transform.transform.translation.x=feed['x']
        transform.transform.rotation.z=math.sin(feed['yaw']/2);transform.transform.rotation.w=math.cos(feed['yaw']/2)
        broadcaster.sendTransform(transform)
        if not feed['on']:return
        # A past observation is bracketed by measured TF samples; no latest-time fallback.
        total=stamp.sec*10**9+stamp.nanosec-30_000_000
        stamp.sec,stamp.nanosec=divmod(total,10**9)
        msg=ObservationBatch();msg.header.frame_id='odom';msg.header.stamp=stamp
        frame=ObservationFrame();frame.header=copy.deepcopy(msg.header)
        frame.source_id='mid360_main';frame.sensor_frame='front_mid360';frame.calibration_id='mapping-fixture-v1'
        sequence+=1;frame.sequence=sequence;frame.roles=17;frame.deskewed=frame.healthy=True;frame.reference_origin_only=True
        frame.sensor_pose.position.x=3.
        frame.sensor_pose.orientation.z=math.sin(feed['origin_yaw']/2)
        frame.sensor_pose.orientation.w=math.cos(feed['origin_yaw']/2)
        frame.point_cloud=cloud(points,'odom',stamp)
        mode=feed['mode']
        if mode=='calibration':frame.calibration_id='wrong'
        elif mode=='stale':msg.header.stamp.sec-=2;frame.header=copy.deepcopy(msg.header);frame.point_cloud.header=copy.deepcopy(msg.header)
        elif mode=='frame':msg.header.frame_id='map';frame.header=copy.deepcopy(msg.header);frame.point_cloud.header=copy.deepcopy(msg.header)
        elif mode=='nan':frame.point_cloud.data=struct.pack('<f',float('nan'))+bytes(frame.point_cloud.data[4:])
        elif mode=='origin':frame.sensor_pose.orientation.w=0.;frame.sensor_pose.orientation.z=0.
        elif mode=='sequence':frame.sequence=1
        elif mode=='truncated':frame.point_cloud.data=bytes(frame.point_cloud.data[:-1])
        msg.frames=[frame]
        if mode=='multiple':msg.frames.append(copy.deepcopy(frame))
        originals[sequence]=copy.deepcopy(frame)
        publisher.publish(msg)
    timer=node.create_timer(.05,tick)
    def wait(predicate,seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.01)
            if predicate():return True
        return False
    process=None;Path('log').mkdir(exist_ok=True)
    # Keep actual archives alongside the log for inspection, rather than deleting evidence.
    root=Path(tempfile.mkdtemp(prefix='mapping_capture_',dir=Path('log').resolve()))
    def start(log,limit=100,extra=()):
        return subprocess.Popen(['ros2','run','rm_nav_mapping','keyframe_recorder','--ros-args',
            '-p','calibration_id:=mapping-fixture-v1','-p',f'archive_root:={root}',
            '-p',f'max_keyframes:={limit}',*extra],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    def stop(item):
        os.killpg(item.pid,signal.SIGINT)
        try:item.wait(timeout=5)
        except subprocess.TimeoutExpired:os.killpg(item.pid,signal.SIGKILL);item.wait()
    try:
        with Path('log/mapping_capture_smoke.log').open('w') as log:
            process=start(log)
            require(wait(lambda:len(archives)==1 and healthy['value'],8),'Recorder did not create first keyframe')
            feed['origin_yaw']=1.8
            wait(lambda:False,.5)
            require(len(archives)==1,'Rotating LiDAR origin incorrectly triggered chassis keyframe')
            feed['x']=.05;wait(lambda:False,.4)
            require(len(archives)==1,'Sub-threshold chassis displacement triggered')
            feed['x']=.3
            require(wait(lambda:len(archives)==2,1.5),'Chassis translation missing')
            feed['yaw']=.4
            require(wait(lambda:len(archives)==3,1.5),'Chassis yaw trigger missing')
            for path in archives:
                directory=Path(path);metadata=json.loads((directory/'metadata.json').read_text())
                data=(directory/'observation.cdr').read_bytes();frame=deserialize_message(data,ObservationFrame)
                require(frame==originals[frame.sequence],'Original observation was changed before archiving')
                require(metadata['observation_bytes']==len(data) and metadata['stamp_ns']==frame.header.stamp.sec*10**9+frame.header.stamp.nanosec,
                        'Archive source stamp/byte count mismatch')
                transform=np.array(metadata['recorded_odom_T_body']).reshape(4,4)
                xyz=np.array([struct.unpack_from('<fff',frame.point_cloud.data,i*12)+(1.,) for i in range(frame.point_cloud.width)]).T
                local=np.linalg.inv(transform)@xyz
                require(np.allclose(transform@local,xyz,atol=1e-10),'Original geometry cannot be reconstructed')
                optimized=transform.copy();optimized[0,3]+=1.
                rebuilt=optimized@local
                require(np.allclose(rebuilt[0],xyz[0]+1.,atol=1e-10),'Optimized pose applied twice or original pose not removed')
                require(frame.reference_origin_only and not frame.per_point_time,'Archive invented acquisition times')
            for mode in ['calibration','stale','frame','nan','origin','sequence','truncated','multiple']:
                feed['mode']=mode
                require(wait(lambda:not healthy['value'],.8),'Malformed observation accepted: '+mode)
                count=len(archives);wait(lambda:False,.15)
                require(len(archives)==count,'Malformed observation committed a keyframe')
                feed['mode']='normal'
                require(wait(lambda:healthy['value'],.8),'Valid observations failed to recover')
            feed['health']=False;feed['x']=1.
            require(wait(lambda:not healthy['value'],.5),'Chassis health loss not enforced')
            count=len(archives);wait(lambda:False,.3)
            require(len(archives)==count,'Lost chassis health still recorded')
            feed['health']=True
            require(wait(lambda:len(archives)>count and healthy['value'],1.5),'Fresh healthy stream failed to recover')
            feed['on']=False
            require(wait(lambda:not healthy['value'],.6),'Observation dropout stayed healthy')
            count=len(archives);wait(lambda:False,.3)
            require(len(archives)==count,'Dropout replayed cached frame')
            stop(process);process=None
            # A new run creates a distinct session and keeps the previous archive immutable.
            baseline=list(root.rglob('keyframe_*/metadata.json'));before={p:p.read_bytes() for p in baseline}
            archives.clear();feed.update(on=True,x=0.,yaw=0.)
            process=start(log,limit=1)
            require(wait(lambda:len(archives)==1,6),'New session failed to record')
            feed['x']=.4
            require(wait(lambda:statuses and 'fault=true' in statuses[-1],1.5),'Quota did not latch storage fault')
            require(wait(lambda:not healthy['value'],.3),'Storage fault falsely healthy')
            feed['x']=0.;wait(lambda:False,.5)
            require(len(archives)==1,'Fault recovered silently or exceeded quota')
            require(all(p.read_bytes()==data for p,data in before.items()),'Restart overwrote existing frames')
            stop(process);process=None
            # Small byte quota: whole first frame must be refused, never announced or partly committed.
            points=points*4
            archives.clear();process=start(log,extra=('-p','max_archive_bytes:=4096'))
            require(wait(lambda:statuses and 'fault=true' in statuses[-1] and 'bytes/free-space' in statuses[-1],6),
                    'Byte quota did not refuse complete frame')
            require(not archives,'Over-byte-quota frame was announced')
            require(not list(root.rglob('*.partial')),'Successful/quota-only runs left partial commits')
            stop(process);process=None
            archives.clear();feed.update(x=0.,yaw=0.)
            process=start(log)
            require(wait(lambda:len(archives)==1 and healthy['value'],6),'Collision fixture session failed')
            collision=Path(archives[0]).parent/'keyframe_1'
            collision.mkdir();(collision/'sentinel').write_text('must remain unchanged')
            feed['x']=.4
            require(wait(lambda:statuses and 'fault=true' in statuses[-1] and 'no-replace' in statuses[-1],1.5),
                    'Existing archive collision was not refused')
            require(wait(lambda:not healthy['value'],.3),'Commit failure stayed healthy')
            require(len(archives)==1 and (collision/'sentinel').read_text()=='must remain unchanged',
                    'Existing archive overwritten or failed commit announced')
            require(len(list(root.rglob('*.partial')))==1,'Failed commit not retained separately for diagnosis')
            print('PASS actual mapping recorder: chassis-only translation/yaw triggers; original CDR/provenance/source time; '
                  'optimized-pose reconstruction; bad observations/health/dropout refused; durable unique sessions, frame/byte quota fault holds; '
                  f'archive evidence: {root}',flush=True)
    finally:
        node.destroy_timer(timer)
        if process is not None:stop(process)
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
