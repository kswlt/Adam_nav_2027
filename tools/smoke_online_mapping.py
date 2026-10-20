#!/usr/bin/env python3
"""TEST_ONLY: stream committed archives through the real online ROS backend."""
import json, os, tempfile, subprocess, time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.serialization import serialize_message
from rm_nav_interfaces.msg import ObservationFrame
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Path as RosPath
from visualization_msgs.msg import MarkerArray
from smoke_frozen_map_localization import cloud, require
from smoke_loop_enabled_mapping import pose


def main():
    # A dedicated domain prevents the fixture being mistaken for live vehicle input.
    os.environ['ROS_DOMAIN_ID']='197'
    root=Path(tempfile.mkdtemp(prefix='online_mapping_test_'));session=root/'session';session.mkdir()
    print('TEST_ONLY evidence='+str(root),flush=True)
    rclpy.init();node=rclpy.create_node('online_mapping_smoke')
    qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    latest={};subscriptions=[]
    for typ,topic in [(String,'/mapping/online_status'),(RosPath,'/mapping/optimized_path'),
                      (PointCloud2,'/mapping/optimized_map'),(MarkerArray,'/mapping/loop_graph')]:
        subscriptions.append(node.create_subscription(typ,topic,lambda msg,t=topic:latest.update({t:msg}),qos))
    def status():
        return json.loads(latest['/mapping/online_status'].data) if '/mapping/online_status' in latest else {}
    def wait(check,seconds=40):
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.1)
            if check():return
        raise RuntimeError('Timeout; status='+str(status()))
    log=(root/'node.log').open('w')
    proc=subprocess.Popen(['ros2','run','rm_nav_mapping','online_mapping','--ros-args',
        '-p','session:='+str(session),'-p','calibration_id:=TEST_ONLY-online-fixture'],
        stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    originals={}
    try:
        wait(lambda: bool(status()))
        rng=np.random.default_rng(20271009);world=rng.uniform(-3,3,(12000,3));hom=np.c_[world,np.ones(len(world))].T
        route=[(0,0),(.7,0),(1.4,0),(2.1,0),(2.8,.2),(2.8,.9),(2.1,1.2),(1.4,1.1),(.7,.7),(.2,.3),(0,0),(.15,.05)]
        raw_poses=[]
        for i,(x,y) in enumerate(route):
            actual=pose(x,y,.02*i);recorded=pose(x+.025*i,y+.012*i,.025*i);raw_poses.append(recorded)
            frame=ObservationFrame();frame.header.frame_id='odom';frame.header.stamp.sec=100+i
            frame.source_id='mid360_main';frame.sensor_frame='front_mid360';frame.calibration_id='TEST_ONLY-online-fixture'
            frame.sequence=i+1;frame.roles=17;frame.deskewed=frame.healthy=frame.reference_origin_only=True
            frame.sensor_pose.position.x=recorded[0,3];frame.sensor_pose.position.y=recorded[1,3]
            frame.sensor_pose.orientation.z=np.sin(.025*i/2);frame.sensor_pose.orientation.w=np.cos(.025*i/2)
            frame.point_cloud=cloud((recorded@np.linalg.inv(actual)@hom)[:3].T,'odom',frame.header.stamp)
            directory=session/f'keyframe_{i}.partial';directory.mkdir();payload=serialize_message(frame)
            (directory/'observation.cdr').write_bytes(payload)
            metadata={'schema':1,'id':i,'stamp_ns':(100+i)*10**9,'calibration_id':frame.calibration_id,
              'source_id':frame.source_id,'sensor_frame':frame.sensor_frame,'body_frame':'base_footprint',
              'cloud_frame':'odom','observation_file':'observation.cdr','observation_bytes':len(payload),
              'recorded_odom_T_body':recorded.ravel().tolist()}
            (directory/'metadata.json').write_text(json.dumps(metadata))
            if i==0:
                for _ in range(15):rclpy.spin_once(node,timeout_sec=.1)
                require(int(status().get('keyframes',0))==0,'Partial keyframe was ingested')
            final=session/f'keyframe_{i}';directory.rename(final)
            originals[final/'observation.cdr']=(len(payload),(final/'observation.cdr').stat().st_mtime_ns)
            wait(lambda:int(status().get('keyframes',0))==i+1)
        wait(lambda: '/mapping/optimized_path' in latest and len(latest['/mapping/optimized_path'].poses)==12)
        state=status();require(int(state['accepted_loops'])>=1,'No real online loop accepted')
        path=latest['/mapping/optimized_path'];point=path.poses[5].pose.position
        require(np.linalg.norm(np.array([point.x,point.y])-raw_poses[5][:2,3])>.001,'Historical trajectory unchanged')
        require(latest['/mapping/optimized_map'].header==path.header,'Cloud/path snapshot source stamps differ')
        require(latest['/mapping/optimized_map'].width>0,'Optimized map empty')
        require(latest['/mapping/loop_graph'].markers[-1].points,'Loop graph empty')
        require(str(state['upstream_healthy']).lower()=='false','Fixture claimed real upstream health')
        # A malformed committed next frame must latch fault without replacing the last accepted snapshot.
        bad=session/'keyframe_12';bad.mkdir();(bad/'metadata.json').write_text('{}')
        wait(lambda:str(status().get('fault')).lower()=='true')
        require(int(status()['revision'])==12,'Bad archive changed accepted revision')
        require(all((p.stat().st_size,p.stat().st_mtime_ns)==v for p,v in originals.items()),'Input archive changed')
        result={'test_only':True,'accepted_before_fault':state,'fault_status':status(),
                'historical_pose_changed':True,'partial_ignored':True,'originals_unchanged':True,'physical_output':False}
        (root/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2),flush=True)
        print('PASS online ROS archive ingestion, real KISS/GICP/GTSAM, map/path/loop snapshots and malformed-input retention',flush=True)
    finally:
        import signal
        os.killpg(proc.pid,signal.SIGINT)
        try:proc.wait(timeout=20)
        except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
        log.close();node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
