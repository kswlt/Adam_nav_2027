#!/usr/bin/env python3
"""Controlled return-to-start archive: exercise real KISS/GICP loop wiring."""
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile

import numpy as np
from rclpy.serialization import serialize_message
from rm_nav_interfaces.msg import ObservationFrame
from smoke_frozen_map_localization import cloud, require


def pose(x,y,yaw):
    c,s=np.cos(yaw),np.sin(yaw); result=np.eye(4)
    result[:2,:2]=[[c,-s],[s,c]]; result[:2,3]=[x,y]; return result


def main():
    root=Path(tempfile.mkdtemp(prefix='loop_mapping_',dir=Path('log').resolve()))
    session=root/'return_session';session.mkdir()
    rng=np.random.default_rng(20271009)
    # A 12-second out-and-back trajectory. Frame 11 revisits frame 0's area.
    actual_xy=[(0,0),(.7,0),(1.4,0),(2.1,0),(2.8,.2),(2.8,.9),(2.1,1.2),(1.4,1.1),(.7,.7),(.2,.3),(.0,.0),(.15,.05)]
    world=rng.uniform(-3,3,(12000,3)); hom=np.c_[world,np.ones(len(world))].T
    snapshots={}
    for i,(x,y) in enumerate(actual_xy):
        actual=pose(x,y,.02*i)
        # Deliberate bounded odometry drift; returned frame remains within candidate radius.
        recorded=pose(x+.025*i,y+.012*i,.025*i)
        points=(recorded@np.linalg.inv(actual)@hom)[:3].T
        frame=ObservationFrame();frame.header.frame_id='odom';frame.header.stamp.sec=100+i
        frame.source_id='mid360_loop_fixture';frame.sensor_frame='front_mid360';frame.calibration_id='loop-fixture-v1'
        frame.sequence=i+1;frame.roles=17;frame.deskewed=frame.healthy=frame.reference_origin_only=True
        frame.sensor_pose.position.x=recorded[0,3];frame.sensor_pose.position.y=recorded[1,3]
        frame.sensor_pose.orientation.z=np.sin(.025*i/2);frame.sensor_pose.orientation.w=np.cos(.025*i/2)
        frame.point_cloud=cloud(points,'odom',frame.header.stamp)
        payload=serialize_message(frame);directory=session/f'keyframe_{i}';directory.mkdir()
        (directory/'observation.cdr').write_bytes(payload)
        metadata={'schema':1,'id':i,'stamp_ns':(100+i)*10**9,'calibration_id':frame.calibration_id,
                  'source_id':frame.source_id,'sensor_frame':frame.sensor_frame,'body_frame':'base_footprint',
                  'cloud_frame':'odom','observation_file':'observation.cdr','observation_bytes':len(payload),
                  'recorded_odom_T_body':recorded.ravel().tolist()}
        (directory/'metadata.json').write_text(json.dumps(metadata));snapshots.update({directory/'observation.cdr':hashlib.sha256(payload).hexdigest()})
    output=root/'optimized';command=['ros2','run','rm_nav_mapping','offline_graph_optimizer',str(session),str(output),'--enable-loops']
    result=subprocess.run(command,capture_output=True,text=True,timeout=180)
    (root/'optimizer.log').write_text(result.stdout+result.stderr)
    require(result.returncode==0,'Loop-enabled optimizer failed: '+result.stdout+result.stderr)
    data=json.loads((output/'optimized_poses.json').read_text());loops=json.loads((output/'loop_edges.json').read_text())
    require(data['loop_count']==len(loops) and len(loops)>=1,'Controlled return fixture did not produce an accepted loop')
    require(any(edge['algorithm']=='KISS_GICP_bidirectional_validated' for edge in loops),'Loop algorithm provenance missing')
    require(all(hashlib.sha256(path.read_bytes()).hexdigest()==digest for path,digest in snapshots.items()),'Loop optimizer mutated archive')
    require((output/'loop_edges.json').stat().st_size>2,'Loop artifact empty despite accepted loop')
    print('PASS real KISS/GICP bidirectional loop accepted and sent to GTSAM; loops='+str(len(loops))+
          '; evidence='+str(root),flush=True)


if __name__=='__main__':main()
