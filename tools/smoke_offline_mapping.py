#!/usr/bin/env python3
"""Actual VGICP/GTSAM executable on explicit drifting-odometry archive fixtures."""
import copy
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import fcntl
import numpy as np
from rclpy.serialization import serialize_message
from rm_nav_interfaces.msg import ObservationFrame
from smoke_frozen_map_localization import cloud,require


def pose(x,y,yaw):
    result=np.eye(4);c,s=np.cos(yaw),np.sin(yaw)
    result[:2,:2]=[[c,-s],[s,c]];result[:2,3]=[x,y];return result


def main():
    Path('log').mkdir(exist_ok=True)
    root=Path(tempfile.mkdtemp(prefix='offline_mapping_',dir=Path('log').resolve()))
    session=root/'session_fixture';session.mkdir()
    world=np.random.default_rng(2027).uniform(-2,2,(5000,3))
    hom=np.c_[world,np.ones(len(world))].T
    truth=[];initial=[]
    frames=[]
    for i in range(6):
        actual=pose(.25*i,0,.03*i);recorded=pose(.30*i,.02*i,.05*i)
        truth.append(actual);initial.append(recorded)
        original=(recorded@np.linalg.inv(actual)@hom)[:3].T
        frame=ObservationFrame();frame.header.frame_id='odom';frame.header.stamp.sec=100+i
        frame.source_id='mid360_main';frame.sensor_frame='front_mid360';frame.calibration_id='offline-fixture-v1'
        frame.sequence=i+1;frame.roles=17;frame.deskewed=frame.healthy=frame.reference_origin_only=True
        frame.sensor_pose.position.x=recorded[0,3];frame.sensor_pose.position.y=recorded[1,3]
        frame.sensor_pose.orientation.z=np.sin(.05*i/2);frame.sensor_pose.orientation.w=np.cos(.05*i/2)
        frame.point_cloud=cloud(original,'odom',frame.header.stamp)
        data=serialize_message(frame);directory=session/f'keyframe_{i}';directory.mkdir()
        (directory/'observation.cdr').write_bytes(data)
        metadata={'schema':1,'id':i,'stamp_ns':(100+i)*10**9,'calibration_id':frame.calibration_id,
                  'source_id':frame.source_id,'sensor_frame':frame.sensor_frame,'body_frame':'base_footprint',
                  'cloud_frame':'odom','observation_file':'observation.cdr','observation_bytes':len(data),
                  'recorded_odom_T_body':recorded.ravel().tolist()}
        (directory/'metadata.json').write_text(json.dumps(metadata));frames.append(frame)
    snapshot={path:hashlib.sha256(path.read_bytes()).hexdigest() for path in session.rglob('*') if path.is_file()}
    command=['ros2','run','rm_nav_mapping','offline_graph_optimizer']
    def run(source,output):return subprocess.run([*command,str(source),str(output)],capture_output=True,text=True,timeout=120)
    valid=root/'optimized';result=run(session,valid)
    (root/'optimization.log').write_text(result.stdout+result.stderr)
    require(result.returncode==0,'Actual optimizer failed: '+result.stdout+result.stderr)
    data=json.loads((valid/'optimized_poses.json').read_text())
    require(data['coordinate_frame']=='mapping_odom' and not data['official_alignment_applied'] and
            data['calibration_id']=='offline-fixture-v1','Output reference/calibration contract missing')
    optimized=[np.array(item['matrix']).reshape(4,4) for item in data['poses']]
    errors=[np.linalg.norm(a[:3,3]-b[:3,3]) for a,b in zip(truth,optimized)]
    require(len(optimized)==6 and max(errors)<.10,'Graph direction or drift reduction incorrect: '+str(errors))
    require(errors[-1]<np.linalg.norm(truth[-1][:3,3]-initial[-1][:3,3])*.4,'VGICP factors did not remove odometry drift')
    require(data['final_error']<data['initial_error']*.01,'GTSAM failed to reduce inconsistent initial graph objective')
    edges=json.loads((valid/'adjacent_edges.json').read_text())
    require(len(edges)==5 and all(edge['algorithm']=='small_gicp_VGICP' for edge in edges),'Adjacent graph bypassed real VGICP')
    for i,edge in enumerate(edges):
        measurement=np.array(edge['matrix']).reshape(4,4)
        expected=np.linalg.inv(truth[i])@truth[i+1]
        require(np.linalg.norm(measurement[:3,3]-expected[:3,3])<.04,'Wrong relative factor transform')
    content=(valid/'rebuilt_map.pcd').read_bytes();header,binary=content.split(b'DATA binary\n',1)
    rebuilt=np.frombuffer(binary,dtype=np.float32).reshape(-1,3)
    require(0<len(rebuilt)<=1000000 and b'mapping_odom' in header,'Invalid reconstructed PCD')
    nearest=np.sqrt(np.min(np.sum((rebuilt[::max(1,len(rebuilt)//300),None,:]-world[None,:,:])**2,axis=2),axis=1))
    require(nearest.mean()<.05 and np.quantile(nearest,.95)<.1,'Map rebuilt from wrong pose/original geometry')
    require(all(hashlib.sha256(path.read_bytes()).hexdigest()==value for path,value in snapshot.items()),'Optimizer modified original archive')
    repeated=run(session,valid)
    require(repeated.returncode!=0,'Existing optimized output overwritten')
    nested=run(session,session/'nested_output')
    require(nested.returncode!=0 and not (session/'nested_output').exists(),'Optimizer wrote output into original archive session')
    with (session/'recording.lock').open('w') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        active=run(session,root/'active_output')
        require(active.returncode!=0 and 'active' in active.stderr and not (root/'active_output').exists(),
                'Optimizer accepted a session that is still recording')
    hashes=json.loads((valid/'input_hashes.json').read_text())
    require(all(item['observation_sha256']==hashlib.sha256((session/f'keyframe_{item["id"]}'/'observation.cdr').read_bytes()).hexdigest()
                for item in hashes),'Output input hashes incorrect')
    for mode in ['calibration','stamp','id_gap','truncated','bad_pose','bad_origin','nonoverlap','partial']:
        source=root/mode;shutil.copytree(session,source)
        directory=source/'keyframe_2';metadata=json.loads((directory/'metadata.json').read_text())
        if mode=='calibration':metadata['calibration_id']='wrong'
        elif mode=='stamp':metadata['stamp_ns']=100*10**9
        elif mode=='id_gap':shutil.rmtree(directory)
        elif mode=='truncated':(directory/'observation.cdr').write_bytes((directory/'observation.cdr').read_bytes()[:-1])
        elif mode=='bad_pose':metadata['recorded_odom_T_body'][0]=2.
        elif mode in ('bad_origin','nonoverlap'):
            frame=copy.deepcopy(frames[2])
            if mode=='bad_origin':frame.sensor_pose.orientation.w=frame.sensor_pose.orientation.z=0.
            else:frame.point_cloud=cloud(world+[100,0,0],'odom',frame.header.stamp)
            payload=serialize_message(frame);(directory/'observation.cdr').write_bytes(payload);metadata['observation_bytes']=len(payload)
        elif mode=='partial':(source/'keyframe_6.partial').mkdir()
        if mode!='id_gap':(directory/'metadata.json').write_text(json.dumps(metadata))
        output=root/(mode+'_output');rejected=run(source,output)
        (root/(mode+'.log')).write_text(rejected.stdout+rejected.stderr)
        require(rejected.returncode!=0 and not output.exists(),'Malformed session generated an optimized map: '+mode)
    print(f'PASS actual VGICP + GTSAM archive graph: pose errors={errors}, graph error={data["initial_error"]}->{data["final_error"]}; '
          'original-cloud map reconstruction, immutable inputs, no-overwrite and malformed/nonoverlap refusal; evidence='+str(root),flush=True)


if __name__=='__main__':main()
