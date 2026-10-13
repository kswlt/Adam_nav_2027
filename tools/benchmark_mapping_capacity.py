#!/usr/bin/env python3
"""TEST_ONLY actual archive/GICP/GTSAM capacity, explicit synthetic 1500-point scans."""
import json
from pathlib import Path
import subprocess
import tempfile
import numpy as np
from rclpy.serialization import serialize_message
from rm_nav_interfaces.msg import ObservationFrame
from smoke_frozen_map_localization import cloud

def main():
    root=Path(tempfile.mkdtemp(prefix='mapping_capacity_'));print('Retained fixture: '+str(root),flush=True)
    points=np.random.default_rng(2027).uniform(-3,3,(1500,3));reports=[]
    for count in (100,300,500):
        session=root/f'session_{count}';session.mkdir()
        for i in range(count):
            pose=np.eye(4);pose[0,3]=i*.01
            frame=ObservationFrame();frame.header.frame_id='odom';frame.header.stamp.sec=100+i
            frame.source_id='TEST_ONLY_capacity';frame.sensor_frame='front_mid360';frame.calibration_id='TEST_ONLY_capacity'
            frame.sequence=i+1;frame.roles=17;frame.deskewed=frame.healthy=frame.reference_origin_only=True
            frame.sensor_pose.position.x=pose[0,3];frame.sensor_pose.orientation.w=1.
            frame.point_cloud=cloud(points,'odom',frame.header.stamp)
            payload=serialize_message(frame);directory=session/f'keyframe_{i}';directory.mkdir()
            (directory/'observation.cdr').write_bytes(payload)
            metadata={'schema':1,'id':i,'stamp_ns':(100+i)*10**9,'calibration_id':frame.calibration_id,
                'source_id':frame.source_id,'sensor_frame':frame.sensor_frame,'body_frame':'base_footprint',
                'cloud_frame':'odom','observation_file':'observation.cdr','observation_bytes':len(payload),
                'recorded_odom_T_body':pose.ravel().tolist()}
            (directory/'metadata.json').write_text(json.dumps(metadata))
        output=root/f'optimized_{count}'
        result=subprocess.run(['ros2','run','rm_nav_mapping','offline_graph_optimizer',str(session),str(output)],capture_output=True,text=True,timeout=240)
        (root/f'optimizer_{count}.log').write_text(result.stdout+result.stderr)
        if result.returncode:raise RuntimeError(f'{count} frame optimization failed: '+result.stderr)
        data=json.loads((output/'optimized_poses.json').read_text())
        if len(data['poses'])!=count:raise RuntimeError('Silent pose truncation')
        report={'keyframes':count,'points_per_frame':1500,
            'input_bytes':sum(p.stat().st_size for p in session.rglob('*') if p.is_file()),
            'output_bytes':sum(p.stat().st_size for p in output.rglob('*') if p.is_file()),
            'resources':next(line for line in result.stdout.splitlines() if line.startswith('resources '))}
        reports.append(report);print(json.dumps(report),flush=True)
        if count==500:
            (session/'keyframe_500').mkdir()
            refused=root/'must_not_exist'
            result=subprocess.run(['ros2','run','rm_nav_mapping','offline_graph_optimizer',str(session),str(refused)],capture_output=True,text=True,timeout=10)
            if result.returncode==0 or refused.exists() or '2..500' not in result.stderr:raise RuntimeError('501 frame contract not enforced')
    (root/'report.json').write_text(json.dumps({'scope':'TEST_ONLY synthetic geometry; not field-scale point density or accuracy','results':reports},indent=2))
    print('PASS actual 100/300/500-frame optimizer and explicit 501 rejection; '+str(root))
if __name__=='__main__':main()
