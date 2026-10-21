#!/usr/bin/env python3
"""Read-only compact motion extracts; preserve original MCAP and UART evidence."""
import argparse
import csv
import gzip
import json
import math
from pathlib import Path
import tarfile

import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--runtime',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--bags',nargs='+',required=True)
    p.add_argument('--rx',nargs='+',required=True)
    args=p.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    report={'scope':'read-only derived samples; originals retained',
            'control_output':False,'encoder_unit_and_zero_verified':False,
            'encoder_timestamp':'host receive batch, not MCU sample time','bags':{},'serial':{}}
    def stamp(m):return m.header.stamp.sec*10**9+m.header.stamp.nanosec
    topics={'/lio/sensor_odometry':('lio',Odometry),
            '/sensor_reference/odom':('reference',Odometry),
            '/livox/imu':('imu',Imu),'/state/sensor_reference_healthy':('health',Bool)}
    for name in args.bags:
        bag=args.runtime/'bags'/name
        meta=yaml.safe_load((bag/'metadata.yaml').read_text())['rosbag2_bagfile_information']
        row={'duration_s':meta['duration']['nanoseconds']/1e9,
             'start_record_ns':meta['starting_time'].get('nanoseconds_since_epoch',meta['starting_time'].get('nanoseconds')),
             'topic_counts':{t['topic_metadata']['name']:t['message_count'] for t in meta['topics_with_message_count']},
             'extract_counts':{},'state_false_samples':0}
        report['bags'][name]=row
        streams={key:gzip.open(args.output/(name+'_'+key+'.csv.gz'),'wt',newline='') for key in ['lio','reference','imu','health']}
        writers={k:csv.writer(v) for k,v in streams.items()}
        for key in ('lio','reference'):writers[key].writerow(['source_ns','record_ns','x','y','z','qx','qy','qz','qw'])
        writers['imu'].writerow(['source_ns','record_ns','ax','ay','az','gx','gy','gz'])
        writers['health'].writerow(['record_ns','healthy'])
        reader=rosbag2_py.SequentialReader()
        try:
            reader.open(rosbag2_py.StorageOptions(uri=str(bag),storage_id=meta['storage_identifier']),rosbag2_py.ConverterOptions('',''))
            reader.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))
            while reader.has_next():
                topic,data,record_ns=reader.read_next()
                if topic not in topics:continue
                key,cls=topics[topic];m=deserialize_message(data,cls)
                if key in ('lio','reference'):
                    pos,q=m.pose.pose.position,m.pose.pose.orientation
                    values=[stamp(m),record_ns,pos.x,pos.y,pos.z,q.x,q.y,q.z,q.w]
                elif key=='imu':
                    a,g=m.linear_acceleration,m.angular_velocity
                    values=[stamp(m),record_ns,a.x,a.y,a.z,g.x,g.y,g.z]
                else:
                    values=[record_ns,int(m.data)];row['state_false_samples']+=int(not m.data)
                writers[key].writerow(values)
                row['extract_counts'][key]=row['extract_counts'].get(key,0)+1
        finally:
            for stream in streams.values():stream.close()
    for name in args.rx:
        source=args.runtime/name
        summary=json.loads((source/'result.json').read_text()) if (source/'result.json').exists() else {'recording_still_active':True}
        n=0;lo=math.inf;hi=-math.inf;partial=0
        with gzip.open(args.output/(name+'_encoder.csv.gz'),'wt',newline='') as out:
            writer=csv.writer(out);writer.writerow(['host_received_system_ns','host_received_monotonic_ns','contact_angle_raw'])
            with (source/'frames.jsonl').open() as original:
                for line in original:
                    try:frame=json.loads(line)
                    except json.JSONDecodeError:partial+=1;continue
                    a=frame['feedback']['contact_angle']
                    writer.writerow([frame['host_received_system_ns'],frame['host_received_monotonic_ns'],a])
                    n+=1
                    if math.isfinite(a):lo=min(lo,a);hi=max(hi,a)
        report['serial'][name]={'decoder_result':summary,'extract_rows':n,
            'angle_raw_min_max':[lo,hi] if math.isfinite(lo) else None,
            'partial_json_lines':partial}
    (args.output/'summary.json').write_text(json.dumps(report,indent=2)+'\n')
    archive=args.output.with_suffix('.tar.gz')
    if archive.exists():raise FileExistsError(archive)
    with tarfile.open(archive,'w:gz') as tar:tar.add(args.output,arcname=args.output.name)
    print(json.dumps(report,indent=2));print('compact_archive='+str(archive),flush=True)


if __name__=='__main__':main()
