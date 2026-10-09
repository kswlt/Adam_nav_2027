#!/usr/bin/env python3
"""Inspect raw return tags and scene geometry without altering sensor data."""
import argparse
from collections import Counter
import json
from pathlib import Path
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu, PointCloud2


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--bag', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(Path(args.bag).resolve(strict=True)),storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    tags, imus, summaries = Counter(), [], []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == '/livox/imu':
            m = deserialize_message(data, Imu)
            imus.append([m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z,
                         m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z])
        elif topic == '/livox/lidar':
            m = deserialize_message(data, PointCloud2)
            fields = {f.name:f for f in m.fields}
            dtype = np.dtype({'names':['x','y','z','tag'],'formats':['<f4','<f4','<f4','u1'],
                              'offsets':[fields[k].offset for k in ['x','y','z','tag']],
                              'itemsize':m.point_step})
            points = np.frombuffer(m.data,dtype=dtype,count=m.width*m.height)
            values, counts = np.unique(points['tag'],return_counts=True)
            tags.update({int(v):int(n) for v,n in zip(values,counts)})
            xyz = np.column_stack([points[k] for k in ['x','y','z']])
            ranges = np.linalg.norm(xyz,axis=1)
            valid = ((points['tag'] & 0x3f) == 0) & (ranges >= .5) & (ranges <= 30) & np.isfinite(xyz).all(axis=1)
            xyz = xyz[valid]
            eigen = np.linalg.eigvalsh(np.cov(xyz.T)).tolist() if len(xyz)>3 else []
            summaries.append({'stamp_ns':m.header.stamp.sec*10**9+m.header.stamp.nanosec,
                              'accepted_points':int(valid.sum()),'total':len(points),
                              'below_min_range':int((ranges<.5).sum()),
                              'range_quantiles_m':np.quantile(ranges,[0,.25,.5,.75,1]).tolist(),
                              'xyz_cov_eigenvalues':eigen})
    imu = np.asarray(imus)
    report = {'cloud_count':len(summaries),'imu_count':len(imus),'tag_counts':dict(tags),
              'imu_mean_xyz_gyro':imu.mean(axis=0).tolist() if len(imus) else [],
              'imu_std_xyz_gyro':imu.std(axis=0).tolist() if len(imus) else [],
              'accepted_point_range':[min(c['accepted_points'] for c in summaries),
                                      max(c['accepted_points'] for c in summaries)] if summaries else [],
              'first_cloud':summaries[0] if summaries else None,
              'last_cloud':summaries[-1] if summaries else None,
              'scope':'raw diagnostics; geometry covariance is not a localization observability proof'}
    with Path(args.output).open('x') as f: json.dump(report,f,indent=2); f.write('\n')
    print(json.dumps(report,indent=2))


if __name__ == '__main__': main()
