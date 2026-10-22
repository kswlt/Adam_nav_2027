#!/usr/bin/env python3
"""Derived MCAP scheduler based on acquisition times; original CDR/header/frame never edited.

Recorded receive delays must not be mistaken for missing sensor measurements.
Every original receive time is retained in CSV. This is OFFLINE analysis only.
"""
import argparse
import csv
import heapq
import json
from pathlib import Path
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu, PointCloud2


def acquisition_end(topic, data):
    message = deserialize_message(data, Imu if topic == '/livox/imu' else PointCloud2)
    stamp = message.header.stamp.sec*10**9+message.header.stamp.nanosec
    if stamp <= 0: raise ValueError('Invalid source header stamp')
    if topic == '/livox/imu': return stamp
    fields = [f for f in message.fields if f.name == 'timestamp']
    if len(fields) != 1 or fields[0].datatype != 8 or fields[0].count != 1 or message.height != 1 or message.is_bigendian:
        raise ValueError('Requires pinned native MID360 float64 absolute-nanosecond point times')
    if message.width < 1 or len(message.data) < message.width*message.point_step or fields[0].offset+8 > message.point_step:
        raise ValueError('Invalid native cloud layout')
    times = np.ndarray((message.width,), dtype='<f8', buffer=message.data,
                       offset=fields[0].offset, strides=(message.point_step,))
    if (not np.isfinite(times).all() or times.min() < stamp-200000000 or times.max() > stamp+200000000):
        raise ValueError('Point times outside native scan acquisition interval')
    return max(stamp, round(float(times.max())))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    source = Path(args.bag).resolve(strict=True); output = Path(args.output).absolute()
    if output.exists(): raise FileExistsError('Derived output exists; original and previous evidence are retained')
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(source), storage_id='mcap'), rosbag2_py.ConverterOptions('',''))
    names = ['/livox/lidar','/livox/imu']; counts = dict.fromkeys(names,0)
    reader.set_filter(rosbag2_py.StorageFilter(topics=names))
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(output), storage_id='mcap'), rosbag2_py.ConverterOptions('',''))
    for topic in reader.get_all_topics_and_types():
        if topic.name in names: writer.create_topic(topic)
    heap = []; sequence = 0; maximum = last_written = 0; max_buffer = 0
    with (output/'source_receive_times.csv').open('w',newline='') as stream:
        records = csv.writer(stream)
        records.writerow(['original_sequence','topic','original_receive_ns','derived_schedule_ns'])
        def flush(limit):
            nonlocal last_written
            while heap and heap[0][0] <= limit:
                schedule, order, topic, data, received = heapq.heappop(heap)
                if schedule < last_written: raise ValueError('Source reorder exceeds bounded 1-second buffer')
                writer.write(topic, data, schedule)  # exact original serialized bytes
                records.writerow([order,topic,received,schedule])
                last_written = schedule; counts[topic] += 1
        while reader.has_next():
            topic, data, received = reader.read_next()
            schedule = acquisition_end(topic,data)
            sequence += 1
            heapq.heappush(heap,(schedule,sequence,topic,data,received))
            maximum = max(maximum,schedule)
            max_buffer = max(max_buffer,len(heap))
            if len(heap) > 10000: raise ValueError('Source buffering exceeds offline memory bound')
            flush(maximum-1000000000)
        flush(2**63-1)
    del writer
    if not all(counts.values()): raise ValueError('Incomplete source topics')
    report = {'original_bag':str(source),'derived_bag':str(output),'message_counts':counts,
              'original_serialized_messages_preserved':True,'header_and_frame_unchanged':True,
              'receive_times_preserved_in':'source_receive_times.csv', 'peak_buffered_messages':max_buffer,
              'schedule':'IMU header; LiDAR last original point time (scan completion); sorted with bounded 1s buffer',
              'scope':'OFFLINE diagnostic scheduling only; not live latency or physical navigation acceptance'}
    (output/'scheduling.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


if __name__ == '__main__': main()
