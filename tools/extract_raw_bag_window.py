#!/usr/bin/env python3
"""Extract a separate raw-input MCAP window, preserving serialized bytes and log times."""
import argparse
import json
import math
from pathlib import Path
import rosbag2_py
import yaml


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--bag', type=Path, required=True)
    ap.add_argument('--output', type=Path, required=True)
    ap.add_argument('--start-seconds', type=float, required=True)
    ap.add_argument('--duration', type=float, required=True)
    args = ap.parse_args()
    if (not math.isfinite(args.start_seconds) or args.start_seconds < 0 or
            not math.isfinite(args.duration) or not 0 < args.duration <= 3600):
        raise ValueError('Require finite nonnegative start and positive duration <=3600s')
    if args.output.exists():
        raise ValueError('Output must not exist; original bags are never modified')
    info = yaml.safe_load((args.bag/'metadata.yaml').read_text())['rosbag2_bagfile_information']
    start = info['starting_time']['nanoseconds_since_epoch']+round(args.start_seconds*10**9)
    end = start+round(args.duration*10**9)
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(args.bag), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    names = ['/livox/lidar', '/livox/imu']
    reader.set_filter(rosbag2_py.StorageFilter(topics=names))
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(args.output), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    for topic in reader.get_all_topics_and_types():
        if topic.name in names:
            writer.create_topic(topic)
    counts = dict.fromkeys(names, 0)
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if start <= stamp < end:
            writer.write(topic, data, stamp)
            counts[topic] += 1
    del writer
    if not all(counts.values()):
        raise ValueError(f'Incomplete raw inputs, partial derived bag retained: {counts}')
    report = {'original_bag': str(args.bag), 'derived_bag': str(args.output),
              'start_log_time_ns': start, 'end_log_time_ns_exclusive': end,
              'message_counts': counts, 'serialized_bytes_and_log_times_preserved': True,
              'scope': 'raw inputs only; no recorded estimates, TF, health or control'}
    (args.output/'extraction.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
