#!/usr/bin/env python3
"""Bounded read-only runtime resource monitor for ROS navigation processes."""
import argparse
import json
import os
import re
import shutil
import time
from pathlib import Path
import psutil


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--duration',type=float,default=30.);parser.add_argument('--output',required=True)
    parser.add_argument('--pattern',default='livox_ros_driver2|small_point_lio|foxglove_bridge|live_map_preview|rm_nav')
    args=parser.parse_args()
    if not 5<=args.duration<=300:raise ValueError('duration must be 5..300 seconds')
    regex=re.compile(args.pattern); output=Path(args.output).resolve()
    if output.exists():raise FileExistsError('Output exists; refusing overwrite')
    samples=[];start=time.monotonic();cpu_procs={}
    while time.monotonic()-start<args.duration:
        processes=[]
        for proc in psutil.process_iter(['pid','name','cmdline']):
            try:
                cmd=' '.join(proc.info.get('cmdline') or [])
                if regex.search(cmd):
                    with proc.oneshot():
                        tracked=cpu_procs.setdefault(proc.pid,psutil.Process(proc.pid))
                        tracked.cpu_percent(interval=None)
                        time.sleep(.05)
                        cpu=tracked.cpu_percent(interval=None);mem=proc.memory_info().rss;threads=proc.num_threads()
                    processes.append({'pid':proc.pid,'cmdline':cmd[:500],'cpu_percent':cpu,'rss_bytes':mem,'threads':threads})
            except (psutil.NoSuchProcess,psutil.AccessDenied,psutil.ZombieProcess):continue
        disk=shutil.disk_usage('/')
        samples.append({'elapsed_s':time.monotonic()-start,'processes':processes,
                        'disk_free_bytes':disk.free,'disk_used_percent':disk.used*100/disk.total})
        time.sleep(.45)
    all_proc=[p for s in samples for p in s['processes']]
    summary={}
    for p in all_proc:
        item=summary.setdefault(str(p['pid']),{'pid':p['pid'],'cmdline':p['cmdline'],'cpu_max_percent':0.,'rss_max_bytes':0,'threads_max':0,'samples':0})
        item['cpu_max_percent']=max(item['cpu_max_percent'],p['cpu_percent']);item['rss_max_bytes']=max(item['rss_max_bytes'],p['rss_bytes']);item['threads_max']=max(item['threads_max'],p['threads']);item['samples']+=1
    report={'passed':bool(samples),'duration_s':args.duration,'pattern':args.pattern,'processes':list(summary.values()),
            'disk_free_bytes_min':min(s['disk_free_bytes'] for s in samples),'disk_used_percent_max':max(s['disk_used_percent'] for s in samples),
            'samples':samples,'scope':'read-only resource baseline; not a real-time schedulability proof'}
    output.parent.mkdir(parents=True,exist_ok=True);output.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))


if __name__=='__main__':main()
