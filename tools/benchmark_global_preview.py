#!/usr/bin/env python3
"""Linux isolated-process synthetic preview capacity/RSS benchmark, not LIO load."""
import argparse
import json
import os
from pathlib import Path
import resource
import subprocess
import sys
import time
import numpy as np

sys.path.insert(0,str(Path(__file__).parents[1]/'src/rm_nav_bringup/scripts'))
from voxel_preview import GlobalVoxelPreview


def rss():
    return int(Path('/proc/self/statm').read_text().split()[1])*os.sysconf('SC_PAGE_SIZE')


def measure(limit):
    baseline=rss()
    grid=GlobalVoxelPreview(.15,limit)
    times=[]
    peak_points=0
    segments=[(start,min(start+20000,limit)) for start in range(0,limit,20000)]
    segments.append((limit,limit+20000))
    for start,end in segments:
        index=np.arange(start,end,dtype=np.int64)
        xyz=np.column_stack([index%100,(index//100)%100,index//10000]).astype(np.float32)*.2+.025
        before=time.perf_counter()
        grid.add(xyz)
        times.append(time.perf_counter()-before)
        peak_points=max(peak_points,len(grid))
    return {'scope':'synthetic 20k-point batches in isolated Python process; no ROS/LIO/browser',
            'capacity':limit,'baseline_rss_bytes':baseline,'final_rss_bytes':rss(),
            'process_peak_rss_bytes':resource.getrusage(resource.RUSAGE_SELF).ru_maxrss*1024,
            'retained_points':len(grid),'largest_retained_snapshot_points':peak_points,
            'coarsenings':grid.coarsenings,'effective_voxel_m':grid.voxel,
            'batch_seconds_p50_p95_max':np.percentile(times,[50,95,100]).tolist(),
            'largest_snapshot_xyz_bytes':peak_points*12}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capacity',type=int)
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    if args.capacity:
        print(json.dumps(measure(args.capacity)))
        return
    results=[json.loads(subprocess.check_output([sys.executable,__file__,'--capacity',str(limit)],text=True))
             for limit in (50000,200000)]
    if args.output:
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(results,indent=2)+'\n')
    print(json.dumps(results,indent=2))


if __name__=='__main__':main()
