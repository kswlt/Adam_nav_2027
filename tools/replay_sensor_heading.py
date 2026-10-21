#!/usr/bin/env python3
"""Replay real recorded LIO arrival/source times through the command-heading fix.

Future LIO interpolation is a comparison reference, not independent ground truth.
Rejected prediction ticks represent zero-output intervals, never repaired poses.
"""
import argparse
import json
from pathlib import Path
import sys

import numpy as np
from analyze_gimbal_recordings import read_bag, yaw_of

sys.path.insert(0,str(Path(__file__).parents[1]/'src/rm_nav_bringup/scripts'))
from sensor_reference_math import HeadingTracker


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--raw-root',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    report={'scope':'recorded LIO arrivals at virtual 50Hz command ticks',
            'comparison':'interpolated future LIO, NOT independent attitude ground truth',
            'physical_output':False,'runs':{}}
    for path in sorted(args.raw_root.glob('run_*/*.mcap')):
        lio=read_bag(path)['lio']
        source,arrival,yaw=lio[:,0],lio[:,1],yaw_of(lio)
        if np.any(np.diff(source)<=0) or np.any(np.diff(arrival)<0):
            raise ValueError('Replay requires monotonic recorded sources and arrivals')
        tracker=HeadingTracker()
        index=0
        errors=[]
        rejected=0
        total=0
        for tick in np.arange(arrival[0],min(source[-1],arrival[-1]),.02):
            while index<len(lio) and arrival[index]<=tick:
                tracker.observe(round(source[index]*1e9),yaw[index])
                index+=1
            total+=1
            try:
                predicted,_,_=tracker.predict(round(tick*1e9))
            except ValueError:
                rejected+=1
                continue
            future=float(np.interp(tick,source,yaw))
            held=yaw[index-1]
            wrap=lambda v: float(np.degrees(np.arctan2(np.sin(v),np.cos(v))))
            errors.append([abs(wrap(held-future)),abs(wrap(predicted-future))])
        a=np.asarray(errors)
        row={'ticks':total,'rejected_ticks':rejected,'accepted_ticks':len(errors),
             'held_yaw_error_deg_p50_p95_max':np.percentile(a[:,0],[50,95,100]).tolist(),
             'bounded_prediction_error_deg_p50_p95_max':np.percentile(a[:,1],[50,95,100]).tolist()}
        report['runs'][path.parent.name]=row
        print(path.parent.name,json.dumps(row),flush=True)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')


if __name__=='__main__':main()
