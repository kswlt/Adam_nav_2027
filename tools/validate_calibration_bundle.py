#!/usr/bin/env python3
"""Validate measured CalibrationBundle before production local_state launch."""
import argparse
import json
import math
from pathlib import Path
import yaml


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--file',required=True);parser.add_argument('--report',default='')
    args=parser.parse_args(); path=Path(args.file).resolve(strict=True); data=yaml.safe_load(path.read_text())
    errors=[]
    if data.get('schema_version')!=1:errors.append('schema_version must be 1')
    if data.get('status')!='verified':errors.append('status must be verified after measurement review')
    if not isinstance(data.get('bundle_id'),str) or not data['bundle_id'] or data['bundle_id']=='pending_measurement':errors.append('bundle_id is not measured')
    frames=data.get('frames',{}); names=[frames.get(k) for k in ('base_footprint','chassis','big_gimbal_yaw','imu','lidar')]
    if any(not isinstance(v,str) or not v for v in names):errors.append('all five frames are required')
    if len(set(names))!=5 or 'base_link' in names:errors.append('frames must be unique and base_link-free')
    encoder=data.get('encoder',{}); sign=encoder.get('sign'); offset=encoder.get('zero_offset')
    if encoder.get('joint')!=frames.get('big_gimbal_yaw'):errors.append('encoder joint must be big_gimbal_yaw frame')
    if sign not in (-1,1) or not isinstance(offset,(int,float)) or not math.isfinite(offset):errors.append('measured encoder sign/zero_offset required')
    transforms=data.get('transforms',{})
    for name in ('footprint_to_chassis','chassis_to_yaw_zero','yaw_to_imu','imu_to_lidar'):
        item=transforms.get(name,{})
        t=item.get('translation');q=item.get('quaternion_xyzw')
        if not isinstance(t,list) or len(t)!=3 or not all(isinstance(v,(int,float)) and math.isfinite(v) for v in t):errors.append(name+' translation invalid')
        if not isinstance(q,list) or len(q)!=4 or not all(isinstance(v,(int,float)) and math.isfinite(v) for v in q) or abs(sum(v*v for v in q)-1)>1e-5:errors.append(name+' unit quaternion required')
    report={'passed':not errors,'file':str(path),'bundle_id':data.get('bundle_id'),'status':data.get('status'),'errors':errors,
            'required_measurements':['four rigid transforms in meters/xyzw','absolute yaw sign and zero offset radians',
                                     'frame names','sensor clock offset and synchronization evidence']}
    if args.report:Path(args.report).resolve().write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2));return 0 if not errors else 1


if __name__=='__main__':raise SystemExit(main())
