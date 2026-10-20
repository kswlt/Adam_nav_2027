#!/usr/bin/env python3
"""Fit rigid SE2 from explicitly verified survey/CAD correspondences (no scale)."""
import argparse
import json
import math
from pathlib import Path
import numpy as np
import yaml

def fit(data,max_rmse=.03):
    if (data.get('status')!='verified' or data.get('source_frame')!='mapping_odom' or
        data.get('target_frame')!='map' or not data.get('official_source')):
        raise ValueError('Verified official correspondences and frame provenance required')
    pairs=data.get('correspondences',[]);height=data.get('height_offset_m')
    if not 3<=len(pairs)<=1000 or not isinstance(height,(float,int)) or not math.isfinite(height) or abs(height)>10:
        raise ValueError('3..1000 correspondences and explicit measured height offset required')
    source=np.asarray([p['mapping_odom'] for p in pairs],dtype=float)
    target=np.asarray([p['map'] for p in pairs],dtype=float)
    if source.shape!=(len(pairs),2) or target.shape!=source.shape or not np.isfinite(source).all() or not np.isfinite(target).all():
        raise ValueError('Finite XY survey points required')
    a=source-source.mean(axis=0);b=target-target.mean(axis=0)
    for centered in (a,b):
        singular=np.linalg.svd(centered,compute_uv=False)
        if singular[-1]<.1 or singular[-1]/singular[0]<.01:raise ValueError('Survey baseline/geometry is degenerate')
    u,_,vt=np.linalg.svd(a.T@b)
    rotation=vt.T@u.T
    if np.linalg.det(rotation)<0:raise ValueError('Reflected coordinate correspondences rejected')
    translation=target.mean(axis=0)-rotation@source.mean(axis=0)
    error=np.linalg.norm(source@rotation.T+translation-target,axis=1)
    rmse=float(np.sqrt(np.mean(error**2)))
    if not math.isfinite(max_rmse) or not 0<max_rmse<=.1 or rmse>max_rmse or error.max()>2*max_rmse:
        raise ValueError('Rigid alignment residual exceeds explicit survey tolerance; scale is never fitted')
    return {'status':'verified','source_frame':'mapping_odom','target_frame':'map',
        'official_source':data['official_source'],'translation':[float(translation[0]),float(translation[1]),float(height)],
        'yaw_rad':math.atan2(rotation[1,0],rotation[0,0]),'scale':1.,
        'survey_rmse_m':rmse,'survey_max_error_m':float(error.max()),'correspondence_count':len(pairs),
        'verification_scope':'rigid fit of operator-verified survey input; does not verify CAD provenance or localization truth'}

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--correspondences',required=True);parser.add_argument('--output',required=True)
    parser.add_argument('--max-rmse',type=float,default=.03);args=parser.parse_args()
    source=Path(args.correspondences).resolve(strict=True)
    if source.stat().st_size>1024*1024:raise ValueError('Survey input too large')
    result=fit(json.loads(source.read_text()),args.max_rmse)
    with Path(args.output).open('x') as stream:yaml.safe_dump(result,stream)
    print(json.dumps(result,ensure_ascii=False,indent=2))
if __name__=='__main__':main()
