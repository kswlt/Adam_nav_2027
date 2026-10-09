#!/usr/bin/env python3
"""Create a deterministic, explicitly draft MapBundle manifest from optimizer output."""
import argparse
import hashlib
import json
from pathlib import Path
import os
import tempfile


def sha256(path):
    digest=hashlib.sha256()
    with path.open('rb') as handle:
        for chunk in iter(lambda: handle.read(1024*1024), b''): digest.update(chunk)
    return digest.hexdigest()


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--optimized-dir',required=True);parser.add_argument('--output',required=True)
    args=parser.parse_args(); source=Path(args.optimized_dir).resolve(strict=True); output=Path(args.output).resolve()
    if output.exists(): raise FileExistsError('Output manifest exists; refusing overwrite')
    poses=source/'optimized_poses.json'; map_pcd=source/'rebuilt_map.pcd'; loops=source/'loop_edges.json'; hashes=source/'input_hashes.json'
    for path in (poses,map_pcd,loops,hashes):
        if not path.is_file() or path.is_symlink(): raise ValueError(f'Missing or symlinked artifact: {path.name}')
    pose_data=json.loads(poses.read_text());loop_data=json.loads(loops.read_text());hash_data=json.loads(hashes.read_text())
    if pose_data.get('coordinate_frame')!='mapping_odom' or pose_data.get('official_alignment_applied') is not False:
        raise ValueError('Optimizer output must explicitly remain mapping_odom and unaligned')
    if not isinstance(loop_data,list) or not isinstance(hash_data,list) or not 1<=len(pose_data.get('poses',[]))<=500:
        raise ValueError('Invalid bounded optimizer artifact')
    pcd_hash=sha256(map_pcd); version='mapping-odom-'+pcd_hash[:16]
    manifest={'schema':1,'status':'draft_mapping_odom','version':version,'coordinate_frame':'mapping_odom',
              'official_alignment_applied':False,'localization_map':map_pcd.name,'base_occupancy':'',
              'terrain_reference':'','semantic_terrain':'','metadata':'optimized_poses.json',
              'files':{'rebuilt_map.pcd':pcd_hash,'optimized_poses.json':sha256(poses),
                       'loop_edges.json':sha256(loops),'input_hashes.json':sha256(hashes)},
              'pose_count':len(pose_data['poses']),'loop_count':len(loop_data),'input_frame_count':len(hash_data),
              'source':'rm_nav_mapping/offline_graph_optimizer','publishable_for_frozen_localization':False}
    output.parent.mkdir(parents=True,exist_ok=True)
    fd,tmp=tempfile.mkstemp(prefix=output.name+'.',suffix='.partial',dir=str(output.parent))
    try:
        with os.fdopen(fd,'w',encoding='utf-8') as handle:
            json.dump(manifest,handle,indent=2,sort_keys=True);handle.write('\n');handle.flush();os.fsync(handle.fileno())
        os.replace(tmp,output)
    finally:
        if os.path.exists(tmp): os.unlink(tmp)
    print(json.dumps(manifest,indent=2))


if __name__=='__main__': main()
