#!/usr/bin/env python3
"""Create a deterministic, explicitly draft MapBundle manifest from optimizer output."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import os
import struct
import tempfile


def sha256(path):
    digest=hashlib.sha256()
    with path.open('rb') as handle:
        for chunk in iter(lambda: handle.read(1024*1024), b''): digest.update(chunk)
    return digest.hexdigest()


def validate_binary_xyz_pcd(path):
    """Validate the exact binary XYZ PCD emitted by offline_graph_optimizer."""
    data = path.read_bytes()
    marker = b'DATA binary\n'
    split = data.find(marker)
    if split <= 0:
        raise ValueError('PCD must contain a binary DATA header')
    header = data[:split].decode('ascii', errors='strict')
    fields = {}
    for line in header.splitlines():
        parts = line.split(None, 1)
        if len(parts) == 2:
            fields[parts[0].upper()] = parts[1].strip()
    if fields.get('FIELDS') != 'x y z' or fields.get('SIZE') != '4 4 4' or \
            fields.get('TYPE') != 'F F F' or fields.get('COUNT') != '1 1 1':
        raise ValueError('PCD must be binary float32 XYZ with one value per field')
    try:
        width = int(fields['WIDTH'])
        height = int(fields['HEIGHT'])
        points = int(fields['POINTS'])
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError('PCD width/height/points header is invalid') from error
    if not 1 <= width <= 1_000_000 or height != 1 or points != width:
        raise ValueError('PCD point count is outside the bounded XYZ contract')
    payload = data[split + len(marker):]
    if len(payload) != points * 12:
        raise ValueError('PCD binary payload length does not match POINTS')
    for offset in range(0, len(payload), 12):
        x, y, z = struct.unpack_from('<fff', payload, offset)
        if not all(math.isfinite(value) and abs(value) <= 20_000 for value in (x, y, z)):
            raise ValueError('PCD contains non-finite or out-of-range XYZ')
    return points


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
    poses_data = pose_data.get('poses', [])
    if not pose_data.get('calibration_id') or not pose_data.get('source_id'):
        raise ValueError('Optimizer output must identify calibration and source')
    if not isinstance(loop_data,list) or not isinstance(hash_data,list) or not 1<=len(poses_data)<=500:
        raise ValueError('Invalid bounded optimizer artifact')
    expected_ids = list(range(len(poses_data)))
    if [item.get('id') for item in poses_data] != expected_ids:
        raise ValueError('Optimized pose IDs must be contiguous from zero')
    if len(hash_data) != len(poses_data) or any(
            not isinstance(item, dict) or not isinstance(item.get('id'), int) or
            not isinstance(item.get('stamp_ns'), int) or item.get('stamp_ns') <= 0 or
            not isinstance(item.get('observation_sha256'), str) or
            len(item['observation_sha256']) != 64 or
            any(c not in '0123456789abcdef' for c in item['observation_sha256'].lower())
            for item in hash_data):
        raise ValueError('input_hashes.json does not contain valid source hashes')
    if [item['id'] for item in hash_data] != expected_ids:
        raise ValueError('input_hashes.json IDs do not match optimized poses')
    point_count = validate_binary_xyz_pcd(map_pcd)
    pcd_hash=sha256(map_pcd); version='mapping-odom-'+pcd_hash[:16]
    artifacts = {
        'rebuilt_map.pcd': {'path': 'rebuilt_map.pcd', 'sha256': pcd_hash},
        'optimized_poses.json': {'path': 'optimized_poses.json', 'sha256': sha256(poses)},
        'loop_edges.json': {'path': 'loop_edges.json', 'sha256': sha256(loops)},
        'input_hashes.json': {'path': 'input_hashes.json', 'sha256': sha256(hashes)},
    }
    manifest={'schema':1,'status':'draft_mapping_odom','version':version,'coordinate_frame':'mapping_odom',
              'official_alignment_applied':False,'localization_map':map_pcd.name,'base_occupancy':'',
              'terrain_reference':'','semantic_terrain':'','metadata':'optimized_poses.json',
              'files':{name:item['sha256'] for name,item in artifacts.items()}, 'artifacts':artifacts,
              'pose_count':len(poses_data),'point_count':point_count,'loop_count':len(loop_data),'input_frame_count':len(hash_data),
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
