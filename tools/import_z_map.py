#!/usr/bin/env python3
"""Import an actual Z map snapshot as an RM draft; never publish automatically."""
import argparse
import json
from pathlib import Path
import shutil
import tempfile
import os
import struct
import math
from create_map_bundle import sha256, validate_binary_xyz_pcd
from publish_map_bundle import write_pcd


def read_native_map(path):
    """Read upstream binary XYZ/XYZI without treating intensity as geometry or editing native data."""
    if path.is_symlink() or path.stat().st_size > 64*1024*1024:
        raise ValueError('Invalid/oversized native PCD')
    data = path.read_bytes()
    split = data.find(b'DATA binary\n')
    if not 0 < split < 65536:
        raise ValueError('Native map must use bounded binary PCD header')
    fields = {}
    for line in data[:split].decode('ascii').splitlines():
        row = line.split(None,1)
        if len(row)==2: fields[row[0]] = row[1]
    names = fields.get('FIELDS','').split()
    if names not in (['x','y','z'], ['x','y','z','intensity']):
        raise ValueError('Unsupported native PCD fields')
    dimensions = len(names)
    if (fields.get('SIZE','').split() != ['4']*dimensions or
            fields.get('TYPE','').split() != ['F']*dimensions or
            fields.get('COUNT','').split() != ['1']*dimensions):
        raise ValueError('Native PCD requires float32 XYZ/XYZI')
    count = int(fields.get('POINTS','0'))
    payload = data[split+len(b'DATA binary\n'):]
    if (not 1 <= count <= 1000000 or int(fields.get('WIDTH','0'))!=count or
            fields.get('HEIGHT')!='1' or len(payload)!=count*4*dimensions):
        raise ValueError('Native PCD dimensions/payload invalid')
    points = [tuple(row[:3]) for row in struct.iter_unpack('<'+'f'*dimensions,payload)]
    if any(not all(math.isfinite(v) and abs(v)<=20000 for v in p) for p in points):
        raise ValueError('Native PCD has invalid geometry')
    return points


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--native-bundle', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--calibration-id', required=True)
    args = parser.parse_args()
    source = Path(args.native_bundle).resolve(strict=True)
    output = Path(args.output).absolute()
    if output.exists():
        raise FileExistsError('Draft output already exists; choose a new version')
    if not (source/'keyframes.txt').is_file() or not (source/'clouds').is_dir():
        raise ValueError('Native Z-Localization bundle required')
    points = read_native_map(source/'map.pcd')
    count = len(points)
    rows = [line for line in (source/'keyframes.txt').read_text().splitlines() if line.strip() and not line.lstrip().startswith('#')]
    if not rows:
        raise ValueError('No saved keyframes')
    output.parent.mkdir(parents=True, exist_ok=True)
    staging = Path(tempfile.mkdtemp(prefix=output.name+'.partial-', dir=output.parent))
    try:
        write_pcd(staging/'rebuilt_map.pcd', points, 'mapping_odom; numeric Z mapping-session coordinates')
        validate_binary_xyz_pcd(staging/'rebuilt_map.pcd')
        metadata = {'source': 'z-localization/backend', 'source_coordinate_frame': 'map (mapping session)',
                    'calibration_id': args.calibration_id, 'native_bundle': str(source),
                    'pose_count': len(rows), 'accuracy_verified': False,
                    'note': 'RM mapping_odom denotes this local mapping session; no official alignment applied'}
        (staging/'z_source.json').write_text(json.dumps(metadata, indent=2)+'\n')
        artifacts = {name: {'path': name, 'sha256': sha256(staging/name)}
                     for name in ('rebuilt_map.pcd', 'z_source.json')}
        manifest = {'schema': 1, 'status': 'draft_mapping_odom', 'coordinate_frame': 'mapping_odom',
                    'official_alignment_applied': False, 'publishable_for_frozen_localization': False,
                    'version': 'z-session-'+artifacts['rebuilt_map.pcd']['sha256'][:16],
                    'source': 'z-localization/backend', 'calibration_id': args.calibration_id,
                    'source_coordinate_frame': 'map (mapping session)',
                    'z_native_map_sha256': sha256(source/'map.pcd'),
                    'localization_map': 'rebuilt_map.pcd', 'artifacts': artifacts,
                    'pose_count': len(rows), 'point_count': count}
        (staging/'map_bundle.json').write_text(json.dumps(manifest, indent=2)+'\n')
        os.rename(staging, output)
    except BaseException:
        shutil.rmtree(staging)
        raise
    print(output/'map_bundle.json')


if __name__ == '__main__':
    main()
