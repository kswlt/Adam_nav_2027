#!/usr/bin/env python3
"""Validate draft MapBundle manifest creation and immutable artifact checks."""
import hashlib
import json
from pathlib import Path
import shutil
import struct
import subprocess
import tempfile


def require(value, message):
    if not value: raise RuntimeError(message)


def main():
    root=Path(tempfile.mkdtemp(prefix='map_bundle_',dir=Path('log').resolve()))
    source=root/'optimized';source.mkdir()
    (source/'optimized_poses.json').write_text(json.dumps({'coordinate_frame':'mapping_odom','official_alignment_applied':False,
        'calibration_id':'fixture-calibration','source_id':'fixture-source',
        'poses':[{'id':0,'matrix':[1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1]}]}))
    (source/'loop_edges.json').write_text('[]\n')
    (source/'input_hashes.json').write_text(json.dumps([{'id':0,'stamp_ns':1,'observation_sha256':'0'*64}]))
    point = struct.pack('<fff', 0.0, 0.0, 0.0)
    (source/'rebuilt_map.pcd').write_bytes(
        b'# mapping_odom; official alignment pending\nVERSION .7\nFIELDS x y z\n'
        b'SIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH 1\nHEIGHT 1\n'
        b'VIEWPOINT 0 0 0 1 0 0 0\nPOINTS 1\nDATA binary\n' + point)
    manifest=root/'map_bundle.json'; command=['python3','tools/create_map_bundle.py','--optimized-dir',str(source),'--output',str(manifest)]
    result=subprocess.run(command,capture_output=True,text=True);require(result.returncode==0,result.stdout+result.stderr)
    data=json.loads(manifest.read_text());require(data['status']=='draft_mapping_odom' and not data['publishable_for_frozen_localization'],'Draft status was not conservative')
    require(data['files']['rebuilt_map.pcd']==hashlib.sha256((source/'rebuilt_map.pcd').read_bytes()).hexdigest(),'PCD hash mismatch')
    repeated=subprocess.run(command,capture_output=True,text=True);require(repeated.returncode!=0,'Manifest overwrite accepted')
    broken=root/'broken';shutil.copytree(source,broken);(broken/'rebuilt_map.pcd').unlink()
    rejected=subprocess.run(['python3','tools/create_map_bundle.py','--optimized-dir',str(broken),'--output',str(root/'broken.json')],capture_output=True,text=True)
    require(rejected.returncode!=0 and not (root/'broken.json').exists(),'Missing PCD accepted')
    corrupt=root/'corrupt';shutil.copytree(source,corrupt)
    (corrupt/'rebuilt_map.pcd').write_bytes((corrupt/'rebuilt_map.pcd').read_bytes()[:-1])
    rejected=subprocess.run(['python3','tools/create_map_bundle.py','--optimized-dir',str(corrupt),'--output',str(root/'corrupt.json')],capture_output=True,text=True)
    require(rejected.returncode!=0 and not (root/'corrupt.json').exists(),'Corrupt PCD accepted')
    print('PASS binary XYZ PCD validation, draft MapBundle hash, conservative publish gate, overwrite and invalid-artifact refusal; evidence='+str(root))


if __name__=='__main__': main()
