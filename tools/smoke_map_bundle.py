#!/usr/bin/env python3
"""Validate draft MapBundle manifest creation and immutable artifact checks."""
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import tempfile


def require(value, message):
    if not value: raise RuntimeError(message)


def main():
    root=Path(tempfile.mkdtemp(prefix='map_bundle_',dir=Path('log').resolve()))
    source=root/'optimized';source.mkdir()
    (source/'optimized_poses.json').write_text(json.dumps({'coordinate_frame':'mapping_odom','official_alignment_applied':False,'poses':[{'id':0,'matrix':[1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1]}]}))
    (source/'loop_edges.json').write_text('[]\n');(source/'input_hashes.json').write_text('[]\n')
    (source/'rebuilt_map.pcd').write_bytes(b'DATA binary\nfixture')
    manifest=root/'map_bundle.json'; command=['python3','tools/create_map_bundle.py','--optimized-dir',str(source),'--output',str(manifest)]
    result=subprocess.run(command,capture_output=True,text=True);require(result.returncode==0,result.stdout+result.stderr)
    data=json.loads(manifest.read_text());require(data['status']=='draft_mapping_odom' and not data['publishable_for_frozen_localization'],'Draft status was not conservative')
    require(data['files']['rebuilt_map.pcd']==hashlib.sha256((source/'rebuilt_map.pcd').read_bytes()).hexdigest(),'PCD hash mismatch')
    repeated=subprocess.run(command,capture_output=True,text=True);require(repeated.returncode!=0,'Manifest overwrite accepted')
    broken=root/'broken';shutil.copytree(source,broken);(broken/'rebuilt_map.pcd').unlink()
    rejected=subprocess.run(['python3','tools/create_map_bundle.py','--optimized-dir',str(broken),'--output',str(root/'broken.json')],capture_output=True,text=True)
    require(rejected.returncode!=0 and not (root/'broken.json').exists(),'Missing PCD accepted')
    print('PASS draft MapBundle hash, conservative publish gate, overwrite and missing-artifact refusal; evidence='+str(root))


if __name__=='__main__': main()
