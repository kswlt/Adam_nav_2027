#!/usr/bin/env python3
import json, struct, subprocess, tempfile
from pathlib import Path

def main():
    Path('log').mkdir(exist_ok=True)
    root=Path(tempfile.mkdtemp(prefix='publish_bundle_',dir='log')); opt=root/'optimized'; opt.mkdir()
    pcd=opt/'rebuilt_map.pcd'; pts=[(-1.,-1.,0.),(1.,-1.,.1),(-1.,1.,.2),(1.,1.,.3)]
    pcd.write_bytes(b'# mapping_odom\nVERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH 4\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS 4\nDATA binary\n'+b''.join(struct.pack('<fff',*p) for p in pts))
    import hashlib
    h=hashlib.sha256(pcd.read_bytes()).hexdigest()
    for name,data in [('optimized_poses.json',{'poses':[]}),('loop_edges.json',[]),('input_hashes.json',[])]: (opt/name).write_text(json.dumps(data))
    draft={'schema':1,'status':'draft_mapping_odom','version':'mapping-odom-fixture','coordinate_frame':'mapping_odom','official_alignment_applied':False,'publishable_for_frozen_localization':False,'artifacts':{'rebuilt_map.pcd':{'path':'optimized/rebuilt_map.pcd','sha256':h}}}
    draft_path=root/'draft.json'; draft_path.write_text(json.dumps(draft)); align=root/'alignment.yaml'; align.write_text('status: verified\nsource_frame: mapping_odom\ntarget_frame: map\nofficial_source: test-cad-v1\ntranslation: [2.0, 3.0, 0.0]\nyaw_rad: 0.0\nscale: 1.0\n')
    out=root/'published'; result=subprocess.run(['python3','tools/publish_map_bundle.py','--draft',str(draft_path),'--alignment',str(align),'--output',str(out)],capture_output=True,text=True)
    if result.returncode: raise RuntimeError(result.stdout+result.stderr)
    manifest=json.loads((out/'map_bundle.json').read_text()); assert manifest['status']=='published' and manifest['coordinate_frame']=='map' and manifest['publishable_for_frozen_localization']
    assert (out/'maps'/'localization_map.pcd').is_file() and (out/'maps'/'occupancy.pgm').is_file() and (out/'maps'/'terrain_reference.json').is_file()
    print('PASS official alignment, occupancy, terrain and atomic published MapBundle; evidence='+str(root))
if __name__=='__main__': main()
