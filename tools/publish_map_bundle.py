#!/usr/bin/env python3
"""Promote a draft mapping_odom bundle only with verified official alignment data."""
import argparse, hashlib, json, math, os, shutil, struct, tempfile
from pathlib import Path
try:
    import yaml
except ImportError:
    yaml=None

def load_alignment(path):
    if yaml is not None: return yaml.safe_load(path.read_text())
    result={}
    for line in path.read_text().splitlines():
        if not line.strip() or line.lstrip().startswith('#') or ':' not in line: continue
        key,value=line.split(':',1); value=value.strip();
        if value.startswith('['): result[key.strip()]=json.loads(value.replace("'",'"'))
        elif value.lower() in ('true','false'): result[key.strip()]=value.lower()=='true'
        else:
            try: result[key.strip()]=float(value)
            except ValueError: result[key.strip()]=value
    return result

def digest(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for chunk in iter(lambda:f.read(1024*1024),b''): h.update(chunk)
    return h.hexdigest()

def read_pcd(path):
    raw=path.read_bytes(); marker=b'DATA binary\n'; split=raw.find(marker)
    if split<0: raise ValueError('Only binary XYZ PCD is supported')
    header=raw[:split].decode('ascii'); fields={}
    for line in header.splitlines():
        p=line.split(None,1)
        if len(p)==2: fields[p[0].upper()]=p[1].strip()
    if fields.get('FIELDS')!='x y z' or fields.get('SIZE')!='4 4 4' or fields.get('TYPE')!='F F F' or fields.get('COUNT')!='1 1 1': raise ValueError('PCD is not binary XYZ float32')
    n=int(fields.get('POINTS','0')); payload=raw[split+len(marker):]
    if n<1 or n>10000000 or len(payload)!=12*n: raise ValueError('PCD point count or payload invalid')
    points=[struct.unpack_from('<fff',payload,i*12) for i in range(n)]
    if any(not all(math.isfinite(v) and abs(v)<20000 for v in p) for p in points): raise ValueError('PCD contains invalid coordinates')
    return points

def write_pcd(path, points, frame):
    with path.open('wb') as f:
        f.write((f'# {frame}\nVERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(points)}\nDATA binary\n').encode())
        for p in points: f.write(struct.pack('<fff',*p))

def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--draft',required=True); ap.add_argument('--alignment',required=True); ap.add_argument('--output',required=True); ap.add_argument('--resolution',type=float,default=.05); ap.add_argument('--min-z',type=float,default=-1e9); ap.add_argument('--max-z',type=float,default=1e9); args=ap.parse_args()
    draft_path=Path(args.draft).resolve(strict=True); alignment_path=Path(args.alignment).resolve(strict=True); output=Path(args.output).resolve()
    if output.exists(): raise FileExistsError('Published output exists; refusing overwrite')
    draft=json.loads(draft_path.read_text()); align=load_alignment(alignment_path)
    if draft.get('status')!='draft_mapping_odom' or draft.get('coordinate_frame')!='mapping_odom' or draft.get('official_alignment_applied') is not False: raise ValueError('Input must be an unaligned mapping_odom draft')
    if align.get('status')!='verified' or align.get('source_frame')!='mapping_odom' or align.get('target_frame')!='map' or not align.get('official_source'): raise ValueError('Verified official alignment data required')
    tr=align.get('translation'); yaw=align.get('yaw_rad'); scale=align.get('scale',1.0)
    if not isinstance(tr,list) or len(tr)!=3 or not all(math.isfinite(float(v)) for v in tr) or not math.isfinite(float(yaw)) or float(scale)!=1.0: raise ValueError('Alignment must provide finite rigid translation/yaw with scale 1')
    artifacts=draft.get('artifacts',{}); entry=artifacts.get('rebuilt_map.pcd',{}); rel=entry.get('path')
    if not isinstance(rel,str): raise ValueError('Draft PCD reference missing')
    pcd=(draft_path.parent/rel).resolve(); pcd.relative_to(draft_path.parent); 
    if not pcd.is_file() or pcd.is_symlink() or digest(pcd)!=entry.get('sha256'): raise ValueError('Draft PCD missing or hash mismatch')
    points=read_pcd(pcd); c,s=math.cos(float(yaw)),math.sin(float(yaw)); tx,ty,tz=map(float,tr)
    aligned=[(c*x-s*y+tx,s*x+c*y+ty,z+tz) for x,y,z in points]
    if not aligned: raise ValueError('Aligned map is empty')
    res=float(args.resolution)
    if not math.isfinite(res) or res<=0 or res>.5: raise ValueError('Invalid occupancy resolution')
    xmin=min(p[0] for p in aligned); xmax=max(p[0] for p in aligned); ymin=min(p[1] for p in aligned); ymax=max(p[1] for p in aligned)
    width=max(1,int(math.ceil((xmax-xmin)/res))+1); height=max(1,int(math.ceil((ymax-ymin)/res))+1)
    occupied=set(); terrain={}
    for x,y,z in aligned:
        if args.min_z<=z<=args.max_z:
            ix=min(width-1,max(0,int((x-xmin)/res))); iy=min(height-1,max(0,int((y-ymin)/res))); occupied.add((ix,iy)); key=f'{ix},{iy}'; terrain[key]=min(z,terrain.get(key,z))
    staging=Path(tempfile.mkdtemp(prefix=output.name+'.partial-',dir=str(output.parent))); (staging/'maps').mkdir()
    try:
        write_pcd(staging/'maps'/'localization_map.pcd',aligned,'map; official alignment applied')
        (staging/'maps'/'occupancy.pgm').write_bytes(('P5\n# map\n%d %d\n255\n'%(width,height)).encode()+bytes(255 if (x,y) not in occupied else 0 for y in range(height-1,-1,-1) for x in range(width)))
        (staging/'maps'/'terrain_reference.json').write_text(json.dumps({'frame':'map','resolution':res,'origin':[xmin,ymin],'cells':terrain},sort_keys=True)+'\n')
        published={'schema':1,'status':'published','version':'map-'+digest(staging/'maps'/'localization_map.pcd')[:16],'coordinate_frame':'map','official_alignment_applied':True,'publishable_for_frozen_localization':True,'source_draft_version':draft.get('version'),'official_source':align['official_source'],'alignment':{'translation':tr,'yaw_rad':float(yaw),'scale':1.0},'localization_map':'maps/localization_map.pcd','base_occupancy':'maps/occupancy.pgm','terrain_reference':'maps/terrain_reference.json','metadata':'maps/terrain_reference.json','artifacts':{}}
        for p in (staging/'maps').iterdir(): published['artifacts'][p.name]={'path':'maps/'+p.name,'sha256':digest(p)}
        (staging/'map_bundle.json').write_text(json.dumps(published,indent=2,sort_keys=True)+'\n'); staging.rename(output)
        print(json.dumps(published,indent=2))
    except Exception:
        shutil.rmtree(staging,ignore_errors=True); raise
if __name__=='__main__': main()
