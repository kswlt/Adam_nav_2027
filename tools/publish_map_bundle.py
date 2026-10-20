#!/usr/bin/env python3
"""Promote a draft mapping_odom bundle only with verified official alignment data."""
import argparse, hashlib, json, math, os, shutil, struct, tempfile
from pathlib import Path
from create_map_bundle import validate_binary_xyz_pcd
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
    validate_binary_xyz_pcd(path)
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

def load_free_space(mask_path, metadata_path, official_source):
    """An endpoint PCD cannot prove free space; use a reviewed official map mask."""
    from PIL import Image
    metadata=load_alignment(metadata_path)
    if (metadata.get('status')!='verified' or metadata.get('coordinate_frame')!='map' or
        metadata.get('official_source')!=official_source): raise ValueError('Verified map-frame official mask metadata required')
    resolution=metadata.get('resolution'); origin=metadata.get('origin')
    if (not isinstance(resolution,(float,int)) or not math.isfinite(resolution) or not 0<resolution<=.5 or
        not isinstance(origin,list) or len(origin)!=3 or not all(isinstance(v,(float,int)) and math.isfinite(v) for v in origin) or origin[2]!=0):
        raise ValueError('Mask needs finite axis-aligned map geometry')
    with Image.open(mask_path) as image:
        width,height=image.size
        if image.format!='PPM' or image.mode!='L' or width*height>4000000 or width*height<1:
            raise ValueError('Mask must be bounded 8-bit grayscale PGM')
        pixels=list(image.getdata())
    normalized=[255 if v>=250 else 0 if v<=5 else 205 for v in pixels]
    if 255 not in normalized: raise ValueError('Official mask has no free-space evidence')
    # PGM starts at the top row; OccupancyGrid starts at the bottom row.
    cells=bytearray(v for row in range(height-1,-1,-1) for v in normalized[row*width:(row+1)*width])
    return metadata,width,height,float(resolution),float(origin[0]),float(origin[1]),cells

def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--draft',required=True); ap.add_argument('--alignment',required=True); ap.add_argument('--output',required=True); ap.add_argument('--resolution',type=float,default=.05); ap.add_argument('--min-z',type=float,default=.15); ap.add_argument('--max-z',type=float,default=2.0); ap.add_argument('--free-space-mask'); ap.add_argument('--free-space-metadata'); args=ap.parse_args()
    draft_path=Path(args.draft).resolve(strict=True); alignment_path=Path(args.alignment).resolve(strict=True); output=Path(args.output).resolve()
    if output.exists(): raise FileExistsError('Published output exists; refusing overwrite')
    draft=json.loads(draft_path.read_text()); align=load_alignment(alignment_path)
    if draft.get('status')!='draft_mapping_odom' or draft.get('coordinate_frame')!='mapping_odom' or draft.get('official_alignment_applied') is not False: raise ValueError('Input must be an unaligned mapping_odom draft')
    if align.get('status')!='verified' or align.get('source_frame')!='mapping_odom' or align.get('target_frame')!='map' or not align.get('official_source'): raise ValueError('Verified official alignment data required')
    tr=align.get('translation'); yaw=align.get('yaw_rad'); scale=align.get('scale',1.0)
    if not isinstance(tr,list) or len(tr)!=3 or not all(math.isfinite(float(v)) for v in tr) or not math.isfinite(float(yaw)) or float(scale)!=1.0: raise ValueError('Alignment must provide finite rigid translation/yaw with scale 1')
    artifacts=draft.get('artifacts',{}); entry=artifacts.get('rebuilt_map.pcd',{}); rel=entry.get('path')
    if not isinstance(rel,str): raise ValueError('Draft PCD reference missing')
    relative=Path(rel)
    if relative.is_absolute() or '..' in relative.parts: raise ValueError('Draft PCD path must remain inside the bundle')
    pcd=draft_path.parent
    for part in relative.parts:
        pcd=pcd/part
        if pcd.is_symlink(): raise ValueError('Draft PCD path contains a symlink')
    pcd=pcd.resolve(strict=True); pcd.relative_to(draft_path.parent)
    if not pcd.is_file() or digest(pcd)!=entry.get('sha256'): raise ValueError('Draft PCD missing or hash mismatch')
    points=read_pcd(pcd); c,s=math.cos(float(yaw)),math.sin(float(yaw)); tx,ty,tz=map(float,tr)
    aligned=[(c*x-s*y+tx,s*x+c*y+ty,z+tz) for x,y,z in points]
    if not aligned: raise ValueError('Aligned map is empty')
    res=float(args.resolution)
    if not math.isfinite(res) or res<=0 or res>.5: raise ValueError('Invalid occupancy resolution')
    if not math.isfinite(args.min_z) or not math.isfinite(args.max_z) or args.min_z>=args.max_z: raise ValueError('Invalid obstacle height interval')
    xmin=min(p[0] for p in aligned); xmax=max(p[0] for p in aligned); ymin=min(p[1] for p in aligned); ymax=max(p[1] for p in aligned)
    width=max(1,int(math.ceil((xmax-xmin)/res))+1); height=max(1,int(math.ceil((ymax-ymin)/res))+1)
    if width*height>4000000: raise ValueError('Occupancy grid exceeds 4 million cell budget')
    if bool(args.free_space_mask)!=bool(args.free_space_metadata): raise ValueError('Mask and official metadata must be supplied together')
    mask_metadata=None; cells=bytearray([205])*(width*height)
    if args.free_space_mask:
        mask_metadata,width,height,res,xmin,ymin,cells=load_free_space(
            Path(args.free_space_mask),Path(args.free_space_metadata),align['official_source'])
    occupied=set(); terrain={}
    outside_points=0
    for x,y,z in aligned:
        if not (xmin<=x<xmin+width*res and ymin<=y<ymin+height*res):
            outside_points+=1; continue
        ix=min(width-1,max(0,int((x-xmin)/res))); iy=min(height-1,max(0,int((y-ymin)/res)))
        key=f'{ix},{iy}'; terrain[key]=min(z,terrain.get(key,z))
        if args.min_z<=z<=args.max_z:
            occupied.add((ix,iy))
            cells[iy*width+ix]=0
    output.parent.mkdir(parents=True,exist_ok=True)
    staging=Path(tempfile.mkdtemp(prefix=output.name+'.partial-',dir=str(output.parent)))
    try:
        (staging/'maps').mkdir()
        write_pcd(staging/'maps'/'localization_map.pcd',aligned,'map; official alignment applied')
        # An endpoint-only PCD contains no free-space evidence. Never mark unseen cells free.
        (staging/'maps'/'occupancy.pgm').write_bytes(('P5\n# map\n%d %d\n255\n'%(width,height)).encode()+bytes(cells[y*width+x] for y in range(height-1,-1,-1) for x in range(width)))
        (staging/'maps'/'map.yaml').write_text('image: occupancy.pgm\nmode: trinary\nresolution: '+str(res)+'\norigin: ['+str(xmin)+', '+str(ymin)+', 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
        (staging/'maps'/'terrain_reference.json').write_text(json.dumps({'frame':'map','resolution':res,'origin':[xmin,ymin],'method':'minimum_observed_z','traversability_verified':False,'cells':terrain},sort_keys=True)+'\n')
        published={'schema':1,'status':'published','version':'map-'+digest(staging/'maps'/'localization_map.pcd')[:16],'coordinate_frame':'map','official_alignment_applied':True,'publishable_for_frozen_localization':True,'source_draft_version':draft.get('version'),'official_source':align['official_source'],'alignment':{'translation':tr,'yaw_rad':float(yaw),'scale':1.0},'localization_map':'maps/localization_map.pcd','base_occupancy':'maps/occupancy.pgm','terrain_reference':'maps/terrain_reference.json','metadata':'maps/terrain_reference.json','artifacts':{}}
        published.update({'map_yaml':'maps/map.yaml','navigation_ready':False,
            'free_space_evidence':'unavailable_endpoint_only_pcd','occupancy_unknown_value':205,
            'obstacle_height_range_m':[args.min_z,args.max_z], 'terrain_traversability_verified':False})
        published['occupancy_ignored_outside_points']=outside_points
        if mask_metadata is not None:
            shutil.copyfile(args.free_space_mask,staging/'maps'/'official_free_space.pgm')
            (staging/'maps'/'official_free_space_metadata.json').write_text(json.dumps(mask_metadata,sort_keys=True)+'\n')
            published.update({'navigation_ready':255 in cells,'free_space_evidence':'verified_official_mask',
                'free_space_mask':'maps/official_free_space.pgm','free_space_metadata':'maps/official_free_space_metadata.json',
                'free_space_official_source':mask_metadata['official_source']})
        for p in (staging/'maps').iterdir(): published['artifacts'][p.name]={'path':'maps/'+p.name,'sha256':digest(p)}
        # Changing grid resolution/height filtering/terrain must invalidate consumers
        # even when the localization PCD itself is identical.
        published['version']='map-'+hashlib.sha256(json.dumps(published['artifacts'],sort_keys=True).encode()).hexdigest()[:16]
        (staging/'map_bundle.json').write_text(json.dumps(published,indent=2,sort_keys=True)+'\n'); staging.rename(output)
        print(json.dumps(published,indent=2))
    except Exception:
        shutil.rmtree(staging,ignore_errors=True); raise
if __name__=='__main__': main()
