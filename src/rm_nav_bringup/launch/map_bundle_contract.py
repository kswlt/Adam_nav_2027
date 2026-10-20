"""Shared startup validation; immutable map artifacts are checked once at launch."""
import hashlib
import json
import math
from pathlib import Path
import yaml

def load_published_bundle(filename, navigation=False):
    original=Path(filename).absolute()
    if original.is_symlink(): raise ValueError('MapBundle manifest cannot be a symlink')
    manifest=original.resolve(strict=True)
    bundle=json.loads(manifest.read_text())
    if (bundle.get('schema')!=1 or bundle.get('status')!='published' or
        bundle.get('coordinate_frame')!='map' or bundle.get('official_alignment_applied') is not True or
        bundle.get('publishable_for_frozen_localization') is not True or not bundle.get('version')):
        raise ValueError('An officially aligned published MapBundle is required')
    root=manifest.parent; artifacts=bundle.get('artifacts')
    if not isinstance(artifacts,dict) or not 1<=len(artifacts)<=128: raise ValueError('Invalid artifact list')
    checked={}
    for entry in artifacts.values():
        if not isinstance(entry,dict): raise ValueError('Invalid artifact')
        rel=entry.get('path'); digest=entry.get('sha256')
        if not isinstance(rel,str) or not isinstance(digest,str) or len(digest)!=64: raise ValueError('Missing artifact reference/hash')
        relative=Path(rel)
        if relative.is_absolute() or '..' in relative.parts: raise ValueError('Artifact escapes bundle')
        path=root
        for part in relative.parts:
            path=path/part
            if path.is_symlink(): raise ValueError('Symlink in artifact path')
        path=path.resolve(strict=True);path.relative_to(root)
        if not path.is_file() or path.stat().st_size>256*1024*1024: raise ValueError('Missing/oversized artifact')
        h=hashlib.sha256()
        with path.open('rb') as stream:
            for data in iter(lambda:stream.read(1024*1024),b''): h.update(data)
        if h.hexdigest()!=digest: raise ValueError('MapBundle artifact checksum mismatch: '+rel)
        checked[rel]=path
    def required(key):
        rel=bundle.get(key)
        if rel not in checked: raise ValueError('Unverified required map reference: '+key)
        return checked[rel]
    required('localization_map')
    if navigation:
        if bundle.get('navigation_ready') is not True: raise ValueError('MapBundle lacks verified navigation free-space evidence')
        if (bundle.get('free_space_evidence')!='verified_official_mask' or not bundle.get('official_source') or
            bundle.get('free_space_official_source')!=bundle['official_source']):
            raise ValueError('Navigation requires an officially verified free-space mask')
        required('free_space_mask');required('free_space_metadata')
        map_yaml=required('map_yaml'); occupancy=required('base_occupancy')
        config=yaml.safe_load(map_yaml.read_text())
        if not isinstance(config,dict) or not isinstance(config.get('image'),str): raise ValueError('Invalid map YAML')
        image=(map_yaml.parent/config['image']).resolve(strict=True)
        if image!=occupancy: raise ValueError('Map YAML image disagrees with Bundle occupancy')
        resolution=config.get('resolution'); origin=config.get('origin')
        if (not isinstance(resolution,(float,int)) or not math.isfinite(resolution) or resolution<=0 or
                not isinstance(origin,list) or len(origin)!=3 or not all(isinstance(v,(int,float)) and math.isfinite(v) for v in origin)):
            raise ValueError('Invalid map geometry')
    return bundle,checked
