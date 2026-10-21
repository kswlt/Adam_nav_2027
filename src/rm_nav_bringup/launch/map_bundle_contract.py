"""Shared startup validation; immutable map artifacts are checked once at launch."""
import hashlib
import json
import math
from pathlib import Path
import yaml

def load_published_bundle(filename, navigation=False, expected_profile='official'):
    if expected_profile not in ('official','local_site'):
        raise ValueError('Unknown explicit map profile')
    original=Path(filename).absolute()
    if original.is_symlink(): raise ValueError('MapBundle manifest cannot be a symlink')
    manifest=original.resolve(strict=True)
    bundle=json.loads(manifest.read_text())
    if (bundle.get('schema')!=1 or bundle.get('status')!='published' or
        bundle.get('coordinate_frame')!='map' or
        bundle.get('publishable_for_frozen_localization') is not True or not bundle.get('version')):
        raise ValueError('An aligned published MapBundle is required')
    if bundle.get('publication_profile','official')!=expected_profile:
        raise ValueError('Map publication profile differs from explicit startup profile')
    if expected_profile=='official':
        if bundle.get('official_alignment_applied') is not True:
            raise ValueError('Official startup requires official alignment')
    elif (bundle.get('official_alignment_applied') is not False or bundle.get('local_alignment_applied') is not True or
          not bundle.get('local_source') or bundle.get('coordinate_definition')!='session_mapping_odom'):
        raise ValueError('Local startup requires explicit non-official session coordinates')
    if expected_profile=='local_site':
        alignment=bundle.get('alignment')
        if (not isinstance(alignment,dict) or alignment.get('translation')!=[0,0,0] or
                alignment.get('yaw_rad')!=0 or alignment.get('scale')!=1):
            raise ValueError('Local session coordinates must retain their explicit identity alignment')
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
        source_key='official_source' if expected_profile=='official' else 'local_source'
        evidence='verified_official_mask' if expected_profile=='official' else 'operator_reviewed_local_mask'
        if (bundle.get('free_space_evidence')!=evidence or not bundle.get(source_key) or
            bundle.get('free_space_'+source_key)!=bundle[source_key]):
            raise ValueError('Navigation requires reviewed free-space evidence for the selected profile')
        required('free_space_mask');required('free_space_metadata')
        if expected_profile=='local_site':
            review=json.loads(required('free_space_metadata').read_text())
            if (review.get('status')!='operator_reviewed' or review.get('coordinate_frame')!='map' or review.get('local_source')!=bundle['local_source'] or
                    not isinstance(review.get('reviewer'),str) or not review['reviewer'].strip() or
                    review.get('review_scope')!='free_space_and_obstacles_in_local_map'):
                raise ValueError('Local free-space operator review is missing or mismatched')
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
