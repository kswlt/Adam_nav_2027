"""RM-specific settings layered on pinned upstream defaults, without site assumptions."""
from pathlib import Path
import copy
import yaml


UPSTREAM_COMMIT = '76e126328b670adc92981d9b4900cc059a6f2343'


def frontend_config(defaults):
    config = copy.deepcopy(defaults)
    config['sensor']['body_from_lidar'] = {
        'rotation_xyzw': [0., 0., 0., 1.],
        'translation': [-.011, -.02329, .04412]}
    return config


def backend_config(defaults, mode, prior_map, output_directory):
    if mode not in ('mapping', 'localization'):
        raise ValueError('Explicit mapping or localization mode required')
    if mode == 'localization':
        root = Path(prior_map).resolve(strict=True)
        if not root.is_dir() or not (root/'keyframes.txt').is_file() or not (root/'clouds').is_dir():
            raise ValueError('Z-Localization native bundle required (keyframes.txt + clouds/)')
    else:
        if prior_map:
            raise ValueError('Mapping cannot silently load a prior map')
        target = Path(output_directory)
        if not target.is_absolute() or target.exists():
            raise ValueError('Mapping requires a new absolute native output directory')
        root = None
    config = copy.deepcopy(defaults)
    config['common'].update(frontend='zlio', odom_frame='odom', sensor_frame='front_mid360_imu',
                            map_frame='map', input_cloud_in_local_frame=True)
    # Backend body is the IMU, not the rotating chassis. Axis compensation is downstream.
    config['common']['body'] = {'rotation': {'roll_deg': 0., 'pitch_deg': 0., 'yaw_deg': 0.},
                               'offset': {'x': 0., 'y': 0., 'z': 0.}, 'frame': 'front_mid360_imu'}
    # Descriptor Z offset is defined about our IMU origin, not upstream's assumed ground height.
    config['common']['scan_context']['lidar_height'] = 0.
    config['prior_map'].update(enabled=mode == 'localization', directory=str(root) if root else '')
    config['uwb']['enabled'] = False
    config['startup_hint']['enabled'] = False
    config['output']['legacy']['enabled'] = False  # prevents second map->IMU TF authority
    config['output']['world_cloud'].update(enabled=True, publish_period=.2,
                                           publish_initializing=False, publish_untrusted=False)
    config['trajectory'].update(save_directory=str(Path(output_directory).absolute()),
                                save_on_shutdown=False, save_global_pcd=True)
    config['visualization'].update(enabled=True, publish_prior_map=mode == 'localization',
                                   publish_runtime_map=mode == 'mapping', publish_paths=True,
                                   map_publish_period=2.)
    config['recovery']['parallel_candidates'] = 2
    config['loop']['max_parallel_verifications'] = 2
    return config


def flatten(config, prefix=''):
    out = {}
    for key, value in config.items():
        name = prefix + key
        if isinstance(value, dict):
            out.update(flatten(value, name+'.'))
        else:
            out[name] = value
    return out


def load(path):
    return yaml.safe_load(Path(path).read_text(encoding='utf-8'))


def native_bundle(manifest, bundle):
    """RM occupancy plus native Z priors, explicitly bound to one local map version."""
    import hashlib
    info = bundle.get('z_localization', {})
    if info.get('backend') != 'z-localization' or info.get('upstream_commit') != UPSTREAM_COMMIT:
        raise ValueError('MapBundle lacks pinned native Z-Localization priors; old RM maps require explicit conversion')
    rel = Path(info.get('native_bundle', ''))
    if not rel.parts or rel.is_absolute() or '..' in rel.parts:
        raise ValueError('Invalid native Z bundle reference')
    root = Path(manifest).resolve(strict=True).parent
    path = root
    for part in rel.parts:
        path = path/part
        if path.is_symlink(): raise ValueError('Native bundle path contains a symlink')
    path = path.resolve(strict=True); path.relative_to(root)
    index = path/'keyframes.txt'
    if index.is_symlink() or not (path/'clouds').is_dir() or index.stat().st_size > 16*1024*1024:
        raise ValueError('Invalid native Z index/clouds')
    if hashlib.sha256(index.read_bytes()).hexdigest() != info.get('keyframe_index_sha256'):
        raise ValueError('Native Z keyframe index changed')
    for item in path.rglob('*'):
        if item.is_symlink(): raise ValueError('Native Z map contains a symlink')
    # Upstream performs complete pose/PCD/descriptor validation before accepting priors.
    return path
