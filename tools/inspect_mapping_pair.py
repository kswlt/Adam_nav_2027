#!/usr/bin/env python3
"""Read-only coverage diagnostics at recorded LIO alignment; never admits graph edges."""
import argparse
import json
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
from rclpy.serialization import deserialize_message
from rm_nav_interfaces.msg import ObservationFrame


def load(session, index):
    folder = session/f'keyframe_{index}'
    metadata = json.loads((folder/'metadata.json').read_text())
    observation = deserialize_message((folder/'observation.cdr').read_bytes(), ObservationFrame)
    cloud = observation.point_cloud
    if cloud.header.frame_id != 'odom' or cloud.height != 1 or cloud.is_bigendian:
        raise ValueError('Expected native little-endian odom cloud')
    fields = {f.name: f for f in cloud.fields}
    if any(fields[k].datatype != 7 or fields[k].count != 1 for k in ('x', 'y', 'z')):
        raise ValueError('Expected FLOAT32 XYZ')
    dtype = np.dtype({'names': ['x', 'y', 'z'], 'formats': ['<f4']*3,
                      'offsets': [fields[k].offset for k in ('x', 'y', 'z')],
                      'itemsize': cloud.point_step})
    packed = np.frombuffer(cloud.data, dtype=dtype, count=cloud.width)
    points = np.column_stack([packed[k] for k in ('x', 'y', 'z')]).astype(float)
    if len(points) < 50 or len(points) > 200000 or not np.isfinite(points).all():
        raise ValueError('Invalid/oversized geometry')
    return points, metadata


def metrics(target, source):
    tree = cKDTree(target)
    distance, index = tree.query(source, k=1, workers=1)
    mask = distance <= .5
    report = {'target_points': len(target), 'source_points': len(source),
              'nn_fraction_within_0_5m': float(mask.mean()),
              'nn_rmse_within_0_5m': float(np.sqrt(np.mean(distance[mask]**2))),
              'nn_distance_p50_p95_m': np.percentile(distance, [50, 95]).tolist()}
    if not mask.any():
        return report
    nearest = index[mask]
    _, neighbors = tree.query(target[nearest], k=10, workers=1)
    patches = target[neighbors]
    centered = patches-patches.mean(axis=1, keepdims=True)
    covariance = np.einsum('nki,nkj->nij', centered, centered)/9.
    eigen, axes = np.linalg.eigh(covariance)
    planar = (eigen[:, 2] > 1e-10) & (eigen[:, 0]/np.maximum(eigen[:, 2], 1e-20) < .02)
    normal = axes[:, :, 0]
    delta = source[mask]-target[nearest]
    residual = np.abs(np.einsum('ni,ni->n', normal, delta))
    report['locally_planar_fraction_of_neighbors'] = float(planar.mean())
    report['local_plane_normal_rmse_m'] = float(np.sqrt(np.mean(residual[planar]**2))) if planar.any() else None
    return report


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--session', type=Path, required=True)
    ap.add_argument('--target-id', type=int, required=True)
    ap.add_argument('--source-id', type=int, required=True)
    ap.add_argument('--history', type=int, default=3)
    ap.add_argument('--output', type=Path, required=True)
    args = ap.parse_args()
    if args.target_id < 0 or args.source_id <= args.target_id or not 1 <= args.history <= 8:
        raise ValueError('Invalid frame/window selection')
    target, tm = load(args.session, args.target_id)
    source, sm = load(args.session, args.source_id)
    ids = list(range(max(0, args.target_id-args.history+1), args.target_id+1))
    union = np.concatenate([load(args.session, i)[0] for i in ids])
    if len(union) > 200000:
        raise ValueError('Diagnostic target exceeds bounded memory contract')
    result = {'scope': 'recorded odom alignment; not GICP result, truth or admission gate',
              'source_stamp_ns': sm['stamp_ns'], 'target_stamp_ns': tm['stamp_ns'],
              'pair_forward': metrics(target, source), 'pair_reverse': metrics(source, target),
              'history_ids': ids, 'history_forward': metrics(union, source)}
    with args.output.open('x') as stream:
        json.dump(result, stream, indent=2)
        stream.write('\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
