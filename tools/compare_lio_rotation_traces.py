#!/usr/bin/env python3
"""Compare raw-input replay estimates on common source stamps; no motion truth assumed."""
import argparse
import json
from pathlib import Path
import sys
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/rm_nav_bringup/scripts'))
from sensor_reference_math import sensor_pose


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--trace', type=Path, nargs='+', required=True)
    ap.add_argument('--output', type=Path, required=True)
    args = ap.parse_args()
    traces = [json.loads(p.read_text()) for p in args.trace]
    tables = [{r['stamp_ns']: r for r in rows} for rows in traces]
    common = sorted(set.intersection(*(set(t) for t in tables)))
    if len(common) < 30:
        raise ValueError('Fewer than 30 common source stamps; comparison refused')
    args.output.mkdir(parents=True, exist_ok=False)
    lever = yaml.safe_load((ROOT/'src/rm_nav_bringup/config/vehicle_164_geometry.yaml').read_text())[
        'mount']['axis_to_lidar_at_reference_height']
    offset = [a-b for a, b in zip([-.011, -.02329, .04412], lever)]
    report = {'scope': 'matched replay estimates, not independent ground truth',
              'configured_axis_to_lidar_m': lever, 'common_stamp_count': len(common),
              'source_span_s': (common[-1]-common[0])*1e-9, 'runs': {}}
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 3, figsize=(14, 4), constrained_layout=True)
    for path, table in zip(args.trace, tables):
        rows = [table[s] for s in common]
        imu = np.array([r['position'] for r in rows])
        axis = np.array([sensor_pose(r['position'], r['quaternion_xyzw'], offset)[0] for r in rows])
        imu -= imu[0].copy()
        axis -= axis[0].copy()
        distance = np.linalg.norm(axis[:, :2], axis=1)
        report['runs'][path.parent.name] = {
            'received_count': len(table),
            'imu_xy_max_from_start_m': float(np.linalg.norm(imu[:, :2], axis=1).max()),
            'axis_xy_max_from_start_m': float(distance.max()),
            'axis_xy_end_from_start_m': float(distance[-1]),
            'axis_z_span_m': float(np.ptp(axis[:, 2])),
        }
        axes[0].plot(imu[:, 0], imu[:, 1], label=path.parent.name)
        axes[1].plot(axis[:, 0], axis[:, 1], label=path.parent.name)
        axes[2].plot((np.array(common, dtype=np.int64)-common[0])*1e-9, distance, label=path.parent.name)
    for ax, title in zip(axes, ['IMU position relative to start', 'Compensated axis relative to start',
                                'Compensated axis XY distance']):
        ax.set_title(title)
        ax.grid(True)
    for ax in axes[:2]:
        ax.set(xlabel='x [m]', ylabel='y [m]', aspect='equal')
    axes[2].set(xlabel='source time from first common stamp [s]', ylabel='distance [m]')
    axes[2].legend(fontsize=7)
    fig.savefig(args.output/'comparison.png', dpi=150)
    plt.close(fig)
    (args.output/'comparison.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
