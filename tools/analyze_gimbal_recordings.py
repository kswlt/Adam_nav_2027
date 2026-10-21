#!/usr/bin/env python3
"""Offline diagnostics from original MCAP/UART; no ROS runtime or control output.

Requires numpy, mcap, mcap-ros2-support; matplotlib is used for the figure.
Lever-arm fits are hypotheses, never calibration or chassis ground truth.
"""
import argparse
import json
from pathlib import Path

import numpy as np
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory


def read_bag(path):
    topics = {'/lio/sensor_odometry': 'lio', '/sensor_reference/odom': 'reference',
              '/livox/imu': 'imu'}
    data = {key: [] for key in topics.values()}
    with path.open('rb') as stream:
        reader = make_reader(stream, decoder_factories=[DecoderFactory()])
        for _, channel, message, value in reader.iter_decoded_messages(topics=list(topics)):
            stamp = value.header.stamp.sec * 10**9 + value.header.stamp.nanosec
            if channel.topic == '/livox/imu':
                a, g = value.linear_acceleration, value.angular_velocity
                fields = [a.x, a.y, a.z, g.x, g.y, g.z]
            else:
                p, q = value.pose.pose.position, value.pose.pose.orientation
                fields = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
            data[topics[channel.topic]].append([stamp / 1e9, message.log_time / 1e9, *fields])
    return {key: np.asarray(rows, dtype=float) for key, rows in data.items()}


def yaw_of(lio):
    x, y, z, w = lio[:, 5:9].T
    return np.unwrap(np.arctan2(2*(w*z+x*y), 1-2*(y*y+z*z)))


def lever_fit(xy, yaw):
    """Fit p=c+R(yaw)*lever. A stationary pivot is an unverified assumption."""
    n = len(yaw)
    model = np.zeros((2*n, 4))
    model[:n, 0], model[n:, 1] = 1, 1
    model[:n, 2], model[:n, 3] = np.cos(yaw), -np.sin(yaw)
    model[n:, 2], model[n:, 3] = np.sin(yaw), np.cos(yaw)
    target = np.concatenate([xy[:, 0], xy[:, 1]])
    coefficients = np.linalg.lstsq(model, target, rcond=None)[0]
    predicted = (model @ coefficients).reshape(2, n).T
    residual = np.linalg.norm(xy-predicted, axis=1)
    return {'horizontal_lever_m': coefficients[2:].tolist(),
            'radius_m': float(np.linalg.norm(coefficients[2:])),
            'residual_xy_rms_m': float(np.sqrt(np.mean(residual**2))),
            'residual_p95_m': float(np.percentile(residual, 95)),
            'condition': float(np.linalg.cond(model)),
            'identifiable_yaw_excitation': bool(np.ptp(yaw) >= np.pi/2),
            'mechanical_calibration_verified': False}, predicted


def serial_records(root):
    records = {}
    for path in sorted(root.glob('gimbal*/frames.jsonl')):
        values = []
        with path.open(encoding='utf-8') as stream:
            for line in stream:
                frame = json.loads(line)
                values.append([frame['host_received_system_ns']/1e9,
                               frame['feedback']['contact_angle']])
        array = np.asarray(values)
        # Degree hypothesis: report as a hypothesis, do not feed a runtime TF.
        records[path.parent.name] = (array[:, 0], np.unwrap(np.deg2rad(array[:, 1])))
    return records


def cloud_samples(path):
    """Sample each tenth second; retain original timestamp and frame semantics."""
    factory, last, results = DecoderFactory(), {}, {}
    types = {1:'i1', 2:'u1', 3:'i2', 4:'u2', 5:'i4', 6:'u4', 7:'f4', 8:'f8'}
    with path.open('rb') as stream:
        for schema, channel, message in make_reader(stream).iter_messages(
                topics=['/livox/lidar', '/lio/deskewed_odom_cloud']):
            topic = channel.topic
            if message.log_time-last.get(topic, 0) < 10_000_000_000:
                continue
            last[topic] = message.log_time
            cloud = factory.decoder_for(channel.message_encoding, schema)(message.data)
            names, formats, offsets = [], [], []
            for field in cloud.fields:
                names.append(field.name)
                formats.append(('>' if cloud.is_bigendian else '<')+types[field.datatype])
                offsets.append(field.offset)
            dtype = np.dtype({'names':names, 'formats':formats, 'offsets':offsets,
                              'itemsize':cloud.point_step})
            points = np.ndarray((cloud.height,cloud.width), dtype=dtype, buffer=bytes(cloud.data),
                                strides=(cloud.row_step,cloud.point_step)).reshape(-1)
            xyz = np.column_stack([points[k] for k in ('x','y','z')])
            finite = np.isfinite(xyz).all(axis=1)
            stamp = cloud.header.stamp.sec*10**9+cloud.header.stamp.nanosec
            sample = {'record_ns':message.log_time, 'source_ns':stamp,
                      'frame':cloud.header.frame_id, 'points':len(points),
                      'finite_xyz_fraction':float(finite.mean()),
                      'xyz_p01_p99':np.percentile(xyz[finite],[1,99],axis=0).tolist(),
                      'fields':names}
            if 'timestamp' in names:
                pt = points['timestamp'].astype(float)
                sample['point_timestamp_min_max'] = [float(pt.min()),float(pt.max())]
                sample['point_span_ns_assuming_absolute_ns'] = float(np.ptp(pt))
                sample['point_start_minus_header_ns'] = float(pt.min()-stamp)
            results.setdefault(topic,[]).append(sample)
    return results


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--raw-root', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    serial = serial_records(args.raw_root)
    report = {'source': 'original local MCAP and original frames.jsonl',
              'angle_unit': 'degree hypothesis; firmware semantics NOT VERIFIED',
              'serial_time': 'host RX batch; not MCU acquisition time',
              'control_output': False, 'runs': {}}
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    figure, axes = plt.subplots(4, 3, figsize=(16, 15), constrained_layout=True)
    static_gz_bias = None
    diagnostic_lever = None
    for row, path in enumerate(sorted(args.raw_root.glob('run_*/*.mcap'))):
        data = read_bag(path)
        lio, imu, reference = (data[key] for key in ('lio', 'imu', 'reference'))
        if not len(lio) or not len(imu):
            raise ValueError(f'Missing required input: {path}')
        t, yaw, xyz = lio[:, 0], yaw_of(lio), lio[:, 2:5]
        xy = xyz[:, :2]
        model, predicted = lever_fit(xy, yaw)
        ref_yaw = np.interp(reference[:, 0], t, yaw)
        ref_model, _ = lever_fit(reference[:, 2:4], ref_yaw)
        mask = (imu[:, 0] >= t[0]) & (imu[:, 0] <= t[-1])
        im = imu[mask]
        if row == 0:
            static_gz_bias = float(np.median(im[:, 7]))
            report['first_recording_static_gz_bias_hypothesis_rad_s'] = static_gz_bias
        gyro_yaw = np.degrees(np.trapezoid(im[:, 7], im[:, 0]))
        latency = (lio[:, 1]-t)*1000
        acceleration = np.linalg.norm(im[:, 2:5], axis=1)
        yaw_rate = np.gradient(yaw, t)
        # Best constant gyro-z bias under near-level yaw assumption.
        iz = np.interp(t, im[:, 0], im[:, 7])
        bias = float(np.median(iz-yaw_rate))
        result = {'lio_count': len(lio), 'imu_count': len(imu),
                  'lio_duration_s': float(t[-1]-t[0]),
                  'xy_max_from_start_m': float(np.linalg.norm(xy-xy[0], axis=1).max()),
                  'xy_end_from_start_m': float(np.linalg.norm(xy[-1]-xy[0])),
                  'z_span_m': float(np.ptp(xyz[:, 2])),
                  'yaw_span_deg': float(np.degrees(np.ptp(yaw))),
                  'yaw_end_delta_deg': float(np.degrees(yaw[-1]-yaw[0])),
                  'imu_gz_integral_same_lio_window_deg': float(gyro_yaw),
                  'gyro_minus_lio_rate_median_rad_s': bias,
                  'imu_bias_corrected_integral_deg': float(gyro_yaw-np.degrees(bias*(im[-1,0]-im[0,0]))),
                  'imu_first_run_bias_corrected_integral_deg': float(gyro_yaw-np.degrees(static_gz_bias*(im[-1,0]-im[0,0]))),
                  'imu_acc_norm_g_p50_p99_max': np.percentile(acceleration, [50,99,100]).tolist(),
                  'lio_record_minus_source_ms_p50_p95_max': np.percentile(latency,[50,95,100]).tolist(),
                  'lio_source_nonmonotonic_count': int(np.sum(np.diff(t)<=0)),
                  'imu_source_nonmonotonic_count': int(np.sum(np.diff(imu[:,0])<=0)),
                  'imu_point_lever_fit': model, 'lidar_reference_lever_fit': ref_model,
                  'serial_overlaps': {}, 'cloud_samples':cloud_samples(path)}
        spin = np.abs(yaw_rate) > .2
        if np.count_nonzero(spin) >= 10 and np.ptp(yaw[spin]) >= np.pi/2:
            spin_fit, _ = lever_fit(xy[spin], yaw[spin])
            spin_fit['sample_count'] = int(spin.sum())
            spin_fit['selection'] = 'abs(LIO yaw rate)>0.2rad/s; does not prove stationary chassis'
            result['imu_point_spin_only_fit'] = spin_fit
            ref_spin = np.interp(reference[:,0],t,spin.astype(float)) > .99
            result['lidar_reference_spin_only_fit'], _ = lever_fit(
                reference[ref_spin,2:4],ref_yaw[ref_spin])
            if path.parent.name == 'run_20261021_202408':
                diagnostic_lever = np.asarray(result['lidar_reference_spin_only_fit']['horizontal_lever_m'])
                report['diagnostic_only_lever_source'] = path.parent.name
                report['diagnostic_only_lidar_horizontal_lever_m'] = diagnostic_lever.tolist()
        if diagnostic_lever is not None:
            a, b = diagnostic_lever
            rotated = np.column_stack([a*np.cos(ref_yaw)-b*np.sin(ref_yaw),
                                       a*np.sin(ref_yaw)+b*np.cos(ref_yaw)])
            pivot = reference[:,2:4]-rotated
            result['diagnostic_only_pivot_hypothesis'] = {
                'max_from_start_m':float(np.linalg.norm(pivot-pivot[0],axis=1).max()),
                'end_from_start_m':float(np.linalg.norm(pivot[-1]-pivot[0])),
                'production_applied':False,
                'peak_rotation_induced_speed_m_s':float(np.max(np.abs(yaw_rate))*np.linalg.norm(diagnostic_lever))}
            axes[row,0].plot(pivot[:,0]-pivot[0,0],pivot[:,1]-pivot[0,1],alpha=.7,
                             label='candidate pivot, diagnostic only')
        for name, (et, angle) in serial.items():
            overlap = (t >= et[0]) & (t <= et[-1])
            if np.count_nonzero(overlap) < 10:
                continue  # Never pair by filename or recording order.
            tt, yy = t[overlap], yaw[overlap]
            aa = np.interp(tt, et, angle)
            yy, aa = yy-yy[0], aa-aa[0]
            rmse = np.degrees(np.sqrt(np.mean((yy-aa)**2)))
            correlation = (float(np.corrcoef(yy, aa)[0,1])
                           if np.std(yy)>1e-10 and np.std(aa)>1e-10 else None)
            result['serial_overlaps'][name] = {
                'duration_s': float(tt[-1]-tt[0]),
                'lio_delta_deg': float(np.degrees(yy[-1])),
                'encoder_delta_deg_if_degrees': float(np.degrees(aa[-1])),
                'correlation': correlation, 'delta_difference_rms_deg': float(rmse),
                'sensor_minus_encoder_delta_deg': float(np.degrees(yy[-1]-aa[-1])),
                'sensor_minus_encoder_span_deg': float(np.degrees(np.ptp(yy-aa))),
                'chassis_interpretation': 'hypothesis only: requires actual relative-yaw semantics and mounting rotation'}
            axes[row, 1].plot(tt-t[0], np.degrees(aa), '--', label=f'RX: {name[7:23]} (deg hypothesis)')
        report['runs'][path.parent.name] = result
        axes[row, 0].plot(xy[:,0]-xy[0,0], xy[:,1]-xy[0,1], label='LIO IMU point')
        if model['identifiable_yaw_excitation']:
            axes[row, 0].plot(predicted[:,0]-xy[0,0], predicted[:,1]-xy[0,1], '--', label='stationary-pivot hypothesis fit')
        axes[row, 0].axis('equal')
        axes[row, 0].set(xlabel='x from start [m]', ylabel='y from start [m]', title=path.parent.name)
        axes[row, 1].plot(t-t[0], np.degrees(yaw-yaw[0]), label='LIO yaw (unwrapped)')
        axes[row, 1].set(xlabel='time from LIO start [s]', ylabel='yaw change [deg]')
        axes[row, 2].plot(t-t[0], xyz[:,2]-xyz[0,2], label='z from start [m]')
        axes[row, 2].plot(t-t[0], np.linalg.norm(xy-xy[0],axis=1), label='horizontal distance [m]')
        axes[row, 2].set(xlabel='time from LIO start [s]', ylabel='distance [m]')
        for axis in axes[row]:
            axis.legend(fontsize=7)
            axis.grid(alpha=.3)
        print(path.parent.name, 'LIO samples:', len(lio), 'cloud samples:',
              sum(len(v) for v in result['cloud_samples'].values()), flush=True)
    figure.savefig(args.output/'gimbal_motion.png', dpi=145)
    (args.output/'metrics.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n', encoding='utf-8')


if __name__ == '__main__':
    main()
