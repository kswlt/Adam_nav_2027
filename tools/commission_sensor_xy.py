#!/usr/bin/env python3
"""One bounded operator-authorized sensor-axis pulse through the real Adam bridge.

Not a Nav2 permission publisher or a localization acceptance test. Does not
publish health, TF, yaw, chassis mode or navigation permission. Actual serial
feedback and live sensor-reference state are required, including for PTY tests.
"""
import argparse
import json
import math
import os
from pathlib import Path
import stat
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from my_serial_py.serial_bridge import SerialNode


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port', required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--speed', type=float, default=.05)
    p.add_argument('--duration', type=float, default=.5)
    p.add_argument('--high-speed-commission', action='store_true',
                   help='Explicit operator-authorized 1m/s-class test, at most 1 second')
    p.add_argument('--rotating-gimbal', action='store_true',
                   help='Operator confirms live gimbal scanning; no stationary LiDAR precondition')
    mode = p.add_mutually_exclusive_group(required=True)
    mode.add_argument('--execute-physical', action='store_true')
    mode.add_argument('--test-only-pty', action='store_true')
    args = p.parse_args()
    limit_speed,limit_duration,limit_travel=(1.,1.,1.) if args.high_speed_commission else (.2,.5,.1)
    if not (math.isfinite(args.speed) and 0<args.speed<=limit_speed and
            math.isfinite(args.duration) and 0<args.duration<=limit_duration and args.speed*args.duration<=limit_travel):
        raise ValueError('Command exceeds explicitly selected commissioning envelope')
    displacement_bound=args.speed*args.duration+(.3 if args.high_speed_commission else .1)
    port = Path(args.port).resolve(strict=True)
    is_pty = port.parent == Path('/dev/pts') and port.name.isdecimal()
    if not stat.S_ISCHR(port.stat().st_mode) or is_pty != args.test_only_pty:
        raise ValueError('Explicit mode must match physical character device or TEST_ONLY PTY')
    if args.test_only_pty and os.environ.get('ROS_DOMAIN_ID') in (None, '', '0', '97'):
        raise ValueError('PTY fixture must use an isolated ROS domain')
    args.output.mkdir(parents=True, exist_ok=False)
    ros_args = ['--ros-args', '-p', 'serial_port:='+str(port), '-p', 'yaw_policy:=legacy_cached']
    # Do not consume global behavior commands or claim production motion_allowed.
    for topic in ('cmd_vel','cmd_chassis_mode','cmd_stance','cmd_yaw_angle','region','big_yaw_aligned'):
        ros_args += ['-r', '/'+topic+':=/commissioning/'+topic]
    ros_args += ['-r', '/hardware/serial_healthy:=/commissioning/serial_healthy']
    rclpy.init(args=ros_args)
    executor = SingleThreadedExecutor()
    probe = Node('operator_bounded_sensor_xy')
    bridge = None
    health = {'state':(False, -math.inf), 'cloud':(False, -math.inf)}
    poses = []
    records = []
    start_pose = None
    report = {'mode':'TEST_ONLY' if is_pty else 'PHYSICAL_COMMISSIONING',
              'axes':'horizontal_sensor_yaw_xy', 'speed_mps':args.speed, 'pulse_budget_s':args.duration,
              'nav2_started':False, 'physical_stop_feedback':'NOT_VERIFIED',
              'yaw_field':'original legacy cached zero; not a full chassis/yaw stop',
              'pulse_started':False, 'completed':False}
    report['rotating_gimbal_operator_confirmed']=args.rotating_gimbal
    report['high_speed_commission']=args.high_speed_commission
    report['measured_displacement_abort_m']=displacement_bound
    def flag(key, msg):
        health[key] = msg.data, time.monotonic()
    def pose(msg):
        stamp = msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        xyz = msg.pose.pose.position
        values = [xyz.x,xyz.y,xyz.z]
        if (msg.header.frame_id == 'odom' and msg.child_frame_id == 'front_mid360_imu'
                and all(math.isfinite(v) for v in values)
                and 0 <= (probe.get_clock().now().nanoseconds-stamp)*1e-9 <= .25):
            poses.append(dict(at=time.monotonic(), stamp_ns=stamp, xyz=values))
    probe.create_subscription(Bool,'/state/sensor_reference_healthy',lambda m:flag('state',m),1)
    probe.create_subscription(Bool,'/sensors/front_mid360/cloud_healthy',lambda m:flag('cloud',m),1)
    probe.create_subscription(Odometry,'/lio/sensor_odometry',pose,qos_profile_sensor_data)
    executor.add_node(probe)
    def ready():
        now = time.monotonic()
        return (all(ok and now-at<=.25 for ok,at in health.values())
                and poses and now-poses[-1]['at']<=.25
                and bridge.last_rx is not None and now-bridge.last_rx<=.25
                and bridge.serial_conn is not None)
    def send(x):
        message = Twist(); message.linear.x=x
        bridge.cmd_vel_callback(message)
        records.append(dict(at=time.monotonic(),x=x,y=0.))
    try:
        bridge = SerialNode(); executor.add_node(bridge)
        deadline = time.monotonic()+8.
        stable_since = None
        while time.monotonic()<deadline:
            send(0.); executor.spin_once(timeout_sec=.01)
            recent = [p['xyz'] for p in poses if p['at']>time.monotonic()-1.]
            stationary = len(recent)>=5 and max(math.dist(recent[0],p) for p in recent)<.02
            stable_since = (stable_since or time.monotonic()) if ready() and (stationary or args.rotating_gimbal) else None
            if stable_since and time.monotonic()-stable_since>=1.: break
        if not stable_since or time.monotonic()-stable_since<1.:
            raise RuntimeError('Real state/cloud/LIO/CRC-valid serial input not continuously ready')
        start_pose = poses[-1]['xyz']
        report['pulse_started']=True
        deadline=time.monotonic()+args.duration
        while time.monotonic()<deadline:
            if not ready(): raise RuntimeError('Live state or serial feedback lost; pulse aborted')
            if math.dist(poses[-1]['xyz'],start_pose)>displacement_bound:
                raise RuntimeError('Measured displacement exceeds selected commissioning bound')
            send(args.speed); executor.spin_once(timeout_sec=.01)
        report['completed']=True
    except (Exception,KeyboardInterrupt) as error:
        report['error']=str(error)
    finally:
        # Keep transport alive with explicit fresh zeros after every outcome.
        if bridge is not None:
            deadline=time.monotonic()+2.
            while time.monotonic()<deadline and rclpy.ok():
                send(0.); executor.spin_once(timeout_sec=.01)
            report['serial_last_rx_age_s']=(time.monotonic()-bridge.last_rx) if bridge.last_rx else None
            executor.remove_node(bridge); bridge.destroy_node()
        if start_pose is not None and poses:
            report['estimated_end_displacement_m']=math.dist(poses[-1]['xyz'],start_pose)
            tail=[p['xyz'] for p in poses if p['at']>poses[-1]['at']-1.]
            report['estimated_last1s_span_m']=max((math.dist(a,b) for a in tail for b in tail),default=0.)
        report['command_samples']=len(records)
        report['last_health']={k:{'value':v,'age_s':time.monotonic()-t} for k,(v,t) in health.items()}
        (args.output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
        (args.output/'commands_and_poses.json').write_text(json.dumps({'commands':records,'poses':poses})+'\n')
        executor.remove_node(probe); probe.destroy_node(); executor.shutdown(); rclpy.shutdown()
    print(json.dumps(report,indent=2),flush=True)
    return 0 if report['completed'] else 1


if __name__=='__main__':
    raise SystemExit(main())
