#!/usr/bin/env python3
"""TEST_ONLY LIO -> reference -> compatibility adapter -> real my_serial_py -> PTY.

The fake STM32 uses only /dev/pts. Never opens any physical serial device.
"""
import json
import math
import os
from pathlib import Path
import pty
import signal
import struct
import subprocess
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32, UInt8, UInt32
from my_serial_py.protocol import RX_FORMAT, TX_FORMAT, TX_SIZE, rm_crc16, modbus_crc16


def main():
    os.environ.setdefault('ROS_DOMAIN_ID', '206')
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    master, slave = pty.openpty()
    os.set_blocking(master, False)
    port = os.ttyname(slave)
    os.close(slave)
    rclpy.init()
    node = Node('test_only_legacy_adam_transport')
    pubs = {key: node.create_publisher(typ, topic, 10) for key, typ, topic in (
        ('lio', Odometry, '/lio/sensor_odometry'),
        ('raw', Bool, '/sensors/front_mid360/cloud_healthy'),
        ('permission', Bool, '/nav/motion_allowed'),
        ('command', TwistStamped, '/nav/cmd_vel_safe'),
        ('decoy', Twist, '/cmd_vel'),
        ('yaw', Float32, '/cmd_yaw_angle'),
        ('stance', UInt32, '/cmd_stance'), ('region', UInt8, '/region'),
        ('aligned', UInt8, '/big_yaw_aligned'))}
    feedback, frames, buffer = [], [], bytearray()
    node.create_subscription(Float32, '/contact_angle', lambda m: feedback.append(m.data), 10)
    processes, logs = [], []
    root = Path('log/legacy_adam_transport')
    root.mkdir(parents=True, exist_ok=True)
    body = b'\xa5'+struct.pack(RX_FORMAT, 4, 4, 100, 200, 60, 42, 1200, 1500, 9, -12.5, 1)
    rx = body+struct.pack('<H', rm_crc16(body))

    fixture_yaw, fixture_at = None, time.monotonic()
    def run(seconds, *, yaw=0., xy=None, pose=True, quality=True, allow=True, receive=True):
        nonlocal fixture_yaw, fixture_at
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            assert all(p.poll() is None for p in processes), 'transport process exited'
            pubs['raw'].publish(Bool(data=quality))
            pubs['permission'].publish(Bool(data=allow))
            if pose:
                now = time.monotonic()
                if fixture_yaw is None:
                    fixture_yaw = yaw
                delta = math.atan2(math.sin(yaw-fixture_yaw),math.cos(yaw-fixture_yaw))
                step = 8.*(now-fixture_at)
                fixture_yaw += max(-step,min(step,delta))
                fixture_at = now
                m = Odometry()
                m.header.frame_id, m.child_frame_id = 'odom', 'front_mid360_imu'
                m.header.stamp = node.get_clock().now().to_msg()
                m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(fixture_yaw/2), math.cos(fixture_yaw/2)
                for i in range(6):
                    m.pose.covariance[i*7] = .01
                pubs['lio'].publish(m)
            if xy is not None:
                m = TwistStamped()
                m.header.frame_id, m.header.stamp = 'nav_reference', node.get_clock().now().to_msg()
                m.twist.linear.x, m.twist.linear.y = xy
                pubs['command'].publish(m)
            # Old region/behavior global cmd_vel must not contend with private input.
            decoy = Twist()
            decoy.linear.x = .49
            pubs['decoy'].publish(decoy)
            if receive:
                try:
                    os.write(master, rx)
                except OSError:
                    pass  # PTY slave may not yet be open during startup.
            rclpy.spin_once(node, timeout_sec=.005)
            try:
                buffer.extend(os.read(master, 8192))
            except (BlockingIOError, OSError):
                pass
            while len(buffer) >= TX_SIZE:
                if buffer[0] != 0xaa:
                    del buffer[0]
                    continue
                packet = bytes(buffer[:TX_SIZE])
                assert modbus_crc16(packet[:-2]) == struct.unpack('<H', packet[-2:])[0], 'bad TX CRC'
                frames.append(struct.unpack(TX_FORMAT, packet[:-2]))
                del buffer[:TX_SIZE]
            time.sleep(.01)
    def stopped():
        return frames and max(abs(v) for v in frames[-1][1:3]) < 1e-6
    result = {'mode': 'TEST_ONLY', 'physical_io': False, 'firmware_modified': False}
    try:
        for name, command in (
            ('state', ['ros2', 'run', 'rm_nav_bringup', 'sensor_reference']),
            ('transport', ['ros2', 'launch', 'rm_nav_bringup', 'legacy_adam_transport.launch.py',
                           'transport:=pty', 'serial_port:='+port])):
            log = (root/(name+'.log')).open('w')
            logs.append(log)
            processes.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
        run(2.)
        assert frames and stopped(), 'legacy startup did not send zero xy'
        assert frames[-1][3:8] == (0., 0., 3, 0, 0), 'original cached yaw/default fields changed'
        assert frames[-1][8:] == (1.,)*8
        assert feedback and feedback[-1] == -12.5
        assert pubs['decoy'].get_subscription_count() == 0, 'global cmd_vel accidentally connected'
        pubs['yaw'].publish(Float32(data=31.5))
        pubs['stance'].publish(UInt32(data=2))
        pubs['region'].publish(UInt8(data=5))
        pubs['aligned'].publish(UInt8(data=1))
        run(.25)
        count, start = len(frames), time.monotonic()
        run(1., xy=(.2, -.1))
        result['observed_tx_hz'] = (len(frames)-count)/(time.monotonic()-start)
        assert 35 < result['observed_tx_hz'] < 65, '50Hz cadence outside tolerance'
        # Same physical meaning as fake_vel_transform, followed by original wire negation.
        for theta, xy in ((0., (.2, -.1)), (math.pi/2, (.2, 0.)),
                          (-math.pi/2, (0., .2)), (math.pi, (-.2, .1))):
            run(.5, yaw=theta, xy=xy)
            x, y = xy
            expected = (-(math.cos(theta)*x+math.sin(theta)*y),
                        -(-math.sin(theta)*x+math.cos(theta)*y))
            assert max(abs(a-b) for a, b in zip(frames[-1][1:3], expected)) < 1e-5
            assert frames[-1][3:8] == (31.5, 31.5, 2, 5, 1), 'cached yaw or passthrough changed'
        run(.5, xy=(.2, 0.), quality=False)
        assert stopped(), 'raw quality did not stop wire xy'
        run(.4)
        assert stopped(), 'old command replayed after quality recovery'
        run(.4, xy=(.2, 0.))
        assert not stopped()
        run(.65, xy=(.2, 0.), receive=False)
        assert stopped(), 'RX dropout did not stop wire xy'
        run(.4)
        assert stopped(), 'old command replayed after RX recovery'
        run(.4, xy=(.2, 0.))
        run(.5, xy=(.2, 0.), allow=False)
        assert stopped(), 'permission loss did not stop wire xy'
        run(.4)
        assert stopped(), 'permission recovery replayed a command'
        run(.4, xy=(.2, 0.))
        run(.5, pose=False, xy=(.2, 0.))
        assert stopped(), 'LIO loss did not stop wire xy'
        run(.4)
        assert stopped(), 'LIO recovery replayed a command'
        run(.4, xy=(.2, 0.))
        run(.6)
        assert stopped(), 'command timeout did not stop wire xy'
        assert frames[-1][3:8] == (31.5, 31.5, 2, 5, 1)
        result.update({'passed': True, 'tx_bytes': 54, 'rx_bytes': 26,
            'rotation_and_single_negation': True, 'contact_angle_not_required': True,
            'cached_yaw_passthrough': True, 'global_cmd_vel_isolated': True,
            'raw_quality_stop': True, 'rx_dropout_stop': True, 'permission_stop': True,
            'lio_dropout_stop': True, 'command_timeout_stop': True, 'no_replay': True,
            'full_chassis_stop': 'NOT_VERIFIED'})
        (root/'result.json').write_text(json.dumps(result, indent=2)+'\n')
        print(json.dumps(result, indent=2), flush=True)
    finally:
        for process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
        for log in logs:
            log.close()
        os.close(master)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
