#!/usr/bin/env python3
"""ROS/PTY acceptance with simulated STM32; never opens a physical serial port."""
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
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, UInt8, UInt32
from pb_rm_interfaces.msg import RobotStatus, GameStatus, GameRobotHP, RfidStatus
from my_serial_py.protocol import RX_FORMAT, TX_FORMAT, TX_SIZE, rm_crc16, modbus_crc16


def require(value, description):
    if not value:
        raise RuntimeError(description)


def main():
    require(os.environ.get('ROS_DOMAIN_ID') not in (None, '', '0'), 'Use dedicated ROS_DOMAIN_ID')
    master, slave = pty.openpty()
    os.set_blocking(master, False)
    path = os.ttyname(slave)
    rclpy.init()
    node = Node('serial_pty_acceptance')
    feedback = {}
    for topic, msgtype in [
        ('/referee/robot_status', RobotStatus), ('/referee/game_status', GameStatus),
        ('/referee/all_robot_hp', GameRobotHP), ('/referee/rfid_status', RfidStatus),
        ('/contact_angle', Float32), ('/is_fire', UInt8),
    ]:
        node.create_subscription(msgtype, topic, lambda msg, key=topic: feedback.update({key: msg}), 10)
    pubs = {topic: node.create_publisher(msgtype, topic, 10) for topic, msgtype in [
        ('/cmd_vel', Twist), ('/cmd_yaw_angle', Float32), ('/cmd_stance', UInt32),
        ('/region', UInt8), ('/big_yaw_aligned', UInt8)]}
    buffer = bytearray()
    received = []
    process = None
    log_path = Path('log/serial_pty_smoke.log')
    log_path.parent.mkdir(exist_ok=True)

    def poll():
        rclpy.spin_once(node, timeout_sec=0.01)
        try:
            buffer.extend(os.read(master, 8192))
        except BlockingIOError:
            pass
        while len(buffer) >= TX_SIZE:
            if buffer[0] != 0xaa:
                del buffer[0]
                continue
            frame = bytes(buffer[:TX_SIZE])
            if modbus_crc16(frame[:-2]) != struct.unpack('<H', frame[-2:])[0]:
                del buffer[0]
                continue
            received.append(struct.unpack(TX_FORMAT, frame[:-2]))
            del buffer[:TX_SIZE]

    def wait(predicate, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            poll()
            if predicate():
                return True
        return False

    try:
        with log_path.open('w') as log:
            process = subprocess.Popen([
                'ros2', 'run', 'my_serial_py', 'serial_node', '--ros-args',
                '-p', 'serial_port:=' + path, '-p', 'cmd_vel_timeout:=0.3'],
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            require(wait(lambda: bool(received), 10), 'No TX frame from bridge')
            require(received[-1][1:3] == (-0., -0.), 'Startup motion is nonzero')
            require(wait(lambda: all(p.get_subscription_count() for p in pubs.values()), 10),
                    'ROS subscription discovery timeout')
            body = b'\xa5' + struct.pack(RX_FORMAT, 4, 4, 100, 200, 60, 42, 1200, 1500, 9, -12.5, 1)
            frame = body + struct.pack('<H', rm_crc16(body))
            corrupt = bytearray(frame)
            corrupt[4] ^= 1
            os.write(master, b'noise' + bytes(corrupt) + frame[:8])
            wait(lambda: False, 0.1)
            require(not feedback, 'Corrupt/incomplete feedback was published')
            os.write(master, frame[8:])
            require(wait(lambda: len(feedback) == 6, 5), 'Split valid feedback not published')
            require(feedback['/referee/robot_status'].current_hp == 100, 'HP mapping failed')
            require(feedback['/referee/robot_status'].projectile_allowance_17mm == 42, 'Bullet mapping failed')
            require(feedback['/referee/game_status'].game_type == 4, 'Game type mapping failed')
            require(feedback['/referee/game_status'].stage_remain_time == 60, 'Game time mapping failed')
            require(feedback['/referee/all_robot_hp'].red_base_hp == 1500, 'Base mapping failed')
            require(feedback['/referee/rfid_status'].center_gain_point, 'RFID mapping failed')
            require(feedback['/contact_angle'].data == -12.5, 'Contact angle mapping failed')
            require(feedback['/is_fire'].data == 1, 'Fire mapping failed')
            for topic, msg in [('/cmd_yaw_angle', Float32(data=30.0)),
                               ('/cmd_stance', UInt32(data=2)), ('/region', UInt8(data=5)),
                               ('/big_yaw_aligned', UInt8(data=1))]:
                pubs[topic].publish(msg)
            cmd = Twist()
            cmd.linear.x, cmd.linear.y, cmd.angular.z = 0.5, -0.25, 99.0
            end = time.monotonic() + 1.0
            while time.monotonic() < end:
                pubs['/cmd_vel'].publish(cmd)
                poll()
            require(received[-1][1:8] == (-0.5, 0.25, 30., 30., 2, 5, 1), 'TX fields changed')
            require(received[-1][8:] == (1.,) * 8, 'Reserved fields changed')
            require(wait(lambda: received[-1][1:3] == (-0., -0.), 1), 'Command timeout did not stop')
            require(received[-1][3:5] == (30., 30.), 'Timeout changed yaw angle semantics')
            cmd.linear.x = float('nan')
            pubs['/cmd_vel'].publish(cmd)
            wait(lambda: False, 0.15)
            require(received[-1][1:3] == (-0., -0.), 'Nonfinite command did not stop')
            require(all(math.isfinite(v) for v in received[-1]), 'Nonfinite wire output')
            print('PASS PTY: 54-byte TX, 26-byte RX, split/noise/CRC recovery, ROS fields, '
                  'signs/yaw/reserved compatibility, watchdog/nonfinite stop', flush=True)
    finally:
        if process is not None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        node.destroy_node()
        rclpy.shutdown()
        os.close(master)
        os.close(slave)


if __name__ == '__main__':
    main()
