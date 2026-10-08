"""Adam ROS serial bridge. Wire codec is in protocol.py.

Original topic/field semantics are preserved. Stale planar velocity stops
translation. Yaw is an angle in degrees; angular.z is not sent by this protocol.
The unchanged serialpy_node.py is retained as provenance and regression input.
"""
import math
import threading
import time

import rclpy
from rclpy.node import Node
import serial
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8, Float32, UInt8, UInt32
from pb_rm_interfaces.msg import RobotStatus, GameStatus, GameRobotHP, RfidStatus

from my_serial_py.protocol import FeedbackStream, encode_command


class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')
        self.serial_port = self.declare_parameter('serial_port', '/dev/ttyUSB0').value
        self.baud_rate = self.declare_parameter('baud_rate', 115200).value
        self.cmd_timeout = self.declare_parameter('cmd_vel_timeout', 0.3).value
        if not math.isfinite(self.cmd_timeout) or self.cmd_timeout <= 0:
            raise ValueError('cmd_vel_timeout must be positive and finite')
        self.io_lock = threading.Lock()
        self.serial_conn = None
        self.stop_event = threading.Event()
        self.stance_running_state = 3
        self.region_code = 0
        self.yaw_angle = 0.0
        self.big_yaw_aligned = 0
        self.latest_x = self.latest_y = 0.0
        self.last_command = None
        self.tx_reserved_fields = [1.0] * 8
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(Int8, '/cmd_chassis_mode', self.chassis_mode_callback, 10)
        self.create_subscription(UInt32, '/cmd_stance', self.stance_callback, 10)
        self.create_subscription(Float32, '/cmd_yaw_angle', self.yaw_angle_callback, 10)
        self.create_subscription(UInt8, '/region', self.region_callback, 10)
        self.create_subscription(UInt8, '/big_yaw_aligned', self.big_yaw_aligned_callback, 10)
        self.pubs = {
            'game_status': self.create_publisher(GameStatus, '/referee/game_status', 10),
            'robot_status': self.create_publisher(RobotStatus, '/referee/robot_status', 10),
            'all_robot_hp': self.create_publisher(GameRobotHP, '/referee/all_robot_hp', 10),
            'rfid_status': self.create_publisher(RfidStatus, '/referee/rfid_status', 10),
            'contact_angle': self.create_publisher(Float32, '/contact_angle', 10),
            'is_fire': self.create_publisher(UInt8, '/is_fire', 10),
        }
        self.read_thread = threading.Thread(target=self.serial_read_loop, daemon=True)
        self.read_thread.start()
        self.create_timer(0.02, self.periodic_send_to_stm32)
        self.get_logger().info(
            f'Serial {self.serial_port} @ {self.baud_rate}; RX 26 / TX 54 bytes; '
            f'planar velocity timeout {self.cmd_timeout}s')

    def disconnect_locked(self):
        if self.serial_conn is not None:
            try:
                self.serial_conn.close()
            except serial.SerialException:
                pass
        self.serial_conn = None
        # Do not resume a pre-disconnect motion command on a new connection.
        self.last_command = None
        self.latest_x = self.latest_y = 0.0

    def serial_read_loop(self):
        stream = FeedbackStream()
        while not self.stop_event.is_set():
            data = b''
            try:
                with self.io_lock:
                    if self.serial_conn is None:
                        self.serial_conn = serial.Serial(
                            self.serial_port, self.baud_rate, timeout=0.1, write_timeout=0.1)
                        stream = FeedbackStream()
                    waiting = self.serial_conn.in_waiting
                    if waiting:
                        data = self.serial_conn.read(min(waiting, 4096))
                for feedback in stream.feed(data):
                    self.publish_feedback(feedback)
                self.stop_event.wait(0.005)
            except (serial.SerialException, OSError) as error:
                with self.io_lock:
                    self.disconnect_locked()
                self.get_logger().warning(f'Serial reconnect: {error}')
                self.stop_event.wait(1.0)

    def publish_feedback(self, f):
        rs = RobotStatus()
        rs.current_hp, rs.maximum_hp = f.remain_hp, f.max_hp
        rs.projectile_allowance_17mm = f.bullet_17mm
        self.pubs['robot_status'].publish(rs)
        gs = GameStatus()
        gs.game_type, gs.game_progress = f.game_type, f.game_progress
        gs.stage_remain_time = f.stage_remain_time
        self.pubs['game_status'].publish(gs)
        hp = GameRobotHP()
        hp.red_outpost_hp, hp.red_base_hp = f.outpost_hp, f.base_hp
        self.pubs['all_robot_hp'].publish(hp)
        rfid = RfidStatus()
        rfid.friendly_fortress_gain_point = bool(f.rfid_status & (1 << 0))
        rfid.center_gain_point = bool(f.rfid_status & (1 << 3))
        self.pubs['rfid_status'].publish(rfid)
        if math.isfinite(f.contact_angle):
            self.pubs['contact_angle'].publish(Float32(data=f.contact_angle))
        self.pubs['is_fire'].publish(UInt8(data=f.is_fire))

    def chassis_mode_callback(self, msg):
        # Original behavior: stance controls running_state; ignore mode overrides.
        pass

    def cmd_vel_callback(self, msg):
        with self.io_lock:
            if math.isfinite(msg.linear.x) and math.isfinite(msg.linear.y):
                self.latest_x, self.latest_y = msg.linear.x, msg.linear.y
                self.last_command = time.monotonic()
            else:
                self.latest_x = self.latest_y = 0.0
                self.last_command = None

    def yaw_angle_callback(self, msg):
        if math.isfinite(msg.data):
            self.yaw_angle = float(msg.data)

    def stance_callback(self, msg):
        if 0 <= msg.data <= 255:
            self.stance_running_state = int(msg.data)

    def region_callback(self, msg):
        self.region_code = int(msg.data)

    def big_yaw_aligned_callback(self, msg):
        self.big_yaw_aligned = int(msg.data != 0)

    def periodic_send_to_stm32(self):
        with self.io_lock:
            if self.serial_conn is None:
                return
            fresh = (self.last_command is not None and
                     time.monotonic() - self.last_command <= self.cmd_timeout)
            try:
                packet = encode_command(
                    self.latest_x if fresh else 0.0, self.latest_y if fresh else 0.0,
                    self.yaw_angle, self.stance_running_state, self.region_code,
                    self.big_yaw_aligned, self.tx_reserved_fields)
                written = self.serial_conn.write(packet)
                if written != len(packet):
                    raise serial.SerialException('Partial serial write')
            except (serial.SerialException, OSError) as error:
                self.disconnect_locked()
                self.get_logger().error(f'Serial TX failed: {error}')

    def destroy_node(self):
        self.stop_event.set()
        self.read_thread.join(timeout=2.0)
        with self.io_lock:
            self.disconnect_locked()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SerialNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
