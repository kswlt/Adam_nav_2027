"""Original Adam wire contract, independent of ROS and serial device I/O.

RX uses reflected 0x8408 CRC (init 0xffff, no final XOR).
TX uses Modbus 0xa001 CRC. All fields and CRC bytes are little endian.
"""
from dataclasses import dataclass
import math
import struct

RX_FORMAT = '<BBHHHHHHIfB'
TX_FORMAT = '<BffffBBB8f'
RX_SIZE = 1 + struct.calcsize(RX_FORMAT) + 2
TX_SIZE = struct.calcsize(TX_FORMAT) + 2


def _crc(data, polynomial):
    value = 0xffff
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = (value >> 1) ^ (polynomial if value & 1 else 0)
    return value


def rm_crc16(data):
    return _crc(data, 0x8408)


def modbus_crc16(data):
    return _crc(data, 0xa001)


@dataclass(frozen=True)
class Feedback:
    game_type: int
    game_progress: int
    remain_hp: int
    max_hp: int
    stage_remain_time: int
    bullet_17mm: int
    outpost_hp: int
    base_hp: int
    rfid_status: int
    contact_angle: float
    is_fire: int


def decode_feedback(packet):
    if len(packet) != RX_SIZE or packet[0] != 0xa5:
        raise ValueError('Invalid RX header or length')
    if struct.unpack('<H', packet[-2:])[0] != rm_crc16(packet[:-2]):
        raise ValueError('Invalid RX CRC')
    return Feedback(*struct.unpack(RX_FORMAT, packet[1:-2]))


def encode_command(x, y, yaw_degrees, stance=3, region=0, aligned=0,
                   reserved=(1.0,) * 8):
    if len(reserved) != 8 or not all(math.isfinite(v) for v in (x, y, yaw_degrees, *reserved)):
        raise ValueError('TX float fields must be finite; eight reserved slots required')
    payload = struct.pack(TX_FORMAT, 0xaa, -x, -y, yaw_degrees, yaw_degrees,
                          stance, region, aligned, *reserved)
    return payload + struct.pack('<H', modbus_crc16(payload))


class FeedbackStream:
    """Resynchronize noisy/split streams without retaining unbounded history."""

    def __init__(self):
        self.buffer = bytearray()
        self.crc_errors = 0
        self.discarded_bytes = 0

    def feed(self, data):
        self.buffer.extend(data)
        frames = []
        while self.buffer:
            start = self.buffer.find(b'\xa5')
            if start < 0:
                self.discarded_bytes += len(self.buffer)
                self.buffer.clear()
                break
            if start:
                self.discarded_bytes += start
                del self.buffer[:start]
            if len(self.buffer) < RX_SIZE:
                break
            try:
                frames.append(decode_feedback(self.buffer[:RX_SIZE]))
                del self.buffer[:RX_SIZE]
            except ValueError:
                self.crc_errors += 1
                self.discarded_bytes += 1
                del self.buffer[0]
        return frames
