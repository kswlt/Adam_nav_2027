"""Wire regression tests, corruption recovery and original CRC compatibility."""
import ast
from pathlib import Path
import random
import struct

import pytest

from my_serial_py.protocol import (
    FeedbackStream, RX_FORMAT, RX_SIZE, TX_FORMAT, TX_SIZE,
    decode_feedback, encode_command, modbus_crc16, rm_crc16,
)


def packet():
    body = b'\xa5' + struct.pack(RX_FORMAT, 4, 4, 100, 200, 60, 42, 1200, 1500,
                                  0x12345679, -12.5, 1)
    return body + struct.pack('<H', rm_crc16(body))


def test_crc_known_vectors():
    assert modbus_crc16(b'123456789') == 0x4b37
    assert rm_crc16(b'123456789') == 0x6f91
    assert RX_SIZE == 26
    assert TX_SIZE == 54


def test_original_rx_crc_table():
    # Frozen source from the initial copied package (before this refactoring).
    tree = ast.parse(Path(__file__).with_name('fixtures').joinpath('legacy_crc.py').read_text())
    scope = {}
    exec(compile(tree, '<original-rm-crc>', 'exec'), scope)
    rng = random.Random(2027)
    for length in range(128):
        data = bytes(rng.randrange(256) for _ in range(length))
        assert rm_crc16(data) == scope['get_rm_crc16'](data)


def test_tx_original_layout_and_modbus():
    tx = encode_command(0.5, -0.25, 30.0, 3, 5, 1)
    # Layout built independently using the original format and values.
    payload = struct.pack('<BffffBBB8f', 0xaa, -0.5, 0.25, 30., 30., 3, 5, 1, *([1.] * 8))
    assert tx[:-2] == payload
    assert tx[-2:] == b'\xce\xfd'  # golden CRC from original libscrc.modbus 1.8.1
    assert struct.unpack(TX_FORMAT, tx[:-2])[5:8] == (3, 5, 1)


def test_modbus_original_library_compatibility():
    libscrc = pytest.importorskip('libscrc', reason='Optional original-library cross-check')
    rng = random.Random(2027)
    for _ in range(100):
        x, y, yaw = (rng.uniform(-100, 100) for _ in range(3))
        stance, region, aligned = (rng.randrange(256) for _ in range(3))
        old_payload = struct.pack('<BffffBBB8f', 0xaa, -x, -y, yaw, yaw,
                                  stance, region, aligned, *([1.] * 8))
        old_packet = old_payload + struct.pack('<H', libscrc.modbus(old_payload))
        assert encode_command(x, y, yaw, stance, region, aligned) == old_packet


def test_rx_fields():
    feedback = decode_feedback(packet())
    assert feedback.remain_hp == 100
    assert feedback.rfid_status == 0x12345679
    assert feedback.contact_angle == -12.5
    assert feedback.is_fire == 1


def test_split_noise_bad_crc_and_concatenation():
    frame = packet()
    broken = bytearray(frame)
    broken[6] ^= 1
    stream = FeedbackStream()
    data = b'garbage' + bytes(broken) + frame + frame
    result = []
    for byte in data:
        result.extend(stream.feed(bytes([byte])))
    assert len(result) == 2
    assert stream.crc_errors >= 1
    assert len(stream.buffer) < RX_SIZE
    assert len(stream.feed(frame * 3)) == 3
    assert not stream.buffer


@pytest.mark.parametrize('data', [b'', b'\xa5', b'\x00' * RX_SIZE, b'\xa5' * RX_SIZE])
def test_invalid_packet(data):
    with pytest.raises(ValueError):
        decode_feedback(data)


@pytest.mark.parametrize('value', [float('nan'), float('inf'), -float('inf')])
def test_reject_nonfinite_commands(value):
    with pytest.raises(ValueError):
        encode_command(value, 0, 0)
