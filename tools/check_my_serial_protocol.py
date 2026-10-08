"""Static protocol checks for the frozen my_serial_py baseline."""

import struct


RX_FORMAT = "<BBHHHHHHIfB"
TX_FORMAT = "<BffffBBB8f"


def main() -> None:
    assert struct.calcsize(RX_FORMAT) == 23
    assert 1 + struct.calcsize(RX_FORMAT) + 2 == 26
    # Current source includes header, four floats, three uint8 fields and eight
    # reserved floats: 52-byte payload plus 2-byte Modbus CRC.
    assert struct.calcsize(TX_FORMAT) == 52
    assert struct.calcsize(TX_FORMAT) + 2 == 54
    print("my_serial_py protocol sizes: PASS")


if __name__ == "__main__":
    main()
