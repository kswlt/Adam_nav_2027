#!/usr/bin/env python3
"""Bounded read-only actual MCU RX; never calls write or serial.Serial."""
import argparse
import json
import os
import subprocess
from pathlib import Path
import termios
import time
from my_serial_py.protocol import FeedbackStream


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    owner = subprocess.run(['fuser', args.port], capture_output=True, text=True)
    if owner.returncode != 1:
        raise RuntimeError('Port is occupied or ownership check failed: '+owner.stdout+owner.stderr)
    fd = os.open(args.port, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
    old = termios.tcgetattr(fd)
    raw, frames = bytearray(), []
    decoder = FeedbackStream()
    start = time.monotonic()
    try:
        config = termios.tcgetattr(fd)
        config[0] = config[1] = config[3] = 0
        config[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        config[4] = config[5] = termios.B115200
        config[6][termios.VMIN] = config[6][termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW, config)
        while time.monotonic()-start < 5.:
            try:
                data = os.read(fd,4096)
                raw.extend(data)
                frames.extend(decoder.feed(data))
            except BlockingIOError:
                pass
            time.sleep(.005)
    finally:
        termios.tcsetattr(fd, termios.TCSANOW, old)
        os.close(fd)
    duration = time.monotonic()-start
    args.output.mkdir(exist_ok=False)
    (args.output/'rx.bin').write_bytes(raw)
    report = {'scope':'REAL_MCU_RX_READ_ONLY', 'tx_bytes':0, 'rx_bytes':len(raw),
        'valid_frames':len(frames),'crc_errors':decoder.crc_errors,'discarded_bytes':decoder.discarded_bytes,
        'duration_s':duration,'valid_hz':len(frames)/duration,
        'first':vars(frames[0]) if frames else None, 'last':vars(frames[-1]) if frames else None,
        'contact_angle_semantics_verified':False, 'wheel_or_stop_feedback_verified':False}
    (args.output/'result.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
    return 0 if frames else 1


if __name__ == '__main__':
    raise SystemExit(main())
