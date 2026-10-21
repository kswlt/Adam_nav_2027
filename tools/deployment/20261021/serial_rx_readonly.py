#!/usr/bin/env python3
"""Bounded read-only actual MCU RX; never calls write or serial.Serial."""
import argparse
import json
import math
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
    parser.add_argument('--duration', type=float, default=5.)
    args = parser.parse_args()
    if not math.isfinite(args.duration) or not 0 < args.duration <= 300:
        raise ValueError('Read-only duration must be 0..300 seconds')
    owner = subprocess.run(['fuser', args.port], capture_output=True, text=True)
    if owner.returncode != 1:
        raise RuntimeError('Port is occupied or ownership check failed: '+owner.stdout+owner.stderr)
    args.output.mkdir(exist_ok=False)
    raw_log = (args.output/'rx.bin').open('wb')
    frame_log = (args.output/'frames.jsonl').open('w', buffering=1)
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
        while time.monotonic()-start < args.duration:
            try:
                data = os.read(fd,4096)
                raw.extend(data)
                raw_log.write(data)
                received = decoder.feed(data)
                frames.extend(received)
                wall_ns, mono_ns = time.time_ns(), time.monotonic_ns()
                for frame in received:
                    frame_log.write(json.dumps({'host_received_system_ns':wall_ns,
                        'host_received_monotonic_ns':mono_ns,
                        'timestamp_scope':'host read batch; MCU acquisition time unavailable',
                        'feedback':vars(frame)})+'\n')
                raw_log.flush()
            except BlockingIOError:
                pass
            time.sleep(.005)
    finally:
        termios.tcsetattr(fd, termios.TCSANOW, old)
        os.close(fd)
        raw_log.close()
        frame_log.close()
    duration = time.monotonic()-start
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
