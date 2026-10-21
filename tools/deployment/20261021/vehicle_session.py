#!/usr/bin/env python3
"""Manage only processes tagged as this isolated session; no global process kills.

Physical is an explicit operator-selected component. Candidate remains device-free.
"""
import argparse
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import time

ROOT = Path('/home/asus/nav_2027_vehicle_config_164')
RUNTIME = ROOT/'runtime_20261021_sensor_reference'
RUNNER = Path(__file__).with_name('vehicle_component.bash')
COMPONENTS = ('driver', 'state', 'foxglove', 'capture', 'online', 'candidate', 'bag', 'physical')
MARKER = b'NAV_MANAGED_SESSION=20261021_sensor_reference'


def owned(pid):
    try:
        proc = Path('/proc')/str(pid)
        return (proc.stat().st_uid == os.getuid() and os.getpgid(pid) == pid and
                MARKER in (proc/'environ').read_bytes().split(b'\0'))
    except (OSError, ProcessLookupError):
        return False


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('action', choices=('start', 'stop', 'status'))
    parser.add_argument('component', choices=COMPONENTS)
    parser.add_argument('arguments', nargs='*')
    args = parser.parse_args()
    RUNTIME.mkdir(exist_ok=True)
    with (RUNTIME/(args.component+'.management.lock')).open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        record = RUNTIME/(args.component+'.process.json')
        old = json.loads(record.read_text()) if record.exists() else {}
        pid = old.get('pid', 0)
        running = pid > 0 and owned(pid)
        if args.action == 'status':
            print(json.dumps(dict(old, running=running)))
            return
        if args.action == 'stop':
            if not running:
                print('No live owned instance; no signal sent')
                return
            os.killpg(pid, signal.SIGINT)
            deadline = time.monotonic()+8.
            while owned(pid) and time.monotonic() < deadline:
                time.sleep(.05)
            if owned(pid):
                raise SystemExit('Owned component still stopping; restart refused until exit: '+str(pid))
            print('Owned process group stopped: '+str(pid))
            return
        if running:
            print('Already running: '+str(pid))
            return
        logfile = RUNTIME/(args.component+'_'+time.strftime('%Y%m%d_%H%M%S')+'.log')
        env = dict(os.environ, NAV_MANAGED_SESSION='20261021_sensor_reference')
        with logfile.open('ab') as stream:
            process = subprocess.Popen(['bash', str(RUNNER), args.component, *args.arguments],
                stdout=stream, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                env=env, start_new_session=True)
        data = {'pid': process.pid, 'component': args.component, 'log': str(logfile)}
        temporary = record.with_suffix('.tmp')
        temporary.write_text(json.dumps(data)+'\n')
        temporary.replace(record)
        time.sleep(1.)
        if process.poll() is not None:
            raise SystemExit('Component exited; inspect '+str(logfile))
        print(json.dumps(data))


if __name__ == '__main__':
    main()
