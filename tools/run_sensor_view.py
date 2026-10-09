#!/usr/bin/env python3
"""Foreground session supervisor: lock, process group shutdown, checked exit.
Source ROS, Livox, LIO and project overlays first. Lock covers all ROS domains.
"""
import argparse
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--driver-config',required=True)
    parser.add_argument('--lio-params',required=True)
    parser.add_argument('--port',type=int,default=8765)
    parser.add_argument('--runtime-dir',default=str(Path.home()/'.local/state/adam_nav/sensor_view'))
    parser.add_argument('--check-lock',action='store_true',help='Only acquire/release lock; do not launch sensors')
    args=parser.parse_args()
    if not 1024<=args.port<=65535:raise ValueError('Nonprivileged port required')
    driver=Path(args.driver_config).resolve(strict=True);lio=Path(args.lio_params).resolve(strict=True)
    runtime=Path(args.runtime_dir).resolve();runtime.mkdir(parents=True,exist_ok=True)
    with (runtime/'session.lock').open('a') as lock:
        try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:
            print('Sensor view session already running; refusing duplicate',file=sys.stderr);return 2
        if args.check_lock:return 0
        stopping=False
        child=None
        def stop(signum,frame):
            nonlocal stopping
            stopping=True
            if child is not None and child.poll() is None:
                try:os.killpg(child.pid,signal.SIGINT)
                except ProcessLookupError:pass
        signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
        command=['ros2','launch','rm_nav_bringup','mid360_diagnostic.launch.py',
                 'driver_config:='+str(driver),'lio_params_file:='+str(lio),'port:='+str(args.port)]
        with (runtime/'session.log').open('a') as log:
            child=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            (runtime/'session.json').write_text(json.dumps({'supervisor_pid':os.getpid(),'launch_pid':child.pid,
                'started_unix_s':time.time(),'domain':os.environ.get('ROS_DOMAIN_ID','0'),'command':command,
                'scope':'sensor diagnostic; unverified internal extrinsics; no navigation/serial'},indent=2)+'\n')
            print('Sensor view supervisor PID='+str(os.getpid())+'; log='+str(runtime/'session.log'),flush=True)
            while child.poll() is None:
                if stopping:
                    try:child.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        os.killpg(child.pid,signal.SIGTERM)
                        try:child.wait(timeout=5)
                        except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
                    break
                time.sleep(.2)
            (runtime/'last_exit.json').write_text(json.dumps({'returncode':child.returncode,
                'requested_stop':stopping,'ended_unix_s':time.time()},indent=2)+'\n')
            return 0 if stopping else child.returncode


if __name__=='__main__':raise SystemExit(main())
