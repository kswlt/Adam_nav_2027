#!/usr/bin/env python3
"""Read-only live smoke: HTTP auth, real PTY, ROS state; never save/stop/move."""
import argparse
import base64
import json
from pathlib import Path
import time
from urllib.error import HTTPError
from urllib.request import Request, urlopen


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--url', default='http://127.0.0.1:8878')
    parser.add_argument('--token-file', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    key = args.token_file.read_text().strip()
    def api(path, data=None):
        request = Request(args.url+path, data=json.dumps(data).encode() if data is not None else None,
                          headers={'Authorization': 'Bearer '+key, 'Content-Type': 'application/json'})
        with urlopen(request, timeout=8) as response:
            return json.load(response)
    denied = False
    try:
        urlopen(args.url+'/api/status', timeout=5)
    except HTTPError as exc:
        denied = exc.code == 401
    before = api('/api/status')
    terminal = api('/api/terminal/open', {})['id']
    try:
        command = "printf '__OPERATOR_DOMAIN_%s__\\n' \"$ROS_DOMAIN_ID\"; ros2 topic echo /backend_node/localization_state --once --field data\r"
        api('/api/terminal/write', {'id': terminal, 'text': command})
        deadline = time.monotonic()+12
        text = ''
        while time.monotonic() < deadline:
            result = api('/api/terminal?id='+terminal)
            text = base64.b64decode(result['data']).decode(errors='replace')
            if '__OPERATOR_DOMAIN_98__' in text and 'status=NORMAL' in text:
                break
            time.sleep(.2)
    finally:
        api('/api/terminal/close', {'id': terminal})
    after = api('/api/status')
    owned = lambda s: {c: p.get('pid') for c, p in s['components'].items() if p['running']}
    report = {'unauthenticated_denied': denied, 'terminal_environment_ok': '__OPERATOR_DOMAIN_98__' in text,
              'real_ros_status_received': 'status=NORMAL' in text,
              'component_pids_preserved': owned(before) == owned(after), 'components': owned(after),
              'physical_output': after['physical_output'],
              'map_save_or_mode_switch_executed': False}
    report['pass'] = all(report[k] for k in ('unauthenticated_denied', 'terminal_environment_ok',
                                           'real_ros_status_received', 'component_pids_preserved'))
    args.output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))
    if not report['pass']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
