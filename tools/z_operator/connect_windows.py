#!/usr/bin/env python3
"""One-click Windows SSH tunnel + console. Password is never saved to a file."""
import getpass
import json
from pathlib import Path
import subprocess
import tempfile
import time
from urllib.request import Request, urlopen
import webbrowser

WORKSPACE = '/home/asus/nav_2027_vehicle_config_164/z_migration_20261023'
RUNTIME = WORKSPACE+'/runtime_live_20261023'
HOST_KEY = 'ssh-ed25519 255 SHA256:VMfripMIt2DwA9GBQaJxq5Mt+h+ZjtshsDHgqmzJRIU'


def main():
    plink = Path('C:/Program Files/PuTTY/plink.exe')
    if not plink.exists():
        raise RuntimeError('需要现有PuTTY：C:/Program Files/PuTTY/plink.exe')
    password = getpass.getpass('ASUS password (not saved): ')
    base = [str(plink), '-P', '2233', '-batch', '-pw', password, '-hostkey', HOST_KEY]
    command = ('nohup bash '+WORKSPACE+'/src/Adam_nav_2027/tools/z_operator/start_server.bash > '+
               RUNTIME+'/operator_launcher.log 2>&1 < /dev/null &')
    result = subprocess.run([*base, 'asus@192.168.1.164', command], capture_output=True, text=True)
    if result.returncode:
        raise RuntimeError('ASUS connection failed: '+result.stderr)
    result = subprocess.run([*base, 'asus@192.168.1.164', 'cat '+RUNTIME+'/operator_token'],
                            capture_output=True, text=True)
    if result.returncode:
        raise RuntimeError('Cannot retrieve console token: '+result.stderr)
    token = result.stdout.strip()
    def ready():
        try:
            req = Request('http://127.0.0.1:8878/api/status', headers={'Authorization': 'Bearer '+token})
            with urlopen(req, timeout=2) as response:
                return json.load(response)['domain'] == '98'
        except Exception:
            return False
    if not ready():
        error_path = Path(tempfile.gettempdir())/'z_operator_tunnel_error.log'
        with error_path.open('w') as errors:
            subprocess.Popen([*base, '-N', '-L', '127.0.0.1:8878:127.0.0.1:8878', 'asus@192.168.1.164'],
                             stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=errors,
                             creationflags=subprocess.CREATE_NO_WINDOW)
    deadline = time.monotonic()+12
    while not ready():
        if time.monotonic() > deadline:
            raise RuntimeError('Console tunnel not ready; inspect ASUS operator log / local tunnel error log')
        time.sleep(.3)
    # The token is local-only, never added to a shared Foxglove layout.
    token_file = Path(tempfile.gettempdir())/'z_operator_token'
    token_file.write_text(token)
    installer = Path(__file__).with_name('install_local_panel.py')
    import sys
    subprocess.run([sys.executable, str(installer), '--token-file', str(token_file)], check=False)
    webbrowser.open('http://127.0.0.1:8878/#'+token)
    print('ASUS console connected. Reload Foxglove to load the local panel.')


if __name__ == '__main__':
    try:
        main()
    except Exception as error:
        print(str(error))
        input('Press Enter to close...')
