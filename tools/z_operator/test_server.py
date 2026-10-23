"""POSIX tests: scope ownership, map gating, API auth and a real local PTY."""
import base64
import importlib.util
import json
import os
import signal
import subprocess
import sys
from pathlib import Path
import tempfile
import threading
import time
import unittest
from unittest.mock import patch
from urllib.error import HTTPError
from urllib.request import Request, urlopen
from http.server import ThreadingHTTPServer

spec = importlib.util.spec_from_file_location('operator_server', Path(__file__).with_name('server.py'))
server = importlib.util.module_from_spec(spec)
spec.loader.exec_module(server)


class OperatorTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.manager = server.Manager(self.root/'ws', self.root/'runtime')
        (self.manager.repo/'tools').mkdir(parents=True)
        (self.manager.repo/'tools/z_environment.bash').write_text('export ROS_DOMAIN_ID=98\n')

    def tearDown(self):
        self.temp.cleanup()

    def test_command_keeps_arguments_literal(self):
        command = self.manager.command('mapping', '/map with spaces/$(touch nope)')
        self.assertEqual(command[-1], '/map with spaces/$(touch nope)')
        self.assertIn('exec bash "$@"', command[2])
        self.assertNotIn('physical', command)

    def test_recycled_pid_not_owned(self):
        self.manager.procs['mapping'] = {'pid': os.getpid(), 'ticks': 'wrong'}
        with patch.object(server.os, 'killpg') as kill:
            self.manager.stop('mapping')
            kill.assert_not_called()

    def test_bad_map_name_and_duplicate_do_not_start(self):
        with patch.object(self.manager, 'start') as start:
            for name in ('../escape', 'bad name', '$(x)', ''):
                with self.assertRaises(ValueError):
                    self.manager.execute('start_mapping', {'name': name})
            (self.manager.ws/'maps/existing').mkdir(parents=True)
            with self.assertRaises(ValueError):
                self.manager.execute('start_mapping', {'name': 'existing'})
            start.assert_not_called()

    def test_draft_navigation_rejected_before_start(self):
        file = self.root/'map_bundle.json'
        file.write_text(json.dumps({'status': 'draft'}))
        with patch.object(self.manager, 'start') as start:
            with self.assertRaises(ValueError):
                self.manager.execute('navigation', {'manifest': str(file)})
            start.assert_not_called()

    def test_current_mapping_not_auto_stopped(self):
        with patch.object(self.manager, 'alive', return_value=True), patch.object(self.manager, 'stop') as stop:
            with self.assertRaises(ValueError):
                self.manager.execute('start_mapping', {'name': 'new'})
            stop.assert_not_called()

    def test_http_auth_and_unknown_actions(self):
        http = ThreadingHTTPServer(('127.0.0.1', 0), server.make_handler(self.manager, 'secret', {}))
        thread = threading.Thread(target=http.serve_forever, daemon=True)
        thread.start()
        url = f'http://127.0.0.1:{http.server_port}'
        try:
            with self.assertRaises(HTTPError) as error:
                urlopen(url+'/api/status')
            self.assertEqual(error.exception.code, 401)
            request = Request(url+'/api/status', headers={'Authorization': 'Bearer secret'})
            with urlopen(request) as response:
                self.assertFalse(json.load(response)['physical_output'])
            self.manager.submit('not_an_action', {})
            deadline = time.monotonic()+2
            while self.manager.job['busy'] and time.monotonic() < deadline:
                time.sleep(.01)
            self.assertFalse(self.manager.job['ok'])
        finally:
            http.shutdown()
            http.server_close()

    def test_real_pty_has_loaded_environment(self):
        terminal = server.Terminal(self.manager)
        try:
            terminal.write("printf 'PTY_DOMAIN_%s\\n' \"$ROS_DOMAIN_ID\"\r")
            deadline = time.monotonic()+3
            output = b''
            while time.monotonic() < deadline:
                output = base64.b64decode(terminal.read(0)['data'])
                if b'PTY_DOMAIN_98' in output:
                    break
                time.sleep(.05)
            self.assertIn(b'PTY_DOMAIN_98', output)
            terminal.write('sleep 30\r')
            time.sleep(.1)
            terminal.write('\x03')
            terminal.write("echo PTY_AFTER_INTERRUPT\r")
            time.sleep(.2)
            self.assertIn(b'PTY_AFTER_INTERRUPT', base64.b64decode(terminal.read(0)['data']))
        finally:
            terminal.close()
            terminal.proc.wait(timeout=3)

    def test_console_sigterm_exits_without_leaving_listener(self):
        env = dict(os.environ, ROS_DOMAIN_ID='224', ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
        proc = subprocess.Popen([sys.executable, str(Path(__file__).with_name('server.py')),
                                 '--workspace', str(self.manager.ws), '--runtime', str(self.manager.runtime),
                                 '--port', '0'], stdout=subprocess.PIPE, stderr=subprocess.PIPE, env=env)
        try:
            line = proc.stdout.readline()
            self.assertTrue(line)
            proc.send_signal(signal.SIGTERM)
            self.assertEqual(proc.wait(timeout=5), 0)
        finally:
            if proc.poll() is None:
                proc.kill()
                proc.wait()
            proc.stdout.close()
            proc.stderr.close()


if __name__ == '__main__':
    unittest.main()
