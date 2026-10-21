#!/usr/bin/env python3
"""TEST_ONLY session ownership/idempotency; launches sleep, never ROS or serial."""
import contextlib
import importlib.util
import io
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

spec = importlib.util.spec_from_file_location('vehicle_session', Path(__file__).with_name('vehicle_session.py'))
session = importlib.util.module_from_spec(spec)
spec.loader.exec_module(session)


class SessionTest(unittest.TestCase):
    def setUp(self):
        self.folder = tempfile.TemporaryDirectory()
        self.root = Path(self.folder.name)
        self.runner = self.root/'runner.bash'
        self.runner.write_text('exec sleep 60\n')

    def tearDown(self):
        self.folder.cleanup()

    def invoke(self, action):
        with patch.object(session, 'RUNTIME', self.root), patch.object(session, 'RUNNER', self.runner), \
             patch.object(sys, 'argv', ['vehicle_session.py', action, 'driver']), contextlib.redirect_stdout(io.StringIO()):
            session.main()

    def test_does_not_claim_unmarked_process(self):
        env = dict(os.environ)
        env.pop('NAV_MANAGED_SESSION', None)
        child = subprocess.Popen(['sleep','60'], env=env, start_new_session=True)
        try:
            self.assertFalse(session.owned(child.pid))
            (self.root/'driver.process.json').write_text(json.dumps({'pid':child.pid}))
            self.invoke('stop')
            self.assertIsNone(child.poll(), 'A foreign process was signalled')
        finally:
            child.terminate()
            child.wait()

    def test_start_is_idempotent_and_stop_is_owned(self):
        self.invoke('start')
        pid = json.loads((self.root/'driver.process.json').read_text())['pid']
        try:
            self.assertTrue(session.owned(pid))
            self.invoke('start')
            self.assertEqual(json.loads((self.root/'driver.process.json').read_text())['pid'], pid)
            self.invoke('stop')
            os.waitpid(pid, 0)
            self.assertFalse(session.owned(pid))
        finally:
            if session.owned(pid):
                os.killpg(pid, 15)
                os.waitpid(pid, 0)

    def test_stop_then_start_actually_creates_new_process(self):
        self.invoke('start')
        first = json.loads((self.root/'driver.process.json').read_text())['pid']
        self.invoke('stop')
        os.waitpid(first,0)
        self.invoke('start')
        second = json.loads((self.root/'driver.process.json').read_text())['pid']
        try:
            self.assertNotEqual(first,second)
            self.assertTrue(session.owned(second))
        finally:
            self.invoke('stop')
            os.waitpid(second,0)


if __name__ == '__main__':
    unittest.main()
