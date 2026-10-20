#!/usr/bin/env python3
"""Startup guard tests: construct launch actions without executing any node."""
import importlib.util
from pathlib import Path
import tempfile
import unittest
from launch import LaunchContext

spec = importlib.util.spec_from_file_location('transport', Path(__file__).parents[1]/'launch/legacy_adam_transport.launch.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class TransportTest(unittest.TestCase):
    def actions(self, mode='disabled', port='', enabled='false'):
        context = LaunchContext()
        context.launch_configurations.update(transport=mode, serial_port=port, enable_physical_output=enabled)
        return module.nodes(context)

    def test_disabled_has_no_serial_node(self):
        self.assertEqual(len(self.actions()), 1)

    def test_reject_unknown_mode(self):
        with self.assertRaises(ValueError):
            self.actions('invented')

    def test_pty_cannot_open_non_pty_device(self):
        with self.assertRaises(ValueError):
            self.actions('pty', '/dev/null')

    def test_physical_requires_explicit_enable(self):
        with self.assertRaises(ValueError):
            self.actions('physical', '/dev/null')

    def test_file_is_not_a_serial_device(self):
        with tempfile.NamedTemporaryFile() as handle:
            with self.assertRaises(ValueError):
                self.actions('pty', handle.name)


if __name__ == '__main__':
    unittest.main()
