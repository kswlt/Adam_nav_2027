"""Regression of trust boundaries and vehicle-specific upstream configuration."""
import importlib.util
from pathlib import Path
import sys
import tempfile
import unittest

ROOT = Path(__file__).parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
from z_localization_policy import parse_state, trusted_for_navigation, rotation
spec = importlib.util.spec_from_file_location('zconfig', ROOT/'launch/z_localization_config.py')
cfg = importlib.util.module_from_spec(spec); spec.loader.exec_module(cfg)


class PolicyTest(unittest.TestCase):
    def test_trust_requires_real_localization_fresh_tf_and_current_epoch(self):
        state = parse_state('status=NORMAL;pose_trusted=true;tf_frozen=false;tf_published=true;epoch=2;stamp_ns=1000000000')
        self.assertTrue(trusted_for_navigation(state, 'localization', 2, 1100000000, .25))
        for mode, epoch, now in [('mapping', 2, 1100000000), ('localization', 1, 1100000000),
                                  ('localization', 2, 1300000000), ('localization', 2, 800000000)]:
            self.assertFalse(trusted_for_navigation(state, mode, epoch, now, .25))
        for key, value in [('status', 'OBS_DEGRADED'), ('status', 'LOST'), ('pose_trusted', 'false'),
                           ('tf_frozen', 'true'), ('tf_published', 'false'), ('stamp_ns', 0)]:
            changed = dict(state); changed[key] = value
            self.assertFalse(trusted_for_navigation(changed, 'localization', 2, 1100000000, .25))

    def test_bad_status_and_quaternion_rejected(self):
        for text in ['', 'status=NORMAL', 'status=A;status=B', 'invalid']:
            with self.assertRaises(ValueError): parse_state(text)
        with self.assertRaises(ValueError): rotation((0, 0, 0, 0))
        self.assertAlmostEqual(float(rotation((0, 0, 0, 1))[0, 0]), 1.)

    def test_old_rm_map_is_not_a_native_z_map(self):
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaises(ValueError): cfg.backend_config({}, 'localization', directory, directory)

    def test_vehicle_override_removes_author_site_assumptions(self):
        defaults = {key: {} for key in ('common','prior_map','uwb','startup_hint','trajectory','visualization','recovery','loop')}
        defaults['common'] = {'body': {'offset': {'x': 100.}}, 'scan_context': {'lidar_height': .35}}
        defaults['output'] = {'legacy': {'enabled': True}, 'world_cloud': {}}
        defaults['uwb']['enabled'] = True
        with tempfile.TemporaryDirectory() as directory:
            result = cfg.backend_config(defaults, 'mapping', '', str(Path(directory)/'new_map'))
            self.assertFalse(result['uwb']['enabled'])
            self.assertFalse(result['startup_hint']['enabled'])
            self.assertFalse(result['output']['legacy']['enabled'])
            self.assertEqual(result['common']['body']['frame'], 'front_mid360_imu')
            self.assertEqual(result['common']['body']['offset']['x'], 0.)
            self.assertEqual(result['common']['scan_context']['lidar_height'], 0.)
            self.assertTrue(defaults['uwb']['enabled'])
            with self.assertRaises(ValueError): cfg.backend_config(defaults, 'mapping', '', directory)


if __name__ == '__main__': unittest.main()
