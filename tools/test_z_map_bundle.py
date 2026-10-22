"""TEST_ONLY native Z draft/publication compatibility and tamper rejection."""
import importlib.util
import json
from pathlib import Path
import shutil
import struct
import sys
import unittest
from unittest.mock import patch
sys.path.insert(0, str(Path(__file__).parent))
import import_z_map
from test_local_site_bundle import LocalSiteTest

spec = importlib.util.spec_from_file_location('zconfig', Path(__file__).parents[1]/'src/rm_nav_bringup/launch/z_localization_config.py')
cfg = importlib.util.module_from_spec(spec); spec.loader.exec_module(cfg)


class ZMapTest(unittest.TestCase):
    def setUp(self):
        self.site = LocalSiteTest(); self.site.setUp()
        self.native = self.site.fixture.root/'native'
        (self.native/'clouds').mkdir(parents=True)
        shutil.copy2(self.site.fixture.pcd, self.native/'map.pcd')
        shutil.copy2(self.site.fixture.pcd, self.native/'clouds/000000.pcd')
        (self.native/'keyframes.txt').write_text('# backend_map 1\n0 1.0 0 0 0 0 0 0 1\n')
        self.draft = self.site.fixture.root/'z_draft'
        argv = ['import_z_map.py', '--native-bundle', str(self.native), '--output', str(self.draft), '--calibration-id', 'TEST_ONLY_z']
        with patch.object(sys, 'argv', argv), patch('builtins.print'): import_z_map.main()
        self.site.fixture.draft = self.draft/'map_bundle.json'

    def tearDown(self): self.site.tearDown()

    def publish(self):
        fixture = self.site.fixture
        fixture.align.write_text(json.dumps(self.site.align)); self.site.review.write_text(json.dumps(self.site.metadata))
        fixture.run_publish('--profile', 'local_site', '--z-native-bundle', str(self.native),
                            '--free-space-mask', str(self.site.mask), '--free-space-metadata', str(self.site.review))
        manifest = fixture.output/'map_bundle.json'
        return manifest, json.loads(manifest.read_text())

    def test_draft_is_not_navigation_and_native_prior_is_preserved(self):
        draft = json.loads(self.site.fixture.draft.read_text())
        self.assertFalse(draft['publishable_for_frozen_localization'])
        manifest, bundle = self.publish()
        self.assertTrue(bundle['navigation_ready'])
        self.assertFalse(bundle['official_alignment_applied'])
        native = cfg.native_bundle(manifest, bundle)
        self.assertTrue((native/'clouds/000000.pcd').is_file())
        (native/'keyframes.txt').write_text('changed index')
        with self.assertRaises(ValueError): cfg.native_bundle(manifest, bundle)

    def test_mismatched_snapshot_rejected_without_partial_publication(self):
        (self.native/'map.pcd').write_bytes(b'corrupt')
        with self.assertRaises(ValueError): self.publish()
        self.site.fixture.assert_no_publication()

    def test_old_map_and_escaping_reference_rejected(self):
        manifest, bundle = self.publish()
        with self.assertRaises(ValueError): cfg.native_bundle(manifest, {})
        bundle['z_localization']['native_bundle'] = '../outside'
        with self.assertRaises(ValueError): cfg.native_bundle(manifest, bundle)

    def test_actual_upstream_xyzi_format_and_invalid_payload(self):
        path=self.site.fixture.root/'xyzi.pcd'
        header=b'VERSION .7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nWIDTH 1\nHEIGHT 1\nPOINTS 1\nDATA binary\n'
        path.write_bytes(header+struct.pack('<ffff',1.,2.,3.,42.))
        self.assertEqual(import_z_map.read_native_map(path),[(1.,2.,3.)])
        path.write_bytes(header+b'broken')
        with self.assertRaises(ValueError): import_z_map.read_native_map(path)

    def test_publication_inside_native_cannot_recursively_copy_itself(self):
        self.site.fixture.output=self.native/'published'
        with self.assertRaises(ValueError): self.publish()
        self.assertFalse(self.site.fixture.output.exists())
        self.assertEqual(list(self.native.glob('published.partial-*')),[])


if __name__ == '__main__': unittest.main()
