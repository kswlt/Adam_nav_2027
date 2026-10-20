#!/usr/bin/env python3
"""Endpoint-only MapBundle publication regressions; all geometry is TEST_ONLY."""
import hashlib
import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

TOOLS = Path(__file__).resolve().parent
sys.path.insert(0, str(TOOLS))
import publish_map_bundle as publisher

class PublishBundleTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        self.pcd = self.root / 'source.pcd'
        publisher.write_pcd(self.pcd, [(0,0,0),(0,0,1),(1,1,0)], 'mapping_odom')
        self.draft = self.root / 'draft.json'
        self.align = self.root / 'alignment.yaml'
        self.align.write_text('status: verified\nsource_frame: mapping_odom\ntarget_frame: map\nofficial_source: TEST_ONLY\ntranslation: [0, 0, 0]\nyaw_rad: 0\nscale: 1\n')
        self.output = self.root / 'published'
        self.refresh_draft()

    def tearDown(self):
        self.tmp.cleanup()

    def refresh_draft(self):
        self.draft.write_text(json.dumps({'status':'draft_mapping_odom',
            'coordinate_frame':'mapping_odom','official_alignment_applied':False,
            'version':'test-only','artifacts':{'rebuilt_map.pcd':{
                'path':self.pcd.name,'sha256':publisher.digest(self.pcd)}}}))

    def run_publish(self, *options):
        argv = ['publish_map_bundle.py','--draft',str(self.draft),'--alignment',str(self.align),
                '--output',str(self.output), *options]
        with patch.object(sys, 'argv', argv), patch('builtins.print'):
            publisher.main()

    def assert_no_publication(self):
        self.assertFalse(self.output.exists())
        self.assertEqual(list(self.root.glob('published.partial-*')), [])

    def test_unknown_is_not_free_and_ground_is_separate(self):
        self.run_publish()
        manifest = json.loads((self.output / 'map_bundle.json').read_text())
        self.assertFalse(manifest['navigation_ready'])
        self.assertEqual(manifest['coordinate_frame'], 'map')
        payload = (self.output / 'maps/occupancy.pgm').read_bytes().split(b'255\n',1)[1]
        self.assertIn(205, payload)
        self.assertIn(0, payload)
        self.assertNotIn(255, payload)
        terrain = json.loads((self.output / 'maps/terrain_reference.json').read_text())
        self.assertEqual(terrain['cells']['0,0'], 0)
        self.assertFalse(terrain['traversability_verified'])
        map_yaml = (self.output / manifest['map_yaml']).read_text()
        self.assertIn('image: occupancy.pgm', map_yaml)
        for entry in manifest['artifacts'].values():
            self.assertEqual(publisher.digest(self.output / entry['path']), entry['sha256'])
        with self.assertRaises(FileExistsError):
            self.run_publish()

    def test_corrupt_pcd_rejected_even_with_updated_digest(self):
        self.pcd.write_bytes(self.pcd.read_bytes().replace(b'WIDTH 3', b'WIDTH 4'))
        self.refresh_draft()
        with self.assertRaises(ValueError): self.run_publish()
        self.assert_no_publication()

    def test_hash_mismatch(self):
        self.pcd.write_bytes(self.pcd.read_bytes() + b'broken')
        with self.assertRaises(ValueError): self.run_publish()
        self.assert_no_publication()

    def test_invalid_height_and_cell_budget(self):
        for options in [('--min-z','2','--max-z','1'), ('--min-z','nan'),
                        ('--resolution','0.000001')]:
            with self.assertRaises(ValueError): self.run_publish(*options)
            self.assert_no_publication()

    def test_unverified_alignment(self):
        self.align.write_text(self.align.read_text().replace('verified','unmeasured'))
        with self.assertRaises(ValueError): self.run_publish()
        self.assert_no_publication()

    def test_grid_change_changes_bundle_version(self):
        self.run_publish('--resolution','0.05')
        first=json.loads((self.output/'map_bundle.json').read_text())['version']
        self.output=self.root/'published_other_resolution'
        self.run_publish('--resolution','0.1')
        second=json.loads((self.output/'map_bundle.json').read_text())['version']
        self.assertNotEqual(first, second)

    def test_official_mask_enables_navigation_without_inventing_free_space(self):
        from PIL import Image
        mask=self.root/'official.pgm'; metadata=self.root/'official.yaml'
        image=Image.new('L',(21,21),255); image.putpixel((20,0),205);image.save(mask)
        metadata.write_text('status: verified\ncoordinate_frame: map\nofficial_source: TEST_ONLY\nresolution: 0.05\norigin: [0, 0, 0]\n')
        self.run_publish('--free-space-mask',str(mask),'--free-space-metadata',str(metadata))
        manifest=json.loads((self.output/'map_bundle.json').read_text())
        self.assertTrue(manifest['navigation_ready'])
        self.assertEqual(manifest['free_space_evidence'],'verified_official_mask')
        payload=(self.output/'maps/occupancy.pgm').read_bytes().split(b'255\n',1)[1]
        self.assertIn(255,payload);self.assertIn(205,payload);self.assertIn(0,payload)
        self.output=self.root/'reject_wrong_frame'
        metadata.write_text(metadata.read_text().replace('coordinate_frame: map','coordinate_frame: mapping_odom'))
        with self.assertRaises(ValueError): self.run_publish('--free-space-mask',str(mask),'--free-space-metadata',str(metadata))

    def test_staging_failure_rolls_back(self):
        with patch.object(publisher, 'write_pcd', side_effect=OSError('TEST_ONLY disk failure')):
            with self.assertRaises(OSError): self.run_publish()
        self.assert_no_publication()

    @unittest.skipIf(sys.platform == 'win32', 'Symlink regression requires Linux filesystem')
    def test_symlink_input_rejected(self):
        target = self.root / 'target.pcd'
        self.pcd.rename(target)
        self.pcd.symlink_to(target)
        with self.assertRaises(ValueError): self.run_publish()
        self.assert_no_publication()

if __name__ == '__main__':
    unittest.main(verbosity=2)
