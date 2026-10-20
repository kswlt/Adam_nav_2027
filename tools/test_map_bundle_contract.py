#!/usr/bin/env python3
"""TEST_ONLY startup contract regression; no production ROS nodes are started."""
import importlib.util
import json
from pathlib import Path
import sys
import unittest
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parent))
import test_publish_map_bundle as fixture_module
import publish_map_bundle
location=Path(__file__).resolve().parents[1]/'src/rm_nav_bringup/launch/map_bundle_contract.py'
spec=importlib.util.spec_from_file_location('map_contract',location)
contract=importlib.util.module_from_spec(spec);spec.loader.exec_module(contract)

class StartupContractTest(unittest.TestCase):
    def setUp(self):
        self.fixture=fixture_module.PublishBundleTest();self.fixture.setUp()
    def tearDown(self): self.fixture.tearDown()
    def test_endpoint_only_is_localization_input_but_not_navigation_ready(self):
        f=self.fixture;f.run_publish();filename=f.output/'map_bundle.json'
        contract.load_published_bundle(filename)
        with self.assertRaises(ValueError): contract.load_published_bundle(filename,navigation=True)
        data=json.loads(filename.read_text());data['navigation_ready']=True;filename.write_text(json.dumps(data))
        with self.assertRaises(ValueError): contract.load_published_bundle(filename,navigation=True)
    def test_official_mask_full_navigation_contract(self):
        from PIL import Image
        f=self.fixture;mask=f.root/'official.pgm';metadata=f.root/'official.yaml'
        Image.new('L',(21,21),255).save(mask)
        metadata.write_text('status: verified\ncoordinate_frame: map\nofficial_source: TEST_ONLY\nresolution: 0.05\norigin: [0, 0, 0]\n')
        f.run_publish('--free-space-mask',str(mask),'--free-space-metadata',str(metadata))
        filename=f.output/'map_bundle.json'
        bundle,artifacts=contract.load_published_bundle(filename,navigation=True)
        self.assertIn(bundle['map_yaml'],artifacts)
        (f.output/bundle['map_yaml']).write_text('corrupt')
        with self.assertRaises(ValueError): contract.load_published_bundle(filename,navigation=True)
    def test_escaping_artifact(self):
        f=self.fixture;f.run_publish();filename=f.output/'map_bundle.json'
        data=json.loads(filename.read_text());next(iter(data['artifacts'].values()))['path']='../outside.pcd'
        filename.write_text(json.dumps(data))
        with self.assertRaises(ValueError): contract.load_published_bundle(filename)

if __name__=='__main__': unittest.main(verbosity=2)
