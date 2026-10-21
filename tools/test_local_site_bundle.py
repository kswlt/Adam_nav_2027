"""TEST_ONLY local-site publication and explicit startup-profile regressions."""
import importlib.util
import json
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).parent))
from PIL import Image
import test_publish_map_bundle as publisher_fixture

spec=importlib.util.spec_from_file_location('local_bundle_contract',
    Path(__file__).parents[1]/'src/rm_nav_bringup/launch/map_bundle_contract.py')
contract=importlib.util.module_from_spec(spec)
spec.loader.exec_module(contract)


class LocalSiteTest(unittest.TestCase):
    def setUp(self):
        self.fixture=publisher_fixture.PublishBundleTest()
        self.fixture.setUp()
        self.align={'status':'operator_defined','source_frame':'mapping_odom','target_frame':'map',
                    'local_source':'TEST_ONLY_ROOM','coordinate_definition':'session_mapping_odom',
                    'translation':[0,0,0],'yaw_rad':0,'scale':1}
        self.metadata={'status':'operator_reviewed','coordinate_frame':'map','local_source':'TEST_ONLY_ROOM',
                       'resolution':.05,'origin':[0,0,0],'reviewer':'TEST_ONLY_FIXTURE',
                       'review_scope':'free_space_and_obstacles_in_local_map'}
        self.mask=self.fixture.root/'local_mask.pgm'
        self.review=self.fixture.root/'local_review.json'
        Image.new('L',(21,21),255).save(self.mask)

    def tearDown(self):
        self.fixture.tearDown()

    def publish(self, reviewed=True):
        self.fixture.align.write_text(json.dumps(self.align))
        self.review.write_text(json.dumps(self.metadata))
        options=['--profile','local_site']
        if reviewed:
            options+=['--free-space-mask',str(self.mask),'--free-space-metadata',str(self.review)]
        self.fixture.run_publish(*options)
        return self.fixture.output/'map_bundle.json'

    def test_local_site_never_claims_official_alignment(self):
        filename=self.publish()
        bundle,_=contract.load_published_bundle(filename,navigation=True,expected_profile='local_site')
        self.assertFalse(bundle['official_alignment_applied'])
        self.assertTrue(bundle['local_alignment_applied'])
        self.assertTrue(bundle['navigation_ready'])
        self.assertNotIn('official_source',bundle)
        with self.assertRaisesRegex(ValueError,'profile'):
            contract.load_published_bundle(filename,navigation=True)

    def test_localization_only_without_free_space(self):
        filename=self.publish(reviewed=False)
        bundle,_=contract.load_published_bundle(filename,expected_profile='local_site')
        self.assertFalse(bundle['navigation_ready'])
        with self.assertRaises(ValueError):
            contract.load_published_bundle(filename,navigation=True,expected_profile='local_site')

    def test_unreviewed_or_mismatched_masks_refused(self):
        for field,value in [('status','candidate'),('reviewer',''),('local_source','OTHER_ROOM'),
                             ('review_scope','obstacle_endpoints_only')]:
            with self.subTest(field=field):
                old=self.metadata[field]
                self.metadata[field]=value
                with self.assertRaises(ValueError):
                    self.publish()
                self.fixture.assert_no_publication()
                self.metadata[field]=old

    def test_session_coordinates_cannot_hide_arbitrary_alignment(self):
        self.align['translation']=[1,0,0]
        with self.assertRaises(ValueError):
            self.publish()
        self.fixture.assert_no_publication()

    def test_official_default_does_not_accept_local_alignment(self):
        self.fixture.align.write_text(json.dumps(self.align))
        with self.assertRaises(ValueError):
            self.fixture.run_publish()
        self.fixture.assert_no_publication()

    def test_missing_review_cannot_be_replaced_by_manifest_boolean(self):
        filename=self.publish(reviewed=False)
        data=json.loads(filename.read_text())
        data.update(navigation_ready=True,free_space_evidence='operator_reviewed_local_mask',
                    free_space_local_source='TEST_ONLY_ROOM')
        filename.write_text(json.dumps(data))
        with self.assertRaises(ValueError):
            contract.load_published_bundle(filename,navigation=True,expected_profile='local_site')


if __name__=='__main__':
    unittest.main(verbosity=2)
