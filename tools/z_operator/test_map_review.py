import importlib.util
from pathlib import Path
import tempfile
import unittest

spec = importlib.util.spec_from_file_location('map_review',Path(__file__).with_name('map_review.py'))
review = importlib.util.module_from_spec(spec)
spec.loader.exec_module(review)
from publish_map_bundle import write_pcd


class MapReviewTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.native = Path(self.directory.name)/'native'
        (self.native/'clouds').mkdir(parents=True)
        (self.native/'keyframes.txt').write_text('# index timestamp tx ty tz qx qy qz qw\n0 1 0 0 0 0 0 0 1\n')
        write_pcd(self.native/'map.pcd',[(-1,-1,-.2),(0,0,.5),(1,1,2)],'map')

    def tearDown(self):
        self.directory.cleanup()

    def test_projection_has_obstacles_and_unknown_only(self):
        data=review.prepare(self.native,resolution=.1)
        self.assertEqual(set(data['_cells']),{0,205})
        self.assertEqual(data['free_cells'],0)
        self.assertEqual(data['keyframes'],1)
        self.assertEqual(data['point_count'],3)

    def test_polygon_keeps_obstacles_and_unknown_outside(self):
        data={'width':10,'height':10,'_cells':bytearray([205])*100}
        data['_cells'][33]=0
        image=review.reviewed_mask(data,[[[1,1],[5,1],[5,5],[1,5]]])
        self.assertEqual(image.getpixel((3,3)),0)
        self.assertEqual(image.getpixel((2,2)),255)
        self.assertEqual(image.getpixel((8,8)),205)

    def test_invalid_polygons(self):
        polygons=[[[0,0],[5,5],[0,5],[5,0]],[[0,0],[1,1],[2,2]],
                  [[0,0],[0,0],[2,2]],[[0,0],[100,0],[0,5]],[[0,0],[float('nan'),2],[0,5]]]
        for polygon in polygons:
            with self.subTest(polygon=polygon), self.assertRaises(ValueError):
                review.polygon_valid(polygon,10,10)

    def test_polygon_touching_non_adjacent_edge_rejected(self):
        with self.assertRaises(ValueError):
            review.polygon_valid([[0,0],[5,0],[5,5],[2,0],[0,5]],10,10)

    def test_public_preview_never_sends_private_buffer(self):
        data=review.prepare(self.native,.1)
        result=review.public(data)
        self.assertNotIn('_cells',result)
        self.assertIsInstance(result['cells'],str)

    def test_publish_requires_human_confirmation(self):
        for name,confirmed in [('',True),('operator',False),('operator','true')]:
            with self.assertRaises(ValueError):
                review.publish({},[],name,confirmed)

    def test_changed_native_map_rejected(self):
        data=review.prepare(self.native,.1)
        with (self.native/'map.pcd').open('ab') as file:
            file.write(b'changed')
        with self.assertRaises(ValueError):
            review.publish(data,[[[0,0],[3,0],[3,3]]],'test',True)


if __name__=='__main__':
    unittest.main()
