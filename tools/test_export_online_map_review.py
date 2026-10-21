import math
import unittest
from export_online_map_review import review_grid


class ReviewGridTests(unittest.TestCase):
    def test_unknown_is_never_invented_free_and_rows_flip(self):
        w,h,x,y,pixels,terrain = review_grid([(0,0,1),(0,1,-.3)], .5, .1, 2)
        self.assertEqual((w,h,x,y), (1,3,0,0))
        self.assertEqual(pixels, bytes([205,205,0]))
        self.assertNotIn(255, pixels)
        self.assertEqual(terrain['0,2'], -.3)

    def test_minimum_z_is_observation_not_traversability(self):
        result = review_grid([(-1,-1,1),(-1,-1,-.25)], .1, .1, 2)
        self.assertEqual(result[4], bytes([0]))
        self.assertEqual(result[5]['0,0'], -.25)

    def test_invalid_or_unbounded_input_refused(self):
        for points,res,lo,hi in [([], .1,0,1), ([(math.nan,0,0)],.1,0,1),
                                  ([(0,0,0)],0,0,1), ([(0,0,0)],.1,1,0),
                                  ([(0,0,0),(1000,1000,0)], .01,0,1)]:
            with self.assertRaises(ValueError):
                review_grid(points,res,lo,hi)


if __name__ == '__main__':
    unittest.main()
