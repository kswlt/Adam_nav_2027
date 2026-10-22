import sys
from pathlib import Path
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).parents[1]/'scripts'))
from voxel_preview import GlobalVoxelPreview


class PreviewTest(unittest.TestCase):
    def test_earliest_region_retained_after_capacity_pressure(self):
        grid=GlobalVoxelPreview(.1,10)
        grid.add(np.array([[0,0,0],[.05,.05,0]]))
        for x in np.arange(1,21,.2):grid.add(np.array([[x,0,0]]))
        self.assertLessEqual(len(grid),10)
        self.assertGreater(grid.coarsenings,0)
        self.assertLess(grid.points()[:,0].min(),1.)
        self.assertGreater(grid.points()[:,0].max(),19.)

    def test_native_coordinates_and_negative_keys(self):
        grid=GlobalVoxelPreview(.1,100)
        points=np.array([[-1.03,2.12,3.44],[4.31,-5.44,6.78]],dtype=np.float32)
        grid.add(points)
        np.testing.assert_equal(grid.points(),points)
        grid.add(points)
        self.assertEqual(len(grid),2)

    def test_invalid_cloud_rejected_without_erasing_history(self):
        grid=GlobalVoxelPreview(.1,10)
        grid.add([[1,2,3]])
        for points in ([[float('nan'),0,0]],[[1,2]],[[10001,0,0]],[]):
            with self.assertRaises(ValueError):grid.add(points)
        np.testing.assert_equal(grid.points(),[[1,2,3]])


if __name__=='__main__':unittest.main()
