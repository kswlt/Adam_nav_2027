import copy
import math
import unittest
import numpy as np
from fit_official_alignment import fit

class AlignmentTest(unittest.TestCase):
    def setUp(self):
        points=np.array([[0.,0.],[3.,0.],[0.,2.],[2.,2.]])
        c,s=math.cos(.7),math.sin(.7);rotation=np.array([[c,-s],[s,c]])
        self.data={'status':'verified','source_frame':'mapping_odom','target_frame':'map','official_source':'TEST_ONLY',
            'height_offset_m':.1,'correspondences':[{'mapping_odom':a.tolist(),'map':b.tolist()} for a,b in zip(points,points@rotation.T+[20.,-10.])]}
    def test_rigid_fit(self):
        result=fit(self.data)
        self.assertTrue(np.allclose(result['translation'],[20.,-10.,.1]))
        self.assertAlmostEqual(result['yaw_rad'],.7);self.assertEqual(result['scale'],1.)
    def test_scale_rejected(self):
        for pair in self.data['correspondences']:pair['map']=[v*1.2 for v in pair['map']]
        with self.assertRaises(ValueError):fit(self.data)
    def test_degenerate_and_unverified(self):
        data=copy.deepcopy(self.data);data['status']='draft'
        with self.assertRaises(ValueError):fit(data)
        for pair in self.data['correspondences']:pair['mapping_odom']=[0.,0.]
        with self.assertRaises(ValueError):fit(self.data)
    def test_reflection_rejected(self):
        for pair in self.data['correspondences']:pair['map'][0]*=-1
        with self.assertRaises(ValueError):fit(self.data)
if __name__=='__main__':unittest.main(verbosity=2)
