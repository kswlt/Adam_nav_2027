"""Regression for the user's corrected mechanical meaning, not survey accuracy."""
from pathlib import Path
import unittest
import math
import yaml


class VehicleGeometryTest(unittest.TestCase):
    def test_left_16cm_is_gimbal_to_lidar(self):
        root=Path(__file__).parents[1]
        mount=yaml.safe_load((root/'config/vehicle_164_geometry.yaml').read_text())['mount']
        self.assertEqual(mount['gimbal_axis_to_lidar_xy'],[0.,.16])
        self.assertAlmostEqual(math.hypot(*mount['axis_to_lidar_at_reference_height'][:2]),.1534362568228761)
        self.assertFalse(mount['reference_geometry_verified'])
        self.assertTrue(mount['reference_fit_debug_approved'])
        self.assertIsNone(mount['yaw_axis_offset_from_vehicle_center_xy'])
        self.assertFalse(mount['lidar_coaxial_with_yaw'])
        self.assertTrue(mount['horizontal_geometry_user_confirmed'])

    def test_full_chassis_bundle_stays_draft(self):
        root=Path(__file__).parents[2]
        bundle=yaml.safe_load((root/'rm_nav_frames/config/vehicle_164_yaw_mid360.draft.yaml').read_text())
        self.assertEqual(bundle['status'],'draft')
        self.assertFalse(bundle['field_calibration_verified'])
        self.assertIsNone(bundle['transforms']['chassis_to_yaw_zero']['translation'])
        a=bundle['transforms']['yaw_to_imu']['translation']
        b=bundle['transforms']['imu_to_lidar']['translation']
        for actual,expected in zip((x+y for x,y in zip(a,b)),(.015293518836158204,.15267217555738066,.05)):
            self.assertAlmostEqual(actual,expected)


if __name__=='__main__':unittest.main()
