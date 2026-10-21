#!/usr/bin/env python3
import importlib.util
import math
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location('geometry', Path(__file__).parents[1]/'scripts/sensor_reference_math.py')
geometry = importlib.util.module_from_spec(spec)
spec.loader.exec_module(geometry)


class GeometryTest(unittest.TestCase):
    def test_velocity_frame_rotation(self):
        for angle in (0, .3, math.pi/2, -math.pi, 2.7):
            x, y = geometry.sensor_velocity(.3, -.2, angle)
            self.assertAlmostEqual(math.cos(angle)*x-math.sin(angle)*y, .3)
            self.assertAlmostEqual(math.sin(angle)*x+math.cos(angle)*y, -.2)

    def test_factory_lever_arm(self):
        p, yaw = geometry.sensor_pose((1, 2, 3), (0, 0, math.sqrt(.5), math.sqrt(.5)), (-.011, -.02329, .04412))
        self.assertAlmostEqual(p[0], 1.02329)
        self.assertAlmostEqual(p[1], 1.989)
        self.assertAlmostEqual(yaw, math.pi/2)

    def test_coaxial_lidar_stays_fixed_when_imu_rotates_about_it(self):
        # Physical fixture: LiDAR centre on yaw axis; IMU describes its small
        # internal lever-arm circle. Navigation reference must remain fixed.
        offset=(-.011,-.02329,.04412)
        centre=(2.,3.,.25)
        for i in range(72):
            angle=i*math.pi/36
            c,s=math.cos(angle),math.sin(angle)
            rotated=(c*offset[0]-s*offset[1],s*offset[0]+c*offset[1],offset[2])
            imu=tuple(a-b for a,b in zip(centre,rotated))
            position,_=geometry.sensor_pose(imu,(0,0,math.sin(angle/2),math.cos(angle/2)),offset)
            for actual,expected in zip(position,centre):self.assertAlmostEqual(actual,expected)

    def test_eccentric_lidar_is_not_the_stationary_gimbal_axis(self):
        # Hypothetical 16cm yaw-to-LiDAR offset, NOT the confirmed chassis-to-yaw
        # installation. This demonstrates why the two offsets are not equivalent.
        offset=(-.011,-.02329,.04412)
        positions=[]
        for angle in (0.,math.pi):
            c,s=math.cos(angle),math.sin(angle)
            lidar=(.16*c,.16*s,.25)
            rotated=(c*offset[0]-s*offset[1],s*offset[0]+c*offset[1],offset[2])
            imu=tuple(a-b for a,b in zip(lidar,rotated))
            position,_=geometry.sensor_pose(imu,(0,0,math.sin(angle/2),math.cos(angle/2)),offset)
            positions.append(position)
        self.assertAlmostEqual(math.dist(*positions),.32)

    def test_fixed_world_direction_needs_changing_sensor_command(self):
        # +sensor-X becomes +world-Y at 90deg. A +world-X request must instead
        # become -sensor-Y, independently of chassis yaw/contact_angle.
        self.assertEqual(geometry.sensor_velocity(1.,0.,0.),(1.,0.))
        x,y=geometry.sensor_velocity(1.,0.,math.pi/2)
        self.assertAlmostEqual(x,0.)
        self.assertAlmostEqual(y,-1.)

    def test_spinning_envelope(self):
        polygon = geometry.circle_polygon()
        required = geometry.spinning_radius()
        self.assertLess(required, .54)
        for i, a in enumerate(polygon):
            b = polygon[(i+1) % len(polygon)]
            distance = abs(a[0]*b[1]-a[1]*b[0])/math.hypot(a[0]-b[0], a[1]-b[1])
            self.assertGreaterEqual(distance+1e-12, .54)
        for angle in range(360):
            for x, y in ((.25, .25), (.25, -.25), (-.25, .25), (-.25, -.25)):
                # Chassis corners relative to off-centre yaw axis, independently spinning.
                self.assertLessEqual(math.hypot(x, y-.16), required)

    def test_invalid_inputs(self):
        with self.assertRaises(ValueError):
            geometry.sensor_velocity(float('nan'), 0, 0)
        with self.assertRaises(ValueError):
            geometry.sensor_pose((0, 0, 0), (0, 0, 0, 0), (0, 0, 0))


if __name__ == '__main__':
    unittest.main()
