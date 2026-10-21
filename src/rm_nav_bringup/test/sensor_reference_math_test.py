#!/usr/bin/env python3
import importlib.util
import math
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location('geometry', Path(__file__).parents[1]/'scripts/sensor_reference_math.py')
geometry = importlib.util.module_from_spec(spec)
spec.loader.exec_module(geometry)


class GeometryTest(unittest.TestCase):
    def test_measured_axis_compensation_preserves_stationary_pivot(self):
        factory = (-.011,-.02329,.04412)
        # Synthetic measured lever, never the recorded/production installation.
        lever = (.03,.18,.08)
        offset = geometry.reference_offset(factory,'gimbal_axis',lever,True,'TEST_ONLY_axis')
        for angle in (0.,1.,2.,-3.,-1.):
            c,s = math.cos(angle), math.sin(angle)
            imu = (2-c*offset[0]+s*offset[1],3-s*offset[0]-c*offset[1],.2-offset[2])
            point,_ = geometry.sensor_pose(imu,(0,0,math.sin(angle/2),math.cos(angle/2)),offset)
            for a,b in zip(point,(2.,3.,.2)):self.assertAlmostEqual(a,b)

    def test_geometry_requires_explicit_measured_mode_and_new_id(self):
        factory = (-.011,-.02329,.04412)
        for mode,verified,identity in [('gimbal_axis',False,'measured_v2'),
                                       ('gimbal_axis',True,''),
                                       ('gimbal_axis',True,'vehicle164_sensor_axis_factory_v1'),
                                       ('lidar_center',True,'measured_v2')]:
            with self.assertRaises(ValueError):
                geometry.reference_offset(factory,mode,(.1,0.,0.),verified,identity)
        self.assertEqual(geometry.reference_offset(factory,'lidar_center',(0,0,0),False,''),factory)

    def test_heading_latency_correction_across_wrap(self):
        tracker = geometry.HeadingTracker()
        tracker.observe(1_000_000_000,math.pi-.05)
        tracker.observe(1_100_000_000,-math.pi+.05)
        yaw,age,correction = tracker.predict(1_180_000_000)
        self.assertAlmostEqual(age,.08)
        self.assertAlmostEqual(correction,.08)
        vx,vy=geometry.sensor_velocity(.3,-.1,yaw)
        actual=math.pi+.13
        self.assertAlmostEqual(math.cos(actual)*vx-math.sin(actual)*vy,.3)
        self.assertAlmostEqual(math.sin(actual)*vx+math.cos(actual)*vy,-.1)

    def test_heading_expiry_and_angle_budget(self):
        tracker=geometry.HeadingTracker()
        tracker.observe(1_000_000_000,0.)
        with self.assertRaises(ValueError):tracker.predict(1_010_000_000)
        tracker.observe(1_100_000_000,.4)
        for stamp in (1_090_000_000,1_180_000_000,1_230_000_000):
            with self.assertRaises(ValueError):tracker.predict(stamp)
        with self.assertRaises(ValueError):tracker.observe(1_100_000_000,.4)
        with self.assertRaises(ValueError):tracker.predict(1_110_000_000)

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
