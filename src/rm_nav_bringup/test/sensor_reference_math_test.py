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
