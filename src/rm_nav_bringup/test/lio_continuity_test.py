import math
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).parents[1]/'scripts'))
from lio_continuity import InputContinuity, PoseContinuity


class InputTest(unittest.TestCase):
    def arm(self):
        guard = InputContinuity()
        self.assertFalse(guard.observe('imu',100,100,1))
        self.assertTrue(guard.observe('cloud',100,100,1))
        return guard

    def test_waits_for_both_without_buffering(self):
        g = InputContinuity()
        for n in range(2000):
            self.assertFalse(g.observe('imu',100+n*.005,100+n*.005,1+n*.005))
        self.assertEqual(len(g.seen),1)
        self.assertFalse(g.tick(20))

    def test_gap_is_latched_and_no_replay(self):
        for stream in ('imu','cloud'):
            g = self.arm()
            self.assertFalse(g.observe(stream,101,101,2))
            self.assertTrue(g.fault)
            self.assertFalse(g.observe('imu',101.01,101.01,2.01))
            self.assertFalse(g.observe('cloud',101.01,101.01,2.01))

    def test_single_sensor_loss_detected_even_with_other_active(self):
        g = self.arm()
        for i in range(1,8):
            g.observe('imu',100+i*.1,100+i*.1,1+i*.1)
        self.assertTrue(g.fault)

    def test_asynchronous_scan_start_and_imu_do_not_false_latch(self):
        g = self.arm()
        for i in range(1,35):
            stamp = 100+i*.01
            self.assertTrue(g.observe('imu',stamp,stamp,1+i*.01))
        self.assertTrue(g.observe('cloud',100.2,100.34,1.34))
        self.assertFalse(g.fault)

    def test_valid_original_streams(self):
        g = self.arm()
        for i in range(1,200):
            stamp = 100+i*.005
            self.assertTrue(g.observe('imu',stamp,stamp,1+i*.005))
            if i % 20 == 0:
                self.assertTrue(g.observe('cloud',stamp,stamp,1+i*.005))
        self.assertFalse(g.fault)

    def test_bad_time_refused(self):
        for stamp,now in [(math.nan,100),(0,0)]:
            g = InputContinuity()
            self.assertFalse(g.observe('imu',stamp,now,1))
            self.assertTrue(g.fault)
        g = self.arm()
        self.assertFalse(g.observe('imu',99.9,100,1.01))

    def test_startup_backlog_cannot_initialize_and_does_not_latch(self):
        g = InputContinuity()
        self.assertFalse(g.observe('imu',100,103,1))
        self.assertFalse(g.observe('cloud',100,103,1))
        self.assertFalse(g.fault)
        self.assertFalse(g.observe('imu',103,103,2))
        self.assertTrue(g.observe('cloud',103,103,2))
        self.assertFalse(g.observe('imu',103.01,104,2.01))
        self.assertTrue(g.fault)


class PoseTest(unittest.TestCase):
    def test_observed_kilometre_fault_cannot_recover_on_next_frame(self):
        g = PoseContinuity()
        g.check(1000000000,(.003,.003,0),(0,0,0,1))
        with self.assertRaises(ValueError):
            g.check(1100000000,(-4626.836,-1126.821,0),(0,0,0,1))
        with self.assertRaises(ValueError):
            g.check(1200000000,(.003,.003,0),(0,0,0,1))

    def test_long_gap_not_bridged(self):
        g = PoseContinuity()
        g.check(1000000000,(0,0,0),(0,0,0,1))
        with self.assertRaisesRegex(ValueError,'gap'):
            g.check(4775400000000,(0,0,0),(0,0,0,1))

    def test_realistic_rotation_and_sign_equivalent_quaternions(self):
        g = PoseContinuity()
        for n in range(100):
            yaw = n*.01
            q = (0,0,math.sin(yaw/2),math.cos(yaw/2))
            if n % 2:
                q = tuple(-v for v in q)
            g.check(1000000000+n*10000000,(n*.005,0,0),q)
        self.assertFalse(g.fault)

    def test_impossible_orientation_jump(self):
        g = PoseContinuity()
        g.check(1000000000,(0,0,0),(0,0,0,1))
        with self.assertRaisesRegex(ValueError,'motion'):
            g.check(1010000000,(0,0,0),(0,0,1,0))


if __name__ == '__main__':
    unittest.main()
