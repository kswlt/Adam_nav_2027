"""Bounded input and pose admission; never repairs timestamps or estimates poses."""
import math


class InputContinuity:
    def __init__(self, max_gap=.5, max_age=.25, max_skew=.25):
        if not all(math.isfinite(v) and v > 0 for v in (max_gap,max_age,max_skew)):
            raise ValueError('Invalid LIO input continuity limits')
        self.max_gap, self.max_age, self.max_skew = max_gap,max_age,max_skew
        self.seen, self.stamps = {}, {}
        self.armed = False
        self.fault = ''

    def fail(self, reason):
        self.fault = self.fault or reason+'; restart state and start a new mapping session'
        return False

    def tick(self, now):
        if self.armed and any(now-t > self.max_gap for t in self.seen.values()):
            self.fail('LIO input arrival gap')
        return self.armed and not self.fault

    def observe(self, stream, stamp, source_now, arrival_now):
        if self.fault:
            return False
        if stream not in ('imu','cloud'):
            raise ValueError('Unknown LIO stream')
        if not all(math.isfinite(v) for v in (stamp,source_now,arrival_now)) or stamp <= 0:
            return self.fail('Nonfinite/invalid LIO input timestamp')
        if not -.05 <= source_now-stamp <= self.max_age:
            # DDS discovery can deliver queued old samples during startup. They
            # cannot initialize the filter, but do not poison a never-armed run.
            if not self.armed:
                return False
            return self.fail('LIO input source age invalid')
        if stream in self.stamps:
            dt = stamp-self.stamps[stream]
            if dt <= 0:
                return self.fail('Nonmonotonic LIO input timestamp')
            if self.armed and (dt > self.max_gap or arrival_now-self.seen[stream] > self.max_gap):
                return self.fail('LIO input source/arrival gap')
        self.seen[stream], self.stamps[stream] = arrival_now, stamp
        if not self.armed:
            if len(self.seen) != 2 or max(arrival_now-t for t in self.seen.values()) > self.max_skew:
                return False
            if abs(self.stamps['imu']-self.stamps['cloud']) > self.max_skew:
                return False
            self.armed = True
        elif not self.tick(arrival_now):
            return False
        # Separate DDS callbacks are not synchronized. A live IMU may arrive
        # before the next scan (whose header marks scan start). After arming,
        # each stream's own source age and gap are the continuity contract;
        # comparing the two latest callbacks creates false faults on valid scans.
        return True


class PoseContinuity:
    def __init__(self, max_gap=1., max_speed=6., max_angular_speed=12.):
        if not all(math.isfinite(v) and v > 0 for v in (max_gap,max_speed,max_angular_speed)):
            raise ValueError('Invalid LIO pose continuity limits')
        self.max_gap, self.max_speed, self.max_angular_speed = max_gap,max_speed,max_angular_speed
        self.previous = None
        self.fault = ''

    def check(self, stamp_ns, point, quaternion):
        if self.fault:
            raise ValueError(self.fault)
        if self.previous is not None:
            old_stamp, old_point, old_q = self.previous
            dt = (stamp_ns-old_stamp)*1e-9
            norm = math.sqrt(sum(v*v for v in quaternion)*sum(v*v for v in old_q))
            dot = min(1., abs(sum(a*b for a,b in zip(quaternion,old_q)))/norm)
            angle = 2*math.acos(dot)
            if not 0 < dt <= self.max_gap:
                self.fault = 'LIO pose source gap/nonmonotonic time; explicit state restart required'
            elif math.dist(point,old_point) > self.max_speed*dt+.05 or angle > self.max_angular_speed*dt+.05:
                self.fault = 'LIO pose motion exceeds physical continuity limits; explicit state restart required'
            if self.fault:
                raise ValueError(self.fault)
        self.previous = stamp_ns, tuple(point), tuple(quaternion)
