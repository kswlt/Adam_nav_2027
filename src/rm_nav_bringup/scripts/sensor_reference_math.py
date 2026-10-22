"""Geometry for sensor-axis navigation; no chassis orientation is inferred."""
import math


def reference_offset(imu_to_lidar, reference_point, axis_to_lidar, verified, calibration_id,
                     fit_debug_approved=False):
    """Factory IMU->LiDAR plus configured LiDAR->axis, all in sensor axes.

    Chassis->gimbal is deliberately not an input to this transformation.
    """
    if (len(imu_to_lidar) != 3 or len(axis_to_lidar) != 3 or
            not all(math.isfinite(v) for v in (*imu_to_lidar, *axis_to_lidar))):
        raise ValueError('Reference geometry requires finite three-element translations')
    if reference_point == 'lidar_center':
        if any(v != 0 for v in axis_to_lidar):
            raise ValueError('Nonzero axis geometry requires explicit gimbal_axis mode')
        return tuple(imu_to_lidar)
    if (reference_point != 'gimbal_axis' or not (verified or fit_debug_approved) or
            (verified and fit_debug_approved) or
            (fit_debug_approved and 'debug' not in calibration_id.lower()) or not calibration_id.strip() or
            calibration_id == 'vehicle164_sensor_axis_factory_v1'):
        raise ValueError('gimbal_axis requires measured geometry or approved fit-debug geometry with a distinct ID')
    return tuple(a-b for a,b in zip(imu_to_lidar, axis_to_lidar))


class HeadingTracker:
    """Short constant-rate yaw extrapolation from two actual LIO samples.

    No TF timestamp is changed, no chassis heading is inferred. Bounds limit
    extrapolation, not physical prediction error under arbitrary acceleration.
    """
    def __init__(self, max_age=.12, max_correction=.25, max_rate=12.):
        if (not all(math.isfinite(v) and v > 0 for v in (max_age,max_correction,max_rate))
                or max_age > .25 or max_correction > .5):
            raise ValueError('Invalid heading extrapolation limits')
        self.max_age, self.max_correction, self.max_rate = max_age,max_correction,max_rate
        self.reset()

    def reset(self):
        self.sample, self.rate = None, None

    def observe(self, stamp_ns, yaw):
        if stamp_ns <= 0 or not math.isfinite(yaw):
            self.reset()
            raise ValueError('Invalid heading sample')
        self.rate = None if self.sample is None else self.rate
        if self.sample is not None:
            old_stamp, old_yaw = self.sample
            dt = (stamp_ns-old_stamp)*1e-9
            if dt <= 0:
                self.reset()
                raise ValueError('Nonmonotonic heading source timestamp')
            delta = math.atan2(math.sin(yaw-old_yaw), math.cos(yaw-old_yaw))
            self.rate = delta/dt if dt <= .25 else None
            if self.rate is not None and abs(self.rate) > self.max_rate:
                self.reset()
                raise ValueError('Heading rate exceeds physical limit')
        self.sample = stamp_ns, yaw

    def predict(self, now_ns):
        if self.sample is None or self.rate is None:
            raise ValueError('Waiting for two continuous real LIO headings')
        stamp, yaw = self.sample
        age = (now_ns-stamp)*1e-9
        correction = self.rate*age
        if (not 0 <= age <= self.max_age or abs(correction) > self.max_correction):
            raise ValueError('Heading extrapolation time/angle budget exceeded')
        return yaw+correction, age, correction


def sensor_pose(position, quaternion, imu_to_lidar):
    values = [*position, *quaternion, *imu_to_lidar]
    if not all(math.isfinite(v) for v in values):
        raise ValueError('Non-finite sensor pose')
    x, y, z, w = quaternion
    if abs(x*x+y*y+z*z+w*w-1) > 1e-3:
        raise ValueError('Non-unit sensor quaternion')
    # Factory MID-360 internal rotation is identity. Translation is in IMU axes.
    a, b, c = imu_to_lidar
    offset = ((1-2*(y*y+z*z))*a+2*(x*y-z*w)*b+2*(x*z+y*w)*c,
              2*(x*y+z*w)*a+(1-2*(x*x+z*z))*b+2*(y*z-x*w)*c,
              2*(x*z-y*w)*a+2*(y*z+x*w)*b+(1-2*(x*x+y*y))*c)
    yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
    return tuple(p+d for p, d in zip(position, offset)), yaw


def sensor_velocity(vx, vy, sensor_yaw):
    if not all(math.isfinite(v) for v in (vx, vy, sensor_yaw)):
        raise ValueError('Non-finite command')
    c, s = math.cos(sensor_yaw), math.sin(sensor_yaw)
    return c*vx+s*vy, -s*vx+c*vy


def spinning_radius(length=.5, width=.5, *, offset_y, margin=.05):
    return math.hypot(length/2, width/2+abs(offset_y))+margin


def circle_polygon(radius=.54, count=32):
    # Circumscribed polygon: edges, not only vertices, cover the requested circle.
    r = radius/math.cos(math.pi/count)
    return [[r*math.cos(2*math.pi*i/count), r*math.sin(2*math.pi*i/count)]
            for i in range(count)]
