"""Geometry for sensor-axis navigation; no chassis orientation is inferred."""
import math


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


def spinning_radius(length=.5, width=.5, offset_y=.16, margin=.05):
    return math.hypot(length/2, width/2+abs(offset_y))+margin


def circle_polygon(radius=.54, count=32):
    # Circumscribed polygon: edges, not only vertices, cover the requested circle.
    r = radius/math.cos(math.pi/count)
    return [[r*math.cos(2*math.pi*i/count), r*math.sin(2*math.pi*i/count)]
            for i in range(count)]
