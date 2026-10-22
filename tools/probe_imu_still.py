#!/usr/bin/env python3
"""Read-only initialization check; IMU consistency is not independent motion truth."""
import json
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
import numpy as np


def main():
    rclpy.init()
    node=Node('read_only_initialization_imu_probe')
    samples=[]
    def receive(m):
        a,g=m.linear_acceleration,m.angular_velocity
        age=(node.get_clock().now().nanoseconds-m.header.stamp.sec*10**9-m.header.stamp.nanosec)/1e9
        if 0<=age<.25:samples.append([a.x,a.y,a.z,g.x,g.y,g.z])
    node.create_subscription(Imu,'/livox/imu',receive,qos_profile_sensor_data)
    end=time.monotonic()+6
    while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
    okay=False
    result={'scope':'real IMU read-only; not independent stationary truth','samples':len(samples)}
    if len(samples)>=200 and np.isfinite(samples).all():
        values=np.asarray(samples)
        gyro=np.linalg.norm(values[:,3:],axis=1)
        acc=np.linalg.norm(values[:,:3],axis=1)
        okay=bool(np.quantile(gyro,.95)<.03 and .8<np.median(acc)<1.2 and np.std(acc)<.02)
        result.update(gyro_norm_p95_rad_s=float(np.quantile(gyro,.95)),acc_norm_median_g=float(np.median(acc)),acc_norm_std_g=float(np.std(acc)))
    result['initialization_imu_consistent_with_stillness']=okay
    print(json.dumps(result))
    node.destroy_node()
    rclpy.shutdown()
    return 0 if okay else 1


if __name__=='__main__':raise SystemExit(main())
