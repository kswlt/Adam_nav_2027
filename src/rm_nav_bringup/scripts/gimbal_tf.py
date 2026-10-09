#!/usr/bin/env python3
"""Time-stamped absolute encoder to calibrated yaw TF; never consumes contact_angle."""
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


class GimbalTF(Node):
    def __init__(self):
        super().__init__('gimbal_tf')
        self.frames = {key:self.declare_parameter(key,'').value for key in
                       ('footprint_frame','chassis_frame','yaw_frame','imu_frame','lidar_frame')}
        self.calibration = self.declare_parameter('calibration_id','').value
        self.joint = self.declare_parameter('yaw_joint','big_gimbal_yaw').value
        self.sign = self.declare_parameter('yaw_sign',1.0).value
        self.offset = self.declare_parameter('yaw_offset',0.0).value
        if not self.calibration or not all(self.frames.values()) or len(set(self.frames.values())) != 5 or 'base_link' in self.frames.values() or abs(self.sign)!=1 or not math.isfinite(self.offset):
            raise ValueError('Explicit calibration, unique frames and valid encoder conversion required')
        self.transforms = {}
        for name in ('footprint_to_chassis','chassis_to_yaw_zero','yaw_to_imu','imu_to_lidar'):
            values = self.declare_parameter(name,[float('nan')]*7).value
            if len(values)!=7 or not all(math.isfinite(v) for v in values) or abs(sum(v*v for v in values[3:])-1)>1e-5:
                raise ValueError('Measured translation/quaternion required for '+name)
            self.transforms[name] = values
        self.dynamic = TransformBroadcaster(self)
        self.static = StaticTransformBroadcaster(self)
        self.health = self.create_publisher(Bool,'/state/gimbal_healthy',10)
        self.stamp = 0
        self.received = 0.0
        self.valid = False
        static = [self.transform('footprint_frame','chassis_frame','footprint_to_chassis'),
                  self.transform('yaw_frame','imu_frame','yaw_to_imu'),
                  self.transform('imu_frame','lidar_frame','imu_to_lidar')]
        # Nav2's base_link aliases the calibrated chassis reference, not the yawing sensor.
        alias = TransformStamped()
        alias.header.frame_id,alias.child_frame_id = self.frames['chassis_frame'],'base_link'
        alias.transform.rotation.w = 1.0
        alias.header.stamp = self.get_clock().now().to_msg()
        static.append(alias)
        self.static.sendTransform(static)
        self.create_subscription(JointState,'/hardware/gimbal_joint_states',self.encoder,qos_profile_sensor_data)
        self.create_timer(0.05,self.tick)

    def transform(self,parent,child,calibration):
        values = self.transforms[calibration]
        result = TransformStamped()
        result.header.frame_id,result.child_frame_id = self.frames[parent],self.frames[child]
        result.header.stamp = self.get_clock().now().to_msg()
        result.transform.translation.x,result.transform.translation.y,result.transform.translation.z = values[:3]
        q = result.transform.rotation
        q.x,q.y,q.z,q.w = values[3:]
        return result

    def encoder(self,msg):
        stamp = msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds-stamp)/1e9
        if msg.header.frame_id != self.frames['chassis_frame'] or stamp<=self.stamp or not -.05<=age<=.2 or msg.name.count(self.joint)!=1:
            self.valid=False;return
        index=msg.name.index(self.joint)
        if index>=len(msg.position) or not math.isfinite(msg.position[index]):
            self.valid=False;return
        # Require two consecutive samples with <=0.1 s gap before declaring time coverage.
        self.valid = self.stamp>0 and 0<stamp-self.stamp<=100_000_000
        self.stamp,self.received = stamp,time.monotonic()
        angle = self.sign*msg.position[index]+self.offset
        out = self.transform('chassis_frame','yaw_frame','chassis_to_yaw_zero')
        out.header.stamp = msg.header.stamp
        q = out.transform.rotation
        x,y,z,w=q.x,q.y,q.z,q.w
        s,c=math.sin(angle/2),math.cos(angle/2)
        q.x,q.y,q.z,q.w=x*c+y*s,y*c-x*s,z*c+w*s,w*c-z*s
        self.dynamic.sendTransform(out)

    def tick(self):
        age=(self.get_clock().now().nanoseconds-self.stamp)/1e9
        self.health.publish(Bool(data=self.valid and -.05<=age<=.2 and time.monotonic()-self.received<=.2))


def main():
    rclpy.init()
    node=GimbalTF()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
