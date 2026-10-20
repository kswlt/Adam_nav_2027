#!/usr/bin/env python3
"""Omni candidate -> standard forward controllers; never opens a serial port."""
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy
from geometry_msgs.msg import Twist,TwistStamped
from std_msgs.msg import Bool,Float32,Float64MultiArray,Header,String
from rm_nav_interfaces.msg import DriveFeedback

class ControlInput(Node):
    def __init__(self):
        super().__init__('ros2_control_input')
        self.frame=self.declare_parameter('base_frame','base_link').value
        self.timeout=float(self.declare_parameter('timeout',.2).value)
        if not math.isfinite(self.timeout) or not 0<self.timeout<=1:raise ValueError('Invalid lease timeout')
        self.v=None;self.yaw=None;self.v_at=self.yaw_at=self.safe_at=0.;self.safe=None
        self.velocity=self.create_publisher(Float64MultiArray,'/omni_velocity_controller/commands',1)
        self.angle=self.create_publisher(Float64MultiArray,'/omni_yaw_controller/commands',1)
        self.lease=self.create_publisher(Header,'/hardware/command_lease',1)
        self.measured=self.create_publisher(TwistStamped,'/hardware/measured_twist',1)
        self.ready=self.create_publisher(Bool,'/hardware/drive_ready',1)
        self.feedback=None;self.feedback_at=0.;self.sequence=0
        self.create_subscription(DriveFeedback,'/hardware/drive_feedback',self.receive_feedback,1)
        self.create_subscription(Twist,'/hardware/controller_input/cmd_vel',self.receive_velocity,1)
        self.create_subscription(Float32,'/hardware/controller_input/cmd_yaw_angle',self.receive_yaw,1)
        self.create_subscription(TwistStamped,'/nav/cmd_vel_safe',self.receive_safe,1)
        description=self.declare_parameter('robot_description','').value
        if description:
            self.description=self.create_publisher(String,'/robot_description',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.description.publish(String(data=description))
        self.create_timer(.02,self.tick)
    def receive_velocity(self,m):
        self.v=[m.linear.x,m.linear.y] if all(math.isfinite(v) for v in (m.linear.x,m.linear.y)) and m.linear.z==m.angular.x==m.angular.y==m.angular.z==0 else None
        self.v_at=time.monotonic()
    def receive_yaw(self,m):
        self.yaw=math.radians(m.data) if math.isfinite(m.data) else None
        self.yaw_at=time.monotonic()
    def receive_safe(self,m):
        age=(self.get_clock().now().nanoseconds-(m.header.stamp.sec*10**9+m.header.stamp.nanosec))/1e9
        v=m.twist
        self.safe=m if (m.header.frame_id==self.frame and 0<=age<=self.timeout and
            all(math.isfinite(x) for x in (v.linear.x,v.linear.y,v.angular.z)) and v.linear.z==v.angular.x==v.angular.y==0) else None
        self.safe_at=time.monotonic()
    def receive_feedback(self,m):
        v=m.measured_twist
        age=(self.get_clock().now().nanoseconds-(m.header.stamp.sec*10**9+m.header.stamp.nanosec))/1e9
        valid=(m.header.frame_id==self.frame and 0<=age<=self.timeout and m.sequence>self.sequence and
               all(math.isfinite(x) for x in (v.linear.x,v.linear.y,v.angular.z,m.yaw_rad)) and
               v.linear.z==v.angular.x==v.angular.y==0 and
               (not m.stopped or (math.hypot(v.linear.x,v.linear.y)<=.01 and abs(v.angular.z)<=.01)))
        self.feedback=m if valid else None;self.feedback_at=time.monotonic()
        if valid:self.sequence=m.sequence;self.measured.publish(TwistStamped(header=m.header,twist=v))
        else:self.v=None;self.safe=None
    def tick(self):
        at=time.monotonic()
        feedback_ok=(self.feedback is not None and self.feedback.drive_ready and at-self.feedback_at<=self.timeout and
            0<=(self.get_clock().now().nanoseconds-(self.feedback.header.stamp.sec*10**9+self.feedback.header.stamp.nanosec))/1e9<=self.timeout)
        self.ready.publish(Bool(data=feedback_ok))
        if not feedback_ok or self.v is None or self.yaw is None or self.safe is None or any(at-t>self.timeout for t in (self.v_at,self.yaw_at,self.safe_at)):
            self.velocity.publish(Float64MultiArray(data=[0.,0.]))
            return  # No command lease: HardwareInterface independently stops/latches.
        age=(self.get_clock().now().nanoseconds-(self.safe.header.stamp.sec*10**9+self.safe.header.stamp.nanosec))/1e9
        if not 0<=age<=self.timeout:return
        stopped=self.safe.twist.linear.x==self.safe.twist.linear.y==self.safe.twist.angular.z==0
        self.velocity.publish(Float64MultiArray(data=[0.,0.] if stopped else self.v))
        self.angle.publish(Float64MultiArray(data=[self.feedback.yaw_rad if stopped else self.yaw]))
        self.lease.publish(Header(stamp=self.get_clock().now().to_msg(),frame_id=self.frame))

def main():
    rclpy.init();node=ControlInput()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
