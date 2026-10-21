#!/usr/bin/env python3
"""TEST_ONLY 10Hz rotating LIO / 50Hz xy adapter; no serial or hardware nodes."""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String


def main():
    os.environ.setdefault('ROS_DOMAIN_ID','208')
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE']='LOCALHOST'
    source=Path(__file__).parents[1]/'src/rm_nav_bringup/scripts/sensor_reference.py'
    with tempfile.TemporaryFile(mode='w+') as log:
        process=subprocess.Popen([sys.executable,str(source)],stdout=log,stderr=log,start_new_session=True)
        rclpy.init()
        node=Node('test_only_rotating_sensor_heading')
        pose=node.create_publisher(Odometry,'/lio/sensor_odometry',10)
        raw=node.create_publisher(Bool,'/sensors/front_mid360/cloud_healthy',10)
        allowed=node.create_publisher(Bool,'/nav/motion_allowed',10)
        command=node.create_publisher(TwistStamped,'/nav/cmd_vel_safe',1)
        state={'status':{},'shadow':None}
        epoch=node.get_clock().now().nanoseconds
        errors=[]
        def receive(message):
            state['shadow']=message
            stamp=message.header.stamp.sec*10**9+message.header.stamp.nanosec
            elapsed=(stamp-epoch)*1e-9
            if 1.5<elapsed<3.4 and math.hypot(message.twist.linear.x,message.twist.linear.y)>.1:
                angle=1.2*elapsed
                x,y=message.twist.linear.x,message.twist.linear.y
                errors.append(math.hypot(math.cos(angle)*x-math.sin(angle)*y-.2,
                                         math.sin(angle)*x+math.cos(angle)*y))
        node.create_subscription(TwistStamped,'/nav/sensor_cmd_vel_shadow',receive,10)
        node.create_subscription(String,'/nav/sensor_command_status',
                                 lambda m:state.update(status=json.loads(m.data)),10)
        next_pose=0.
        def step(elapsed, send_pose=True):
            nonlocal next_pose
            stamp=node.get_clock().now().to_msg()
            raw.publish(Bool(data=True))
            allowed.publish(Bool(data=True))
            if send_pose and elapsed>=next_pose:
                next_pose=elapsed+.1
                angle=1.2*((stamp.sec*10**9+stamp.nanosec-epoch)*1e-9)
                m=Odometry()
                m.header.stamp,m.header.frame_id,m.child_frame_id=stamp,'odom','front_mid360_imu'
                m.pose.pose.orientation.z,m.pose.pose.orientation.w=math.sin(angle/2),math.cos(angle/2)
                for index in range(6):m.pose.covariance[index*7]=.01
                pose.publish(m)
            cmd=TwistStamped()
            cmd.header.stamp,cmd.header.frame_id=stamp,'nav_reference'
            cmd.twist.linear.x=.2
            command.publish(cmd)
            rclpy.spin_once(node,timeout_sec=.005)
            time.sleep(.005)
        try:
            start=time.monotonic()
            while time.monotonic()-start<3.5:
                assert process.poll() is None,'sensor node exited'
                step(time.monotonic()-start)
            assert len(errors)>30,('insufficient moving output',len(errors),state)
            p95=sorted(errors)[int(.95*(len(errors)-1))]
            assert p95<.006,('10Hz heading lag not corrected',p95)
            stop=time.monotonic()
            while time.monotonic()-stop<.3:
                step(time.monotonic()-start,False)
            output=state['shadow']
            assert output is not None and math.hypot(output.twist.linear.x,output.twist.linear.y)<1e-9
            assert node.count_publishers('/cmd_vel')==0
            assert node.count_publishers('/cmd_yaw_angle')==0
            print(json.dumps({'result':'PASS','mode':'TEST_ONLY','lio_hz':10,
                              'sensor_yaw_rate_rad_s':1.2,'world_velocity_error_p95_m_s':p95,
                              'checked_samples':len(errors),'pose_loss_zero_output':True,
                              'physical_publishers':0}))
        except Exception:
            log.seek(0)
            print(log.read(),file=sys.stderr)
            raise
        finally:
            os.killpg(process.pid,signal.SIGINT)
            try:process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGKILL)
                process.wait()
            node.destroy_node()
            rclpy.shutdown()


if __name__=='__main__':main()
