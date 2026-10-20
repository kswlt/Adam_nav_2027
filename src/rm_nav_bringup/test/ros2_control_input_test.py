import math
import os
import signal
import subprocess
import sys
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist,TwistStamped
from std_msgs.msg import Float32,Float64MultiArray,Header
from rm_nav_interfaces.msg import DriveFeedback

def main():
    os.environ['ROS_DOMAIN_ID']='185';os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE']='LOCALHOST'
    rclpy.init();node=Node('test_only_control_input');messages={'v':[],'y':[],'lease':[]}
    for kind,topic,key in [(Float64MultiArray,'/omni_velocity_controller/commands','v'),
                          (Float64MultiArray,'/omni_yaw_controller/commands','y'),(Header,'/hardware/command_lease','lease')]:
        node.create_subscription(kind,topic,lambda m,k=key:messages[k].append(m),10)
    pubs=[node.create_publisher(t,name,1) for t,name in [(Twist,'/hardware/controller_input/cmd_vel'),
        (Float32,'/hardware/controller_input/cmd_yaw_angle'),(TwistStamped,'/nav/cmd_vel_safe'),(DriveFeedback,'/hardware/drive_feedback')]]
    process=subprocess.Popen([sys.executable,sys.argv[1]],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
    def spin(seconds):
        until=time.monotonic()+seconds
        while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.01)
    try:
        deadline=time.monotonic()+10
        while not all(p.get_subscription_count() for p in pubs) and time.monotonic()<deadline:spin(.05)
        assert all(p.get_subscription_count() for p in pubs),'Discovery failed'
        assert not messages['lease'],'Lease fabricated before feedback'
        for i in range(35):
            stamp=node.get_clock().now().to_msg()
            feedback=DriveFeedback();feedback.header=Header(stamp=stamp,frame_id='base_link');feedback.sequence=i+1;feedback.drive_ready=True
            pubs[3].publish(feedback)
            safe=TwistStamped();safe.header=feedback.header;safe.twist.linear.x=.2;safe.twist.linear.y=-.1;pubs[2].publish(safe)
            velocity=Twist();velocity.linear.x=.2;velocity.linear.y=-.1;pubs[0].publish(velocity);pubs[1].publish(Float32(data=30.))
            spin(.03)
        assert messages['lease'] and list(messages['v'][-1].data)==[.2,-.1]
        assert abs(messages['y'][-1].data[0]-math.pi/6)<1e-6
        # A stop arriving before a delayed Omni candidate must dominate that candidate.
        for i in range(8):
            feedback.header.stamp=node.get_clock().now().to_msg();feedback.sequence+=1;pubs[3].publish(feedback)
            safe.header=feedback.header;safe.twist=Twist();pubs[2].publish(safe)
            pubs[0].publish(velocity);pubs[1].publish(Float32(data=30.));spin(.03)
        assert list(messages['v'][-1].data)==[0.,0.] and messages['y'][-1].data[0]==0.,'Stop replayed a prior candidate/yaw'
        spin(.4);count=len(messages['lease']);spin(.15)
        assert len(messages['lease'])==count and list(messages['v'][-1].data)==[0.,0.],'Expired input refreshed command lease'
        assert process.poll() is None
        print('PASS real ROS controller input: degree/radian adapter, true feedback prerequisite, independent lease expiry')
    finally:
        process.send_signal(signal.SIGINT)
        try:process.wait(timeout=3)
        except subprocess.TimeoutExpired:process.kill();process.wait()
        node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
