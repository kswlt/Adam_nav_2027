#!/usr/bin/env python3
"""TEST_ONLY actual Jazzy manager/controller loading; hardware stays inactive."""
import os
from pathlib import Path
import signal
import subprocess
import time
import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import ListControllers,ListHardwareComponents
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from tf2_msgs.msg import TFMessage

def main():
    if os.environ.get('ROS_DOMAIN_ID') in (None,'','0'):raise RuntimeError('Isolated domain required')
    rclpy.init();node=Node('test_only_controller_manager_probe');outputs=[];transforms=[]
    node.create_subscription(Twist,'/cmd_vel',lambda m:outputs.append(m),1)
    node.create_subscription(Float32,'/cmd_yaw_angle',lambda m:outputs.append(m),1)
    node.create_subscription(TFMessage,'/tf',lambda m:transforms.append(m),1)
    clients=[node.create_client(t,'/controller_manager/'+name) for t,name in
             [(ListControllers,'list_controllers'),(ListHardwareComponents,'list_hardware_components')]]
    log=Path('log/ros2_control_smoke.log');log.parent.mkdir(exist_ok=True)
    process=None
    try:
        with log.open('w') as handle:
            process=subprocess.Popen(['ros2','launch','rm_nav_bringup','ros2_control_backend.launch.py'],stdout=handle,stderr=subprocess.STDOUT,start_new_session=True)
            for client in clients:
                if not client.wait_for_service(timeout_sec=20):raise RuntimeError('Controller manager service missing')
            deadline=time.monotonic()+20
            while time.monotonic()<deadline:
                future=clients[0].call_async(ListControllers.Request());rclpy.spin_until_future_complete(node,future,timeout_sec=3)
                result=future.result()
                if result and len(result.controller)==2 and all(c.state=='inactive' for c in result.controller):break
                rclpy.spin_once(node,timeout_sec=.2)
            else:raise RuntimeError('Both standard forward controllers did not configure inactive')
            future=clients[1].call_async(ListHardwareComponents.Request());rclpy.spin_until_future_complete(node,future,timeout_sec=3)
            result=future.result()
            if not result or len(result.component)!=1 or result.component[0].state.label!='inactive':raise RuntimeError('Hardware not inactive')
            if outputs or transforms:raise RuntimeError('Disabled backend published physical output or invented TF')
            print('PASS actual Jazzy controller_manager, two standard ForwardCommandControllers, inactive real hardware plugin; no physical output/TF')
    finally:
        if process:
            os.killpg(process.pid,signal.SIGINT)
            try:process.wait(timeout=5)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
        node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
