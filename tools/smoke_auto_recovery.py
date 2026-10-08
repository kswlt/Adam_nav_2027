#!/usr/bin/env python3
"""Real local-GICP timeout -> bounded automatic recovery -> true KISS candidates."""
import math
import os
from pathlib import Path
import random
import signal
import subprocess
import rclpy
from rm_nav_interfaces.msg import RecoveryState
from smoke_recovery_transaction import Plant
from smoke_frozen_map_localization import cloud,require


def main():
    require(os.environ.get('ROS_DOMAIN_ID') not in (None,'','0'),'Use isolated Domain')
    rclpy.init()
    node=Plant(publish_motion_permission=False)
    processes=[]
    log_path=Path('log/auto_recovery_smoke.log');log_path.parent.mkdir(exist_ok=True)
    try:
        with log_path.open('w') as log:
            command=['ros2','launch','rm_nav_bringup','frozen_map_localization.launch.py',
                     'map_version:=transaction-v1','enable_recovery:=true','enable_auto_recovery:=true',
                     'recovery_timeout:=10.0','field_bounds:=[-12.0,12.0,-12.0,12.0]']
            processes.append(subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
            require(node.wait(lambda:node.submap.get_subscription_count()>0,15),'Matcher missing')
            rng=random.Random(42)
            points=[tuple(rng.uniform(-5,5) for _ in range(3)) for _ in range(5000)]
            node.frozen.publish(cloud(points,'map',node.get_clock().now().to_msg()))
            node.wait(lambda:False,0.3)
            for _ in range(8):
                node.submap.publish(cloud(points,'odom',node.get_clock().now().to_msg()))
                if node.wait(lambda:node.healthy is True and node.transform is not None,0.25):break
            require(node.healthy is True,'Initial real GICP failed')
            original=node.transform.transform
            # No service request is made. Losing accepted observations triggers the manager.
            require(node.wait(lambda:node.state is not None and node.state.phase==RecoveryState.SEARCHING,2),
                    'Timeout did not automatically create bounded session')
            require(node.wait(lambda:node.healthy is False and node.state.recovery_id>0,0.5),
                    'Auto recovery did not suspend health')
            c,s=math.cos(0.2),math.sin(0.2)
            source=[(c*(x-0.6)+s*y,-s*(x-0.6)+c*y,z) for x,y,z in points]
            node.submap.publish(cloud(source,'odom',node.get_clock().now().to_msg()))
            require(node.wait(lambda:node.estimate is not None and node.estimate.method==node.estimate.KISS_GICP,3),
                    'Automatic session did not run real KISS')
            node.wait(lambda:False,1.05)
            node.submap.publish(cloud(source[20:],'odom',node.get_clock().now().to_msg()))
            require(node.wait(lambda:node.state.phase==RecoveryState.CONFIRMED,3),'Auto KISS candidate not confirmed')
            require(node.transform.transform==original and node.healthy is False,'Auto candidate directly changed TF')
            print('PASS actual GICP timeout automatically starts bounded KISS/GICP; candidate confirmed without direct TF/health release',flush=True)
    finally:
        for process in processes:os.killpg(process.pid,signal.SIGINT)
        for process in processes:
            try:process.wait(timeout=5)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
