#!/usr/bin/env python3
"""TEST_ONLY real cancellation service with injected owned/late action handles."""
import os,sys,time
from pathlib import Path
from concurrent.futures import Future
from types import SimpleNamespace
assert os.environ.get('ROS_DOMAIN_ID')=='229','Requires isolated TEST_ONLY domain 229'
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/rm_nav_bringup/scripts'))
import rclpy
from rclpy.executors import SingleThreadedExecutor
from std_srvs.srv import Trigger
from std_msgs.msg import Bool
from task_supervisor import TaskSupervisor
rclpy.init();supervisor=TaskSupervisor();client_node=rclpy.create_node('cancel_test_only')
executor=SingleThreadedExecutor();executor.add_node(supervisor);executor.add_node(client_node)
flags=[];sub=client_node.create_subscription(Bool,'/nav/motion_enable',lambda m:flags.append(m.data),10)
class Handle:
    accepted=True
    def __init__(self):self.cancelled=0
    def cancel_goal_async(self):self.cancelled+=1
try:
 client=client_node.create_client(Trigger,'/nav/cancel_task');assert client.wait_for_service(timeout_sec=3)
 for phase in ('PLANNING','STARTING','NAVIGATING'):
  nav,planner=Handle(),Handle();supervisor.navigation_handle=nav;supervisor.planner_handle=planner
  supervisor.phase=phase
  f=client.call_async(Trigger.Request());end=time.monotonic()+3
  while not f.done() and time.monotonic()<end:executor.spin_once(timeout_sec=.02)
  assert f.done() and f.result().success
  assert supervisor.phase=='FAULT' and nav.cancelled==1 and planner.cancelled==1
  # A late acceptance must be cancelled, never restore NAVIGATING permission.
  late=Handle();future=Future();future.set_result(late)
  supervisor.navigation_accepted(supervisor.task_id,future)
  assert late.cancelled==1 and supervisor.phase=='FAULT'
 end=time.monotonic()+.3
 while time.monotonic()<end:executor.spin_once(timeout_sec=.02)
 assert flags and not any(flags),'Cancellation granted motion permission'
 print('PASS: actual service, owned handles only, late acceptance cancelled, permission false; TEST_ONLY injected handles, not real Nav2/hardware')
finally:
 executor.remove_node(supervisor);executor.remove_node(client_node)
 supervisor.destroy_node();client_node.destroy_node();rclpy.shutdown()
