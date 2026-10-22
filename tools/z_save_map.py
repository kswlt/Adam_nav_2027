#!/usr/bin/env python3
"""Drain ZLIO and save a committed graph snapshot, checking both service outcomes."""
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


def call(node, name):
    client = node.create_client(Trigger, name)
    if not client.wait_for_service(timeout_sec=5.):
        raise RuntimeError('Service unavailable: '+name)
    future = client.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=120.)
    if not future.done() or future.exception() is not None:
        raise RuntimeError('Service failed/timed out: '+name)
    response = future.result()
    if not response.success:
        raise RuntimeError(name+': '+response.message)
    print(name+': '+response.message, flush=True)


def main():
    rclpy.init()
    node = Node('rm_z_map_save_client')
    try:
        call(node, '/stop')
        call(node, '/backend_node/save_map')
        print('地图快照已保存；前端已停止。开始下一次任务前重新启动状态链。')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__': main()
