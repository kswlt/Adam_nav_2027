#!/usr/bin/env python3
"""Publish a latched JSON MapBundle status for Foxglove Raw Messages."""
import argparse
import json
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String


class MapStatus(Node):
    def __init__(self, manifest):
        super().__init__('foxglove_map_status')
        self.pub=self.create_publisher(String,'/visualization/map_bundle_status',QoSProfile(
            depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE))
        data=json.loads(Path(manifest).resolve(strict=True).read_text())
        required=('schema','status','version','coordinate_frame','official_alignment_applied',
                  'files','publishable_for_frozen_localization')
        if any(k not in data for k in required) or data['status']!='draft_mapping_odom' or data['coordinate_frame']!='mapping_odom' or data['official_alignment_applied'] is not False:
            raise ValueError('Only explicit draft mapping_odom MapBundle manifests are accepted')
        self.msg=String();self.msg.data=json.dumps(data,sort_keys=True)
        self.pub.publish(self.msg);self.timer=self.create_timer(1.0,lambda:self.pub.publish(self.msg))
        self.get_logger().info(f'MapBundle {data["version"]}: {data["status"]}, publishable=false')


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--manifest',required=True);args,_=parser.parse_known_args()
    rclpy.init();node=MapStatus(args.manifest)
    try:rclpy.spin(node)
    finally:node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
