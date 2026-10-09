#!/usr/bin/env python3
"""Publish optimized keyframe and loop graph JSON as latched Foxglove markers."""
import argparse
import json
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray


class GraphPublisher(Node):
    def __init__(self, poses_path, loops_path, frame):
        super().__init__('foxglove_graph_publisher')
        self.pub = self.create_publisher(MarkerArray, '/visualization/optimization_graph', QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
        self.msg = self.load(poses_path, loops_path, frame)
        self.timer = self.create_timer(1.0, lambda: self.pub.publish(self.msg))
        self.get_logger().info(f'Publishing {len(self.msg.markers)} optimization markers')

    @staticmethod
    def point(matrix): return (float(matrix[3]), float(matrix[7]), float(matrix[11]))

    def load(self, poses_path, loops_path, frame):
        poses = json.loads(Path(poses_path).resolve(strict=True).read_text())
        loops = json.loads(Path(loops_path).resolve(strict=True).read_text()) if loops_path else []
        entries = poses.get('poses', [])
        if not 1 <= len(entries) <= 500: raise ValueError('Pose graph must contain 1..500 poses')
        points = [self.point(e['matrix']) for e in entries]
        result = MarkerArray()
        trace = Marker(); trace.header.frame_id=frame; trace.ns='optimized_trajectory'; trace.id=0
        trace.type=Marker.LINE_STRIP; trace.action=Marker.ADD; trace.scale.x=.035
        trace.color.r=.1; trace.color.g=.8; trace.color.b=1.; trace.color.a=1.
        for x,y,z in points:
            from geometry_msgs.msg import Point
            p=Point();p.x=x;p.y=y;p.z=z;trace.points.append(p)
        result.markers.append(trace)
        keyframes = Marker(); keyframes.header.frame_id=frame; keyframes.ns='keyframes'; keyframes.id=1
        keyframes.type=Marker.SPHERE_LIST; keyframes.action=Marker.ADD; keyframes.scale.x=.12; keyframes.scale.y=.12; keyframes.scale.z=.12
        keyframes.color.r=1.; keyframes.color.g=.8; keyframes.color.a=1.
        for x,y,z in points:
            from geometry_msgs.msg import Point
            p=Point();p.x=x;p.y=y;p.z=z;keyframes.points.append(p)
        result.markers.append(keyframes)
        if len(loops)>1000: raise ValueError('Loop marker budget exceeded')
        for index, edge in enumerate(loops):
            a,b=int(edge['from']),int(edge['to'])
            if not 0<=a<len(points) or not 0<=b<len(points): raise ValueError('Loop index out of range')
            marker=Marker();marker.header.frame_id=frame;marker.ns='accepted_loops';marker.id=100+index
            marker.type=Marker.LINE_LIST;marker.action=Marker.ADD;marker.scale.x=.06
            marker.color.r=1.;marker.color.g=.15;marker.color.a=1.
            from geometry_msgs.msg import Point
            for point in (points[a],points[b]):
                p=Point();p.x,p.y,p.z=point;marker.points.append(p)
            result.markers.append(marker)
        return result


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--poses',required=True);parser.add_argument('--loops',default='')
    parser.add_argument('--frame',default='mapping_odom');args,_=parser.parse_known_args()
    rclpy.init();node=GraphPublisher(args.poses,args.loops,args.frame)
    try:rclpy.spin(node)
    finally:node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
