"""Foxglove bridge plus optional latched PCD map publisher; visualization only."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def nodes(context):
    map_pcd = LaunchConfiguration('map_pcd').perform(context)
    actions = [Node(package='foxglove_bridge', executable='foxglove_bridge', name='foxglove_bridge',
                    parameters=[{'port': int(LaunchConfiguration('port').perform(context)),
                                 'address': LaunchConfiguration('address').perform(context),
                                 'send_buffer_limit': 100 * 1024 * 1024}], output='screen')]
    actions.append(Node(package='rm_nav_bringup', executable='foxglove_trace_publisher',
                        name='foxglove_trace_publisher',
                        output='screen'))
    if map_pcd:
        actions.append(Node(package='rm_nav_bringup', executable='pcd_map_publisher',
                            name='foxglove_map_cloud',
                            arguments=['--pcd', map_pcd, '--frame', LaunchConfiguration('map_frame').perform(context),
                                       '--topic', '/visualization/map_cloud'], output='screen'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='8765'),
        DeclareLaunchArgument('address', default_value='0.0.0.0'),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('map_pcd', default_value='', description='Optional binary XYZ PCD to publish latched'),
        OpaqueFunction(function=nodes),
    ])
