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
    graph_poses = LaunchConfiguration('graph_poses').perform(context)
    if graph_poses:
        actions.append(Node(package='rm_nav_bringup', executable='foxglove_graph_publisher',
                            name='foxglove_graph_publisher',
                            arguments=['--poses', graph_poses, '--loops', LaunchConfiguration('graph_loops').perform(context),
                                       '--frame', LaunchConfiguration('map_frame').perform(context)], output='screen'))
    manifest = LaunchConfiguration('map_bundle').perform(context)
    if manifest:
        actions.append(Node(package='rm_nav_bringup', executable='foxglove_map_status',
                            name='foxglove_map_status', arguments=['--manifest', manifest], output='screen'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='8765'),
        DeclareLaunchArgument('address', default_value='0.0.0.0'),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('map_pcd', default_value='', description='Optional binary XYZ PCD to publish latched'),
        DeclareLaunchArgument('graph_poses', default_value='', description='Optional optimized_poses.json'),
        DeclareLaunchArgument('graph_loops', default_value='', description='Optional loop_edges.json'),
        DeclareLaunchArgument('map_bundle', default_value='', description='Optional map_bundle.json manifest'),
        OpaqueFunction(function=nodes),
    ])
