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
                                 'send_buffer_limit': int(LaunchConfiguration('send_buffer_limit').perform(context))}], output='screen')]
    actions.append(Node(package='rm_nav_bringup', executable='foxglove_trace_publisher',
                        name='foxglove_trace_publisher',
                        output='screen'))
    actions.append(Node(package='rm_nav_bringup', executable='live_map_preview',
                        condition=IfCondition(LaunchConfiguration('enable_live_preview')),
                        arguments=['--max-voxels', LaunchConfiguration('preview_max_voxels').perform(context),
                                   '--voxel', LaunchConfiguration('preview_voxel').perform(context),
                                   '--publish-hz', LaunchConfiguration('preview_publish_hz').perform(context)],
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
        # 小缓冲优先丢弃旧数据，避免高带宽点云在 WebSocket 排队形成秒级延迟。
        DeclareLaunchArgument('send_buffer_limit', default_value=str(4*1024*1024)),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('enable_live_preview', default_value='true'),
        DeclareLaunchArgument('preview_max_voxels', default_value='50000'),
        DeclareLaunchArgument('preview_voxel', default_value='0.15'),
        DeclareLaunchArgument('preview_publish_hz', default_value='1.0'),
        DeclareLaunchArgument('map_pcd', default_value='', description='Optional binary XYZ PCD to publish latched'),
        DeclareLaunchArgument('graph_poses', default_value='', description='Optional optimized_poses.json'),
        DeclareLaunchArgument('graph_loops', default_value='', description='Optional loop_edges.json'),
        DeclareLaunchArgument('map_bundle', default_value='', description='Optional map_bundle.json manifest'),
        OpaqueFunction(function=nodes),
    ])
