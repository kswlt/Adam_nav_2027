"""可重复的 MID-360 → LaserScan → Nav2 Costmap 联调启动器。

仅使用测试地图和显式 legacy 静态标定，禁止用于物理底盘。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup = get_package_share_directory('rm_nav_bringup')
    map_yaml = LaunchConfiguration('map_yaml')
    calibration = LaunchConfiguration('calibration_file')
    lio_params = LaunchConfiguration('lio_params_file')
    return LaunchDescription([
        DeclareLaunchArgument(
            'profile', default_value='LEGACY_DEBUG',
            description='TEST_ONLY 使用测试 TF；LEGACY_DEBUG 依赖外部真实 LIO；PRODUCTION 拒绝此 smoke 启动'),
        DeclareLaunchArgument('map_yaml', default_value='/tmp/smoke_map.yaml'),
        DeclareLaunchArgument(
            'calibration_file', default_value=os.path.join(
                get_package_share_directory('rm_nav_frames'), 'config',
                'local_state_calibration.legacy_adam_static.yaml')),
        DeclareLaunchArgument(
            'lio_params_file', default_value='/home/asus/nav_deps/src/small_point_lio/config/mid360.yaml'),
        OpaqueFunction(function=lambda context: _profile_nodes(
            context, bringup, map_yaml, calibration, lio_params)),
    ])


def _profile_nodes(context, bringup, map_yaml, calibration, lio_params):
    profile = LaunchConfiguration('profile').perform(context)
    if profile == 'PRODUCTION':
        raise RuntimeError(
            'PRODUCTION 禁止使用 costmap_mid360_smoke.launch.py；请启动正式地图、MapOdomManager、实测标定和真实状态链')
    if profile not in ('TEST_ONLY', 'LEGACY_DEBUG'):
        raise RuntimeError('profile 必须是 TEST_ONLY、LEGACY_DEBUG 或 PRODUCTION')
    nodes = []
    if profile == 'TEST_ONLY':
        # 仅用于 Costmap 消息/障碍层回归；该静态 TF 不得进入生产 Domain。
        nodes.extend([
            Node(package='tf2_ros', executable='static_transform_publisher',
                 name='test_only_map_to_odom', output='screen',
                 arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'odom']),
            Node(package='tf2_ros', executable='static_transform_publisher',
                 name='test_only_odom_to_base_footprint', output='screen',
                 arguments=['0', '0', '0', '0', '0', '0', '1', 'odom', 'base_footprint']),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(
                os.path.join(bringup, 'launch', 'legacy_static_tf.launch.py'))),
        ])
    else:
        nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'local_state.launch.py')),
            launch_arguments={
                'calibration_file': calibration,
                'lio_params_file': lio_params,
                'enable_lio': 'false',
                'enable_mid360_guard': 'false',
                'allow_legacy_static_calibration': 'true',
            }.items()))
    nodes.extend([
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             output='screen', parameters=[{'yaml_filename': map_yaml, 'frame_id': 'map'}],
             remappings=[('map', '/map')]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_map', output='screen',
             parameters=[{'autostart': True, 'node_names': ['map_server']}]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(bringup, 'launch', 'mppi_baseline.launch.py')),
            launch_arguments={
                'enable_mid360_scan': 'true',
                'enable_task_supervisor': 'false',
            }.items()),
    ])
    return nodes
