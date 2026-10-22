"""Pinned ZLIO frontend, official Livox PointCloud2 absolute nanosecond timestamps."""
import importlib.util
import os
from pathlib import Path
import tempfile
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start(context):
    spec = importlib.util.spec_from_file_location('zconfig', Path(__file__).with_name('z_localization_config.py'))
    cfg = importlib.util.module_from_spec(spec); spec.loader.exec_module(cfg)
    value = lambda name: LaunchConfiguration(name).perform(context)
    configuration = cfg.frontend_config(cfg.load(Path(get_package_share_directory('frontend'))/'config/lio.yaml'))
    configuration['sensor']['body_from_lidar'] = {
        'translation': yaml.safe_load(value('imu_to_lidar_translation')),
        'rotation_xyzw': yaml.safe_load(value('imu_to_lidar_quaternion'))}
    fd, filename = tempfile.mkstemp(prefix='rm_zlio_', suffix='.yaml')
    with os.fdopen(fd, 'w') as stream:
        yaml.safe_dump(configuration, stream)
    sim = LaunchConfiguration('use_sim_time').perform(context) == 'true'
    def cleanup(_context):
        Path(filename).unlink(missing_ok=True)
        return []
    return [
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
        Node(package='frontend', executable='zlio_node', name='zlio', output='screen', parameters=[{
            'config_file': filename, 'input_type': 'pointcloud2',
            'lidar_topic': '/lio/input/cloud', 'imu_topic': '/lio/input/imu',
            'imu_acceleration_scale': 9.81, 'point_time_field': 'timestamp',
            'point_time_scale': 1.e-9, 'point_time_absolute': True,
            'world_frame': 'odom', 'body_frame': value('sensor_frame'),
            'publish_tf': False, 'coordinated_reset': True, 'reset_mode': 'warm',
            'use_sim_time': sim}]),
        Node(package='rm_nav_bringup', executable='z_localization_bridge', output='screen',
             parameters=[{'use_sim_time': sim, 'role': 'frontend', 'sensor_frame': value('sensor_frame')}]),
    ]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('sensor_frame', default_value='front_mid360_imu'),
        DeclareLaunchArgument('imu_to_lidar_translation', default_value='[-0.011,-0.02329,0.04412]'),
        DeclareLaunchArgument('imu_to_lidar_quaternion', default_value='[0.0,0.0,0.0,1.0]'),
        OpaqueFunction(function=start)])
