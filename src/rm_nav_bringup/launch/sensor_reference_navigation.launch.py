"""Translation-only TDT/Nav2 shadow profile; requires a real published MapBundle."""
import importlib.util
import math
import os
from pathlib import Path
import tempfile
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start(context):
    share = Path(get_package_share_directory('rm_nav_bringup'))
    spec = importlib.util.spec_from_file_location('bundle_contract', Path(__file__).with_name('map_bundle_contract.py'))
    contract = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(contract)
    manifest = LaunchConfiguration('map_bundle').perform(context)
    bundle, artifacts = contract.load_published_bundle(manifest, navigation=True)
    sim = LaunchConfiguration('use_sim_time').perform(context)
    profile_spec = importlib.util.spec_from_file_location('sensor_profile', Path(__file__).with_name('sensor_reference_profile.py'))
    profile = importlib.util.module_from_spec(profile_spec)
    profile_spec.loader.exec_module(profile)
    params = profile.configure(yaml.safe_load((share/'config/mppi_baseline.yaml').read_text(encoding='utf-8')), bundle['version'], share)
    fd, filename = tempfile.mkstemp(prefix='sensor_nav_', suffix='.yaml')
    with os.fdopen(fd, 'w') as stream:
        yaml.safe_dump(params, stream)
    def cleanup(_context):
        Path(filename).unlink(missing_ok=True)
        return []
    return [
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             parameters=[{'yaml_filename': str(artifacts[bundle['map_yaml']]), 'frame_id': 'map', 'use_sim_time': sim == 'true'}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_map',
             parameters=[{'autostart': True, 'node_names': ['map_server'], 'use_sim_time': sim == 'true'}]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch/frozen_map_localization.launch.py')),
             launch_arguments={'map_bundle': manifest, 'map_version': bundle['version'], 'use_sim_time': sim,
                 'tracking_frame': 'nav_reference', 'hint_body_frame': 'nav_reference',
                 'odom_topic': '/sensor_reference/odom', 'state_health_topic': '/state/sensor_reference_healthy'}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch/mppi_baseline.launch.py')),
             launch_arguments={'params_file': filename, 'use_sim_time': sim,
                 'navigation_frame': 'nav_reference', 'state_health_topic': '/state/sensor_reference_healthy',
                 'odom_topic': '/sensor_reference/odom', 'enable_omni_adapter': 'false',
                 'enable_physical_output': 'false', 'enable_mid360_scan': 'true',
                 'pointcloud_topic': '/lio/deskewed_odom_cloud', 'scan_target_frame': 'nav_reference',
                 'scan_min_height': LaunchConfiguration('scan_min_height').perform(context),
                 'scan_max_height': LaunchConfiguration('scan_max_height').perform(context)}.items()),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map_bundle', description='Official aligned published navigation-ready manifest'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('scan_min_height', default_value='-0.10', description='Relative to odom z=0, tune against actual ground returns'),
        DeclareLaunchArgument('scan_max_height', default_value='1.50'),
        OpaqueFunction(function=start),
    ])
