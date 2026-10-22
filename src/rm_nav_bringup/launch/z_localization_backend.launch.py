"""Z-Localization owns map->odom; no old matcher/MapOdomManager/legacy TF."""
import importlib.util
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    spec = importlib.util.spec_from_file_location('zconfig', Path(__file__).with_name('z_localization_config.py'))
    cfg = importlib.util.module_from_spec(spec); spec.loader.exec_module(cfg)
    config = cfg.backend_config(cfg.load(Path(get_package_share_directory('backend'))/'config/backend.yaml'),
                                value('mode'), value('prior_map'), value('save_directory'))
    params = cfg.flatten(config)
    params['use_sim_time'] = value('use_sim_time') == 'true'
    return [Node(package='backend', executable='backend_node', name='backend_node', output='screen', parameters=[params]),
            Node(package='rm_nav_bringup', executable='z_localization_bridge', name='z_localization_health',
                 output='screen', parameters=[{'role': 'backend', 'mode': value('mode'),
                    'map_version': value('map_version'), 'use_sim_time': params['use_sim_time']}])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='mapping'),
        DeclareLaunchArgument('prior_map', default_value=''),
        DeclareLaunchArgument('save_directory', default_value=str(Path.home()/'adam_z_maps/session')),
        DeclareLaunchArgument('map_version', default_value='unpublished_mapping_session'),
        DeclareLaunchArgument('use_sim_time', default_value='false'), OpaqueFunction(function=start)])
