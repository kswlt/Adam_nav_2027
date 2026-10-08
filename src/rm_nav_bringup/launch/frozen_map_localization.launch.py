"""External frozen-map/deskewed odom-submap inputs; sole map->odom authority."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    parameters = {
        'map_version': ParameterValue(LaunchConfiguration('map_version'), value_type=str),
        'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
    }
    return LaunchDescription([
        DeclareLaunchArgument('map_version', description='Frozen MapBundle version or content hash'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='rm_nav_localization', executable='frozen_map_matcher',
             name='frozen_map_matcher', parameters=[parameters], output='screen'),
        Node(package='rm_nav_localization', executable='map_odom_manager',
             name='map_odom_manager', parameters=[parameters], output='screen'),
    ])
