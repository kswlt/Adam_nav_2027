"""Explicit mapping-only archive process; does not modify match-mode map TF."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('archive_root',description='Absolute persistent archive root; new session per run'),
        DeclareLaunchArgument('calibration_id',description='Same measured calibration ID as local_state'),
        DeclareLaunchArgument('body_frame',default_value='base_footprint'),
        DeclareLaunchArgument('sensor_frame',default_value='front_mid360'),
        DeclareLaunchArgument('use_sim_time',default_value='false'),
        Node(package='rm_nav_mapping',executable='keyframe_recorder',parameters=[{
            'archive_root':LaunchConfiguration('archive_root'),
            'calibration_id':LaunchConfiguration('calibration_id'),
            'body_frame':LaunchConfiguration('body_frame'),
            'sensor_frame':LaunchConfiguration('sensor_frame'),
            'use_sim_time':ParameterValue(LaunchConfiguration('use_sim_time'),value_type=bool)}],output='screen')])
