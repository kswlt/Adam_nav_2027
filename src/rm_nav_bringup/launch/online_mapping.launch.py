"""Attach a bounded online backend to one immutable-keyframe recorder session.

No sensor driver, control output, or map/odom TF publisher is started here.
The recorder may remain active: only atomically committed frame directories are read.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    args = [DeclareLaunchArgument('session', description='Absolute path to this recorder session'),
            DeclareLaunchArgument('calibration_id', description='Expected recorder calibration ID')]
    defaults = {'max_keyframes': ('500', int), 'max_cloud_points': ('20000', int),
                'max_map_points': ('500000', int), 'max_candidates': ('2', int),
                'map_voxel_m': ('0.15', float), 'loop_budget_s': ('3.0', float),
                'min_loop_time_s': ('10.0', float), 'min_loop_frames': ('10', int),
                'use_sim_time': ('false', bool)}
    params = {name: LaunchConfiguration(name) for name in ('session', 'calibration_id')}
    for name, (default, typ) in defaults.items():
        args.append(DeclareLaunchArgument(name, default_value=default))
        params[name] = ParameterValue(LaunchConfiguration(name), value_type=typ)
    return LaunchDescription(args + [Node(package='rm_nav_mapping', executable='online_mapping',
                                         parameters=[params], output='screen')])
