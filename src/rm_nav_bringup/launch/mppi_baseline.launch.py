"""MPPI Omni baseline; map, localization and observations come from external nodes.

The output is isolated at /nav/cmd_vel_safe (TwistStamped). Hardware adapters
must explicitly subscribe after validating the serial yaw/frame contract.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('rm_nav_bringup'), 'config', 'mppi_baseline.yaml')
    params = LaunchConfiguration('params_file')
    sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    autostart = ParameterValue(LaunchConfiguration('autostart'), value_type=bool)
    specs = [
        ('nav2_controller', 'controller_server', [('cmd_vel', '/nav/cmd_vel_raw')]),
        ('nav2_planner', 'planner_server', []),
        ('nav2_behaviors', 'behavior_server', [('cmd_vel', '/nav/cmd_vel_raw')]),
        ('nav2_bt_navigator', 'bt_navigator', []),
        ('nav2_velocity_smoother', 'velocity_smoother', [
            ('cmd_vel', '/nav/cmd_vel_raw'),
            ('cmd_vel_smoothed', '/nav/cmd_vel_smoothed')]),
        ('nav2_collision_monitor', 'collision_monitor', []),
    ]
    nodes = [Node(package=pkg, executable=name, name=name, output='screen',
                  parameters=[params, {'use_sim_time': sim_time}], remappings=remaps)
             for pkg, name, remaps in specs]
    nodes.append(Node(
        package='rm_nav_bringup', executable='task_supervisor', name='task_supervisor',
        output='screen', parameters=[{'use_sim_time': sim_time}],
        condition=IfCondition(LaunchConfiguration('enable_task_supervisor'))))
    nodes.append(Node(
        package='rm_nav_control', executable='motion_gate', name='motion_gate',
        output='screen', parameters=[{'use_sim_time': sim_time}]))
    nodes.append(Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen',
        parameters=[{'use_sim_time': sim_time, 'autostart': autostart,
                     'node_names': [name for _, name, _ in specs]}]))
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('enable_task_supervisor', default_value='true'),
        *nodes,
    ])
