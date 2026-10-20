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
    scan_adapter = Node(
        package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
        name='mid360_pointcloud_to_scan', output='screen',
        condition=IfCondition(LaunchConfiguration('enable_mid360_scan')),
        parameters=[{
            'target_frame': LaunchConfiguration('scan_target_frame'),
            'transform_tolerance': 0.05,
            'min_height': ParameterValue(LaunchConfiguration('scan_min_height'), value_type=float),
            'max_height': ParameterValue(LaunchConfiguration('scan_max_height'), value_type=float),
            'angle_min': -3.141592653589793,
            'angle_max': 3.141592653589793,
            'angle_increment': 0.008726646259971648,
            'scan_time': 0.1,
            'range_min': 0.15,
            'range_max': 30.0,
            'use_inf': True,
            'inf_epsilon': 1.0,
        }],
        remappings=[
            ('cloud_in', LaunchConfiguration('pointcloud_topic')),
            ('scan', LaunchConfiguration('scan_topic')),
        ])
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
        remappings=[('/state/chassis_healthy', LaunchConfiguration('state_health_topic')),
                    ('/odom', LaunchConfiguration('odom_topic'))],
        condition=IfCondition(LaunchConfiguration('enable_task_supervisor'))))
    nodes.append(Node(
        package='rm_nav_control', executable='motion_gate', name='motion_gate',
        output='screen', parameters=[{'use_sim_time': sim_time, 'base_frame': LaunchConfiguration('navigation_frame')}],
        remappings=[('/state/chassis_healthy', LaunchConfiguration('state_health_topic'))]))
    nodes.append(Node(
        package='rm_nav_control', executable='omni_pid_adapter', name='omni_pid_adapter',
        output='screen', condition=IfCondition(LaunchConfiguration('enable_omni_adapter')),
        parameters=[LaunchConfiguration('omni_params_file'), {'use_sim_time': sim_time,
                     'enable_physical_output': ParameterValue(LaunchConfiguration('enable_physical_output'), value_type=bool),
                     'readiness_topic': LaunchConfiguration('readiness_topic'),
                     'require_measured_feedback': ParameterValue(LaunchConfiguration('require_measured_feedback'), value_type=bool)}]))
    nodes.append(Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen',
        parameters=[{'use_sim_time': sim_time, 'autostart': autostart,
                     'node_names': [name for _, name, _ in specs]}]))
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('navigation_frame', default_value='base_link'),
        DeclareLaunchArgument('state_health_topic', default_value='/state/chassis_healthy'),
        DeclareLaunchArgument('odom_topic', default_value='/odom'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('enable_task_supervisor', default_value='true'),
        DeclareLaunchArgument('enable_omni_adapter', default_value='false',
                              description='启动 Omni PID/YawManager 到 my_serial_py 的安全适配器'),
        DeclareLaunchArgument('enable_physical_output', default_value='false',
                              description='必须在真实反馈、急停和人工批准后才允许 true'),
        DeclareLaunchArgument('omni_params_file', default_value=os.path.join(
            get_package_share_directory('rm_nav_bringup'), 'config', 'omni_pid_adapter.yaml')),
        DeclareLaunchArgument('require_measured_feedback', default_value='true',
                              description='false 使用纯速度前馈；不伪造轮速，drive_ready 和真实 odom 仍必需'),
        DeclareLaunchArgument('readiness_topic',default_value='/hardware/drive_ready',
                              description='Legacy transport-only commissioning may explicitly use /hardware/serial_healthy; this is not measured drive readiness'),
        DeclareLaunchArgument(
            'enable_mid360_scan', default_value='false',
            description='启用 MID-360 PointCloud2 到 LaserScan 适配；需要已验证 TF'),
        DeclareLaunchArgument(
            'pointcloud_topic', default_value='/sensors/front_mid360/guarded_points'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('scan_target_frame', default_value='base_link'),
        DeclareLaunchArgument('scan_min_height', default_value='-0.25'),
        DeclareLaunchArgument('scan_max_height', default_value='0.35'),
        scan_adapter,
        *nodes,
    ])
