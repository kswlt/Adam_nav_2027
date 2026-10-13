"""临时复用原版 Adam 静态 TF，仅用于 Costmap/Foxglove 联调。

这些数值来自原版 rm_static_tf，不是本车重新测量的 CalibrationBundle。
该启动文件不替代 local_state.launch.py 的 verified 标定门，也不发布动态云台 TF。
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _static(name, xyz, quat, parent, child, condition=None):
    return Node(
        package='tf2_ros', executable='static_transform_publisher', name=name,
        output='screen',condition=condition,
        arguments=[*map(str, xyz), *map(str, quat), parent, child])


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('projection_owner',default_value='false'),
        DeclareLaunchArgument('alias_parent',default_value='base_footprint'),
        _static('legacy_base_footprint_to_chassis',
                (0.0, 0.0, 0.076), (0.0, 0.0, 0.0, 1.0),
                'base_footprint', 'chassis',UnlessCondition(LaunchConfiguration('projection_owner'))),
        _static('legacy_base_footprint_to_base_link',
                (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0),
                LaunchConfiguration('alias_parent'), 'base_link'),
        _static('legacy_chassis_to_front_mid360',
                (0.16, 0.0, 0.18), (1.0, 0.0, 0.0, 0.0),
                'chassis', 'front_mid360'),
        _static('legacy_chassis_to_front_mid360_imu',
                (0.16, 0.0, 0.18), (1.0, 0.0, 0.0, 0.0),
                'chassis', 'front_mid360_imu'),
    ])
