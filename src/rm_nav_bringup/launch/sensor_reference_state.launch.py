"""Sensor-axis navigation state. No chassis TF, encoder, EKF or serial output."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    sim = {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}
    health = [('/state/chassis_healthy', '/state/sensor_reference_healthy')]
    calibration = 'vehicle164_sensor_axis_factory_v1'
    return LaunchDescription([
        DeclareLaunchArgument('lio_params_file', description='small_point_lio MID360 driver/time parameters'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('enable_lio', default_value='true'),
        Node(package='rm_nav_sensors', executable='mid360_cloud_guard', parameters=[sim], output='screen'),
        Node(package='rm_nav_bringup', executable='lio_input_guard', parameters=[sim], output='screen'),
        Node(package='small_point_lio', executable='small_point_lio_node', name='small_point_lio',
             condition=IfCondition(LaunchConfiguration('enable_lio')),
             parameters=[LaunchConfiguration('lio_params_file'), sim, {
                 'publish_tf': False, 'odom_frame': 'odom', 'state_frame': 'front_mid360_imu',
                 'lidar_frame': 'front_mid360', 'lidar_type': 'livox_pointcloud2',
                 'lidar_topic': '/lio/input/cloud', 'imu_topic': '/lio/input/imu',
                 'extrinsic_est_en': False, 'extrinsic_T': [-.011, -.02329, .04412],
                 'extrinsic_R': [1., 0., 0., 0., 1., 0., 0., 0., 1.]}],
             remappings=[('/Odometry', '/lio/sensor_odometry'),
                         ('/cloud_registered', '/lio/deskewed_odom_cloud')], output='screen'),
        Node(package='rm_nav_bringup', executable='sensor_reference', parameters=[sim], output='screen'),
        Node(package='rm_nav_sensors', executable='lio_observation_adapter',
             parameters=[sim, {'calibration_id': calibration, 'sensor_frame': 'front_mid360'}],
             remappings=health, output='screen'),
        Node(package='rm_nav_localization', executable='observation_submap',
             parameters=[sim, {'calibration_id': calibration, 'sensor_frame': 'front_mid360'}],
             remappings=health, output='screen'),
    ])
