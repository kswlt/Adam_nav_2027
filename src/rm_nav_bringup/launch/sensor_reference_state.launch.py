"""Sensor-axis navigation state. No chassis TF, encoder, EKF or serial output."""
from launch import LaunchDescription
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    sim = {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}
    health = [('/state/chassis_healthy', '/state/sensor_reference_healthy')]
    calibration = ParameterValue(LaunchConfiguration('calibration_id'), value_type=str)
    return LaunchDescription([
        DeclareLaunchArgument('lio_params_file', default_value='', description='LEGACY_DEBUG only: small_point_lio parameters'),
        DeclareLaunchArgument('legacy_lio', default_value='false', description='Explicit historical small_point_lio regression only'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('enable_lio', default_value='true'),
        DeclareLaunchArgument('reference_point', default_value='lidar_center'),
        DeclareLaunchArgument('gimbal_axis_to_lidar', default_value='[0.0,0.0,0.0]',
                              description='Measured or approved fit axis->LiDAR in sensor axes, NOT chassis->axis'),
        DeclareLaunchArgument('reference_geometry_verified', default_value='false'),
        DeclareLaunchArgument('reference_fit_debug_approved', default_value='false',
                              description='Explicit operator approval of recorded-fit geometry; requires debug calibration ID'),
        DeclareLaunchArgument('calibration_id', default_value='vehicle164_sensor_axis_factory_v1'),
        DeclareLaunchArgument('heading_max_extrapolation', default_value='0.12'),
        DeclareLaunchArgument('heading_max_correction', default_value='0.25'),
        Node(package='rm_nav_sensors', executable='mid360_cloud_guard', parameters=[sim], output='screen'),
        Node(package='rm_nav_bringup', executable='lio_input_guard', parameters=[sim], output='screen'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(get_package_share_directory('rm_nav_bringup'))/'launch/z_localization_frontend.launch.py')),
             condition=IfCondition(PythonExpression(["'", LaunchConfiguration('enable_lio'), "' == 'true' and '", LaunchConfiguration('legacy_lio'), "' != 'true'"])),
             launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()),
        Node(package='small_point_lio', executable='small_point_lio_node', name='small_point_lio',
             condition=IfCondition(PythonExpression(["'", LaunchConfiguration('enable_lio'), "' == 'true' and '", LaunchConfiguration('legacy_lio'), "' == 'true'"])),
             parameters=[LaunchConfiguration('lio_params_file'), sim, {
                 'publish_tf': False, 'odom_frame': 'odom', 'state_frame': 'front_mid360_imu',
                 'lidar_frame': 'front_mid360', 'lidar_type': 'livox_pointcloud2',
                 'lidar_topic': '/lio/input/cloud', 'imu_topic': '/lio/input/imu',
                 'extrinsic_est_en': False, 'extrinsic_T': [-.011, -.02329, .04412],
                 'extrinsic_R': [1., 0., 0., 0., 1., 0., 0., 0., 1.]}],
             remappings=[('/Odometry', '/lio/sensor_odometry'),
                         ('/cloud_registered', '/lio/deskewed_odom_cloud')], output='screen'),
        Node(package='rm_nav_bringup', executable='sensor_reference', parameters=[sim, {
            'reference_point': ParameterValue(LaunchConfiguration('reference_point'), value_type=str),
            'gimbal_axis_to_lidar': ParameterValue(LaunchConfiguration('gimbal_axis_to_lidar')),
            'reference_geometry_verified': ParameterValue(LaunchConfiguration('reference_geometry_verified'), value_type=bool),
            'reference_fit_debug_approved': ParameterValue(LaunchConfiguration('reference_fit_debug_approved'), value_type=bool),
            'calibration_id': calibration,
            'heading_max_extrapolation': ParameterValue(LaunchConfiguration('heading_max_extrapolation'), value_type=float),
            'heading_max_correction': ParameterValue(LaunchConfiguration('heading_max_correction'), value_type=float)}], output='screen'),
        Node(package='rm_nav_sensors', executable='lio_observation_adapter',
             parameters=[sim, {'calibration_id': calibration, 'sensor_frame': 'front_mid360'}],
             remappings=health, output='screen'),
        Node(package='rm_nav_localization', executable='observation_submap',
             parameters=[sim, {'calibration_id': calibration, 'sensor_frame': 'front_mid360'}],
             remappings=health, output='screen'),
    ])
