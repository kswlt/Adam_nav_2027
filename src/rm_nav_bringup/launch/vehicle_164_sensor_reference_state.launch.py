"""Vehicle164 user-approved 15.34cm fit; no chassis pose or field verification is invented."""
import json
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share=Path(get_package_share_directory('rm_nav_bringup'))
    geometry=yaml.safe_load((share/'config/vehicle_164_geometry.yaml').read_text(encoding='utf-8'))
    mount=geometry['mount']
    if (mount.get('horizontal_geometry_user_confirmed') is not True or
            mount.get('axis_reference')!='point_on_yaw_axis_at_lidar_height'):
        raise ValueError('Missing user-confirmed horizontal yaw-axis geometry')
    return LaunchDescription([
        DeclareLaunchArgument('lio_params_file',default_value='',description='LEGACY_DEBUG only; ZLIO uses pinned upstream configuration'),
        DeclareLaunchArgument('legacy_lio',default_value='false'),
        DeclareLaunchArgument('use_sim_time',default_value='false'),
        DeclareLaunchArgument('enable_lio',default_value='true'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch/sensor_reference_state.launch.py')),
            launch_arguments={
                'lio_params_file':LaunchConfiguration('lio_params_file'),
                'use_sim_time':LaunchConfiguration('use_sim_time'),
                'enable_lio':LaunchConfiguration('enable_lio'),
                'legacy_lio':LaunchConfiguration('legacy_lio'),
                'reference_point':'gimbal_axis',
                'gimbal_axis_to_lidar':json.dumps(mount['axis_to_lidar_at_reference_height']),
                'reference_geometry_verified':str(mount['reference_geometry_verified']).lower(),
                'reference_fit_debug_approved':str(mount['reference_fit_debug_approved']).lower(),
                'calibration_id':mount['reference_calibration_id']}.items()),
    ])
