"""External frozen-map inputs; optionally require a published aligned MapBundle."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def nodes(context):
    import json
    from pathlib import Path
    bundle_path=LaunchConfiguration('map_bundle').perform(context)
    map_version=LaunchConfiguration('map_version').perform(context)
    if bundle_path:
        bundle=json.loads(Path(bundle_path).resolve(strict=True).read_text())
        if (bundle.get('status')!='published' or bundle.get('coordinate_frame')!='map' or
                bundle.get('official_alignment_applied') is not True or
                bundle.get('publishable_for_frozen_localization') is not True or
                bundle.get('version')!=map_version):
            raise ValueError('MapBundle is not an aligned published map or version does not match')
    parameters={'map_version':ParameterValue(LaunchConfiguration('map_version'),value_type=str),
                'use_sim_time':ParameterValue(LaunchConfiguration('use_sim_time'),value_type=bool)}
    return [
        Node(package='rm_nav_localization',executable='frozen_map_matcher',name='frozen_map_matcher',parameters=[parameters],output='screen'),
        Node(package='rm_nav_localization',executable='map_odom_manager',name='map_odom_manager',parameters=[parameters,{
            'enable_recovery':ParameterValue(LaunchConfiguration('enable_recovery'),value_type=bool),
            'enable_recovery_transaction':ParameterValue(LaunchConfiguration('enable_recovery_transaction'),value_type=bool),
            'enable_auto_recovery':ParameterValue(LaunchConfiguration('enable_auto_recovery'),value_type=bool),
            'recovery_timeout':ParameterValue(LaunchConfiguration('recovery_timeout'),value_type=float),
            'field_bounds':ParameterValue(LaunchConfiguration('field_bounds'))}],output='screen')]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map_version',description='Frozen MapBundle version or content hash'),
        DeclareLaunchArgument('map_bundle',default_value='',description='Optional published MapBundle manifest; draft maps are rejected'),
        DeclareLaunchArgument('use_sim_time',default_value='false'),
        DeclareLaunchArgument('enable_recovery',default_value='false'),
        DeclareLaunchArgument('enable_recovery_transaction',default_value='false'),
        DeclareLaunchArgument('enable_auto_recovery',default_value='false'),
        DeclareLaunchArgument('recovery_timeout',default_value='5.0'),
        DeclareLaunchArgument('field_bounds',default_value='[0.0,0.0,0.0,0.0]',description='Measured [min_x,max_x,min_y,max_y]; required for recovery'),
        OpaqueFunction(function=nodes)])
