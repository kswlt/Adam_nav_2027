"""Native Z prior localization by default; explicit legacy matcher regression retained."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def nodes(context):
    import importlib.util
    from pathlib import Path
    bundle_path=LaunchConfiguration('map_bundle').perform(context)
    map_version=LaunchConfiguration('map_version').perform(context)
    map_nodes=[]
    legacy=LaunchConfiguration('legacy_localization').perform(context)=='true'
    if not legacy:
        from launch.actions import IncludeLaunchDescription
        from launch.launch_description_sources import PythonLaunchDescriptionSource
        if not bundle_path:
            raise ValueError('Z-Localization requires a published RM MapBundle with native Z priors')
        spec=importlib.util.spec_from_file_location('rm_nav_bundle_contract',Path(__file__).with_name('map_bundle_contract.py'))
        contract=importlib.util.module_from_spec(spec);spec.loader.exec_module(contract)
        bundle,_=contract.load_published_bundle(bundle_path,expected_profile=LaunchConfiguration('map_profile').perform(context))
        if bundle['version']!=map_version: raise ValueError('MapBundle version mismatch')
        spec=importlib.util.spec_from_file_location('zconfig',Path(__file__).with_name('z_localization_config.py'))
        config=importlib.util.module_from_spec(spec);spec.loader.exec_module(config)
        native=config.native_bundle(bundle_path,bundle)
        return [IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(__file__).with_name('z_localization_backend.launch.py'))),
            launch_arguments={'mode':'localization','prior_map':str(native),'map_version':map_version,
                              'use_sim_time':LaunchConfiguration('use_sim_time')}.items())]
    if bundle_path:
        spec=importlib.util.spec_from_file_location('rm_nav_bundle_contract',Path(__file__).with_name('map_bundle_contract.py'))
        contract=importlib.util.module_from_spec(spec);spec.loader.exec_module(contract)
        bundle,artifacts=contract.load_published_bundle(bundle_path,expected_profile=LaunchConfiguration('map_profile').perform(context))
        if bundle['version']!=map_version: raise ValueError('MapBundle version mismatch')
        map_nodes.append(Node(package='rm_nav_bringup',executable='pcd_map_publisher',
            name='frozen_map_bundle_publisher',output='screen',arguments=[
                '--pcd',str(artifacts[bundle['localization_map']]),'--frame','map',
                '--topic','/localization/frozen_map','--rate','0.2']))
    parameters={'map_version':ParameterValue(LaunchConfiguration('map_version'),value_type=str),
                'use_sim_time':ParameterValue(LaunchConfiguration('use_sim_time'),value_type=bool)}
    return map_nodes+[
        Node(package='rm_nav_localization',executable='initial_pose_hint',parameters=[parameters, {'body_frame':LaunchConfiguration('hint_body_frame')}],output='screen'),
        Node(package='rm_nav_localization',executable='frozen_map_matcher',name='frozen_map_matcher',parameters=[parameters],output='screen'),
        Node(package='rm_nav_localization',executable='map_odom_manager',name='map_odom_manager',parameters=[parameters,{
            'tracking_frame':LaunchConfiguration('tracking_frame'),
            'enable_recovery':ParameterValue(LaunchConfiguration('enable_recovery'),value_type=bool),
            'enable_recovery_transaction':ParameterValue(LaunchConfiguration('enable_recovery_transaction'),value_type=bool),
            'enable_auto_recovery':ParameterValue(LaunchConfiguration('enable_auto_recovery'),value_type=bool),
            'recovery_timeout':ParameterValue(LaunchConfiguration('recovery_timeout'),value_type=float),
            'field_bounds':ParameterValue(LaunchConfiguration('field_bounds'))}],
            remappings=[('/odom',LaunchConfiguration('odom_topic')),
                        ('/state/chassis_healthy',LaunchConfiguration('state_health_topic'))],output='screen')]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('legacy_localization',default_value='false',description='LEGACY_DEBUG only: old frozen matcher/MapOdomManager regression'),
        DeclareLaunchArgument('tracking_frame',default_value='base_link'),
        DeclareLaunchArgument('hint_body_frame',default_value='base_footprint'),
        DeclareLaunchArgument('odom_topic',default_value='/odom'),
        DeclareLaunchArgument('state_health_topic',default_value='/state/chassis_healthy'),
        DeclareLaunchArgument('map_version',description='Frozen MapBundle version or content hash'),
        DeclareLaunchArgument('map_bundle',default_value='',description='Optional published MapBundle manifest; draft maps are rejected'),
        DeclareLaunchArgument('map_profile',default_value='official',description='Explicit official or local_site map contract'),
        DeclareLaunchArgument('use_sim_time',default_value='false'),
        DeclareLaunchArgument('enable_recovery',default_value='false'),
        DeclareLaunchArgument('enable_recovery_transaction',default_value='false'),
        DeclareLaunchArgument('enable_auto_recovery',default_value='false'),
        DeclareLaunchArgument('recovery_timeout',default_value='5.0'),
        DeclareLaunchArgument('field_bounds',default_value='[0.0,0.0,0.0,0.0]',description='Measured [min_x,max_x,min_y,max_y]; required for recovery'),
        OpaqueFunction(function=nodes)])
