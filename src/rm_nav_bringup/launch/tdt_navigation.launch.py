"""Production TDT profile: verified Bundle, real TF/state/sensors remain required."""
from pathlib import Path
import importlib.util
import tempfile
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction,IncludeLaunchDescription,RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def start(context):
    share=Path(get_package_share_directory('rm_nav_bringup'))
    spec=importlib.util.spec_from_file_location('rm_nav_bundle_contract',Path(__file__).with_name('map_bundle_contract.py'))
    contract=importlib.util.module_from_spec(spec);spec.loader.exec_module(contract)
    manifest=LaunchConfiguration('map_bundle').perform(context)
    map_profile=LaunchConfiguration('map_profile').perform(context)
    bundle,artifacts=contract.load_published_bundle(manifest,navigation=True,expected_profile=map_profile)
    params=yaml.safe_load(Path(LaunchConfiguration('params_file').perform(context)).read_text())
    controller=params['controller_server']['ros__parameters']
    controller['controller_plugins']=['FollowPath']
    controller['FollowPath']={'plugin':'rm_nav_control::TdtOmniController',
        'map_bundle_version':bundle['version'],'environment_version':bundle['version']+'/costmap',
        'max_speed':0.5,'max_acceleration':1.0,'max_jerk':5.0,'max_yaw_rate':1.0,
        'max_yaw_acceleration':1.5,'max_yaw_jerk':5.0,'solve_budget':0.04,'sample_period':0.02,
        'lookahead':1.0,'replan_period':0.2,'input_timeout':0.25}
    # Actual footprint is an explicit deployment input, not the legacy example.
    footprint=yaml.safe_load(LaunchConfiguration('footprint').perform(context))
    if not isinstance(footprint,list) or not 3<=len(footprint)<=128: raise ValueError('Actual footprint polygon required')
    import math
    if any(not isinstance(p,list) or len(p)!=2 or not all(isinstance(v,(int,float)) and not isinstance(v,bool) and math.isfinite(v) for v in p) for p in footprint):
        raise ValueError('Invalid footprint polygon')
    for name in ('local_costmap','global_costmap'):
        params[name][name]['ros__parameters']['footprint']=str(footprint)
    fd,filename=tempfile.mkstemp(prefix='rm_nav_tdt_',suffix='.yaml')
    import os
    with os.fdopen(fd,'w') as stream: yaml.safe_dump(params,stream)
    def cleanup(_context):
        Path(filename).unlink(missing_ok=True)
        return []
    nav_args={key:LaunchConfiguration(key).perform(context) for key in (
        'use_sim_time','enable_task_supervisor','enable_omni_adapter','enable_physical_output',
        'require_measured_feedback','readiness_topic','enable_mid360_scan','pointcloud_topic')}
    nav_args['params_file']=filename
    return [
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
        Node(package='nav2_map_server',executable='map_server',name='map_server',output='screen',
            parameters=[{'yaml_filename':str(artifacts[bundle['map_yaml']]),'frame_id':'map','use_sim_time':nav_args['use_sim_time']=='true'}]),
        Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='lifecycle_manager_map',
            parameters=[{'autostart':True,'node_names':['map_server']}]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch/frozen_map_localization.launch.py')),
            launch_arguments={'map_bundle':manifest,'map_profile':map_profile,'map_version':bundle['version'],
                'use_sim_time':nav_args['use_sim_time']}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch/mppi_baseline.launch.py')),
            launch_arguments=nav_args.items())]

def generate_launch_description():
    share=Path(get_package_share_directory('rm_nav_bringup'))
    return LaunchDescription([
        DeclareLaunchArgument('map_bundle',description='Published navigation-ready official MapBundle manifest'),
        DeclareLaunchArgument('map_profile',default_value='official',description='Explicit official or local_site map contract'),
        DeclareLaunchArgument('footprint',description='Actual measured polygon [[x,y],...] in metres'),
        DeclareLaunchArgument('params_file',default_value=str(share/'config/mppi_baseline.yaml')),
        DeclareLaunchArgument('use_sim_time',default_value='false'),
        DeclareLaunchArgument('enable_task_supervisor',default_value='true'),
        DeclareLaunchArgument('enable_omni_adapter',default_value='true'),
        DeclareLaunchArgument('enable_physical_output',default_value='false'),
        DeclareLaunchArgument('require_measured_feedback',default_value='true'),
        DeclareLaunchArgument('readiness_topic',default_value='/hardware/drive_ready'),
        DeclareLaunchArgument('enable_mid360_scan',default_value='true'),
        DeclareLaunchArgument('pointcloud_topic',default_value='/sensors/front_mid360/guarded_points'),
        OpaqueFunction(function=start)])
