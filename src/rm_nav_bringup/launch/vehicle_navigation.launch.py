"""Select exactly one control backend; local_state/driver remain explicit inputs."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction,IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def start(context):
    share=Path(get_package_share_directory('rm_nav_bringup'))
    values={name:LaunchConfiguration(name).perform(context) for name in (
        'map_bundle','footprint','enable_physical_output','require_measured_feedback','readiness_topic')}
    backend=LaunchConfiguration('control_backend').perform(context)
    if backend not in ('direct_omni','ros2_control'):raise ValueError('Unknown control backend')
    if backend=='ros2_control' and (values['require_measured_feedback']!='true' or values['readiness_topic']!='/hardware/drive_ready'):
        raise ValueError('ros2_control requires actual measured drive feedback/readiness')
    values['enable_omni_adapter']=str(backend=='direct_omni').lower()
    actions=[IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch/tdt_navigation.launch.py')),launch_arguments=values.items())]
    if backend=='ros2_control':
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch/ros2_control_backend.launch.py')),
            launch_arguments={'enable_physical_output':values['enable_physical_output']}.items()))
    if LaunchConfiguration('start_serial').perform(context)=='true':
        if values['enable_physical_output']!='true':raise ValueError('Serial opening requires explicit physical-output selection')
        actions.append(Node(package='my_serial_py',executable='serial_node',parameters=[{
            'serial_port':LaunchConfiguration('serial_port').perform(context)}],output='screen'))
    return actions

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map_bundle'),DeclareLaunchArgument('footprint'),
        DeclareLaunchArgument('control_backend',default_value='direct_omni'),
        DeclareLaunchArgument('enable_physical_output',default_value='false'),
        DeclareLaunchArgument('require_measured_feedback',default_value='true'),
        DeclareLaunchArgument('readiness_topic',default_value='/hardware/drive_ready'),
        DeclareLaunchArgument('start_serial',default_value='false'),
        DeclareLaunchArgument('serial_port',default_value='/dev/ttyUSB0'),OpaqueFunction(function=start)])
