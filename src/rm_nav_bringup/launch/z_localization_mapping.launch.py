"""Complete RM mapping state + ZLIO + online loop/graph/map. No serial or Nav2."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = Path(get_package_share_directory('rm_nav_bringup'))/'launch'
    def include(name, args):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/name)), launch_arguments=args.items())
    sim = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('save_directory', description='New absolute native Z map output directory'),
        include('vehicle_164_sensor_reference_state.launch.py', {'use_sim_time': sim}),
        include('z_localization_backend.launch.py', {'use_sim_time': sim, 'mode': 'mapping',
                'save_directory': LaunchConfiguration('save_directory')}),
    ])
