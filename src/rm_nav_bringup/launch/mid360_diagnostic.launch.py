"""Single sensor diagnostic view; upstream internal extrinsics are unverified.
No chassis/encoder substitutes, navigation, serial or motion command nodes.
"""
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def nodes(context):
    driver_config = Path(LaunchConfiguration('driver_config').perform(context)).resolve(strict=True)
    lio_file = Path(LaunchConfiguration('lio_params_file').perform(context)).resolve(strict=True)
    params = yaml.safe_load(lio_file.read_text())['small_point_lio']['ros__parameters']
    params.update({'lidar_type':'livox_pointcloud2', 'lidar_topic':'/sensors/front_mid360/guarded_points',
                   'imu_topic':'/livox/imu', 'lidar_frame':'front_mid360', 'state_frame':'front_mid360_imu',
                   'odom_frame':'odom', 'publish_tf':False, 'acc_norm':1.0, 'save_pcd':False, 'max_distance':30.0})
    driver = Node(package='livox_ros_driver2', executable='livox_ros_driver2_node',
                  parameters=[{'xfer_format':0, 'multi_topic':0, 'data_src':0, 'publish_freq':10.0,
                               'output_data_type':0, 'frame_id':'front_mid360', 'user_config_path':str(driver_config)}],
                  output='screen')
    guard = Node(package='rm_nav_sensors', executable='mid360_cloud_guard', output='screen')
    lio = Node(package='small_point_lio', executable='small_point_lio_node', parameters=[params],
               remappings=[('/Odometry','/lio/sensor_odometry'),('/cloud_registered','/lio/deskewed_odom_cloud')],
               output='screen')
    processes = [driver,guard,lio]
    # If a component actually exits, stop the entire diagnostic session.
    handlers = [RegisterEventHandler(OnProcessExit(target_action=p, on_exit=[
        EmitEvent(event=Shutdown(reason='diagnostic component exited'))])) for p in processes]
    return handlers+processes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('driver_config',description='Absolute MID360 host/device JSON'),
        DeclareLaunchArgument('lio_params_file',description='Pinned upstream MID360 yaml; internal extrinsics remain diagnostic only'),
        DeclareLaunchArgument('port',default_value='8765'),
        OpaqueFunction(function=nodes)])
