"""Raw upstream LIO -> time-aligned SE3 resolver -> upstream robot_localization."""
import math
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition


def nodes(context):
    with open(LaunchConfiguration('calibration_file').perform(context),encoding='utf-8') as handle:
        bundle=yaml.safe_load(handle)
    if bundle.get('status')!='verified' or not bundle.get('bundle_id'):
        raise ValueError('Measured CalibrationBundle with verified status required')
    frames=bundle['frames']
    keys=('base_footprint','chassis','big_gimbal_yaw','imu','lidar')
    if any(not frames.get(k) for k in keys) or len({frames[k] for k in keys})!=5 or 'base_link' in {frames[k] for k in keys}:
        raise ValueError('Distinct calibration frames required')
    transforms=bundle['transforms']
    tf_params={name:transforms[name]['translation']+transforms[name]['quaternion_xyzw'] for name in
               ('footprint_to_chassis','chassis_to_yaw_zero','yaw_to_imu','imu_to_lidar')}
    for values in tf_params.values():
        if len(values)!=7 or not all(math.isfinite(v) for v in values) or abs(sum(v*v for v in values[3:])-1)>1e-5:
            raise ValueError('Finite measured transforms with unit quaternions required')
    x,y,z,w=transforms['imu_to_lidar']['quaternion_xyzw']
    rotation=[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w),
              2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w),
              2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]
    sim=ParameterValue(LaunchConfiguration('use_sim_time'),value_type=bool)
    ekf={'use_sim_time':sim,'frequency':50.0,'sensor_timeout':0.2,'two_d_mode':False,
         'publish_tf':True,'map_frame':'map','odom_frame':'odom',
         'base_link_frame':frames['base_footprint'],'world_frame':'odom',
         'pose0':'/state/lio_pose','pose0_config':[True]*6+[False]*9,
         'pose0_differential':False,'pose0_relative':False,'pose0_queue_size':5,
         'print_diagnostics':True}
    if LaunchConfiguration('enable_wheel_odometry').perform(context)=='true':
        ekf.update({'odom0':'/hardware/wheel_odometry','odom0_config':
                   [False]*6+[True,True,False,False,False,True]+[False]*3})
    if LaunchConfiguration('enable_chassis_imu').perform(context)=='true':
        ekf.update({'imu0':'/hardware/chassis_imu','imu0_config':
                   [False]*9+[True,True,True]+[False]*3})
    return [
        Node(package='rm_nav_bringup',executable='gimbal_tf',parameters=[tf_params,{
            'use_sim_time':sim,'calibration_id':bundle['bundle_id'],
            'footprint_frame':frames['base_footprint'],'chassis_frame':frames['chassis'],
            'yaw_frame':frames['big_gimbal_yaw'],'imu_frame':frames['imu'],'lidar_frame':frames['lidar'],
            'yaw_sign':float(bundle['encoder']['sign']),'yaw_offset':float(bundle['encoder']['zero_offset']),
            'yaw_joint':bundle['encoder']['joint']}],output='screen'),
        Node(package='small_point_lio',executable='small_point_lio_node',name='small_point_lio',
             condition=IfCondition(LaunchConfiguration('enable_lio')),
             parameters=[LaunchConfiguration('lio_params_file'),{'use_sim_time':sim,
                 'publish_tf':False,'odom_frame':'odom','state_frame':frames['imu'],'lidar_frame':frames['lidar'],
                 'extrinsic_est_en':False,'extrinsic_T':transforms['imu_to_lidar']['translation'],
                 'extrinsic_R':rotation}],
             remappings=[('/Odometry','/lio/sensor_odometry'),('/cloud_registered','/lio/deskewed_odom_cloud')],output='screen'),
        Node(package='rm_nav_localization',executable='sensor_pose_resolver',parameters=[{
            'use_sim_time':sim,'calibration_id':bundle['bundle_id'],'sensor_frame':frames['imu'],
            'body_frame':frames['base_footprint'],'require_encoder_health':True}],output='screen'),
        Node(package='robot_localization',executable='ekf_node',name='chassis_ekf',parameters=[ekf],
             remappings=[('odometry/filtered','/state/chassis')],output='screen'),
        Node(package='rm_nav_localization',executable='chassis_state_bridge',parameters=[{
            'use_sim_time':sim,'calibration_id':bundle['bundle_id'],
            'state_reference_frame':frames['base_footprint']}],output='screen'),
        Node(package='rm_nav_sensors',executable='lio_observation_adapter',parameters=[{
            'use_sim_time':sim,'calibration_id':bundle['bundle_id'],'sensor_frame':frames['lidar']}],output='screen'),
        Node(package='rm_nav_localization',executable='observation_submap',parameters=[{
            'use_sim_time':sim,'calibration_id':bundle['bundle_id'],'sensor_frame':frames['lidar']}],output='screen')]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('calibration_file',description='Verified measured calibration YAML'),
        DeclareLaunchArgument('lio_params_file',description='Driver topics, timing and filter parameters'),
        DeclareLaunchArgument('use_sim_time',default_value='false'),
        DeclareLaunchArgument('enable_lio',default_value='true'),
        DeclareLaunchArgument('enable_wheel_odometry',default_value='false'),
        DeclareLaunchArgument('enable_chassis_imu',default_value='false'),
        OpaqueFunction(function=nodes)])
