"""Raw upstream LIO -> time-aligned SE3 resolver -> upstream robot_localization."""
import math
import os
import json
from ament_index_python.packages import get_package_share_directory
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition


def nodes(context):
    with open(LaunchConfiguration('calibration_file').perform(context),encoding='utf-8') as handle:
        bundle=yaml.safe_load(handle)
    allow_legacy = LaunchConfiguration('allow_legacy_static_calibration').perform(context) == 'true'
    legacy = allow_legacy and bundle.get('status') == 'legacy_static'
    fixed=bundle.get('sensor_mount')=='fixed_chassis'
    if legacy and fixed:raise ValueError('Legacy and fixed-chassis profiles cannot be combined')
    user_mount=(fixed and bundle.get('status')=='user_provided_factory_internal' and
        LaunchConfiguration('allow_user_mount').perform(context)=='true')
    if bundle.get('status') != 'verified' and not legacy and not user_mount:
        raise ValueError('Measured CalibrationBundle with verified status required')
    if allow_legacy and bundle.get('status') == 'legacy_static':
        print('[WARN] 使用原版 Adam 静态外参，仅用于联调；禁止作为实车标定验收')
    if not bundle.get('bundle_id'):
        raise ValueError('Calibration bundle_id required')
    frames=bundle['frames']
    keys=('base_footprint','chassis','imu','lidar') if fixed else ('base_footprint','chassis','big_gimbal_yaw','imu','lidar')
    if any(not frames.get(k) for k in keys) or len({frames[k] for k in keys})!=len(keys) or 'base_link' in {frames[k] for k in keys}:
        raise ValueError('Distinct calibration frames required')
    transforms=bundle['transforms']
    names=('chassis_to_imu','imu_to_lidar') if fixed else ('footprint_to_chassis','chassis_to_yaw_zero','yaw_to_imu','imu_to_lidar')
    tf_params={name:transforms[name]['translation']+transforms[name]['quaternion_xyzw'] for name in names}
    for values in tf_params.values():
        if len(values)!=7 or not all(math.isfinite(v) for v in values) or abs(sum(v*v for v in values[3:])-1)>1e-5:
            raise ValueError('Finite measured transforms with unit quaternions required')
    x,y,z,w=transforms['imu_to_lidar']['quaternion_xyzw']
    rotation=[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w),
              2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w),
              2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]
    sim=ParameterValue(LaunchConfiguration('use_sim_time'),value_type=bool)
    z_frontend = (LaunchConfiguration('enable_lio').perform(context)=='true' and
                  LaunchConfiguration('legacy_lio').perform(context)!='true')
    raw_guard = LaunchConfiguration('enable_mid360_guard').perform(context)=='true' or z_frontend
    lio_input = {'lidar_type':'livox_pointcloud2',
                 'lidar_topic':'/sensors/front_mid360/guarded_points'} if raw_guard else {}
    ekf={'use_sim_time':sim,'frequency':50.0,'sensor_timeout':0.2,'two_d_mode':False,
         'publish_tf':False,'map_frame':'map','odom_frame':'odom',
         'base_link_frame':frames['chassis'],'world_frame':'odom',
         'pose0':'/state/lio_pose','pose0_config':[True]*6+[False]*9,
         'pose0_differential':False,'pose0_relative':False,'pose0_queue_size':5,
         'print_diagnostics':True}
    if LaunchConfiguration('enable_wheel_odometry').perform(context)=='true':
        ekf.update({'odom0':'/hardware/wheel_odometry','odom0_config':
                   [False]*6+[True,True,False,False,False,True]+[False]*3})
    if LaunchConfiguration('enable_chassis_imu').perform(context)=='true':
        ekf.update({'imu0':'/hardware/chassis_imu','imu0_config':
                   [False]*9+[True,True,True]+[False]*3})
    fixed_nodes=[]
    if fixed:
        for name,parent,child,values in (
            ('fixed_chassis_imu',frames['chassis'],frames['imu'],tf_params['chassis_to_imu']),
            ('fixed_imu_lidar',frames['imu'],frames['lidar'],tf_params['imu_to_lidar']),
            ('fixed_chassis_alias',frames['chassis'],'base_link',[0.,0.,0.,0.,0.,0.,1.])):
            fixed_nodes.append(Node(package='tf2_ros',executable='static_transform_publisher',name=name,
                arguments=[*map(str,values),parent,child],parameters=[{'use_sim_time':sim}],output='screen'))
    if user_mount:print('[WARN] 用户给定车体安装位置与工厂内部外参；现场联合标定仍未验收')
    if legacy and frames!={'base_footprint':'base_footprint','chassis':'chassis','big_gimbal_yaw':'big_gimbal_yaw','imu':'front_mid360_imu','lidar':'front_mid360'}:
        raise ValueError('Legacy hard-coded TF requires the original frame names')
    return [
        Node(package='rm_nav_sensors',executable='mid360_cloud_guard',
             condition=IfCondition(str(raw_guard).lower()),
             parameters=[{'use_sim_time':sim,'sensor_frame':frames['lidar']}],output='screen'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('rm_nav_bringup'), 'launch', 'legacy_static_tf.launch.py')),
            launch_arguments={'projection_owner':'true','alias_parent':frames['chassis']}.items())
        if legacy else (fixed_nodes[0] if fixed else Node(package='rm_nav_bringup',executable='gimbal_tf',parameters=[tf_params,{
            'use_sim_time':sim,'calibration_id':bundle['bundle_id'],
            'footprint_frame':frames['base_footprint'],'chassis_frame':frames['chassis'],
            'yaw_frame':frames['big_gimbal_yaw'],'imu_frame':frames['imu'],'lidar_frame':frames['lidar'],
            'yaw_sign':float(bundle['encoder']['sign']),'yaw_offset':float(bundle['encoder']['zero_offset']),
            'yaw_joint':bundle['encoder']['joint']}],output='screen')),
        *fixed_nodes[1:],
        Node(package='rm_nav_bringup',executable='lio_input_guard',
             condition=IfCondition(PythonExpression(["'",LaunchConfiguration('enable_lio'),"' == 'true' and '",LaunchConfiguration('legacy_lio'),"' != 'true'"])),
             parameters=[{'use_sim_time':sim}],output='screen'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('rm_nav_bringup'),'launch','z_localization_frontend.launch.py')),
             condition=IfCondition(PythonExpression(["'",LaunchConfiguration('enable_lio'),"' == 'true' and '",LaunchConfiguration('legacy_lio'),"' != 'true'"])),
             launch_arguments={'use_sim_time':LaunchConfiguration('use_sim_time'),'sensor_frame':frames['imu'],
                 'imu_to_lidar_translation':json.dumps(transforms['imu_to_lidar']['translation']),
                 'imu_to_lidar_quaternion':json.dumps(transforms['imu_to_lidar']['quaternion_xyzw'])}.items()),
        Node(package='small_point_lio',executable='small_point_lio_node',name='small_point_lio',
             condition=IfCondition(PythonExpression(["'",LaunchConfiguration('enable_lio'),"' == 'true' and '",LaunchConfiguration('legacy_lio'),"' == 'true'"])),
             parameters=[LaunchConfiguration('lio_params_file'),{'use_sim_time':sim,
                 'publish_tf':False,'odom_frame':'odom','state_frame':frames['imu'],'lidar_frame':frames['lidar'],
                 'extrinsic_est_en':False,'extrinsic_T':transforms['imu_to_lidar']['translation'],
                 'extrinsic_R':rotation},lio_input],
             remappings=[('/Odometry','/lio/sensor_odometry'),('/cloud_registered','/lio/deskewed_odom_cloud')],output='screen'),
        Node(package='rm_nav_localization',executable='sensor_pose_resolver',parameters=[{
            'use_sim_time':sim,'calibration_id':bundle['bundle_id'],'sensor_frame':frames['imu'],
            'body_frame':frames['chassis'],'require_encoder_health':not (legacy or fixed),
            'require_raw_cloud_health':raw_guard}],output='screen'),
        Node(package='robot_localization',executable='ekf_node',name='chassis_ekf',parameters=[ekf],
             remappings=[('odometry/filtered','/state/chassis')],output='screen'),
        Node(package='rm_nav_localization',executable='base_footprint_projection',parameters=[{
            'use_sim_time':sim,'odom_frame':'odom','footprint_frame':frames['base_footprint'],
            'chassis_frame':frames['chassis']}],output='screen'),
        Node(package='rm_nav_localization',executable='chassis_state_bridge',parameters=[{
            'use_sim_time':sim,'calibration_id':bundle['bundle_id'],
            'state_reference_frame':frames['chassis'],'require_encoder_health':not (legacy or fixed)}],output='screen'),
        Node(package='rm_nav_sensors',executable='lio_observation_adapter',parameters=[{
            'use_sim_time':sim,'calibration_id':bundle['bundle_id'],'sensor_frame':frames['lidar']}],output='screen'),
        Node(package='rm_nav_localization',executable='observation_submap',parameters=[{
            'use_sim_time':sim,'calibration_id':bundle['bundle_id'],'sensor_frame':frames['lidar']}],output='screen')]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('calibration_file',description='Verified measured calibration YAML'),
        DeclareLaunchArgument('lio_params_file',default_value='',description='LEGACY_DEBUG only: small_point_lio settings'),
        DeclareLaunchArgument('legacy_lio',default_value='false'),
        DeclareLaunchArgument('use_sim_time',default_value='false'),
        DeclareLaunchArgument('enable_lio',default_value='true'),
        DeclareLaunchArgument('enable_mid360_guard',default_value='false',
                             description='Enable native Livox scan gate and raw-quality state interlock; use true for MID360 hardware'),
        DeclareLaunchArgument('enable_wheel_odometry',default_value='false'),
        DeclareLaunchArgument('enable_chassis_imu',default_value='false'),
        DeclareLaunchArgument('allow_legacy_static_calibration',default_value='false',
                             description='显式允许原版静态外参联调；默认拒绝，不能用于正式验收'),
        DeclareLaunchArgument('allow_user_mount',default_value='false',
                             description='Explicit fixed chassis mount and factory internal extrinsics; does not mark field calibration verified'),
        OpaqueFunction(function=nodes)])
