"""Alternative transport backend. Hardware/controllers start inactive; no TF broadcaster."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def start(context):
    share=Path(get_package_share_directory('rm_nav_bringup'))
    physical=LaunchConfiguration('enable_physical_output').perform(context)=='true'
    # These are logical chassis commands, not wheel joints or invented wheel states.
    description='<robot name="adam_command_interfaces"><link name="command_root"/>'
    parent='command_root'
    for name,kind,axis in [('command_vx','prismatic','1 0 0'),('command_vy','prismatic','0 1 0'),('command_yaw','continuous','0 0 1')]:
        child=name+'_link'
        description+=f'<link name="{child}"/><joint name="{name}" type="{kind}"><parent link="{parent}"/><child link="{child}"/><axis xyz="{axis}"/><limit lower="-1000" upper="1000" effort="1" velocity="1"/></joint>'
        parent=child
    description+='<ros2_control name="AdamSerial" type="system"><hardware><plugin>rm_nav_hardware/RmSerialHardwareInterface</plugin>'
    for key,value in {'allow_hardware_activation':str(physical).lower(),'base_frame':'base_link',
                      'velocity_topic':'/cmd_vel' if physical else '/hardware/ros2_control/cmd_vel',
                      'yaw_topic':'/cmd_yaw_angle' if physical else '/hardware/ros2_control/cmd_yaw_angle'}.items():
        description+=f'<param name="{key}">{value}</param>'
    description+='</hardware>'
    for name,interface in [('command_vx','velocity'),('command_vy','velocity'),('command_yaw','position')]:
        description+=f'<joint name="{name}"><command_interface name="{interface}"/><state_interface name="{interface}"/></joint>'
    description+='</ros2_control></robot>'
    return [
        Node(package='controller_manager',executable='ros2_control_node',parameters=[str(share/'config/ros2_control.yaml')],output='screen'),
        Node(package='rm_nav_bringup',executable='ros2_control_input',parameters=[{'robot_description':description}],output='screen'),
        Node(package='rm_nav_control',executable='omni_pid_adapter',parameters=[str(share/'config/omni_pid_adapter.yaml'),
            {'enable_physical_output':True,'require_measured_feedback':True}],
            remappings=[('/cmd_vel','/hardware/controller_input/cmd_vel'),('/cmd_yaw_angle','/hardware/controller_input/cmd_yaw_angle')]),
        Node(package='controller_manager',executable='spawner',arguments=['omni_velocity_controller','omni_yaw_controller','--inactive'],output='screen')]

def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('enable_physical_output',default_value='false'),OpaqueFunction(function=start)])
