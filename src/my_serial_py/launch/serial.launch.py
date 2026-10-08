from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    
    rm_serial = Node(
        package="my_serial_py",
        executable="serial_node",
        name="serial",
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate': ParameterValue(LaunchConfiguration('baud_rate'), value_type=int),
            'cmd_vel_timeout': ParameterValue(LaunchConfiguration('cmd_vel_timeout'), value_type=float),
        }],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0'),
            DeclareLaunchArgument('baud_rate', default_value='115200'),
            DeclareLaunchArgument('cmd_vel_timeout', default_value='0.3'),
            rm_serial
        ]
    )
