"""Original Adam wire transport; defaults to a device-free candidate output.

PTY is TEST_ONLY. Physical cutover must be explicit and cannot share the old
serial owner. No sensor, localization or original region/yaw controller is started.
"""
from pathlib import Path
import stat
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def nodes(context):
    mode = LaunchConfiguration('transport').perform(context)
    if mode not in ('disabled', 'pty', 'physical'):
        raise ValueError('transport must be disabled, pty or physical')
    port = LaunchConfiguration('serial_port').perform(context)
    if mode != 'disabled':
        resolved = Path(port).resolve(strict=True)
        if not stat.S_ISCHR(resolved.stat().st_mode):
            raise ValueError('Serial port must be a real character device')
        is_pty = resolved.parent == Path('/dev/pts') and resolved.name.isdecimal()
        if mode == 'pty' and not is_pty:
            raise ValueError('TEST_ONLY pty mode accepts only /dev/pts/<number>')
        if mode == 'physical' and (is_pty or
                LaunchConfiguration('enable_physical_output').perform(context) != 'true'):
            raise ValueError('Physical mode requires explicit enable_physical_output=true and a physical port')
    actions = [Node(package='rm_nav_bringup', executable='legacy_adam_adapter',
                    parameters=[{'require_serial_health': mode != 'disabled'}], output='screen')]
    if mode != 'disabled':
        actions.append(Node(package='my_serial_py', executable='serial_node', name='legacy_adam_serial',
            parameters=[{'serial_port': port, 'baud_rate': 115200, 'yaw_policy': 'legacy_cached',
                         'cmd_vel_timeout': .3}],
            remappings=[('/cmd_vel', '/hardware/legacy_adam/cmd_vel'),
                        ('/hardware/serial_healthy', '/hardware/legacy_adam/serial_healthy')], output='screen'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('transport', default_value='disabled'),
        DeclareLaunchArgument('serial_port', default_value='', description='Explicit port; no /dev/ttyUSB0 fallback'),
        DeclareLaunchArgument('enable_physical_output', default_value='false'),
        OpaqueFunction(function=nodes),
    ])
