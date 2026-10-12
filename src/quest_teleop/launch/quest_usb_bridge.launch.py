from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='/'),
        DeclareLaunchArgument('listen_port', default_value='45454'),
        DeclareLaunchArgument('bind_address', default_value='127.0.0.1'),
        DeclareLaunchArgument('command_timeout_ms', default_value='100'),
        DeclareLaunchArgument('max_clock_uncertainty_ms', default_value='10'),
        DeclareLaunchArgument('max_clock_sync_age_ms', default_value='3000'),
        DeclareLaunchArgument('max_source_future_ms', default_value='10'),
        Node(
            package='quest_teleop',
            executable='quest_usb_bridge',
            name='quest_usb_bridge',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[{
                'listen_port': LaunchConfiguration('listen_port'),
                'bind_address': LaunchConfiguration('bind_address'),
                'command_timeout_ms': LaunchConfiguration('command_timeout_ms'),
                'max_clock_uncertainty_ms': LaunchConfiguration('max_clock_uncertainty_ms'),
                'max_clock_sync_age_ms': LaunchConfiguration('max_clock_sync_age_ms'),
                'max_source_future_ms': LaunchConfiguration('max_source_future_ms'),
            }],
        ),
    ])
