from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

def generate_launch_description():
    def source(package, filename):
        return PythonLaunchDescriptionSource(str(Path(get_package_share_directory(package))/'launch'/filename))
    return LaunchDescription([
        DeclareLaunchArgument('runtime_config'),
        DeclareLaunchArgument('operator_confirm', default_value='false'),
        IncludeLaunchDescription(source('camera_bringup','cameras.launch.py'),launch_arguments={'cameras':'both'}.items()),
        IncludeLaunchDescription(source('teleop_bringup','teleoperation.launch.py'),launch_arguments={'runtime_config':LaunchConfiguration('runtime_config'),'operator_confirm':LaunchConfiguration('operator_confirm')}.items()),
    ])
