"""Start the shared arm owner with guarded homing available; no startup home motion."""

from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(
                        Path(get_package_share_directory("teleop_controllers"))
                        / "launch/cartesian_impedance_controller.launch.py"
                    )
                ),
                launch_arguments={"homing_enabled": "true"}.items(),
            )
        ]
    )
