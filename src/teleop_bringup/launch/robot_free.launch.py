"""All configured mappers plus optional continuous recording; no arm or Hand owners."""

from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from quest_teleop.configuration import load_arms, select_arms, launch_bool
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def setup(context):
    value = lambda key: LaunchConfiguration(key).perform(context)

    def include(pkg, name, args):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(Path(get_package_share_directory(pkg)) / "launch" / name)
            ),
            launch_arguments=args.items(),
        )

    actions = [
        include(
            "quest_teleop",
            "quest_teleop.launch.py",
            dict(
                arms_config=value("arms_config"),
                mode="all",
                start_mapper="true",
                start_bridge=value("start_bridge"),
                listen_port=value("listen_port"),
                mock_measured_pose=value("mock_measured_pose"),
            ),
        )
    ]
    if launch_bool(value("recording_enabled")):
        actions.append(
            include(
                "recording",
                "recording.launch.py",
                dict(
                    mode="all",
                    recording_mode="continuous",
                    **{
                        k: value(k)
                        for k in (
                            "arms_config",
                            "recording_config",
                            "recording_topics",
                            "recording_output_root",
                        )
                    },
                ),
            )
        )
    return actions


def generate_launch_description():
    defaults = dict(
        arms_config=str(
            Path(get_package_share_directory("quest_teleop")) / "config/arms.yaml"
        ),
        start_bridge="false",
        listen_port="45454",
        mock_measured_pose="false",
        recording_enabled="false",
        recording_config=str(
            Path(get_package_share_directory("recording")) / "config/recording.yaml"
        ),
        recording_topics="",
        recording_output_root="",
    )
    return LaunchDescription(
        [DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()]
        + [OpaqueFunction(function=setup)]
    )
