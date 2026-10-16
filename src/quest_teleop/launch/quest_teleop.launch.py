"""Quest transport, selected-arm mapping and optional measured-pose fixture."""

from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from quest_teleop.configuration import load_arms, select_arms, launch_bool
from quest_teleop.runtime_configuration import BRIDGE_DEFAULTS


def setup(context):
    value = lambda key: LaunchConfiguration(key).perform(context)
    configured = load_arms(value("arms_config"))
    mode = value("mode")
    arms = (
        configured
        if mode == "all"
        else select_arms(configured, mode, value("single_controller"))
    )
    actions = []
    if launch_bool(value("start_bridge")):
        port = int(value("listen_port"))
        if not 1 <= port <= 65535:
            raise ValueError("listen_port must be 1..65535")
        params = {
            k: int(value(k)) if isinstance(v, int) else value(k)
            for k, v in BRIDGE_DEFAULTS.items()
            if k != "namespace"
        }
        actions.append(
            Node(
                package="quest_teleop",
                executable="quest_usb_bridge",
                name="quest_usb_bridge",
                namespace=value("namespace"),
                output="screen",
                parameters=[params],
            )
        )
    for arm in arms:
        if launch_bool(value("start_mapper")):
            actions.append(
                Node(
                    package="quest_teleop",
                    executable="pose_mapper",
                    name="pose_mapper",
                    namespace=arm.namespace,
                    output="screen",
                    parameters=[arm.parameters],
                )
            )
        if launch_bool(value("mock_measured_pose")):
            actions.append(
                Node(
                    package="quest_teleop",
                    executable="mock_measured_pose",
                    namespace=arm.namespace,
                    output="screen",
                    parameters=[
                        {
                            k: arm.parameters[k]
                            for k in ("base_frame", "measured_pose_topic")
                        }
                    ],
                )
            )
    return actions


def generate_launch_description():
    defaults = dict(
        arms_config=str(
            Path(get_package_share_directory("quest_teleop")) / "config/arms.yaml"
        ),
        mode="single",
        single_controller="right",
        start_bridge="true",
        start_mapper="true",
        mock_measured_pose="false",
        **{k: str(v) for k, v in BRIDGE_DEFAULTS.items()},
    )
    return LaunchDescription(
        [DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()]
        + [OpaqueFunction(function=setup)]
    )
