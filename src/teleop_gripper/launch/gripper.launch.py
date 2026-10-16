"""Selected-arm Hand workers with optional one-time empty-hand startup calibration."""

from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from quest_teleop.configuration import load_arms, select_arms, launch_bool
from quest_teleop.runtime_configuration import load_runtime


def setup(context):
    value = lambda key: LaunchConfiguration(key).perform(context)
    backend = value("backend")
    confirmed = launch_bool(value("operator_confirm"))
    if backend not in ("mock", "hardware"):
        raise ValueError("backend must be mock/hardware")
    if backend == "hardware" and not confirmed:
        raise ValueError("hardware requires operator_confirm:=true")
    configured = load_arms(value("arms_config"))
    active = select_arms(configured, value("mode"), value("single_controller"))
    settings = load_runtime(value("runtime_config"), configured)
    actions = []
    for arm in active:
        if backend == "hardware" and not arm.robot_ip:
            raise ValueError(f"{arm.name}: hardware needs robot_ip")
        hand = dict(settings.get("hand", {}))
        hand.update(settings.get("arm_settings", {}).get(arm.name, {}).get("hand", {}))
        actions.append(
            Node(
                package="teleop_gripper",
                executable="franka_hand_worker",
                namespace=arm.namespace,
                output="screen",
                parameters=[
                    dict(
                        hand,
                        backend=backend,
                        robot_ip=arm.robot_ip,
                        operator_confirm=confirmed,
                        calibrate_on_start=True,
                        controller_id=arm.parameters["controller_id"],
                        controller_state_topic=arm.parameters["controller_state_topic"],
                    )
                ],
            )
        )
    return actions


def generate_launch_description():
    defaults = dict(
        mode="single",
        single_controller="right",
        backend="mock",
        operator_confirm="false",
        arms_config=str(
            Path(get_package_share_directory("quest_teleop")) / "config/arms.yaml"
        ),
        runtime_config=str(
            Path(get_package_share_directory("teleop_gripper")) / "config/gripper.yaml"
        ),
    )
    return LaunchDescription(
        [DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()]
        + [OpaqueFunction(function=setup)]
    )
