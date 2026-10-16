"""Compose the pipeline from an explicitly selected runtime YAML."""

from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from quest_teleop.configuration import read_mapping, load_arms, select_arms, launch_bool
from quest_teleop.runtime_configuration import (
    PIPELINE_DEFAULTS,
    unified_settings,
    load_runtime,
)

REMOVED = set(PIPELINE_DEFAULTS) | {
    "listen_port",
    "recording_mode",
    "recording_topics",
    "recording_output_root",
    "left_controller_state_topic",
    "recording_namespace",
    "controller_config",
    "gripper_config",
    "safety_config",
    "arms_config",
    "recording_config",
}


def setup(context):
    value = lambda key: LaunchConfiguration(key).perform(context)
    if REMOVED & set(context.launch_configurations):
        raise ValueError(
            "overall bringup accepts only runtime_config and operator_confirm; set all pipeline options in YAML"
        )
    path = value("runtime_config")
    pipeline, bridge, recording = unified_settings(read_mapping(path))
    if pipeline["backend"] not in ("mock", "hardware"):
        raise ValueError("backend must be mock/hardware")
    confirmed = launch_bool(value("operator_confirm"))
    if pipeline["backend"] == "hardware" and not confirmed:
        raise ValueError("hardware requires explicit operator_confirm:=true")
    configured = load_arms(path)
    active = select_arms(configured, pipeline["mode"], pipeline["single_controller"])
    settings = load_runtime(path, configured)
    # Validate ownership and dual geometry before starting any child (including Hand).
    if pipeline["backend"] == "hardware" and (
        pipeline["start_controller"] or pipeline["start_hand"]
    ):
        if any(not arm.robot_ip for arm in active):
            raise ValueError("hardware needs each selected robot_ip")
        if pipeline["start_controller"] and pipeline["mode"] == "dual":
            if any(
                "world_from_base_xyz_xyzw"
                not in settings.get("arm_settings", {}).get(a.name, {})
                for a in active
            ):
                raise ValueError(
                    "hardware dual mode needs both surveyed world_from_base_xyz_xyzw transforms"
                )
    if pipeline["start_controller"] and pipeline["mock_measured_pose"]:
        raise ValueError(
            "mock_measured_pose requires start_controller:=false to avoid competing pose publishers"
        )
    common = dict(
        arms_config=path,
        mode=pipeline["mode"],
        single_controller=pipeline["single_controller"],
    )
    text = lambda v: str(v).lower() if isinstance(v, bool) else str(v)
    actions = []

    def include(package, launch, args):
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(Path(get_package_share_directory(package)) / "launch" / launch)
                ),
                launch_arguments={k: text(v) for k, v in args.items()}.items(),
            )
        )

    if (
        pipeline["start_bridge"]
        or pipeline["start_mapper"]
        or pipeline["mock_measured_pose"]
    ):
        include(
            "quest_teleop",
            "quest_teleop.launch.py",
            dict(
                common,
                **bridge,
                start_bridge=pipeline["start_bridge"],
                start_mapper=pipeline["start_mapper"],
                mock_measured_pose=pipeline["mock_measured_pose"],
            ),
        )
    if pipeline["start_controller"]:
        include(
            "teleop_controllers",
            "cartesian_impedance_controller.launch.py",
            dict(
                common,
                backend=pipeline["backend"],
                operator_confirm=confirmed,
                runtime_config=path,
                safety_config=path,
                cbf_enabled=pipeline["cbf_enabled"],
                homing_enabled=pipeline["homing_enabled"],
            ),
        )
    if pipeline["start_hand"]:
        include(
            "teleop_gripper",
            "gripper.launch.py",
            dict(
                common,
                backend=pipeline["backend"],
                operator_confirm=confirmed,
                runtime_config=path,
            ),
        )
    if pipeline["recording_enabled"]:
        # Check recording schema/topics before any process can be started.
        from recording.configuration import recording_plan

        recording_plan(path, active)
        include(
            "recording",
            "recording.launch.py",
            dict(
                common,
                recording_config=path,
                recording_mode=recording["mode"],
                left_controller_state_topic=recording["left_controller_state_topic"],
                recording_namespace=recording["namespace"],
            ),
        )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "runtime_config",
                description="Required path to the complete single/dual Panda runtime YAML",
            ),
            DeclareLaunchArgument(
                "operator_confirm",
                default_value="false",
                description="Explicit hardware authorization; not required for mock",
            ),
            OpaqueFunction(function=setup),
        ]
    )
