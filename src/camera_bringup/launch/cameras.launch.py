"""Camera-only launch; neither process failure shuts down the other."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from camera_bringup.configuration import load_camera


def setup(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    selected = value("cameras")
    if selected not in ("wrist", "scene", "both"):
        raise ValueError("cameras must be wrist, scene or both")
    roles = ("wrist", "scene") if selected == "both" else (selected,)
    configured = [(role, load_camera(value(role + "_config"), role, value(role + "_profile"))) for role in roles]
    if len({params["serial_no"] for _, params in configured}) != len(configured):
        raise ValueError("selected cameras must have different serial numbers")
    actions = []
    for role, params in configured:
        actions.extend([
            LogInfo(msg=f"{role}: serial={params['serial_no'][1:]} color={params['rgb_camera.color_profile']} depth={params['depth_module.depth_profile']}; no automatic fallback"),
            Node(package="realsense2_camera", executable="realsense2_camera_node",
                 namespace="/cameras", name=role, parameters=[params], output="screen"),
        ])
    return actions


def generate_launch_description():
    root = Path(get_package_share_directory("camera_bringup")) / "config"
    args = [DeclareLaunchArgument("cameras", default_value="both")]
    for role in ("wrist", "scene"):
        args.extend([
            DeclareLaunchArgument(role + "_config", default_value=str(root / (role + ".yaml"))),
            DeclareLaunchArgument(role + "_profile", default_value="target"),
        ])
    return LaunchDescription(args + [OpaqueFunction(function=setup)])
