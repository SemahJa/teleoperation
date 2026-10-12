"""Load embedded CBF/coordinator parameters for subsequently launched arm owners.

The filter is a linked realtime library, not a second FCI or ROS service owner.
Include this launch in the same scoped group as its controller node.
"""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from quest_teleop.configuration import load_arms, select_arms, launch_bool
from launch.actions import LogInfo
from launch_ros.actions import SetParameter
from quest_teleop.runtime_configuration import load_runtime

def setup(context):
    value = lambda key: LaunchConfiguration(key).perform(context)
    arms = load_arms(value('arms_config'))
    settings = load_runtime(value('safety_config'), arms)
    params = {'cbf.'+k:v for k,v in settings.get('cbf', {}).items()}
    params['cbf.enabled'] = launch_bool(value('cbf_enabled'))
    return [SetParameter(name=k, value=v) for k,v in params.items()] + [
        LogInfo(msg='Loaded safety_filter parameters for the embedded arm-owner coordinator')]


def generate_launch_description():
    defaults = dict(arms_config=str(Path(get_package_share_directory('quest_teleop'))/'config/arms.yaml'),
        safety_config=str(Path(get_package_share_directory('safety_filter'))/'config/safety.yaml'), cbf_enabled='true')
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k,v in defaults.items()] +
                             [OpaqueFunction(function=setup)])
