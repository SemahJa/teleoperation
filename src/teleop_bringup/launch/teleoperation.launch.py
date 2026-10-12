"""Compose independently selectable Quest, arm, homing, gripper, CBF and recording features."""
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
    common = {k:value(k) for k in ('arms_config','mode','single_controller')}
    if launch_bool(value('start_controller')) and launch_bool(value('mock_measured_pose')):
        raise ValueError('mock_measured_pose requires start_controller:=false to avoid competing pose publishers')
    actions = []
    def include(package, launch, args):
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
            Path(get_package_share_directory(package))/'launch'/launch)), launch_arguments=args.items()))
    if launch_bool(value('start_bridge')) or launch_bool(value('start_mapper')) or launch_bool(value('mock_measured_pose')):
        include('quest_teleop','quest_teleop.launch.py',dict(common,
            start_bridge=value('start_bridge'), start_mapper=value('start_mapper'), listen_port=value('listen_port'),
            mock_measured_pose=value('mock_measured_pose')))
    if launch_bool(value('start_controller')):
        include('teleop_controllers','cartesian_impedance_controller.launch.py',dict(common,
            backend=value('backend'), operator_confirm=value('operator_confirm'),
            runtime_config=value('controller_config') or value('runtime_config'),
            safety_config=value('safety_config') or value('runtime_config'),
            cbf_enabled=value('cbf_enabled'), homing_enabled=value('homing_enabled')))
    if launch_bool(value('start_hand')):
        include('teleop_gripper','gripper.launch.py',dict(common,
            backend=value('backend'), operator_confirm=value('operator_confirm'),
            runtime_config=value('gripper_config') or value('runtime_config')))
    if launch_bool(value('recording_enabled')):
        include('recording','recording.launch.py',dict(common, **{k:value(k) for k in
            ('recording_mode','recording_config','recording_topics','recording_output_root',
             'left_controller_state_topic','recording_namespace')}))
    return actions


def generate_launch_description():
    defaults = dict(mode='single', single_controller='right', backend='mock', operator_confirm='false',
        arms_config=str(Path(get_package_share_directory('quest_teleop'))/'config/arms.yaml'),
        runtime_config=str(Path(get_package_share_directory('teleop_bringup'))/'config/teleoperation_runtime.yaml'),
        controller_config='', gripper_config='', safety_config='',
        start_controller='true', homing_enabled='true', cbf_enabled='true',
        start_mapper='true', start_hand='true', start_bridge='false', listen_port='45454', mock_measured_pose='false',
        recording_enabled='true', recording_mode='menu',
        recording_config=str(Path(get_package_share_directory('recording'))/'config/recording.yaml'),
        recording_topics='', recording_output_root='', left_controller_state_topic='', recording_namespace='/quest')
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k,v in defaults.items()] +
                             [OpaqueFunction(function=setup)])
