"""Per-arm Quest mapping and optional recording; never starts an FCI owner."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from quest_teleop.configuration import launch_bool, load_arms, recording_plan


def setup(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    arms = load_arms(value('arms_config'))
    actions = []
    if launch_bool(value('start_bridge')):
        port = int(value('listen_port'))
        if not 1 <= port <= 65535:
            raise ValueError('listen_port must be in [1,65535]')
        actions.append(Node(package='quest_teleop', executable='quest_usb_bridge',
                            name='quest_usb_bridge', output='screen',
                            parameters=[{'listen_port': port, 'bind_address': '127.0.0.1'}]))
    mock = launch_bool(value('mock_measured_pose'))
    for arm in arms:
        actions.append(Node(package='quest_teleop', executable='pose_mapper', name='pose_mapper',
                            namespace=arm.namespace, output='screen', parameters=[arm.parameters]))
        if mock:
            actions.append(Node(package='quest_teleop', executable='mock_measured_pose',
                                name='mock_measured_pose', namespace=arm.namespace, output='screen',
                                parameters=[{key: arm.parameters[key] for key in
                                             ('base_frame', 'measured_pose_topic')}]))
    if launch_bool(value('recording_enabled')):
        plan = recording_plan(value('recording_config'), arms, value('recording_topics'),
                              value('recording_output_root'))
        Path(plan.output).parent.mkdir(parents=True, exist_ok=True)
        actions.extend([LogInfo(msg=f'Recording selected topics to {plan.output}: {plan.topics}'),
                        ExecuteProcess(cmd=plan.command, output='screen')])
    return actions


def generate_launch_description():
    share = Path(get_package_share_directory('quest_teleop'))
    defaults = dict(arms_config=str(share / 'config/arms.yaml'), start_bridge='false',
                    listen_port='45454', mock_measured_pose='false', recording_enabled='false',
                    recording_config=str(share / 'config/recording.yaml'),
                    recording_topics='', recording_output_root='')
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()]
                             + [OpaqueFunction(function=setup)])
