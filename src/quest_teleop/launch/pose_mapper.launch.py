from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    names = {
        'namespace': '/',
        'controller_id': 'right',
        'controller_state_topic': '/quest_usb_bridge/controller_state',
        'measured_pose_topic': '/panda_1/teleop_anchor_pose',
        'command_topic': '/panda_1/cartesian_teleop_command',
        'monitor_topic': '/panda_1/target_pose',
        'base_frame': 'panda_1/panda_link0',
        'reference_to_robot_xyzw': '[0.0, 0.0, 0.0, 1.0]',
        'translation_scale': '1.0',
        'rotation_scale': '1.0',
        'feedforward_scale': '1.0',
        'twist_filter_cutoff_hz': '10.0',
        'command_timeout_ms': '100',
        'max_clock_uncertainty_ms': '10.0',
        'max_source_future_ms': '10.0',
    }
    args = [DeclareLaunchArgument(name, default_value=value) for name, value in names.items()]
    params = {name: LaunchConfiguration(name) for name in names
              if name not in ('reference_to_robot_xyzw', 'namespace')}
    params['reference_to_robot_xyzw'] = ParameterValue(
        LaunchConfiguration('reference_to_robot_xyzw'))
    return LaunchDescription(args + [Node(
        package='quest_teleop', executable='pose_mapper', name='pose_mapper',
        namespace=LaunchConfiguration('namespace'), output='screen', parameters=[params])])
