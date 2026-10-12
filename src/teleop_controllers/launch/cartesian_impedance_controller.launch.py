"""Exclusive arm owners: Cartesian impedance, optional homing and embedded safety."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from quest_teleop.configuration import load_arms, select_arms, launch_bool
from launch.actions import EmitEvent, GroupAction, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from quest_teleop.runtime_configuration import load_runtime

def setup(context):
    value = lambda key: LaunchConfiguration(key).perform(context)
    mode = value('mode'); backend = value('backend')
    if backend not in ('mock','hardware'): raise ValueError('backend must be mock/hardware')
    confirmed = launch_bool(value('operator_confirm'))
    if backend == 'hardware' and not confirmed: raise ValueError('hardware requires operator_confirm:=true')
    configured = load_arms(value('arms_config'))
    active = select_arms(configured, mode, value('single_controller'))
    settings = load_runtime(value('runtime_config'), configured)
    params = dict(mode=mode, backend=backend, operator_confirm=confirmed,
        homing_enabled=launch_bool(value('homing_enabled')))
    for arm in active:
        p = arm.parameters
        local = settings.get('arm_settings', {}).get(arm.name, {})
        controller = dict(settings.get('controller', {})); controller.update(local.get('controller', {}))
        if backend == 'hardware' and not arm.robot_ip: raise ValueError(f'{arm.name}: hardware needs robot_ip')
        fields = dict(controller, robot_ip=arm.robot_ip, base_frame=p['base_frame'],
            controller_id=p['controller_id'], controller_state_topic=p['controller_state_topic'],
            command_topic=p['command_topic'], measured_pose_topic=p['measured_pose_topic'],
            state_topic=arm.state_topic, command_timeout_ms=float(p['command_timeout_ms']),
            max_clock_uncertainty_ms=p['max_clock_uncertainty_ms'], max_source_future_ms=p['max_source_future_ms'],
            trigger_release=p['trigger_release'])
        if 'world_from_base_xyz_xyzw' in local: fields['world_from_base_xyz_xyzw'] = local['world_from_base_xyz_xyzw']
        elif backend == 'hardware' and mode == 'dual':
            raise ValueError(f'{arm.name}: hardware dual mode needs surveyed world_from_base_xyz_xyzw')
        prefix = p['controller_id']+'.' if mode == 'dual' else ''
        if mode == 'dual': fields['namespace'] = arm.namespace
        params.update({prefix+k:v for k,v in fields.items()})
    owner = Node(package='teleop_controllers', executable='cartesian_impedance_controller',
        namespace=active[0].namespace if mode == 'single' else '/', output='screen', parameters=[params])
    safety = IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
        Path(get_package_share_directory('safety_filter'))/'launch/safety_filter.launch.py')),
        launch_arguments=dict(arms_config=value('arms_config'),
            safety_config=value('safety_config') or value('runtime_config'), cbf_enabled=value('cbf_enabled')).items())
    return [GroupAction(actions=[safety, owner, RegisterEventHandler(OnProcessExit(target_action=owner,
        on_exit=[EmitEvent(event=Shutdown(reason='arm owner exited; no automatic restart'))]))])]


def generate_launch_description():
    defaults = dict(mode='single', single_controller='right', backend='mock', operator_confirm='false',
        arms_config=str(Path(get_package_share_directory('quest_teleop'))/'config/arms.yaml'),
        runtime_config=str(Path(get_package_share_directory('teleop_controllers'))/'config/controllers.yaml'),
        safety_config=str(Path(get_package_share_directory('safety_filter'))/'config/safety.yaml'),
        homing_enabled='true', cbf_enabled='true')
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k,v in defaults.items()] +
                             [OpaqueFunction(function=setup)])
