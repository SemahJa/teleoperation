"""One coupled CBF coordinator, exclusive arm owners, mappers and Menu recorder."""
import json
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from quest_teleop.configuration import load_arms, select_arms, launch_bool, read_mapping, recording_plan


def setup(context):
    value = lambda key: LaunchConfiguration(key).perform(context)
    mode = value('mode'); backend = value('backend')
    if backend not in ('mock', 'hardware'): raise ValueError('backend must be mock/hardware')
    confirmed = launch_bool(value('operator_confirm'))
    if backend == 'hardware' and not confirmed:
        raise ValueError('hardware requires explicit operator_confirm:=true')
    configured = load_arms(value('arms_config'))
    active = select_arms(configured, mode, value('single_controller'))
    settings = read_mapping(value('runtime_config'))
    if set(settings) - {'controller', 'hand', 'cbf', 'arm_settings'}:
        raise ValueError('unknown runtime configuration section')
    for section in ('controller', 'hand', 'cbf', 'arm_settings'):
        if not isinstance(settings.get(section, {}), dict): raise ValueError(f'{section} must be a mapping')
    allowed_controller = {'lambda_translation','lambda_rotation','gain_translation','gain_rotation',
        'maximum_torque_rate','tool_offset_m','envelope_margin_m','envelope_center','torque_limits',
        'homing_speed_factor','homing_timeout_seconds','homing_stop_duration_seconds',
        'homing_stop_acceleration','homing_stop_timeout_seconds','homing_stiffness','homing_damping'}
    allowed_hand = {'speed_mps','grasp_force_n','grasp_width_m','epsilon_inner_m','epsilon_outer_m',
        'open_width_m','grip_press','grip_release','grip_debounce_ms','command_timeout_ms',
        'max_clock_uncertainty_ms','mock_operation_duration_ms'}
    allowed_cbf = {'table_height','table_minimum_clearance','self_minimum_clearance',
        'inter_arm_minimum_clearance','maximum_failures','obstacle_centers','obstacle_radii'}
    if set(settings.get('cbf', {})) - allowed_cbf: raise ValueError('unknown CBF setting')
    if set(settings.get('arm_settings', {})) - {a.name for a in configured}:
        raise ValueError('arm_settings contains unknown arm names')
    # Ownership, input topics and command frame come from the arm configuration.
    protected = {'mode', 'backend', 'operator_confirm', 'robot_ip', 'namespace', 'base_frame',
                 'controller_id', 'controller_state_topic', 'command_topic', 'measured_pose_topic', 'state_topic'}
    params = dict(mode=mode, backend=backend, operator_confirm=confirmed)
    params.update({'cbf.' + k: v for k, v in settings.get('cbf', {}).items()})
    actions = []
    for arm in active:
        p = arm.parameters
        local = settings.get('arm_settings', {}).get(arm.name, {})
        if not isinstance(local, dict) or set(local) - {'controller', 'hand', 'world_from_base_xyz_xyzw'}:
            raise ValueError(f'{arm.name}: invalid arm_settings')
        controller = dict(settings.get('controller', {})); controller.update(local.get('controller', {}))
        hand = dict(settings.get('hand', {})); hand.update(local.get('hand', {}))
        if set(controller) - allowed_controller or set(hand) - allowed_hand:
            raise ValueError('unknown controller or Hand setting')
        if protected & (controller.keys() | hand.keys()):
            raise ValueError('runtime settings cannot override ownership/assignment/topic identities')
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
        prefix = p['controller_id'] + '.' if mode == 'dual' else ''
        if mode == 'dual': fields['namespace'] = arm.namespace
        params.update({prefix+k: v for k,v in fields.items()})
        actions.append(Node(package='quest_teleop', executable='pose_mapper', name='pose_mapper',
            namespace=arm.namespace, output='screen', parameters=[p]))
        if launch_bool(value('start_hand')):
            actions.append(Node(package='franka_teleop_controller', executable='franka_hand_worker',
                namespace=arm.namespace, output='screen', parameters=[dict(hand,
                    backend=backend, robot_ip=arm.robot_ip, operator_confirm=confirmed,
                    controller_id=p['controller_id'], controller_state_topic=p['controller_state_topic'])]))
    owner = Node(package='franka_teleop_controller', executable='cartesian_impedance_controller',
        namespace=active[0].namespace if mode == 'single' else '/', output='screen', parameters=[params])
    actions.extend([owner, RegisterEventHandler(OnProcessExit(target_action=owner,
        on_exit=[EmitEvent(event=Shutdown(reason='arm owner exited; no automatic restart'))]))])
    if launch_bool(value('start_bridge')):
        port = int(value('listen_port'))
        if not 1 <= port <= 65535: raise ValueError('listen_port must be 1..65535')
        actions.append(Node(package='quest_teleop', executable='quest_usb_bridge', name='quest_usb_bridge',
            output='screen', parameters=[dict(listen_port=port, bind_address='127.0.0.1')]))
    if launch_bool(value('recording_enabled')):
        plan = recording_plan(value('recording_config'), active, value('recording_topics'), value('recording_output_root'))
        inactive = [a.namespace for a in configured if a.name not in {b.name for b in active}]
        left = next((a.parameters['controller_state_topic'] for a in configured if a.parameters['controller_id']=='left'),
                    '/quest_usb_bridge/left/controller_state')
        left = value('left_controller_state_topic') or left
        actions.append(Node(package='quest_teleop', executable='menu_recorder', namespace=value('recording_namespace'),
            output='screen', parameters=[dict(controller_state_topic=left,
                topics_json=json.dumps(plan.topics), inactive_namespaces_json=json.dumps(inactive),
                output_root=str(Path(plan.output).parent), storage_id=plan.command[4])]))
    return actions


def generate_launch_description():
    share = Path(get_package_share_directory('franka_teleop_controller'))
    quest = Path(get_package_share_directory('quest_teleop'))
    defaults = dict(mode='single', single_controller='right', backend='mock', operator_confirm='false',
        arms_config=str(quest/'config/arms.yaml'), runtime_config=str(share/'config/teleoperation_runtime.yaml'),
        start_hand='true', start_bridge='false', listen_port='45454', recording_enabled='true',
        recording_config=str(quest/'config/recording.yaml'), recording_topics='', recording_output_root='',
        left_controller_state_topic='', recording_namespace='/quest')
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k,v in defaults.items()] +
                             [OpaqueFunction(function=setup)])
