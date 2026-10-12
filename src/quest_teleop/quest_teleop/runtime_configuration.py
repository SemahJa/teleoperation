"""Validate feature settings and per-arm overrides shared by feature launches."""
from quest_teleop.configuration import read_mapping
ALLOWED = {
 'controller': {'lambda_translation','lambda_rotation','gain_translation','gain_rotation',
 'maximum_torque_rate','tool_offset_m','envelope_margin_m','envelope_center','torque_limits',
 'homing_speed_factor','homing_timeout_seconds','homing_stop_duration_seconds',
 'homing_stop_acceleration','homing_stop_timeout_seconds','homing_stiffness','homing_damping'},
 'hand': {'speed_mps','grasp_force_n','grasp_width_m','epsilon_inner_m','epsilon_outer_m',
 'open_width_m','grip_press','grip_release','grip_debounce_ms','command_timeout_ms',
 'max_clock_uncertainty_ms','mock_operation_duration_ms'},
 'cbf': {'table_height','table_minimum_clearance','self_minimum_clearance',
 'inter_arm_minimum_clearance','maximum_failures','obstacle_centers','obstacle_radii'}}
def load_runtime(path, arms):
    data = read_mapping(path)
    if set(data) - (set(ALLOWED) | {'arm_settings'}):
        raise ValueError('unknown runtime configuration section')
    for key, allowed in ALLOWED.items():
        section = data.get(key, {})
        if not isinstance(section, dict) or set(section) - allowed:
            raise ValueError(f'unknown or invalid {key} setting')
    local = data.get('arm_settings', {})
    if not isinstance(local, dict) or set(local) - {a.name for a in arms}:
        raise ValueError('arm_settings contains unknown arm names')
    for name, settings in local.items():
        if not isinstance(settings, dict) or set(settings) - {'controller','hand','world_from_base_xyz_xyzw'}:
            raise ValueError(f'{name}: invalid arm_settings')
        for section in ('controller','hand'):
            values = settings.get(section, {})
            if not isinstance(values, dict) or set(values) - ALLOWED[section]:
                raise ValueError(f'{name}: invalid {section} override')
    return data
