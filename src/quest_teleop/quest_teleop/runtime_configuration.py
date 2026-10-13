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
    data = feature_document(read_mapping(path))
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

# A document with `pipeline` is the combined-bringup schema. Standalone feature
# documents retain their original schemas and installed package defaults.
PIPELINE_DEFAULTS = dict(mode='single', single_controller='right', backend='mock',
    start_controller=True, homing_enabled=True, cbf_enabled=True, start_mapper=True,
    start_hand=True, start_bridge=False, mock_measured_pose=False, recording_enabled=True)
BRIDGE_DEFAULTS = dict(namespace='/', listen_port=45454, bind_address='127.0.0.1',
    command_timeout_ms=100, max_clock_uncertainty_ms=10, max_clock_sync_age_ms=3000,
    max_source_future_ms=10, client_receive_timeout_ms=500)
RECORDING_DEFAULTS = dict(mode='menu', namespace='/quest', left_controller_state_topic='',
    include_default_topics=True, topics=[], output_root='/ws/log/quest_bags', storage_id='sqlite3', menu_debounce_seconds=0.35)
UNIFIED_KEYS = set(ALLOWED) | {'pipeline', 'quest', 'arms', 'arm_settings', 'recording'}


def unified_settings(data):
    """Validate the combined document before any launch can open hardware."""
    if set(data) - UNIFIED_KEYS:
        raise ValueError('unknown unified runtime configuration section')
    for key in UNIFIED_KEYS:
        if key in data and not isinstance(data[key], dict):
            raise ValueError(f'{key} must be a mapping')
    if not data.get('arms'):
        raise ValueError('unified runtime requires a nonempty arms mapping')
    raw = data.get('pipeline', {})
    if set(raw) - set(PIPELINE_DEFAULTS):
        raise ValueError('unknown pipeline setting; operator_confirm is a launch-only confirmation')
    pipeline = dict(PIPELINE_DEFAULTS, **raw)
    for key, default in PIPELINE_DEFAULTS.items():
        if isinstance(default, bool) and not isinstance(pipeline[key], bool):
            raise ValueError(f'pipeline.{key} must be true/false')
    for key, values in dict(mode=('single','dual'), single_controller=('left','right'), backend=('mock','hardware')).items():
        if pipeline[key] not in values: raise ValueError(f'invalid pipeline.{key}')
    quest = data.get('quest', {})
    if set(quest) - {'bridge'} or not isinstance(quest.get('bridge', {}), dict):
        raise ValueError('quest supports a bridge mapping; mapper settings belong to arms')
    raw = quest.get('bridge', {})
    if set(raw) - set(BRIDGE_DEFAULTS): raise ValueError('unknown Quest bridge setting')
    bridge = dict(BRIDGE_DEFAULTS, **raw)
    for key, default in BRIDGE_DEFAULTS.items():
        if isinstance(default, int) and (isinstance(bridge[key], bool) or not isinstance(bridge[key], int)):
            raise ValueError(f'quest.bridge.{key} must be an integer')
        if isinstance(default, str) and (not isinstance(bridge[key], str) or not bridge[key]):
            raise ValueError(f'quest.bridge.{key} must be a nonempty string')
    if not 1 <= bridge['listen_port'] <= 65535: raise ValueError('listen_port must be 1..65535')
    if any(bridge[k] <= 0 for k in ('command_timeout_ms','max_clock_uncertainty_ms',
            'max_clock_sync_age_ms','client_receive_timeout_ms')) or bridge['max_source_future_ms'] < 0:
        raise ValueError('invalid bridge freshness limit')
    ns = bridge['namespace']
    from quest_teleop.configuration import full_topic
    if ns != '/': full_topic(ns)
    raw = data.get('recording', {})
    if set(raw) - set(RECORDING_DEFAULTS): raise ValueError('unknown recording setting')
    recording = dict(RECORDING_DEFAULTS, **raw)
    if recording['mode'] not in ('menu','continuous'): raise ValueError('recording.mode must be menu/continuous')
    if not isinstance(recording['namespace'], str): raise ValueError('invalid recording namespace')
    if recording['namespace'] != '/': full_topic(recording['namespace'])
    import math
    debounce = recording['menu_debounce_seconds']
    if isinstance(debounce, bool) or not isinstance(debounce, (float,int)) or not math.isfinite(debounce) or debounce < .35:
        raise ValueError('recording.menu_debounce_seconds must be finite and at least 0.35')
    left = recording['left_controller_state_topic']
    if not isinstance(left, str): raise ValueError('invalid Menu input topic')
    if left: full_topic(left)
    return pipeline, bridge, recording


def feature_document(data):
    if 'pipeline' in data:
        unified_settings(data)
        return {key: value for key, value in data.items() if key in set(ALLOWED) | {'arm_settings'}}
    return data


def bridge_controller_topic(data, side):
    namespace = unified_settings(data)[1]['namespace'] if 'pipeline' in data else '/'
    return namespace.rstrip('/') + '/quest_usb_bridge/' + side + '/controller_state'
