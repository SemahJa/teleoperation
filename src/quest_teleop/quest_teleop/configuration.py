"""Validated per-arm mapper configuration and explicit rosbag recording plans."""
from dataclasses import dataclass
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import re
import uuid

import yaml


_TOKEN = re.compile(r'^[A-Za-z_][A-Za-z0-9_]*$')


def full_topic(value, namespace='/'):
    if not isinstance(value, str) or not value or value.endswith('/'):
        raise ValueError(f'invalid ROS topic: {value!r}')
    resolved = value if value.startswith('/') else namespace.rstrip('/') + '/' + value
    if not all(_TOKEN.fullmatch(part) for part in resolved[1:].split('/')):
        raise ValueError(f'invalid ROS topic: {resolved!r}')
    return resolved


def read_mapping(path):
    with Path(path).expanduser().open(encoding='utf-8') as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise ValueError(f'{path}: expected a YAML mapping')
    return data


def number(value, field, minimum=0.0, inclusive=False):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f'{field} must be finite numeric data')
    if value < minimum or (not inclusive and value == minimum):
        raise ValueError(f'{field} must be {">=" if inclusive else ">"} {minimum}')
    return float(value)


def boolean(value, field):
    if not isinstance(value, bool):
        raise ValueError(f'{field} must be true or false')
    return value


def launch_bool(value):
    if value.lower() not in ('true', 'false'):
        raise ValueError(f'expected true/false launch argument, got {value!r}')
    return value.lower() == 'true'


@dataclass
class ArmConfig:
    name: str
    namespace: str
    parameters: dict
    robot_ip: str
    state_topic: str


_DEFAULTS = dict(translation_scale=1.0, rotation_scale=1.0, feedforward_scale=1.0,
                 twist_filter_cutoff_hz=10.0, command_timeout_ms=100,
                 max_clock_uncertainty_ms=10.0, max_source_future_ms=10.0,
                 min_derivative_dt_ms=1.0, max_derivative_dt_ms=100.0,
                 trigger_engage=0.75, trigger_release=0.25)


def load_arms(path):
    data = read_mapping(path)
    if set(data) != {'arms'} or not isinstance(data['arms'], dict) or not data['arms']:
        raise ValueError('arms configuration must contain one nonempty arms mapping')
    result = []
    namespaces, frames, outputs, inputs, ips = set(), set(), set(), set(), set()
    allowed = set(_DEFAULTS) | {'namespace', 'base_frame', 'controller', 'robot_ip',
        'controller_enabled', 'operator_confirm', 'command_topic', 'target_pose_topic',
        'measured_pose_topic', 'controller_state_topic', 'state_topic', 'reference_to_robot_xyzw'}
    for name, raw in data['arms'].items():
        if not isinstance(name, str) or not _TOKEN.fullmatch(name) or not isinstance(raw, dict):
            raise ValueError('each arm needs a valid name and parameter mapping')
        unknown = set(raw) - allowed
        if unknown:
            raise ValueError(f'{name}: unknown configuration keys: {sorted(unknown)}')
        namespace = raw.get('namespace', '/' + name)
        namespace = full_topic(namespace)
        if namespace in namespaces:
            raise ValueError('arm namespaces must be unique')
        namespaces.add(namespace)
        frame = raw.get('base_frame', namespace.lstrip('/') + '/panda_link0')
        if not isinstance(frame, str) or not frame or frame.startswith('/') or frame in frames:
            raise ValueError('base frames must be distinct nonempty frame IDs without leading slash')
        frames.add(frame)
        controller = raw.get('controller', 'right')
        if controller not in ('left', 'right'):
            raise ValueError(f'{name}: controller must be left or right')
        for key in ('controller_enabled', 'operator_confirm'):
            if boolean(raw.get(key, False), key):
                raise ValueError(f'{name}: robot-free launch does not activate hardware; {key} must be false')
        ip = raw.get('robot_ip', '')
        if not isinstance(ip, str) or (ip and ip in ips):
            raise ValueError('nonempty robot_ip values must be unique strings')
        if ip:
            ips.add(ip)
        p = dict(_DEFAULTS)
        p.update({key: raw[key] for key in _DEFAULTS if key in raw})
        for key in _DEFAULTS:
            p[key] = number(p[key], f'{name}.{key}', inclusive=key in (
                'feedforward_scale', 'max_source_future_ms', 'trigger_release'))
        if p['feedforward_scale'] > 1.0 or not (p['trigger_release'] < p['trigger_engage'] <= 1.0):
            raise ValueError(f'{name}: invalid feedforward scale or trigger hysteresis')
        if p['min_derivative_dt_ms'] > p['max_derivative_dt_ms']:
            raise ValueError(f'{name}: derivative interval limits reversed')
        if not p['command_timeout_ms'].is_integer():
            raise ValueError('command_timeout_ms must be an integer number of milliseconds')
        p['command_timeout_ms'] = int(p['command_timeout_ms'])
        basis = raw.get('reference_to_robot_xyzw', [0.0, 0.0, 0.0, 1.0])
        if not isinstance(basis, list) or len(basis) != 4 or any(
                isinstance(x, bool) or not isinstance(x, (float, int)) or not math.isfinite(x) for x in basis):
            raise ValueError(f'{name}: reference_to_robot_xyzw needs four finite numbers')
        norm = math.sqrt(sum(x*x for x in basis))
        if not math.isfinite(norm) or norm < 1e-9:
            raise ValueError(f'{name}: invalid calibration quaternion')
        p.update(controller_id=controller, base_frame=frame,
                 reference_to_robot_xyzw=[float(x/norm) for x in basis])
        for key, default in [('measured_pose_topic', 'teleop_anchor_pose'),
                             ('command_topic', 'cartesian_teleop_command'),
                             ('monitor_topic', 'target_pose')]:
            config_key = 'target_pose_topic' if key == 'monitor_topic' else key
            topic = full_topic(raw.get(config_key, default), namespace)
            if key == 'measured_pose_topic':
                if topic in inputs:
                    raise ValueError('measured pose topics must be unique per arm')
                inputs.add(topic)
            else:
                if topic in outputs:
                    raise ValueError('command and monitor topics must be unique per arm')
                outputs.add(topic)
            p[key] = topic
        p['controller_state_topic'] = full_topic(raw.get('controller_state_topic',
            f'/quest_usb_bridge/{controller}/controller_state'), namespace)
        result.append(ArmConfig(name, namespace, p, ip,
                                full_topic(raw.get('state_topic', 'robot_state'), namespace)))
    if outputs & inputs:
        raise ValueError('mapper outputs must not alias measured-pose inputs')
    if any(arm.parameters['controller_state_topic'] in outputs | inputs for arm in result):
        raise ValueError('raw controller input must not alias mapper output or measured pose')
    return result


def default_recording_topics(arms):
    topics = ['/quest_usb_bridge/left/controller_state']
    for arm in arms:
        p = arm.parameters
        topics.extend([p['controller_state_topic'], p['command_topic'], p['monitor_topic'],
                       p['measured_pose_topic'], arm.state_topic,
                       full_topic('watchdog_events', arm.namespace),
                       full_topic('gripper_events', arm.namespace),
                       full_topic('controller_status', arm.namespace),
                       full_topic('homing_status', arm.namespace),
                       full_topic('cbf_status', arm.namespace),
                       full_topic('joint_states', arm.namespace)])
    return list(dict.fromkeys(topics))


@dataclass
class RecordingPlan:
    topics: list
    output: str
    command: list


def recording_plan(path, arms, topics_override='', output_root_override=''):
    config = read_mapping(path)
    if set(config) - {'include_default_topics', 'topics', 'output_root', 'storage_id'}:
        raise ValueError('unknown recording configuration key')
    if topics_override:
        topics = json.loads(topics_override)
    else:
        topics = config.get('topics', [])
        if not isinstance(topics, list):
            raise ValueError('recording topics must be a list')
        if boolean(config.get('include_default_topics', True), 'include_default_topics'):
            topics = default_recording_topics(arms) + topics
    if not isinstance(topics, list) or not topics:
        raise ValueError('enabled recording needs at least one explicitly selected topic')
    topics = list(dict.fromkeys(full_topic(topic) for topic in topics))
    root = output_root_override or config.get('output_root', '')
    if not isinstance(root, str) or not root or not Path(root).expanduser().is_absolute():
        raise ValueError('recording output_root must be an absolute container path')
    storage = config.get('storage_id', 'sqlite3')
    if not isinstance(storage, str) or not _TOKEN.fullmatch(storage):
        raise ValueError('invalid rosbag storage plugin name')
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    output = str(Path(root).expanduser() / f'quest_{stamp}_{uuid.uuid4().hex[:8]}')
    # Arguments are passed directly; no shell or topic glob expansion.
    return RecordingPlan(topics, output, ['ros2', 'bag', 'record', '--storage', storage,
                                         '--output', output, *topics])


def select_arms(arms, mode='single', single_controller='right'):
    """Select active owners; a one-arm configuration can use either hand."""
    from dataclasses import replace
    if mode not in ('single', 'dual') or single_controller not in ('left', 'right'):
        raise ValueError('mode must be single/dual; single_controller must be left/right')
    if mode == 'dual':
        if len(arms) != 2 or {a.parameters['controller_id'] for a in arms} != {'left', 'right'}:
            raise ValueError('dual mode needs exactly two arms assigned to left and right')
        return sorted(arms, key=lambda a: a.parameters['controller_id'])
    matches = [a for a in arms if a.parameters['controller_id'] == single_controller]
    if len(arms) == 1:
        arm = arms[0]
    elif len(matches) == 1:
        arm = matches[0]
    else:
        raise ValueError('single mode needs one unambiguous configured arm for the selected controller')
    params = dict(arm.parameters)
    old = params['controller_id']
    params['controller_id'] = single_controller
    if params['controller_state_topic'] == f'/quest_usb_bridge/{old}/controller_state':
        params['controller_state_topic'] = f'/quest_usb_bridge/{single_controller}/controller_state'
    elif old != single_controller:
        raise ValueError('set the custom controller_state_topic for the selected single controller in arms_config')
    return [replace(arm, parameters=params)]
