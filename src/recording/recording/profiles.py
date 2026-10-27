"""Explicit topic/QoS profiles. No hardware launch or source resampling."""
from quest_teleop.configuration import full_topic

CAMERA_SUFFIXES = ('color/image_raw','aligned_depth_to_color/image_raw',
                   'color/camera_info','aligned_depth_to_color/camera_info',
                   'color/metadata','depth/metadata')
TIMING_TOPICS = ('/demonstration_timing/clock_observations','/demonstration_timing/recording_health')


def demonstration_topics(arms):
    result = [f'/cameras/{role}/{suffix}' for role in ('wrist','scene') for suffix in CAMERA_SUFFIXES]
    result += ['/tf','/tf_static',*TIMING_TOPICS]
    for arm in arms:
        result += [full_topic(name,arm.namespace) for name in
                   ('joint_states','tracking_samples','tracking_status','target_pose','teleop_anchor_pose',
                    'measured_ee_pose','read_only_state','gripper_events','controller_status','homing_status','cbf_status','watchdog_events')]
        result.append(arm.state_topic)
    # The existing Hand worker exposes status/events, not a separate command or
    # continuously timestamped measured-state topic. Raw Quest carries grip/buttons.
    return list(dict.fromkeys(result))


def qos_overrides(topics):
    result={}
    for topic in topics:
        if topic=='/tf_static' or topic.endswith('/gripper_events'):
            result[topic]={'reliability':'reliable','durability':'transient_local','history':'keep_last','depth':100}
        elif topic.startswith('/cameras/') or topic in ('/tf',*TIMING_TOPICS) or topic.endswith(('/tracking_samples','/joint_states','/controller_state','/cartesian_teleop_command','/measured_ee_pose','/read_only_state')):
            result[topic]={'reliability':'best_effort','durability':'volatile','history':'keep_last','depth':30}
    return result


def selected_topics(topics,inactive,published,late):
    eligible=[t for t in topics if not any(t==ns or t.startswith(ns+'/') for ns in inactive)]
    return eligible if late else [t for t in eligible if t in published]
