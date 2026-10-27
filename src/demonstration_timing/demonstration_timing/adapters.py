"""Small-message adapters; no control publishers and no invented execution times."""
import math


def finite(values):
    return bool(values) and all(isinstance(x,(int,float)) and math.isfinite(x) for x in values)


def command_key(message):
    return [message.get('session_id',''),message.get('sequence',0),message.get('reference_space_generation',0)]


def adapt(topic, message, context):
    """Input is a lossless ROS-field dictionary (nonfinite values retained as null).

    context supplies host identity, local arrival clocks and ROS-clock segment.
    Caller retains all original fields in payload, including all native timestamps.
    """
    header=message.get('header',{}).get('stamp',{})
    stamp=header.get('sec',0)*1_000_000_000+header.get('nanosec',0)
    base=dict(context,header_ros_ns=stamp,payload=message,uncertainty_ns=None,
              clock_id=context['ros_clock_id'],timestamp_ns=stamp,
              semantic='ROS status/sample time; physical capture uncertainty unspecified',
              valid=stamp>0,stream=topic.rsplit('/',1)[-1],segment='0')
    session=str(message.get('session_id',''))+':'+str(message.get('reference_space_generation',0))
    if topic.endswith('/tracking_samples'):
        base.update(stream='robot_state',timestamp_ns=message['state_time_pc_monotonic_ns'],
                    clock_id=context['host_clock_id'],clock_segment='0',segment=session+':'+message.get('reason',''),
                    uncertainty_ns=0,valid=message['state_time_pc_monotonic_ns']>0 and finite(message.get('joint_position',[])),
                    semantic='host FCI callback entry observation, not robot encoder acquisition')
        # Only joint positions may be interpolated; original full message is retained.
        app=dict(base,stream='command_application',record_id=context['record_id']+':application',
                 timestamp_ns=message['command_issue_time_pc_monotonic_ns'],
                 command_key=command_key(message),receipt_monotonic_ns=message['command_receipt_time_pc_monotonic_ns'],
                 valid=bool(message.get('motion_enabled') and message.get('requested_pose_valid')
                            and message.get('cartesian_reference_valid') and not message.get('homing_active')
                            and not message.get('safety_fallback') and message['command_issue_time_pc_monotonic_ns']>0),
                 semantic='host torque-output issuance observation, not actuator acknowledgement')
        return [base,app]
    if topic.endswith('/cartesian_teleop_command'):
        base.update(stream='teleoperation_command',clock_id=context['host_clock_id'],clock_segment='0',segment=session,
                    timestamp_ns=message['source_time_pc_monotonic_ns'],
                    uncertainty_ns=message.get('clock_uncertainty_ns'),command_key=command_key(message),
                    valid=bool(message.get('source_time_valid') and message.get('clock_sync_valid') and message.get('input_valid')
                               and message.get('motion_enabled') and message['source_time_pc_monotonic_ns']>0),
                    semantic='Quest acquisition mapped to host monotonic; command publication/application are distinct')
    elif topic.endswith('/controller_state'):
        base.update(stream='quest_state',clock_id=context['host_clock_id'],clock_segment='0',segment=session,
                    timestamp_ns=message['source_time_pc_monotonic_ns'],uncertainty_ns=message.get('clock_uncertainty_ns'),
                    valid=bool(message.get('source_time_valid') and message.get('clock_sync_valid') and message.get('connection_valid')
                               and message.get('position_valid') and message.get('orientation_valid')),
                    command_key=command_key(message),semantic='Quest acquisition converted by existing USB clock synchronization')
    elif topic.endswith('/joint_states'):
        base.update(stream='joint_states',segment='joint_names:'+','.join(message.get('name',[])),valid=stamp>0 and finite(message.get('position',[])),
                    payload={'joint_position':message.get('position',[]),'original_joint_state':message},semantic='ROS projection of host callback state time; prefer TrackingSample native monotonic')
    elif topic.endswith('/gripper_events'):
        width=message.get('width')
        base.update(stream='gripper_state',valid=bool(stamp>0 and message.get('calibrated') and not message.get('busy')
                    and message.get('state')!='unknown' and width is not None and math.isfinite(width)),
                    semantic='status construction; readback age/operation timing unavailable; no gripper interpolation')
    return [base]
