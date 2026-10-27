#!/usr/bin/env python3
"""Inspect original robot-native/host receipt provenance in an existing episode."""
import argparse
from collections import Counter,defaultdict
import json,math,statistics
from pathlib import Path
import yaml
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from demonstration_timing.clocks import ClockModel


def stats(values):
    return {'count':len(values),'min':min(values),'median':statistics.median(values),'max':max(values)} if values else {'count':0}


def inspect(episode):
    bag=Path(episode).resolve();manifest=json.loads((bag/'episode_manifest.json').read_text())
    md=yaml.safe_load((bag/'metadata.yaml').read_text())['rosbag2_bagfile_information']
    r=rosbag2_py.SequentialReader();r.open(rosbag2_py.StorageOptions(uri=str(bag),storage_id=md['storage_identifier']),rosbag2_py.ConverterOptions('',''))
    selected=[t.name for t in r.get_all_topics_and_types() if t.name.endswith(('/joint_states','/measured_ee_pose','/read_only_state'))]
    r.set_filter(rosbag2_py.StorageFilter(topics=selected));meta=[];joints={};poses={};counts=Counter()
    while r.has_next():
        topic,raw,_=r.read_next();counts[topic]+=1;namespace=topic.rsplit('/',1)[0]
        if topic.endswith('/read_only_state'):
            doc=json.loads(deserialize_message(raw,String).data)
            if doc.get('schema')=='panda_read_only_state_v1':meta.append((namespace,doc))
        else:
            message=deserialize_message(raw,JointState if topic.endswith('/joint_states') else PoseStamped)
            stamp=message.header.stamp.sec*10**9+message.header.stamp.nanosec
            (joints if topic.endswith('/joint_states') else poses)[(namespace,stamp)]=message
    models=[ClockModel(**m) for m in manifest.get('clock_models',[])];matches=pose_matches=0;missing_joints=missing_pose=0;errors=[];projection=[];native_steps=[];host_steps=[];sequences=[];prior={};native_q_matches=0
    for namespace,d in meta:
        key=(namespace,d['header_ros_ns']);joint=joints.get(key);pose=poses.get(key)
        if not d.get('read_only') or not d.get('hardware_backend') or d.get('execution_timestamp') is not None:errors.append('invalid read-only/execution provenance')
        if d['host_clock_id']!=manifest.get('host_clock_id'):errors.append('host monotonic identity differs from recorder')
        if joint is None:missing_joints+=1
        elif list(joint.position)!=d['joint_position'] or list(joint.velocity)!=d['joint_velocity'] or list(joint.effort)!=d['measured_torque']:errors.append('standard JointState differs from native measured values')
        else:matches+=1
        if pose is None:missing_pose+=1
        elif [pose.pose.position.x,pose.pose.position.y,pose.pose.position.z]!=d['O_T_EE'][12:15]:errors.append('derived Pose translation differs from raw SDK transform')
        else:pose_matches+=1
        native=json.loads(d['native_robot_state_text'])
        if native['q']==d['joint_position'] and native['dq']==d['joint_velocity']:native_q_matches+=1
        else:errors.append('serialized original SDK state differs from measured state')
        previous=prior.get((namespace,d['session_id']))
        if previous:
            native_steps.append(d['robot_time_ms']-previous['robot_time_ms']);host_steps.append(d['receipt_host_monotonic_ns']-previous['receipt_host_monotonic_ns']);sequences.append(d['sample_sequence']-previous['sample_sequence'])
        prior[(namespace,d['session_id'])]=d
        applicable=[m for m in models if m.source==manifest.get('ros_clock_id') and m.reference==d['host_clock_id'] and m.valid_from_ns<=d['header_ros_ns']<=m.valid_to_ns]
        if len(applicable)==1:
            m=applicable[0];projection.append(m.convert(d['header_ros_ns'],m.source,m.segment)-d['receipt_host_monotonic_ns'])
    if not meta:errors.append('no native read-only provenance samples recorded')
    if any(x<=0 for x in native_steps) or any(x<=0 for x in host_steps):errors.append('non-monotonic native/receipt timestamps')
    alignment=manifest.get('alignment',{})
    return {'episode':str(bag),'message_counts':dict(counts),'native_metadata_samples':len(meta),'joint_samples_matched_exactly':matches,'pose_translations_matched_exactly':pose_matches,'native_state_q_dq_preserved_exactly':native_q_matches,'metadata_without_joint_sample':missing_joints,'metadata_without_pose_sample':missing_pose,'robot_modes':dict(Counter(d['robot_mode'] for _,d in meta)),'robot_addresses':sorted({d.get('robot_address','') for _,d in meta}),'robot_native_clock_ids':sorted({d['robot_clock_id'] for _,d in meta}),'native_sample_step_ms':stats(native_steps),'host_receipt_step_ns':stats(host_steps),'sample_sequence_steps':stats(sequences),'read_call_wait_ns':stats([d['receipt_host_monotonic_ns']-d['read_before_host_monotonic_ns'] for _,d in meta]),'ros_stamp_mapped_to_host_minus_read_receipt_ns':stats(projection),'raw_rotation_rounding_residual':stats([d['raw_rotation_orthogonality_max_abs_error'] for _,d in meta]),'derived_rotation_projection_correction':stats([d['derived_pose_rotation_projection_max_abs_error'] for _,d in meta]),'alignment':alignment,'errors':sorted(set(errors)),'source_capture_uncertainty_ns':None,'execution_timing_available':False,'semantics':'ROS joint headers are read-return observation times. Native robot time is preserved, not equated to host time. Read-wait duration and host clock-fit residual do not bound encoder acquisition age or camera exposure timing. Robot state alignment remains provisional; no gripper/control measurements invented.'}


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('episode');p.add_argument('--output');a=p.parse_args();result=inspect(a.episode)
    out=Path(a.output) if a.output else Path(a.episode)/'read_only_state_validation.json';out.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2));raise SystemExit(bool(result['errors']))


if __name__=='__main__':main()
