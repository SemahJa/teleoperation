"""Native input evidence and independent supervision; no robot publishers."""
from collections import Counter
import gzip
import json
import numpy as np
from recording.clutch_filter import latched_axis


def choose_command(commands,key,source):
    """Disarmed watchdog repeats cannot erase an earlier applied command."""
    candidates=commands.get(tuple(key),[])
    if isinstance(candidates,dict):candidates=[candidates]
    valid=[c for c in candidates if c['valid'] and c['source']==source]
    unique={json.dumps(c,sort_keys=True):c for c in valid}
    return next(iter(unique.values())) if len(unique)==1 else None


def read_native(bag,models,ledger,convert_ros):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from rosidl_runtime_py.convert import message_to_ordereddict
    from recording.export_lerobot import pose,key
    topics=['/quest_usb_bridge/right/controller_state','/panda_1/cartesian_teleop_command','/panda_1/gripper_events','/panda_1/joint_states']
    reader=rosbag2_py.SequentialReader();reader.open(rosbag2_py.StorageOptions(uri=str(bag),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
    types={t.name:t.type for t in reader.get_all_topics_and_types()}
    topics += [t for t,typ in types.items() if typ=='realsense2_camera_msgs/msg/Metadata']
    reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
    classes={};quest=[];gripper=[];commands={};counts=Counter();errors=Counter();examples={};metadata={};last_completion=None;joint_bad=0
    ledger.parent.mkdir(parents=True,exist_ok=True)
    with gzip.open(ledger,'wt') as f:
        while reader.has_next():
            topic,raw,receipt=reader.read_next();counts[topic]+=1
            try:
                if topic not in classes:classes[topic]=get_message(types[topic])
                m=deserialize_message(raw,classes[topic]);doc=message_to_ordereddict(m)
            except Exception as exc:
                errors[topic]+=1;examples.setdefault(topic,str(exc));continue
            f.write(json.dumps(dict(topic=topic,bag_receipt_ros_ns=receipt,message=doc),separators=(',',':'))+'\n')
            if topic.endswith('/controller_state'):
                native=int(m.source_time_pc_monotonic_ns)
                header=m.header.stamp.sec*10**9+m.header.stamp.nanosec;t=native or convert_ros(header,models)
                if t is None:continue
                valid=bool(native>0 and m.connection_valid and m.session_focused and m.source_time_valid and m.clock_sync_valid and m.clock_uncertainty_ns<=10_000_000 and m.reference_space=='LOCAL' and m.session_id)
                x=float(getattr(m,'thumbstick_x',0.));stick=bool(getattr(m,'thumbstick_active',False))
                quest.append(dict(t=t,header=header,receipt=receipt,identity=(m.session_id,int(m.reference_space_generation)),sequence=int(m.sequence),valid=valid,
                    pose_valid=bool(m.pose_action_active and m.position_valid and m.orientation_valid and m.position_tracked and m.orientation_tracked),
                    trigger=float(m.trigger_value),trigger_active=bool(m.trigger_active),x=x,stick=stick and np.isfinite(x) and abs(x)<=1,
                    acquisition=int(m.acquisition_time_ns),location=int(m.openxr_location_time_ns),uncertainty=int(m.clock_uncertainty_ns)))
            elif topic.endswith('/cartesian_teleop_command'):
                c=dict(source=int(m.source_time_pc_monotonic_ns),acquisition=int(m.source_acquisition_time_ns),location=int(m.openxr_location_time_ns),
                    valid=bool(m.motion_enabled and m.input_valid and m.source_time_valid and m.clock_sync_valid),pose=pose(m.target_pose))
                commands.setdefault(key(m),[]).append(c)
            elif topic.endswith('/gripper_events'):
                header=m.header.stamp.sec*10**9+m.header.stamp.nanosec;t=convert_ros(header,models)
                completed=int(m.operations_completed)
                fresh=last_completion is not None and completed>last_completion;last_completion=completed
                if t is None:continue
                gripper.append(dict(t=t,header=header,receipt=receipt,busy=bool(m.busy),calibrated=bool(m.calibrated),hardware=bool(m.hardware_backend),
                    operation=m.operation,state=m.state,width=float(m.width),maximum_width=float(m.maximum_width),grasped=bool(m.grasp_confirmed),fresh_readback=fresh,
                    started=int(m.operations_started),completed=completed,neutral_required=bool(m.neutral_required),force_command_n=float(m.commanded_grasp_force_n),message=m.message))
            elif topic.endswith('/joint_states'):
                if len(m.name)!=len(m.position) or not np.isfinite(m.position).all() or (m.velocity and (len(m.velocity)!=len(m.name) or not np.isfinite(m.velocity).all())):joint_bad+=1
            else:
                meta=json.loads(m.json_data);entry=metadata.setdefault(topic,dict(keys=set(),clock_domains=set(),first=None,last=None,duplicates=0,estimated_frame_number_gaps=0,nonmonotonic_frame_numbers=0))
                entry['keys'].update(meta);entry['clock_domains'].add(str(meta.get('clock_domain','not_recorded')))
                frame=meta.get('frame_number')
                if frame is not None:
                    n=int(frame);last=entry['last']
                    if last is not None:
                        if n==last:entry['duplicates']+=1
                        elif n<last:entry['nonmonotonic_frame_numbers']+=1
                        else:entry['estimated_frame_number_gaps']+=max(0,n-last-1)
                    if entry['first'] is None:entry['first']=n
                    entry['last']=n
    del reader
    if quest:
        directions=latched_axis([q['x'] for q in quest],[q['valid'] and q['stick'] for q in quest],[q['identity'] for q in quest],np.array([q['t'] for q in quest]),initial_neutral_required=False)
        for q,d in zip(quest,directions):q['stick_direction']=int(d)
    for entry in metadata.values():entry['keys']=sorted(entry['keys']);entry['clock_domains']=sorted(entry['clock_domains'])
    return quest,gripper,commands,dict(counts=counts,decode_errors=errors,error_examples=examples,joint_state_invalid_messages=joint_bad,camera_metadata=metadata,ledger=str(ledger),gripper_convention='right_thumbstick_x_negative_closes_positive_opens; unavailable_legacy_axes_masked')


def summarize_supervision(streams,arrays,segments,tolerance_ns=50_000_000):
    qs=streams['quest'];rows=len(arrays.get('action',[]));sample=Counter();by_t={q['t']:q for q in qs};runs=[];current=None;previous=None
    for q in qs:
        d=q['stick_direction'] if q['valid'] and q['stick'] else 0
        reset=previous is not None and (q['identity']!=previous['identity'] or not 0<q['t']-previous['t']<=100_000_000)
        if current and (current['direction']!=d or reset):
            current.update(end_ns=previous['t'],next_direction=d,next_time_ns=q['t']);runs.append(current);current=None
        if d and current is None:current=dict(direction=d,start_ns=q['t'])
        previous=q
    if current:current.update(end_ns=previous['t'],next_direction=None,next_time_ns=None);runs.append(current)
    kept=[];directions=[];gripper_only=0;arm_only=0;both=0;placeholder=0
    for i in range(rows):
        mask=arrays['action_valid'][i];arm=bool(np.all(mask[:7]));grip=bool(mask[7]);q=by_t.get(int(arrays['source.quest_source_time_ns'][i,0]))
        if grip:
            assert q and q['valid'] and q['stick']
            assert abs(float(arrays['action'][i,7])-q['x'])<1e-6,'Active gripper input altered'
            d=q['stick_direction'];sample[{1:'opening',-1:'closing',0:'neutral'}[d]]+=1
            kept.append(q['t']);directions.append(d)
            if d and not arm:gripper_only+=1
            if d and float(arrays['action'][i,7])==0:placeholder+=1
        else:sample['invalid']+=1
        arm_only+=int(arm and not grip);both+=int(arm and grip)
    for run in runs:
        run['retained_active_samples']=sum(run['start_ns']<=t<=run['end_ns'] and d==run['direction'] for t,d in zip(kept,directions))
        run['neutral_edge_retained']=None if run['next_direction']!=0 else any(run['next_time_ns']<=t<=run['next_time_ns']+200_000_000 and d==0 for t,d in zip(kept,directions))
        run['covered_by_grid']=run['retained_active_samples']>0
        if not run['covered_by_grid']:run['unrepresented_reason']='No eligible 30Hz observation/action sample; inspect missing RGB/state, clock validity or subframe pulse. Native run preserved in ledger.'
    images={}
    for name,values in streams.items():
        if not name.endswith(('_rgb','_depth')):continue
        stamps=np.array([r['header'] for r in values],np.int64);diff=np.diff(stamps)
        images[name]=dict(mapped_frames=len(values),duplicates=int((diff==0).sum()),nonmonotonic=int((diff<0).sum()),estimated_header_gaps=int(np.maximum(np.rint(diff[diff>50_000_000]/(1e9/30)).astype(int)-1,0).sum()),meaning='Gaps observed in recorded mapped headers; publisher vs recorder attribution unknown')
    operations=Counter();previous=None
    for g in streams['gripper']:
        if previous is None:
            if g['busy']:operations['left_censored_'+g['operation']]+=1
        elif g.get('started',0)>previous.get('started',0):operations[g['operation']]+=1
        previous=g
    return dict(native_arm_enabled_samples=sum(bool(r['motion'] and not r['homing'] and not r['fallback']) for r in streams.get('state',[])),native_gripper_busy_samples=sum(bool(r['hardware'] and r['busy'] and r['operation'] in ['open','close']) for r in streams['gripper']),samples_30hz={k:sample[k] for k in ['opening','closing','neutral','invalid']},gripper_only_non_neutral_frames=gripper_only,
        arm_only_valid_frames=arm_only,both_valid_frames=both,active_gripper_replaced_with_zero=placeholder,native_gripper_runs=runs,
        unrepresented_gripper_runs=sum(not r['covered_by_grid'] for r in runs),unrepresented_neutral_edges=sum(r['neutral_edge_retained'] is False for r in runs),
        observed_operation_start_evidence=dict(operations),confirmed_grasp_statuses=sum(g['grasped'] for g in streams['gripper']),image_timing=images,
        note='Command statuses are issue evidence, not exact SDK dispatch or physical execution timestamps. Neutral stops unfinished motion and preserves an existing confirmed grasp.')
