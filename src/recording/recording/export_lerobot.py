#!/usr/bin/env python3
"""Read-only ROS2 -> official LeRobot v3 export, splitting removed clutch pauses.

Requires the pinned dataset-only environment documented in LEROBOT_EXPORT.md.
No ROS publishers, robot connections, raw-data modification, or Hub uploads.
"""
import argparse
from collections import Counter, defaultdict
from dataclasses import asdict
import hashlib
import gzip
import importlib.metadata
import json
from pathlib import Path
import shutil
import subprocess
import sys
import time
import numpy as np
import yaml
from recording.clutch_filter import TemporalIndex,latched_axis,context_mask,continuous_segments,recorded_arm_gate
from recording.image_tools import image_view
from demonstration_timing.clocks import ClockModel,ClockError
from recording.supervision import choose_command,read_native,summarize_supervision

VERSION=3
SERIALS={'wrist':'238222073675','scene':'934222072657'}
ROLES=('wrist','scene')
LOW_TOPICS={'/panda_1/tracking_samples':'tracking','/panda_1/cartesian_teleop_command':'command',
 '/quest_usb_bridge/right/controller_state':'quest','/panda_1/gripper_events':'gripper'}
TASK='Grasp and lift the cable.'


def dump(path,value):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    tmp=path.with_suffix(path.suffix+'.tmp');tmp.write_text(json.dumps(value,indent=2,allow_nan=False)+'\n');tmp.replace(path)


def sha(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda:f.read(8*1024*1024),b''):h.update(block)
    return h.hexdigest()


def pose(p):return [p.position.x,p.position.y,p.position.z,p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w]

def good_pose(p):return np.isfinite(p).all() and abs(np.linalg.norm(p[3:])-1)<.01

def key(m):return (m.session_id,int(m.reference_space_generation),int(m.sequence))


def camera_topics(manifest):
    info=manifest.get('camera_device_info',{})
    result={}
    for physical,serial in SERIALS.items():
        found=[role for role,device in info.items() if device.get('serial_number')==serial]
        if len(found)!=1:raise ValueError(f'Unknown/ambiguous camera role for serial {serial}')
        role=found[0]
        result[physical+'_rgb']=f'/cameras/{role}/color/image_raw'
        result[physical+'_depth']=f'/cameras/{role}/aligned_depth_to_color/image_raw'
    return result


def convert_ros(stamp,models):
    candidates=[]
    for m in models:
        try:candidates.append(m.convert(int(stamp),m.source,m.segment))
        except ClockError:pass
    return candidates[0] if len(candidates)==1 else None


def read_episode(bag,manifest,topics):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from rosidl_runtime_py.convert import message_to_ordereddict
    models=[ClockModel(**m) for m in manifest.get('clock_models',[]) if m['source']==manifest.get('ros_clock_id') and m['reference']==manifest.get('host_clock_id')]
    if not models:raise ValueError('No measured ROS -> host-monotonic clock relationship')
    if len({m.reference for m in models})!=1:raise ValueError('Incompatible host reference clocks')
    streams=defaultdict(list);commands={};counts=Counter();camera_info={};classes={};completion=None
    images={topic:name for name,topic in topics.items()}
    info_topics={t.rsplit('/',1)[0]+'/camera_info':name for name,t in topics.items()}
    reader=rosbag2_py.SequentialReader();reader.open(rosbag2_py.StorageOptions(uri=str(bag),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
    types={t.name:t.type for t in reader.get_all_topics_and_types()}
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(LOW_TOPICS)+list(images)+list(info_topics)))
    while reader.has_next():
        topic,raw,received=reader.read_next();ordinal=counts[topic];counts[topic]+=1
        if topic in images:
            h,_=image_view(raw);mapped=convert_ros(h['stamp_ns'],models)
            if mapped is not None:streams[images[topic]].append(dict(t=mapped,header=h['stamp_ns'],receipt=received,ordinal=ordinal,frame_id=h['frame_id']))
            continue
        if topic in info_topics and topic in camera_info:continue
        if topic not in classes:classes[topic]=get_message(types[topic])
        m=deserialize_message(raw,classes[topic])
        if topic in info_topics:
            camera_info[topic]=dict(message_to_ordereddict(m));continue
        kind=LOW_TOPICS[topic]
        if kind=='tracking':
            t=int(m.state_time_pc_monotonic_ns)
            streams['state'].append(dict(t=t,identity=(m.session_id,int(m.reference_space_generation)),key=key(m),
                issue=int(m.command_issue_time_pc_monotonic_ns),receipt=int(m.command_receipt_time_pc_monotonic_ns),
                source=int(m.command_source_time_pc_monotonic_ns),control=int(m.control_time_pc_monotonic_ns),cycle=int(m.control_cycle),
                motion=bool(m.motion_enabled),requested_valid=bool(m.requested_pose_valid),reference_valid=bool(m.cartesian_reference_valid),
                homing=bool(m.homing_active),fallback=bool(m.safety_fallback),hardware=bool(m.hardware_backend),
                q=list(m.joint_position),dq=list(m.joint_velocity),pose=pose(m.measured_pose)))
        elif kind=='command':
            commands[key(m)]=dict(source=int(m.source_time_pc_monotonic_ns),acquisition=int(m.source_acquisition_time_ns),
                location=int(m.openxr_location_time_ns),valid=bool(m.motion_enabled and m.input_valid and m.source_time_valid and m.clock_sync_valid),pose=pose(m.target_pose))
        elif kind=='quest':
            raw_t=int(m.source_time_pc_monotonic_ns)
            # An invalid status may only terminate validity; it is never used as capture data.
            t=raw_t or convert_ros(m.header.stamp.sec*10**9+m.header.stamp.nanosec,models)
            if t is None:continue
            valid=bool(raw_t>0 and m.connection_valid and m.session_focused and m.source_time_valid and m.clock_sync_valid
                       and m.clock_uncertainty_ns<=10_000_000 and m.reference_space=='LOCAL')
            streams['quest'].append(dict(t=t,identity=(m.session_id,int(m.reference_space_generation)),valid=valid,
                pose_valid=bool(m.pose_action_active and m.position_valid and m.orientation_valid and m.position_tracked and m.orientation_tracked),
                trigger=float(m.trigger_value),trigger_active=bool(m.trigger_active),x=float(m.thumbstick_x),stick=bool(m.thumbstick_active),
                acquisition=int(m.acquisition_time_ns),location=int(m.openxr_location_time_ns),uncertainty=int(m.clock_uncertainty_ns)))
        elif kind=='gripper':
            header=m.header.stamp.sec*10**9+m.header.stamp.nanosec;t=convert_ros(header,models)
            fresh_readback=completion is not None and int(m.operations_completed)>completion
            completion=int(m.operations_completed)
            if t is None:continue
            streams['gripper'].append(dict(t=t,header=header,busy=bool(m.busy),calibrated=bool(m.calibrated),hardware=bool(m.hardware_backend),
                operation=m.operation,state=m.state,width=float(m.width),grasped=bool(m.grasp_confirmed),fresh_readback=fresh_readback))
    del reader
    for name in ('state',):
        if not streams[name]:raise ValueError(f'Missing required telemetry: {name}')
    qs=streams['quest'];times=np.array([q['t'] for q in qs],dtype=np.int64);identities=[q['identity'] for q in qs]
    sticks=latched_axis([q['x'] for q in qs],[q['valid'] and q['stick'] for q in qs],identities,times,initial_neutral_required=False)
    for q,stick in zip(qs,sticks):q.update(stick_direction=int(stick))
    return streams,commands,models,camera_info,dict(counts)


def prepare(bag,source_number,work,args):
    manifest_path=bag/'episode_manifest.json';manifest=json.loads(manifest_path.read_text())
    if manifest.get('status')=='recording' or manifest.get('phase') in ('recording','stopping','validating'):
        raise ValueError(f'Active episode: {bag.name}')
    topics=camera_topics(manifest)
    files=list(bag.glob('*.mcap'))+[bag/'metadata.yaml',manifest_path]
    side=Path(str(bag)+'.manifest.json')
    if side.exists():files.append(side)
    inventory={str(p):{'bytes':p.stat().st_size,'sha256':sha(p)} for p in files}
    params=dict(version=VERSION,source_number=source_number,fps=args.fps,tolerance_ms=args.tolerance_ms,pre_ms=args.pre_ms,post_ms=args.post_ms,min_frames=args.min_frames,topics=topics,inventory=inventory)
    plan_json=work/(bag.name+'.json');plan_npz=work/(bag.name+'.npz')
    if plan_json.exists() and plan_npz.exists():
        old=json.loads(plan_json.read_text())
        if old.get('parameters')==params:
            print(json.dumps({'reused_plan':bag.name,'retained':old['summary']['retained_frames']}),flush=True);return old
    # Cache parsed, small telemetry separately from pixels for repeatable selection.
    parsed=work/(bag.name+'.parsed.json.gz')
    cached=None
    if parsed.exists():
        with gzip.open(parsed,'rt') as f:cached=json.load(f)
    if cached and cached.get('inventory')==inventory and cached.get('version') in (2,VERSION):
        streams=cached['streams'];commands={tuple(k):v for k,v in cached['commands']}
        for stream in streams.values():
            for row in stream:
                if 'key' in row:row['key']=tuple(row['key'])
                if 'identity' in row:row['identity']=tuple(row['identity'])
        models=[ClockModel(**m) for m in cached['models']];camera_info=cached['camera_info'];counts=cached['counts']
    else:
        streams,commands,models,camera_info,counts=read_episode(bag,manifest,topics)
        temporary=parsed.with_suffix('.tmp')
        with gzip.open(temporary,'wt') as f:json.dump(dict(version=VERSION,inventory=inventory,streams=streams,commands=list(commands.items()),models=[asdict(m) for m in models],camera_info=camera_info,counts=counts),f)
        temporary.replace(parsed)
    if cached and cached.get('inventory')==inventory and cached.get('version')==VERSION and cached.get('native'):
        native=cached['native']
    else:
        q,g,commands,native=read_native(bag,models,work/(bag.name+'.native.jsonl.gz'),convert_ros)
        streams['quest']=q;streams['gripper']=g
        temporary=parsed.with_suffix('.tmp')
        with gzip.open(temporary,'wt') as f:json.dump(dict(version=VERSION,inventory=inventory,streams=streams,commands=list(commands.items()),models=[asdict(m) for m in models],camera_info=camera_info,counts=counts,native=native),f)
        temporary.replace(parsed)
    indexes={name:TemporalIndex(rows) for name,rows in streams.items() if name!='gripper'}
    issues=[]
    for row in streams['state']:
        if row['issue']>0:issues.append(dict(row,t=row['issue']))
    app_index=TemporalIndex(issues);grip_index=TemporalIndex(streams['gripper'])
    fresh_index=TemporalIndex([r for r in streams['gripper'] if r['fresh_readback'] and r['calibrated'] and not r['busy'] and r['state']!='unknown' and r['hardware'] and np.isfinite(r['width']) and 0<=r['width']<=r.get('maximum_width',.085)+1e-5])
    start=min(r['t'] for r in streams['state']);end=max(r['t'] for r in streams['state'])
    grid=np.arange((end-start)*args.fps//10**9+1,dtype=np.int64)*10**9//args.fps+start
    tol=int(args.tolerance_ms*1e6);rows=[];active=[];keys=[];reasons=Counter()
    for t0 in grid:
        t=int(t0);state=indexes['state'].pick(t,tol,causal=True);quest=indexes['quest'].pick(t,100_000_000,causal=True)
        row=None;why=None;key_value=None;activity=False
        if state is None or not state['hardware'] or not np.isfinite(state['q']).all() or not good_pose(state['pose']):why='robot_missing_invalid_or_homing'
        else:
            if quest is None:
                quest=dict(t=0,identity=None,epoch=None,valid=False,pose_valid=False,trigger=0.,trigger_active=False,x=0.,stick=False,stick_direction=0,acquisition=0,location=0,uncertainty=0)
            if not quest['valid']:reasons['quest_unavailable_observation_retained_when_other_activity']+=1
            rgb=[indexes.get(role+'_rgb',TemporalIndex([])).pick(t,tol) for role in ROLES]
            if rgb[0] is None:why='wrist_rgb_missing_or_clock_ambiguous'
            else:
                app=app_index.pick(t,tol,causal=True);command=choose_command(commands,app['key'],app['source']) if app else None
                arm_active=bool(app and app['motion'] and app['requested_valid'] and app['reference_valid'] and not app['homing'] and not app['fallback'] and not state['homing'] and not state['fallback'])
                arm_valid=bool(arm_active and command and good_pose(command['pose']) and 0<command['source']<=app['receipt']<=app['issue']<=t)
                if arm_active and not arm_valid:reasons['arm_motion_without_unambiguous_causal_target_masked']+=1
                if True:
                    depth=[indexes.get(role+'_depth',TemporalIndex([])).pick(t,tol) for role in ROLES]
                    grip=grip_index.pick(t,500_000_000,causal=True)
                    measured_grip=fresh_index.pick(t,100_000_000,causal=True)
                    gripper_valid=bool(quest['valid'] and quest['stick'] and np.isfinite(quest['x']) and abs(quest['x'])<=1)
                    busy=bool(grip and grip['hardware'] and grip['calibrated'] and grip['busy'] and grip['operation'] in ('open','close'))
                    activity=bool(arm_active or (gripper_valid and quest['stick_direction']) or busy)
                    action=(command['pose'] if arm_valid else [0.,0.,0.,0.,0.,0.,1.])+[quest['x'] if gripper_valid else 0.]
                    row={'observation.state':state['q']+state['pose'],'observation.joint_velocity':state['dq'],
                        'action':action,'action_valid':[int(arm_valid)]*7+[int(gripper_valid)],
                        'observation.gripper_width':[measured_grip['width'] if measured_grip else 0.],
                        'observation.gripper_width_valid':[int(measured_grip is not None)],
                        'observation.gripper_width_age_s':[(t-measured_grip['t'])/1e9 if measured_grip else -1.],
                        'observation.grasp_confirmed':[int(bool(measured_grip and measured_grip['grasped']))],
                        'observation.trigger':[quest['trigger']],'observation.arm_motion_enabled':[int(arm_active)],
                        'observation.depth_valid':[int(d is not None) for d in depth],
                        'observation.rgb_valid':[int(r is not None) for r in rgb],
                        'observation.arm_action_valid':[int(arm_valid)],'observation.gripper_action_valid':[int(gripper_valid)],
                        'observation.gripper_status_valid':[int(bool(grip and grip['hardware'] and grip['calibrated']))],
                        'observation.gripper_busy':[int(bool(grip and grip['busy']))],
                        'observation.gripper_holding':[int(bool(grip and grip['grasped']))],
                        'observation.gripper_status_age_s':[(t-grip['t'])/1e9 if grip else -1.],
                        'source.episode_number':[source_number],'source.time_ns':[t],
                        'source.rgb_header_ros_ns':[r['header'] if r else 0 for r in rgb],
                        'source.rgb_bag_receipt_ros_ns':[r['receipt'] if r else 0 for r in rgb],
                        'source.rgb_match_delta_ns':[r['t']-t if r else 0 for r in rgb],
                        'source.depth_header_ros_ns':[d['header'] if d else 0 for d in depth],
                        'source.robot_state_time_ns':[state['t']],'source.control_cycle':[state['cycle']],
                        'source.control_time_ns':[state['control']],
                        'source.command_acquisition_time_ns':[command['acquisition'] if arm_valid else 0],
                        'source.command_openxr_location_time_ns':[command['location'] if arm_valid else 0],
                        'source.command_sequence':[app['key'][2] if arm_valid else 0],
                        'source.command_reference_generation':[app['key'][1] if arm_valid else 0],
                        'source.quest_clock_uncertainty_ns':[quest['uncertainty']],
                        'source.command_source_time_ns':[command['source'] if arm_valid else 0],
                        'source.command_receipt_time_ns':[app['receipt'] if arm_valid else 0],
                        'source.command_issue_time_ns':[app['issue'] if arm_valid else 0],
                        'source.quest_source_time_ns':[quest['t']],'source.quest_acquisition_time_ns':[quest['acquisition']],
                        'source.openxr_location_time_ns':[quest['location']],
                        'source.gripper_status_ros_ns':[measured_grip['header'] if measured_grip else 0],
                        'source.capture_uncertainty_known':[0],
                        'source.clock_mapping_uncertainty_ns':[max(m.uncertainty_ns for m in models)],
                        '_rgb_refs':[r['ordinal'] if r else -1 for r in rgb],'_depth_refs':[d['ordinal'] if d else -1 for d in depth]}
                    key_value=(state['epoch'],quest['epoch'],*[r['epoch'] if r else None for r in rgb])
        if why:reasons[why]+=1
        rows.append(row);active.append(activity);keys.append(key_value)
    keep=context_mask(grid,np.array(active,dtype=bool),int(args.pre_ms*1e6),int(args.post_ms*1e6))
    for i,row in enumerate(rows):keep[i]=bool(keep[i] and row is not None)
    segments=continuous_segments(keep,keys,args.min_frames)
    selected=[];segment_info=[]
    for a,b in segments:
        first=len(selected);selected.extend(range(a,b));segment_info.append({'row_start':first,'row_end':len(selected),'source_start_ns':int(grid[a]),'source_end_ns':int(grid[b-1]),'frames':b-a})
    arrays={}
    if selected:
        for name in rows[selected[0]]:arrays[name]=np.array([rows[i][name] for i in selected],dtype=np.int64 if name.startswith('source.') or name.startswith('_') else np.float32)
        # Masks use uint8 for robust statistics in the pinned official SDK.
        for name in ['action_valid','observation.gripper_width_valid','observation.grasp_confirmed','observation.arm_motion_enabled','observation.depth_valid','source.capture_uncertainty_known','observation.rgb_valid','observation.arm_action_valid','observation.gripper_action_valid','observation.gripper_status_valid','observation.gripper_busy','observation.gripper_holding']:
            arrays[name]=arrays[name].astype(np.uint8)
    np.savez_compressed(plan_npz,**arrays)
    supervision=summarize_supervision(streams,arrays,segment_info)
    summary={'grid_frames':len(grid),'retained_frames':len(selected),'segments':len(segments),
             'inactive_grid_frames':int((~context_mask(grid,np.array(active,dtype=bool),int(args.pre_ms*1e6),int(args.post_ms*1e6))).sum()),
             'invalid_reasons':dict(reasons),'source_seconds':(end-start)/1e9,'retained_seconds':len(selected)/args.fps,
             'gripper_readback_valid_frames':int(arrays['observation.gripper_width_valid'].sum()) if selected else 0}
    result={'source_id':bag.name,'source_number':source_number,'source_dir':str(bag),'parameters':params,'camera_topics':topics,
            'clock_models':[asdict(m) for m in models],'camera_info':camera_info,'message_counts':counts,'native_evidence':native,'supervision':supervision,
            'source_sessions':sorted({r['identity'][0] for r in streams['state']}),
            'source_validation_status':manifest.get('validation_status','legacy_passed' if manifest.get('readable') else 'unverified'),
            'segments':segment_info,'summary':summary,'npz':str(plan_npz)}
    dump(plan_json,result);print(json.dumps({'prepared':bag.name,**summary}),flush=True);return result


def features():
    specs={
      'observation.state':('float32',14,[f'joint_{i+1}_rad' for i in range(7)]+['eef_x_m','eef_y_m','eef_z_m','eef_qx','eef_qy','eef_qz','eef_qw']),
      'observation.joint_velocity':('float32',7,[f'joint_{i+1}_rad_s' for i in range(7)]),
      'action':('float32',8,['target_x_m','target_y_m','target_z_m','target_qx','target_qy','target_qz','target_qw','right_thumbstick_x_intent']),
      'action_valid':('uint8',8,None),
      'observation.gripper_width':('float32',1,['width_m']),
      'observation.gripper_width_valid':('uint8',1,None),
      'observation.gripper_width_age_s':('float32',1,None),
      'observation.grasp_confirmed':('uint8',1,None),
      'observation.trigger':('float32',1,None),
      'observation.arm_motion_enabled':('uint8',1,None),
      'observation.depth_valid':('uint8',2,['wrist','scene']),
      'observation.rgb_valid':('uint8',2,['wrist','scene']),
      'observation.arm_action_valid':('uint8',1,None),'observation.gripper_action_valid':('uint8',1,None),
      'observation.gripper_status_valid':('uint8',1,None),'observation.gripper_busy':('uint8',1,None),
      'observation.gripper_holding':('uint8',1,None),'observation.gripper_status_age_s':('float32',1,None),
    }
    scalar=['episode_number','time_ns','robot_state_time_ns','control_cycle','command_source_time_ns','command_receipt_time_ns',
            'command_issue_time_ns','quest_source_time_ns','quest_acquisition_time_ns','openxr_location_time_ns',
            'gripper_status_ros_ns','clock_mapping_uncertainty_ns','depth_block','depth_offset','control_time_ns',
            'command_acquisition_time_ns','command_openxr_location_time_ns','command_sequence','command_reference_generation','quest_clock_uncertainty_ns']
    for k in scalar:specs['source.'+k]=('int64',1,None)
    for k in ['rgb_header_ros_ns','rgb_bag_receipt_ros_ns','rgb_match_delta_ns','depth_header_ros_ns']:specs['source.'+k]=('int64',2,['wrist','scene'])
    specs['source.capture_uncertainty_known']=('uint8',1,None)
    result={k:{'dtype':dtype,'shape':(size,),'names':names} for k,(dtype,size,names) in specs.items()}
    for role in ROLES:result['observation.images.'+role]={'dtype':'video','shape':(3,480,640),'names':['channels','height','width']}
    return result


def source_frames(plan,arrays):
    """Stream only requested frames; bounded look-ahead, no PNG intermediates."""
    import rosbag2_py
    rows=len(arrays['_rgb_refs']);topics=plan['camera_topics'];reverse={v:k for k,v in topics.items()}
    wanted=defaultdict(set)
    for c,role in enumerate(ROLES):
        wanted[role+'_rgb'].update(map(int,arrays['_rgb_refs'][:,c]))
        wanted[role+'_depth'].update(int(x) for x in arrays['_depth_refs'][:,c] if x>=0)
    reader=rosbag2_py.SequentialReader();reader.open(rosbag2_py.StorageOptions(uri=plan['source_dir'],storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(reverse)))
    counters=Counter();cache={}
    for i in range(rows):
        refs={role+'_rgb':int(arrays['_rgb_refs'][i,c]) for c,role in enumerate(ROLES)}
        refs.update({role+'_depth':int(arrays['_depth_refs'][i,c]) for c,role in enumerate(ROLES)})
        while any((name,ordinal) not in cache for name,ordinal in refs.items() if ordinal>=0):
            if not reader.has_next():raise ValueError(f'EOF before requested frames: {plan["source_id"]}, row {i}')
            topic,raw,_=reader.read_next();name=reverse[topic];ordinal=counters[name];counters[name]+=1
            if ordinal not in wanted[name]:continue
            h,pixels=image_view(raw);rgb=name.endswith('_rgb')
            assert (h['width'],h['height'],h['encoding'])==(640,480,'rgb8' if rgb else '16UC1'),h
            if rgb:a=np.frombuffer(pixels,dtype=np.uint8).reshape(480,h['step'])[:,:640*3].reshape(480,640,3).copy()
            else:a=np.frombuffer(pixels,dtype=('>u2' if h['is_bigendian'] else '<u2')).reshape(480,h['step']//2)[:,:640].astype(np.uint16,copy=True)
            cache[name,ordinal]=a
        yield {name:(cache[name,ordinal] if ordinal>=0 else None) for name,ordinal in refs.items()}
        for name in topics:
            next_ordinal=int(arrays['_rgb_refs' if name.endswith('_rgb') else '_depth_refs'][i+1,ROLES.index(name.split('_')[0])]) if i+1<rows else 10**12
            for k in [k for k in cache if k[0]==name and k[1]<next_ordinal]:cache.pop(k)
    del reader


def build_dataset(plans,args):
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    import torch
    torch.set_num_threads(2)
    root=args.output_root
    ds=LeRobotDataset.create(repo_id='SemahJa/panda_cable_clutch_trimmed',root=root,fps=args.fps,features=features(),
        robot_type='franka_panda_quest',video_backend='pyav',vcodec='h264',streaming_encoding=True,
        encoder_queue_maxsize=8,encoder_threads=2,metadata_buffer_size=1)
    ds._streaming_encoder.crf=18;ds._streaming_encoder.g=30
    provenance=root/'provenance';provenance.mkdir();segment_records=[]
    try:
        for plan in plans:
            dump(provenance/'sources'/(plan['source_id']+'.json'),plan)
            for name in ('episode_manifest.json','metadata.yaml'):
                target=provenance/'source_manifests'/plan['source_id']/name;target.parent.mkdir(parents=True,exist_ok=True)
                shutil.copy2(Path(plan['source_dir'])/name,target)
            ledger=Path(plan['native_evidence']['ledger'])
            target=provenance/'native_inputs'/ledger.name;target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(ledger,target)
            if not plan['summary']['retained_frames']:continue
            with np.load(plan['npz']) as arrays:
                frames=iter(source_frames(plan,arrays));cursor=0
                for segment_number,segment in enumerate(plan['segments']):
                    ep=ds.meta.total_episodes;block=0;depths={r:[] for r in ROLES};depth_times=[];depth_valid=[]
                    depth_dir=root/'auxiliary'/'aligned_depth'/f'episode_{ep:06d}';depth_dir.mkdir(parents=True)
                    def flush_depth():
                        nonlocal block
                        if not depth_times:return
                        np.savez_compressed(depth_dir/f'block_{block:06d}.npz',
                            **{r:np.stack(depths[r]) for r in ROLES},source_time_ns=np.array(depth_times,dtype=np.int64),valid=np.array(depth_valid,dtype=np.uint8))
                        for r in ROLES:depths[r].clear()
                        depth_times.clear();depth_valid.clear();block+=1
                    for j in range(segment['row_start'],segment['row_end']):
                        images=next(frames);assert j==cursor;cursor+=1
                        frame={k:np.array(arrays[k][j],dtype=spec['dtype']) for k,spec in features().items() if k in arrays}
                        frame['source.depth_block']=np.array([block],dtype=np.int64);frame['source.depth_offset']=np.array([len(depth_times)],dtype=np.int64)
                        frame['task']=TASK
                        for c,role in enumerate(ROLES):
                            frame['observation.images.'+role]=images[role+'_rgb'] if images[role+'_rgb'] is not None else np.zeros((480,640,3),dtype=np.uint8)
                            d=images[role+'_depth'];depths[role].append(d if d is not None else np.zeros((480,640),dtype=np.uint16))
                        depth_times.append(int(arrays['source.time_ns'][j,0]));depth_valid.append(arrays['observation.depth_valid'][j])
                        # Apply backpressure rather than the SDK's queue-full drop policy.
                        wait_started=time.monotonic()
                        while any(q.full() for q in ds._streaming_encoder._frame_queues.values()):
                            if not all(t.is_alive() for t in ds._streaming_encoder._threads.values()):raise RuntimeError('Video encoder worker stopped')
                            if time.monotonic()-wait_started>60:raise RuntimeError('Video encoder stalled for 60 seconds')
                            time.sleep(.002)
                        ds.add_frame(frame)
                        if len(depth_times)>=64:flush_depth()
                    flush_depth();ds.save_episode(parallel_encoding=False)
                    if any(ds._streaming_encoder._dropped_frames.values()):raise RuntimeError('Video encoder dropped frames')
                    record=dict(dataset_episode_index=ep,source_episode_number=plan['source_number'],source_episode_id=plan['source_id'],
                        source_segment=segment_number,success=plan.get('success'),outcome=plan.get('outcome','unknown'),source_group=plan.get('source_group','unknown'),**segment)
                    segment_records.append(record);dump(provenance/'segments.json',segment_records)
                    print(json.dumps({'saved_dataset_episode':ep,'source_episode':plan['source_number'],'frames':segment['frames']}),flush=True)
        ds.finalize()
    finally:
        if ds._streaming_encoder is not None:ds._streaming_encoder.close()
    if ds.meta.total_episodes==0:raise ValueError('No admissible continuous segments were found')
    return segment_records


def validate_source_labels(segments,plans):
    expected={p['source_id']:(p.get('outcome','unknown'),p.get('success')) for p in plans}
    for segment in segments:
        if (segment.get('outcome','unknown'),segment.get('success'))!=expected[segment['source_episode_id']]:
            raise ValueError('Source outcome label changed: '+segment['source_episode_id'])
    return dict(Counter(p.get('outcome','unknown') for p in plans if p['summary']['retained_frames']))


def validate_dataset(args,segments,plans):
    labels=validate_source_labels(segments,plans)
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    ds=LeRobotDataset('SemahJa/panda_cable_clutch_trimmed',root=args.output_root,video_backend='pyav',
        delta_timestamps={'action':[0.,1/args.fps,2/args.fps]},download_videos=False)
    expected=sum(s['frames'] for s in segments)
    assert len(ds)==expected and ds.num_episodes==len(segments)
    cursor=0
    for segment in segments:
        n=segment['frames'];first=ds[cursor];last=ds[cursor+n-1]
        assert int(first['episode_index'])==segment['dataset_episode_index']
        assert int(last['episode_index'])==segment['dataset_episode_index']
        assert first['observation.images.scene'].shape==(3,480,640)
        assert first['observation.images.wrist'].shape==(3,480,640)
        assert last['action_is_pad'].tolist()==[False,True,True]
        for item in (first,last):
            mask=item['action_valid'].bool()
            if mask[:7].all():
                source=int(item['source.command_source_time_ns']);receipt=int(item['source.command_receipt_time_ns']);issue=int(item['source.command_issue_time_ns']);anchor=int(item['source.time_ns'])
                assert 0<source<=receipt<=issue<=anchor
        cursor+=n
    # Check every numeric row, not just the sampled image boundaries.
    import pyarrow.parquet as pq
    import av
    columns=['action_valid','source.command_source_time_ns','source.command_receipt_time_ns',
             'source.command_issue_time_ns','source.time_ns','source.depth_block','source.depth_offset',
             'observation.depth_valid','episode_index']
    tables=[pq.read_table(p,columns=columns) for p in sorted((args.output_root/'data').rglob('*.parquet'))]
    import pyarrow as pa
    table=pa.concat_tables(tables).to_pydict()
    def column(name):return np.asarray(table[name])
    arm=column('action_valid')[:,0].astype(bool)
    source=column('source.command_source_time_ns').reshape(-1);receipt=column('source.command_receipt_time_ns').reshape(-1)
    issue=column('source.command_issue_time_ns').reshape(-1);anchor=column('source.time_ns').reshape(-1)
    assert np.all((source[arm]>0)&(source[arm]<=receipt[arm])&(receipt[arm]<=issue[arm])&(issue[arm]<=anchor[arm]))
    depth_frames=0;depth_missing=np.zeros(2,dtype=np.int64)
    for segment in segments:
        ep=segment['dataset_episode_index'];selected=np.flatnonzero(column('episode_index')==ep)
        assert len(selected)==segment['frames']
        for block_path in sorted((args.output_root/'auxiliary/aligned_depth'/f'episode_{ep:06d}').glob('*.npz')):
            block=int(block_path.stem.split('_')[-1]);indices=selected[column('source.depth_block').reshape(-1)[selected]==block]
            with np.load(block_path) as z:
                assert np.array_equal(column('source.depth_offset').reshape(-1)[indices],np.arange(len(indices)))
                assert np.array_equal(z['source_time_ns'],anchor[indices])
                assert np.array_equal(z['valid'],column('observation.depth_valid')[indices])
                for role in ROLES:assert z[role].dtype==np.uint16 and z[role].shape==(len(indices),480,640)
                depth_missing+=(1-z['valid']).sum(axis=0).astype(np.int64);depth_frames+=len(indices)
    assert depth_frames==expected
    video_counts={}
    for role in ROLES:
        count=0
        for path in (args.output_root/'videos'/('observation.images.'+role)).rglob('*.mp4'):
            with av.open(str(path)) as container:count+=container.streams.video[0].frames
        assert count==expected,(role,count,expected)
        video_counts[role]=count
    raw_ok=True
    for plan in plans:
        for path,original in plan['parameters']['inventory'].items():
            if sha(path)!=original['sha256']:raw_ok=False;raise ValueError(f'Raw source changed during export: {path}')
    return dict(official_loader='lerobot==0.4.4 / LeRobotDataset',format='v3.0',episodes=len(segments),frames=len(ds),
                fps=args.fps,raw_source_hashes_unchanged=raw_ok,episode_boundary_padding_verified=True,
                both_camera_decoding_verified=True,video_frame_counts=video_counts,lossless_depth_frames=depth_frames,depth_missing_frames=depth_missing.tolist(),all_action_causality_verified=True,outcome_labels_preserved=True,outcomes_by_contributing_source=labels,success_labels='operator-provided where available; unknown otherwise; no automatic training selection')


def select_bags(input_root,episode_ids=None,max_source_episodes=None):
    """Explicit IDs are ordered by recorded MCAP start time, never directory mtime."""
    bags={p.name:p for p in input_root.iterdir() if p.is_dir() and (p/'metadata.yaml').exists()}
    if episode_ids:
        if max_source_episodes is not None:raise ValueError('Do not combine explicit episode IDs with --max-source-episodes')
        if len(episode_ids)!=len(set(episode_ids)):raise ValueError('Duplicate episode ID')
        missing=set(episode_ids)-set(bags)
        if missing:raise ValueError('Unknown episode IDs: '+', '.join(sorted(missing)))
        def start(p):
            md=yaml.safe_load((p/'metadata.yaml').read_text())['rosbag2_bagfile_information']
            return (int(md['starting_time']['nanoseconds_since_epoch']),p.name)
        return sorted((bags[name] for name in episode_ids),key=start)
    ordered=sorted(bags.values())
    return ordered[:max_source_episodes] if max_source_episodes is not None else ordered


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input-root',type=Path,required=True);parser.add_argument('--output-root',type=Path,required=True)
    parser.add_argument('--episode-id',action='append',help='Exact recording ID; repeat to select explicitly. Ordered by MCAP start time.');parser.add_argument('--work-root',type=Path);parser.add_argument('--fps',type=int,default=30)
    parser.add_argument('--tolerance-ms',type=float,default=50.);parser.add_argument('--pre-ms',type=float,default=100.);parser.add_argument('--post-ms',type=float,default=200.)
    parser.add_argument('--validate-only',action='store_true');parser.add_argument('--min-frames',type=int,default=2);parser.add_argument('--prepare-only',action='store_true');parser.add_argument('--max-source-episodes',type=int)
    args=parser.parse_args();args.input_root=args.input_root.resolve();args.output_root=args.output_root.resolve()
    if args.output_root==args.input_root or args.output_root.is_relative_to(args.input_root):parser.error('Output must be outside raw recordings')
    if args.validate_only:
        plans=[json.loads(p.read_text()) for p in sorted((args.output_root/'provenance/sources').glob('*.json'))]
        segments=json.loads((args.output_root/'provenance/segments.json').read_text())
        result=validate_dataset(args,segments,plans);dump(args.output_root/'export_validation.json',result)
        print(json.dumps(result,indent=2));return
    if args.output_root.exists():parser.error('Output already exists; choose a new directory. Nothing will be deleted.')
    if not args.fps>0 or args.tolerance_ms<=0 or args.min_frames<2:parser.error('Invalid FPS/tolerance/minimum segment length')
    work=(args.work_root or args.output_root.parent/'.export_work'/args.output_root.name).resolve();work.mkdir(parents=True,exist_ok=True)
    try:bags=select_bags(args.input_root,args.episode_id,args.max_source_episodes)
    except ValueError as e:parser.error(str(e))
    if not bags:parser.error('No selected recordings')
    plans=[prepare(bag,i,work,args) for i,bag in enumerate(bags)]
    dump(work/'summary.json',[p['summary']|{'source_id':p['source_id']} for p in plans])
    if args.prepare_only:return
    segments=build_dataset(plans,args)
    result=validate_dataset(args,segments,plans);dump(args.output_root/'export_validation.json',result)
    dump(args.output_root/'provenance'/'export_parameters.json',{'exporter_version':VERSION,'fps':args.fps,'tolerance_ms':args.tolerance_ms,'pre_context_ms':args.pre_ms,'post_context_ms':args.post_ms,'input_root':str(args.input_root),'source_summaries':[{**p['summary'],'source_id':p['source_id']} for p in plans]})
    packages=['lerobot','torch','torchvision','numpy','pandas','pyarrow','datasets','huggingface-hub','av','Pillow','opencv-python-headless','draccus','accelerate']
    environment='Python '+sys.version+'\n'+'\n'.join(name+'=='+importlib.metadata.version(name) for name in packages)+'\n'
    (args.output_root/'provenance/environment.txt').write_text(environment)
    pinned=subprocess.run([sys.executable,'-m','pip','freeze','--local'],capture_output=True,text=True,check=True)
    (args.output_root/'provenance/requirements-local.txt').write_text(pinned.stdout)
    documentation=Path(__file__).resolve().parents[1]/'LEROBOT_EXPORT.md'
    (args.output_root/'README.md').write_text('---\ntask_categories:\n- robotics\ntags:\n- lerobot\n- franka-panda\n- teleoperation\n---\n\n'+documentation.read_text())
    dump(args.output_root/'auxiliary/aligned_depth/schema.json',{'encoding':'16UC1','dtype':'uint16','roles':list(ROLES),'metric_scale_m_per_unit':None,'missing_value':0,'require_valid_mask':True,'extrinsics_calibrated':False})
    print(json.dumps(result,indent=2),flush=True)


if __name__=='__main__':main()
