#!/usr/bin/env python3
"""Read/validate an existing bag and derive its manifest/timing; never record a bag."""
import argparse
from collections import Counter,defaultdict
from dataclasses import asdict
from decimal import Decimal
import hashlib
import json
import math
from pathlib import Path
import time
import yaml
from demonstration_timing.clocks import ClockModel, HostClockBridge
from demonstration_timing.metrics import SegmentTracker, stream_metrics
from demonstration_timing.diagnostics import analyze, json_safe
from demonstration_timing.adapters import adapt
from demonstration_timing.alignment import align
from recording.episode_monitor import atomic_json
from recording.image_tools import image_view


def loss_comparison(observed,recorded):
    """Exclude episode boundary differences. This still cannot localize shared loss."""
    observed=Counter({int(k):v for k,v in observed.items()});recorded=Counter(recorded)
    if not observed or not recorded:return {'common_window':None,'observer_only':None,'bag_only':None}
    low=max(min(observed),min(recorded));high=min(max(observed),max(recorded))
    a=Counter({t:n for t,n in observed.items() if low<=t<=high})
    b=Counter({t:n for t,n in recorded.items() if low<=t<=high})
    return {'common_window':[low,high],'observer_only':sum((a-b).values()),'bag_only':sum((b-a).values()),
            'interpretation':'Observer-only identities indicate recorder path misses; gaps shared by both cannot be attributed to the publisher alone.'}


def validate_episode(uri):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from rosidl_runtime_py.convert import message_to_ordereddict
    import numpy as np
    bag=Path(uri).resolve();sidecar=Path(str(bag)+'.manifest.json')
    manifest=json.loads(sidecar.read_text()) if sidecar.exists() else {'schema_version':1,'episode_id':bag.name,'requested_topics':[],'status':'unknown_origin','clock_provenance':'unverified'}
    started=time.monotonic();warnings=list(manifest.get('warnings',[]));errors=[]
    counts=Counter();serialized_bytes=0;stamps=defaultdict(Counter);profiles=defaultdict(set)
    fingerprints=defaultdict(dict);image_samples={};camera_info={};events=[];records=[];clock_messages=[]
    first_bag=last_bag=None;tracker=SegmentTracker();topic_types={};first_header={};last_header={}
    metadata_path=bag/'metadata.yaml'
    storage=manifest.get('storage_id','mcap')
    if metadata_path.exists():
        md=yaml.safe_load(metadata_path.read_text())['rosbag2_bagfile_information'];storage=md['storage_identifier']
    manifest['storage_id']=storage
    try:
        reader=rosbag2_py.SequentialReader()
        actual=bag
        if not metadata_path.exists() and bag.is_dir():
            files=list(bag.glob('*.mcap'))
            if len(files)==1:actual=files[0]
        reader.open(rosbag2_py.StorageOptions(uri=str(actual),storage_id=storage),rosbag2_py.ConverterOptions('',''))
        topic_types={t.name:t.type for t in reader.get_all_topics_and_types()}
        classes={}
        host=manifest.get('host_clock_id','unverified_host_monotonic')
        ros=manifest.get('ros_clock_id','unverified_ros')
        same_host=manifest.get('clock_provenance')=='same_host'
        models=[ClockModel(**m) for m in manifest.get('clock_models',[])]
        def segment_of(t):
            candidates=[m.segment for m in models if m.source==ros and m.valid_from_ns<=t<=m.valid_to_ns]
            return candidates[0] if len(candidates)==1 else 'unmapped'
        while reader.has_next():
            topic,raw,receive_ns=reader.read_next();counts[topic]+=1;serialized_bytes+=len(raw)
            first_bag=receive_ns if first_bag is None else min(first_bag,receive_ns)
            last_bag=receive_ns if last_bag is None else max(last_bag,receive_ns)
            record_id=str(sum(counts.values()));typ=topic_types[topic]
            ctx={'record_id':record_id,'arrival_ros_ns':receive_ns,'arrival_monotonic_ns':None,
                 'host_clock_id':host if same_host else 'unverified_telemetry_monotonic',
                 'ros_clock_id':ros if same_host else 'unverified_telemetry_ros','clock_segment':'unmapped'}
            if typ=='sensor_msgs/msg/Image':
                h,pixels=image_view(raw);stamp=h['stamp_ns'];stamps[topic][stamp]+=1
                first_header.setdefault(topic,stamp);last_header[topic]=stamp
                bpp={'rgb8':3,'bgr8':3,'rgba8':4,'mono8':1,'16UC1':2,'32FC1':4}.get(h['encoding'])
                if not bpp or h['width']<=0 or h['height']<=0 or h['step']<h['width']*bpp:raise ValueError('unsupported/invalid image layout on '+topic)
                profiles[topic].add((h['width'],h['height'],h['encoding'],h['step']))
                targets=manifest.get('observer_payload_fingerprints',{}).get(topic,{})
                if str(stamp) in targets:fingerprints[topic][str(stamp)]=hashlib.sha256(pixels).hexdigest()
                if topic not in image_samples:
                    if h['encoding']=='16UC1':
                        image=np.ndarray((h['height'],h['width']),dtype='>u2' if h['is_bigendian'] else '<u2',buffer=pixels,strides=(h['step'],2))
                        positive=image[image>0]
                        image_samples[topic]={'decoded':True,'nonzero_pixels':int(np.count_nonzero(image)),
                            'minimum_nonzero':int(positive.min()) if positive.size else None,'maximum':int(image.max()),
                            'depth_scale_m_per_unit':None,'depth_scale_note':'Raw wrapper values preserved; metric scale requires verified SDK/calibration metadata.'}
                    else:
                        image=np.ndarray((h['height'],h['width'],bpp),dtype=np.uint8,buffer=pixels,strides=(h['step'],bpp,1))
                        image_samples[topic]={'decoded':True,'first_pixel':image[0,0].tolist()}
                role=topic.split('/')[2] if topic.startswith('/cameras/') else None
                if role in ('wrist','scene'):
                    kind='rgb' if '/color/image_raw' in topic else 'aligned_depth'
                    ctx['clock_segment']=segment_of(stamp)
                    seg,reason=tracker.observe(topic,stamp,(h['frame_id'],ctx['clock_segment']))
                    events.append(dict(ctx,channel=role+'/'+kind,header_ros_ns=stamp,frame_id=h['frame_id']))
                    records.append(dict(ctx,stream=role+('_rgb' if kind=='rgb' else '_depth'),
                       timestamp_ns=stamp,header_ros_ns=stamp,clock_id=ros if same_host else 'unverified_camera_header:'+role,
                       segment=seg,valid=stamp>0 and reason not in ('duplicate','timestamp_reset_or_reorder'),uncertainty_ns=None,
                       payload={'frame_ref':{'bag':str(bag),'topic':topic,'record_id':record_id,'header_ros_ns':stamp,'frame_id':h['frame_id']}}))
                continue
            try:
                if typ not in classes:classes[typ]=get_message(typ)
                message=deserialize_message(raw,classes[typ])
            except (ImportError,AttributeError,ValueError,ModuleNotFoundError) as e:
                warning='message type unavailable for decoding '+topic+': '+str(e)
                if warning not in warnings:warnings.append(warning)
                continue
            if typ=='realsense2_camera_msgs/msg/Metadata':
                role=topic.split('/')[2];kind='color_metadata' if '/color/' in topic else 'depth_metadata'
                stamp=message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec
                parsed=json.loads(message.json_data,parse_float=Decimal);metadata=json.loads(message.json_data)
                event=dict(ctx,channel=role+'/'+kind,header_ros_ns=stamp,metadata=metadata,metadata_json=message.json_data,
                    frame_number=metadata.get('frame_number'),clock_domain=metadata.get('clock_domain','unknown'))
                if 'frame_timestamp' in parsed:event['native_frame_ns']=int(Decimal(str(parsed['frame_timestamp']))*1_000_000)
                events.append(event)
            elif typ=='sensor_msgs/msg/CameraInfo':
                if topic not in camera_info:camera_info[topic]=json_safe(message_to_ordereddict(message))
            elif topic=='/demonstration_timing/clock_observations':
                clock_messages.append(json.loads(message.data))
            elif topic.endswith(('/tracking_samples','/joint_states','/cartesian_teleop_command','/controller_state','/gripper_events')):
                doc=json_safe(message_to_ordereddict(message));header=doc.get('header',{}).get('stamp',{})
                ctx['clock_segment']=segment_of(header.get('sec',0)*1_000_000_000+header.get('nanosec',0))
                for record in adapt(topic,doc,ctx):
                    seg,reason=tracker.observe(record['stream'],record['timestamp_ns'],record['segment'],record['valid'])
                    record['segment']+=':'+seg;record['discontinuity_reason']=reason;records.append(record)
        del reader
        manifest['readable']=True
    except Exception as e:
        errors.append(str(e));manifest['readable']=False
    model_objects=[ClockModel(**m) for m in manifest.get('clock_models',[])]
    # Recover clock anchors from the recorded topic if an interrupted sidecar is sparse.
    if not model_objects and clock_messages:
        anchors=clock_messages
        try:
            host=anchors[0]['host_clock_id'];ros=anchors[0]['ros_clock_id']
            model_objects=HostClockBridge.models(anchors,ros,host)
            manifest.update(host_clock_id=host,ros_clock_id=ros,clock_models=[asdict(m) for m in model_objects])
        except (ValueError,KeyError) as e:warnings.append('clock model unavailable: '+str(e))
    diagnostic=analyze(events,{'wrist':30,'scene':30},50_000_000)
    camera_profiles={}
    for topic,values in profiles.items():
        n=counts[topic];span=last_header[topic]-first_header[topic]
        camera_profiles[topic]={'observed_layouts':[{'width':a,'height':b,'encoding':c,'step':d} for a,b,c,d in sorted(values)],
                               'observed_header_fps':(n-1)*1e9/span if span>0 else None}
    losses={topic:loss_comparison(manifest.get('observer_image_stamps',{}).get(topic,{}),counter) for topic,counter in stamps.items()}
    payload_checks={}
    for topic,values in fingerprints.items():
        target=manifest.get('observer_payload_fingerprints',{}).get(topic,{})
        payload_checks[topic]={'compared':len(values),'mismatched':sum(v!=target[k] for k,v in values.items())}
    for topic,comparison in losses.items():
        if comparison.get('observer_only'):warnings.append(f'{topic}: {comparison["observer_only"]} observer identities absent in bag inside common window')
    for key,value in diagnostic['streams'].items():
        if value.get('estimated_missing_from_header_gaps'):warnings.append(f'{key}: {value["estimated_missing_from_header_gaps"]} estimated frame gaps; attribution requires observer comparison')
    missing=[t for t in manifest.get('requested_topics',[]) if counts[t]==0]
    log=Path(str(bag)+'.recorder.log')
    storage_warnings=[line for line in log.read_text(errors='replace').splitlines() if any(k in line.lower() for k in ('lost','dropp','cache is full','error','warn'))] if log.exists() else []
    if missing:warnings.append('requested streams missing: '+', '.join(missing))
    storage_bytes=sum(p.stat().st_size for p in bag.glob('*') if p.suffix in ('.mcap','.db3') and p.is_file()) if bag.is_dir() else 0
    duration=(last_bag-first_bag)/1e9 if first_bag is not None and last_bag is not None else 0
    manifest.update(recorded_topics={t:{'type':topic_types.get(t),'message_count':n} for t,n in counts.items() if n},
        missing_topics=missing,duration_seconds=duration,first_bag_receipt_ros_ns=first_bag,last_bag_receipt_ros_ns=last_bag,
        storage_bytes=storage_bytes,serialized_bytes=serialized_bytes,storage_bytes_per_second=storage_bytes/duration if duration>0 else None,
        camera_actual_profiles=camera_profiles,camera_info=camera_info,image_decoding=image_samples,
        independent_observer_comparison=losses,pixel_payload_fingerprints=payload_checks,
        timing_diagnostics=diagnostic,storage_warnings=storage_warnings,storage_drop_count=None,
        warnings=list(dict.fromkeys(warnings)),validation_errors=errors,
        data_completeness='incomplete' if missing else 'all_requested_topics_present',
        robot_data_available=any(t.endswith(('/tracking_samples','/joint_states','/robot_state')) and n for t,n in counts.items()),
        gripper_data_available=any(t.endswith('/gripper_events') and n for t,n in counts.items()),
        gripper_contract='Raw Quest grip/buttons and GripperStatus.operation/width only; no dedicated timestamped physical command/continuous readback contract.',
        calibration_status='uncalibrated_extrinsics',capture_synchronization_accuracy_ns=None,
        validation_seconds=time.monotonic()-started)
    if bag.is_dir():
        normalized=bag/'timing_records.jsonl'
        normalized.write_text(''.join(json.dumps(r,separators=(',',':'),allow_nan=False)+'\n' for r in records))
        if str(manifest.get('host_clock_id','')).startswith('host_monotonic:'):
            result=align(records,model_objects,manifest['host_clock_id'],tolerance_ns=50_000_000,allow_unknown=True)
            atomic_json(bag/'alignment_provisional.json',result)
            rows=result['observations']
            manifest['alignment']={'policy':'provisional; unknown capture uncertainty allowed; not exposure synchronization','tolerance_ns':50_000_000,
                'anchor_count':len(rows),'rejected_records':len(result['rejected_records']),
                'matched_counts':{k:sum(not r['missing_data_mask'][k] for r in rows) for k in ('wrist_rgb','wrist_depth','scene_rgb','scene_depth','robot_state','gripper_state','action')}}
        atomic_json(bag/'episode_manifest.json',manifest)
    atomic_json(sidecar,manifest)
    return manifest


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('episode');args=parser.parse_args()
    result=validate_episode(args.episode)
    print(json.dumps({k:result.get(k) for k in ('episode_id','readable','duration_seconds','storage_bytes','recorded_topics','missing_topics','validation_errors','alignment')},indent=2))
    raise SystemExit(0 if result.get('readable') and not result.get('validation_errors') else 1)


if __name__=='__main__':main()
