"""Bounded passive timing monitor. Raw CDR parsing avoids image deserialization."""
import argparse
from dataclasses import asdict
from decimal import Decimal
import json
import math
from pathlib import Path
import struct
import time
import socket
from collections import defaultdict
from .clocks import HostClockBridge, ClockError
from .metrics import stream_metrics, nearest_differences, summary, SegmentTracker
from .adapters import adapt


def image_header(serialized):
    """ROS 2 Humble XCDR1 Image: encapsulation + Time + frame_id string.

    No pixel array is parsed/copied. RMW/rclpy still transports a serialized buffer.
    """
    view=memoryview(serialized)
    if len(view)<16 or bytes(view[:2]) not in (b'\x00\x01',b'\x00\x00'):
        raise ValueError('unsupported CDR encapsulation')
    endian='<' if view[1]==1 else '>'
    sec,nsec,length=struct.unpack_from(endian+'iII',view,4)
    if nsec>=1_000_000_000 or length<1 or 16+length>len(view):
        raise ValueError('malformed Image header')
    return sec*1_000_000_000+nsec,bytes(view[16:16+length-1]).decode('utf8')


def json_safe(value):
    if isinstance(value,dict): return {k:json_safe(v) for k,v in value.items()}
    if isinstance(value,(list,tuple)): return [json_safe(v) for v in value]
    if isinstance(value,float) and not math.isfinite(value): return None
    return value


def analyze(events,fps,tolerance_ns):
    groups=defaultdict(list)
    for row in events: groups[row['channel']].append(row)
    result={'streams':{key:stream_metrics(rows,fps.get(key.split('/')[0])) for key,rows in groups.items()},
            'header_relationships':{},'native_rgb_depth_relationships':{},
            'interpretation':'Nearest header phase offsets are observational; not measured cross-device capture skew.'}
    result['exact_rgb_depth_headers']={}
    for role in ('wrist','scene'):
        a=groups.get(role+'/rgb',[]); b=groups.get(role+'/aligned_depth',[])
        aa={r['header_ros_ns'] for r in a}; bb={r['header_ros_ns'] for r in b}
        result['exact_rgb_depth_headers'][role]={'shared':len(aa & bb),'rgb_without_depth':len(aa-bb),'depth_without_rgb':len(bb-aa)}
        result['header_relationships'][role+'_rgb_to_aligned_depth_ns']=summary(nearest_differences(
            [r['header_ros_ns'] for r in a],[r['header_ros_ns'] for r in b],tolerance_ns))
        # Native clocks are compared only if metadata reports the same SDK domain.
        color=groups.get(role+'/color_metadata',[]); depth=groups.get(role+'/depth_metadata',[])
        native=[]; firmware_readout=[]; exposure_midpoints=[]; projection=[]
        for c in color:
            if 'native_frame_ns' not in c: continue
            candidates=[d for d in depth if d.get('clock_domain')==c.get('clock_domain')
                        and abs(d['header_ros_ns']-c['header_ros_ns'])<=tolerance_ns and 'native_frame_ns' in d]
            if candidates:
                d=min(candidates,key=lambda d:abs(d['header_ros_ns']-c['header_ros_ns']))
                native.append(d['native_frame_ns']-c['native_frame_ns'])
                projection.append(c['header_ros_ns']-c['native_frame_ns'])
                for key,dest in (('hw_timestamp',firmware_readout),('sensor_timestamp',exposure_midpoints)):
                    if key in c['metadata'] and key in d['metadata']:
                        dest.append((d['metadata'][key]-c['metadata'][key])*1000)
        result['native_rgb_depth_relationships'][role]={'sdk_frame_depth_minus_color_ns':summary(native),
            'firmware_readout_depth_minus_color_ns':summary(firmware_readout),
            'firmware_exposure_midpoint_depth_minus_color_ns':summary(exposure_midpoints),
            'color_header_minus_native_frame_ns':summary(projection),
            'warning':'Same-device paired metadata only; firmware counters may wrap; not a calibrated host capture-time bound.'}
    result['header_relationships']['wrist_to_scene_rgb_ns']=summary(nearest_differences(
        [r['header_ros_ns'] for r in groups.get('wrist/rgb',[])],
        [r['header_ros_ns'] for r in groups.get('scene/rgb',[])],tolerance_ns))
    result['metadata_fields']={key:sorted({k for row in rows for k in row.get('metadata',{})})
                               for key,rows in groups.items() if key.endswith('metadata')}
    result['metadata_clock_domains']={key:sorted({r.get('clock_domain','unknown') for r in rows})
                                      for key,rows in groups.items() if key.endswith('metadata')}
    return result


def main():
    import rclpy
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
    from sensor_msgs.msg import Image
    from realsense2_camera_msgs.msg import Metadata
    from rosidl_runtime_py.utilities import get_message
    from rosidl_runtime_py.convert import message_to_ordereddict
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--seconds',type=float,default=12)
    p.add_argument('--output',required=True)
    p.add_argument('--wrist-fps',type=float,default=30)
    p.add_argument('--scene-fps',type=float,default=30)
    p.add_argument('--pair-tolerance-ms',type=float,default=50)
    p.add_argument('--metadata-only',action='store_true',help='avoid image subscriptions entirely; aligned header checks then unavailable')
    p.add_argument('--local-camera-clock',action='store_true',help='assert these camera wrapper processes use this host/system ROS clock; otherwise reject clock joining')
    p.add_argument('--telemetry-host-clock-id',help='explicit verified host_monotonic:hostname:boot identity of telemetry publisher; absent means unverified, never silently local')
    p.add_argument('--observe-robot',action='store_true',help='subscribe only to already-discovered matching telemetry types')
    args=p.parse_args()
    if not 0<args.seconds<=300 or min(args.wrist_fps,args.scene_fps,args.pair_tolerance_ms)<=0:
        p.error('duration must be 0..300 seconds; FPS and tolerance positive')
    rclpy.init(); node=rclpy.create_node('demonstration_timestamp_diagnostics')
    if node.get_parameter('use_sim_time').value:
        raise ClockError('simulation clock rejected by host system-clock diagnostics')
    boot=Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    host_id='host_monotonic:'+socket.gethostname()+':'+boot
    ros_id='host_ros_system:'+socket.gethostname()+':'+boot
    bridge=HostClockBridge(); anchors=[]; events=[]; records=[]; errors=[]; subscriptions=[]
    tracker=SegmentTracker(); counter=0
    def anchor():
        before=time.monotonic_ns(); ros=node.get_clock().now().nanoseconds; after=time.monotonic_ns()
        a=bridge.observe(before,ros,after); anchors.append(a)
        return a
    def context():
        nonlocal counter
        counter+=1
        before=time.monotonic_ns(); ros=node.get_clock().now().nanoseconds; after=time.monotonic_ns()
        return {'record_id':str(counter),'arrival_monotonic_ns':(before+after)//2,'arrival_ros_ns':ros,
                'host_clock_id':host_id,'ros_clock_id':ros_id,'clock_segment':str(bridge.segment)}
    def camera_callback(role,kind,is_metadata):
        def receive(msg):
            ctx=context()
            try:
                metadata={}; native=None
                if is_metadata:
                    stamp=msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
                    frame_id=msg.header.frame_id
                    raw=msg.json_data
                    parsed=json.loads(raw,parse_float=Decimal)
                    if 'frame_timestamp' in parsed: native=int(Decimal(str(parsed['frame_timestamp']))*1_000_000)
                    metadata=json.loads(raw)
                else: stamp,frame_id=image_header(msg)
                row=dict(ctx,channel=role+'/'+kind,header_ros_ns=stamp,frame_id=frame_id)
                if is_metadata:
                    row.update(metadata=metadata,metadata_json=raw,clock_domain=metadata.get('clock_domain','unknown'),
                               frame_number=metadata.get('frame_number'))
                    if native is not None: row['native_frame_ns']=native
                identity=(frame_id,metadata.get('clock_domain','header_only'),bridge.segment)
                segment,reason=tracker.observe(row['channel'],native if native is not None else stamp,identity,stamp>0)
                row.update(segment=segment,discontinuity_reason=reason)
                events.append(row)
                stream=role+('_rgb' if kind=='rgb' else '_depth')
                record=dict(ctx,stream=stream,clock_id=ros_id if args.local_camera_clock else 'unverified_camera_header:'+role,timestamp_ns=stamp,header_ros_ns=stamp,
                            segment=segment,valid=stamp>0 and reason not in ('duplicate','timestamp_reset_or_reorder'),
                            uncertainty_ns=None,payload={'frame_ref':{'topic':'/cameras/'+role+'/'+
                                ('color/image_raw' if kind=='rgb' else 'aligned_depth_to_color/image_raw'),
                                'header_ros_ns':stamp,'frame_id':frame_id}},
                            semantic='wrapper-projected frame time; capture uncertainty unmeasured')
                if not is_metadata: records.append(record)
            except (ValueError,KeyError,struct.error) as e:
                errors.append({'channel':role+'/'+kind,'error':str(e)})
        return receive
    for role in ('wrist','scene'):
        for kind,suffix in (('rgb','color/image_raw'),('aligned_depth','aligned_depth_to_color/image_raw')):
            if not args.metadata_only:
                subscriptions.append(node.create_subscription(Image,'/cameras/'+role+'/'+suffix,
                    camera_callback(role,kind,False),qos_profile_sensor_data,raw=True))
        for kind,suffix in (('color_metadata','color/metadata'),('depth_metadata','depth/metadata')):
            subscriptions.append(node.create_subscription(Metadata,'/cameras/'+role+'/'+suffix,
                camera_callback(role,kind,True),qos_profile_sensor_data))
    expected={'/panda_1/tracking_samples':'quest_teleop_interfaces/msg/TrackingSample',
              '/panda_1/joint_states':'sensor_msgs/msg/JointState',
              '/panda_1/cartesian_teleop_command':'quest_teleop_interfaces/msg/CartesianTeleopCommand',
              '/quest_usb_bridge/right/controller_state':'quest_teleop_interfaces/msg/QuestControllerState',
              '/panda_1/gripper_events':'quest_teleop_interfaces/msg/GripperStatus'}
    observed={}
    def discover():
        graph=dict(node.get_topic_names_and_types())
        for topic,msg_type in expected.items():
            if topic in observed or msg_type not in graph.get(topic,[]) or not node.count_publishers(topic): continue
            try:
                cls=get_message(msg_type)
                def receive(msg,topic=topic):
                    ctx=context()
                    ctx['host_clock_id']=args.telemetry_host_clock_id or 'unverified_telemetry_monotonic'
                    # ROS header clock identity of other publishers is not encoded by Header.
                    # Only verified same-host telemetry may share this collector's ROS clock.
                    if args.telemetry_host_clock_id != host_id:
                        ctx['ros_clock_id']='unverified_telemetry_ros:'+topic
                    document=json_safe(message_to_ordereddict(msg))
                    for row in adapt(topic,document,ctx):
                        seg,reason=tracker.observe(row['stream'],row['timestamp_ns'],row['segment'],row['valid'])
                        row['segment']+=':'+seg; row['discontinuity_reason']=reason
                        records.append(row)
                qos=QoSProfile(depth=100) if not topic.endswith('/gripper_events') else QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
                # Best effort accepts both reliable and best-effort telemetry without blocking it.
                if not topic.endswith('/gripper_events'): qos=qos_profile_sensor_data
                subscriptions.append(node.create_subscription(cls,topic,receive,qos))
                observed[topic]=msg_type
            except (ImportError,AttributeError,ModuleNotFoundError) as e:
                errors.append({'topic':topic,'error':str(e)}); observed[topic]='type_unavailable'
    anchor(); timer=node.create_timer(0.1,anchor)
    discover_timer=node.create_timer(1.0,discover) if args.observe_robot else None
    start=time.monotonic(); deadline=start+args.seconds
    while time.monotonic()<deadline: rclpy.spin_once(node,timeout_sec=0.05)
    anchor()
    try:
        models=HostClockBridge.models(anchors,ros_id,host_id)
        model_error=None
    except ClockError as e: models=[]; model_error=str(e)
    indexed={}
    for event in events:
        if 'metadata_json' in event:
            indexed[(event['channel'],event['header_ros_ns'])]=event
    for record in records:
        if record['stream'] in ('wrist_rgb','scene_rgb','wrist_depth','scene_depth'):
            role,kind=record['stream'].split('_')
            meta=indexed.get((role+('/color_metadata' if kind=='rgb' else '/depth_metadata'),record['header_ros_ns']))
            if meta:
                record['payload']['metadata_event_record_id']=meta['record_id']
                record['payload']['frame_number']=meta.get('frame_number')
                record['payload']['native_frame_ns']=meta.get('native_frame_ns')
                record['payload']['sdk_clock_domain']=meta.get('clock_domain')
    report={'schema_version':1,'duration_seconds':time.monotonic()-start,
            'reference_clock':host_id,'ros_clock':ros_id,'use_sim_time':False,
            'clock_anchors':anchors,'clock_models':[asdict(m) for m in models],
            'clock_model_error':model_error,'camera_capture_clock_uncertainty_ns':None,
            'diagnostics':analyze(events,{'wrist':args.wrist_fps,'scene':args.scene_fps},round(args.pair_tolerance_ms*1e6)),
            'events':events,'records':records,'errors':errors,
            'camera_clock_local_asserted':args.local_camera_clock,'telemetry_host_clock_assertion':args.telemetry_host_clock_id,'host_time_namespace_offsets':Path('/proc/self/timens_offsets').read_text() if Path('/proc/self/timens_offsets').exists() else None,'robot_topics_subscribed':observed,'robot_topics_unavailable':[t for t in expected if t not in observed],
            'limitations':['No camera hardware-clock to host capture-time calibration measured.',
                           'No inter-camera hardware clock offset/drift inferred from reception.',
                           'Images are references only; no pixels or complete episodes recorded.']}
    Path(args.output).write_text(json.dumps(report,separators=(',',':'),allow_nan=False))
    print(json.dumps({'output':args.output,'events':len(events),'records':len(records),'clock_models':len(models),
                      'robot_topics_subscribed':observed,'errors':errors,'diagnostics':report['diagnostics']},indent=2))
    node.destroy_node(); rclpy.shutdown()
    if not events or errors or any(not report['diagnostics']['streams'].get(role+'/'+kind,{}).get('samples')
        for role in ('wrist','scene') for kind in (('color_metadata','depth_metadata') if args.metadata_only else ('rgb','aligned_depth','color_metadata','depth_metadata'))):
        raise SystemExit(1)
