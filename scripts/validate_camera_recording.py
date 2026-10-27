#!/usr/bin/env python3
"""Camera-only bounded delivery/recording trial; reuses MenuRecorder and Phase3 metrics."""
import argparse,json,os,signal,subprocess,time,shutil,math
from pathlib import Path
from collections import defaultdict,Counter
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile,ReliabilityPolicy
from sensor_msgs.msg import Image,CameraInfo
from realsense2_camera_msgs.msg import Metadata
from recording.menu_recorder import MenuRecorder
from recording.image_tools import image_view
from recording.validate_episode import loss_comparison
from demonstration_timing.metrics import stream_metrics
p=argparse.ArgumentParser();p.add_argument('--roles',choices=['wrist','scene','both'],default='both');p.add_argument('--seconds',type=float,default=15);p.add_argument('--baseline-seconds',type=float,default=10);p.add_argument('--label',required=True);p.add_argument('--output-root',required=True);p.add_argument('--validation-timeout',type=float,default=600);a=p.parse_args()
if not math.isfinite(a.seconds) or a.seconds<=0 or not math.isfinite(a.baseline_seconds) or a.baseline_seconds<0 or not math.isfinite(a.validation_timeout) or a.validation_timeout<=0:
 p.error('seconds and validation-timeout must be positive; baseline-seconds nonnegative')
if not a.label or Path(a.label).name!=a.label or a.label in ('.','..'):
 p.error('label must be a single directory name')
root=Path(a.output_root).resolve()/a.label;root.mkdir(parents=True,exist_ok=False)
roles=['wrist','scene'] if a.roles=='both' else [a.roles]
topics=[f'/cameras/{r}/{s}' for r in roles for s in ('color/image_raw','aligned_depth_to_color/image_raw','color/camera_info','aligned_depth_to_color/camera_info','color/metadata','depth/metadata')]+['/tf_static','/demonstration_timing/clock_observations','/demonstration_timing/recording_health']
params=dict(profile='demonstration',record_late_topics=True,storage_id='mcap',output_root=str(root/'bags'),topics_json=json.dumps(topics),controller_state_topic='/camera_validation/unused_menu',menu_enabled=False,max_cache_size=268435456,clock_provenance='same_host')
def udp():
 lines=Path('/proc/net/snmp').read_text().splitlines()
 for i,l in enumerate(lines):
  if l.startswith('Udp:'):return dict(zip(l.split()[1:],map(int,lines[i+1].split()[1:])))
def delta(b,e):return {k:e[k]-b[k] for k in b}
rclpy.init();n=MenuRecorder(parameter_overrides=[Parameter(k,value=v) for k,v in params.items()]);samples=defaultdict(list);phase='warmup';subs=[];camera=None;log=(root/'camera.log').open('w');last_progress=0
for topic in topics:
 if not topic.startswith('/cameras/'):continue
 if topic.endswith('image_raw'):
  def cb(raw,t=topic):
   h,_=image_view(raw);samples[(phase,t)].append({'header_ros_ns':h['stamp_ns'],'arrival_ros_ns':n.get_clock().now().nanoseconds,'arrival_monotonic_ns':time.monotonic_ns()})
  subs.append(n.create_subscription(Image,topic,cb,QoSProfile(depth=30,reliability=ReliabilityPolicy.BEST_EFFORT),raw=True))
 elif topic.endswith('metadata'):
  def cb(m,t=topic):
   d=json.loads(m.json_data);samples[(phase,t)].append({'header_ros_ns':m.header.stamp.sec*10**9+m.header.stamp.nanosec,'arrival_ros_ns':n.get_clock().now().nanoseconds,'frame_number':d.get('frame_number'),'metadata':d})
  subs.append(n.create_subscription(Metadata,topic,cb,30))
def spin(seconds):
 global last_progress
 end=time.monotonic()+seconds
 while time.monotonic()<end:
  rclpy.spin_once(n,timeout_sec=.01)
  if time.monotonic()-last_progress>=30:
   last_progress=time.monotonic();print(json.dumps({'label':a.label,'phase':phase,'remaining_seconds':round(end-time.monotonic(),1),'observer_counts':{t:len(v) for (ph,t),v in samples.items() if ph==phase},'disk_free_bytes':shutil.disk_usage(root).free}),flush=True)
  if shutil.disk_usage(root).free<15*1024**3:raise RuntimeError('disk reserve reached')
try:
 camera=subprocess.Popen(['ros2','launch','camera_bringup','cameras.launch.py','cameras:='+a.roles],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
 deadline=time.monotonic()+40
 while time.monotonic()<deadline:
  rclpy.spin_once(n,timeout_sec=.05)
  if (root/'camera.log').read_text().count('RealSense Node Is Up!')>=len(roles):break
 else:raise RuntimeError('camera startup timeout')
 spin(4)
 phase='baseline';u0=udp();spin(a.baseline_seconds);u1=udp()
 phase='recording';n.start();spin(a.seconds);n.stop();phase='finalizing'
 deadline=time.monotonic()+a.validation_timeout
 while time.monotonic()<deadline and (n.process is not None or n.validation is not None):rclpy.spin_once(n,timeout_sec=.01)
 if n.process is not None or n.validation is not None:raise RuntimeError('finalization timeout')
 manifest=json.loads(Path(n.output+'.manifest.json').read_text());counts=defaultdict(Counter)
 for l in (Path(n.output)/'timing_records.jsonl').read_text().splitlines():
  r=json.loads(l)
  if 'frame_ref' in r.get('payload',{}):counts[r['payload']['frame_ref']['topic']][r['timestamp_ns']]+=1
 def metrics(rows):
  d=stream_metrics(rows,30);d['observed_header_fps']=(len(rows)-1)*1e9/(rows[-1]['header_ros_ns']-rows[0]['header_ros_ns']) if len(rows)>1 and rows[-1]['header_ros_ns']>rows[0]['header_ros_ns'] else None;return d
 result={'label':a.label,'roles':roles,'requested_recording_seconds':a.seconds,'baseline_seconds':a.baseline_seconds,'episode':n.output,'readable':manifest.get('readable'),'validation_errors':manifest.get('validation_errors'),'duration_seconds':manifest.get('duration_seconds'),'storage_bytes':manifest.get('storage_bytes'),'storage_bytes_per_second':manifest.get('storage_bytes_per_second'),'environment':{k:os.environ.get(k) for k in ('ROS_DOMAIN_ID','ROS_LOCALHOST_ONLY','RMW_IMPLEMENTATION','FASTRTPS_DEFAULT_PROFILES_FILE')},'baseline_metrics':{t:metrics(v) for (ph,t),v in samples.items() if ph=='baseline'},'recording_observer_metrics':{t:metrics(v) for (ph,t),v in samples.items() if ph=='recording'},'observer_vs_bag':{t:loss_comparison(Counter(r['header_ros_ns'] for r in samples.get(('recording',t),[])),v) for t,v in counts.items()},'bag_stream_metrics':manifest.get('timing_diagnostics',{}).get('streams'),'bag_profiles':manifest.get('camera_actual_profiles'),'storage_warnings':manifest.get('storage_warnings'),'host_udp_baseline_delta':delta(u0,u1),'host_udp_recording_and_validation_delta':delta(u1,udp()),'camera_device_info':manifest.get('camera_device_info'),'alignment':manifest.get('alignment'),'note':'Host UDP counters are global; shared subscriber losses do not prove camera publication gaps. Header matching is not exposure synchronization.'}
 (root/'result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({'label':a.label,'result':str(root/'result.json'),'readable':result['readable'],'bytes':result['storage_bytes']}),flush=True)
finally:
 n.close();n.destroy_node();rclpy.shutdown()
 if camera:
  os.killpg(camera.pid,signal.SIGINT)
  try:camera.wait(timeout=15)
  except subprocess.TimeoutExpired:os.killpg(camera.pid,signal.SIGTERM);camera.wait(timeout=5)
 log.close()
