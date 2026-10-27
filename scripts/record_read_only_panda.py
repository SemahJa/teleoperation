#!/usr/bin/env python3
import argparse,json,os,signal,subprocess,time,math
from datetime import datetime,timezone
import uuid
from ament_index_python.packages import get_package_prefix
from pathlib import Path
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from sensor_msgs.msg import Image
from recording.menu_recorder import MenuRecorder
parser=argparse.ArgumentParser(description="Bounded camera + measured Panda state recording; no control or gripper owners")
parser.add_argument('--robot-ip',required=True);parser.add_argument('--seconds',type=float,default=15.0);parser.add_argument('--sample-hz',type=float,default=100.0);parser.add_argument('--output-root',default='log/read_only_panda_episodes');a=parser.parse_args()
if not math.isfinite(a.seconds) or not 0<a.seconds<=60 or not math.isfinite(a.sample_hz) or not 1<=a.sample_hz<=200:parser.error('seconds must be(0,60], sample-hz1..200')
root=(Path(a.output_root)/('trial_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'_'+uuid.uuid4().hex[:8])).resolve();root.mkdir(parents=True,exist_ok=False)
topics=[f'/cameras/{r}/{s}' for r in ('wrist','scene') for s in ('color/image_raw','aligned_depth_to_color/image_raw','color/camera_info','aligned_depth_to_color/camera_info','color/metadata','depth/metadata')]+['/tf_static','/demonstration_timing/clock_observations','/demonstration_timing/recording_health','/panda_1/joint_states','/panda_1/measured_ee_pose','/panda_1/read_only_state','/panda_1/tracking_samples','/panda_1/gripper_events']
rclpy.init();params=dict(profile='demonstration',record_late_topics=True,storage_id='mcap',output_root=str(root/'bags'),topics_json=json.dumps(topics),controller_state_topic='/read_only_validation/unused_menu',menu_enabled=False,max_cache_size=268435456,clock_provenance='same_host')
n=MenuRecorder(parameter_overrides=[Parameter(k,value=v) for k,v in params.items()]);latest={};images={};procs=[];logs=[];subscriptions=[]
def spin(sec):
 end=time.monotonic()+sec
 while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.01)
def launch(args,name):
 log=(root/(name+'.log')).open('w');logs.append(log);p=subprocess.Popen(args,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);procs.append(p);return p
try:
 for role in ('wrist','scene'):
  for kind in ('color','aligned_depth_to_color'):
   t=f'/cameras/{role}/{kind}/image_raw'
   subscriptions.append(n.create_subscription(Image,t,lambda b,t=t:images.update({t:images.get(t,0)+1}),qos_profile_sensor_data,raw=True))
 subscriptions.append(n.create_subscription(String,'/panda_1/read_only_state',lambda m:latest.update(json.loads(m.data)),qos_profile_sensor_data))
 camera=launch(['ros2','launch','camera_bringup','cameras.launch.py','cameras:=both'],'cameras')
 deadline=time.monotonic()+40
 while time.monotonic()<deadline:
  rclpy.spin_once(n,timeout_sec=.05)
  if len(images)==4 and all(v>30 for v in images.values()):break
 else:raise RuntimeError('camera image readiness timeout')
 reader=launch([str(Path(get_package_prefix('panda_state_bringup'))/'lib/panda_state_bringup/read_only_state'),'--ros-args','-r','__ns:=/panda_1','-p','robot_ip:='+a.robot_ip,'-p','sample_hz:='+str(a.sample_hz),'-p','duration_seconds:='+str(a.seconds+15)],'read_only_panda')
 deadline=time.monotonic()+12
 while time.monotonic()<deadline and not latest:
  rclpy.spin_once(n,timeout_sec=.05)
  if reader.poll() is not None:raise RuntimeError('reader exited before first real state')
 if not latest or not latest.get('read_only') or not latest.get('hardware_backend'):raise RuntimeError('real read-only state unavailable')
 print('REAL_STATE_READY',json.dumps({k:latest.get(k) for k in ('robot_mode','server_version','libfranka_version','host_clock_id')}),flush=True)
 spin(1);n.start();spin(a.seconds);n.stop()
 deadline=time.monotonic()+120
 while time.monotonic()<deadline and (n.process is not None or n.validation is not None):rclpy.spin_once(n,timeout_sec=.01)
 if n.process is not None or n.validation is not None:raise RuntimeError('episode finalization timeout')
 d=json.loads(Path(n.output+'.manifest.json').read_text());result={'episode':n.output,'readable':d.get('readable'),'errors':d.get('validation_errors'),'duration_seconds':d.get('duration_seconds'),'storage_bytes':d.get('storage_bytes'),'topics':d.get('recorded_topics'),'alignment':d.get('alignment'),'last_robot_mode':latest.get('robot_mode'),'reader_exit_before_stop':reader.poll(),'hardware_controller_started':False,'gripper_connection_opened':False};(root/'result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
 assert result['readable'] and not result['errors']
 for topic in ('/panda_1/joint_states','/panda_1/measured_ee_pose','/panda_1/read_only_state'):
  assert result['topics'].get(topic,{}).get('message_count',0)>100
finally:
 n.close();n.destroy_node();rclpy.shutdown()
 for p in reversed(procs):
  if p.poll() is None:
   os.killpg(p.pid,signal.SIGINT)
   try:p.wait(timeout=12)
   except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=5)
 for log in logs:log.close()
