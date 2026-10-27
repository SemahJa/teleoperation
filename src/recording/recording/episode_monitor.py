"""Non-realtime observer: original image-header identities, clock brackets, health.

This is a diagnostic subscriber, never a bag writer. It cannot distinguish a
publication gap from a packet missed by both this observer and the recorder.
"""
from collections import defaultdict, Counter
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import shutil
import socket
import time
from demonstration_timing.clocks import HostClockBridge
from recording.image_tools import image_view
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from rosidl_runtime_py.utilities import get_message
from rosidl_runtime_py.convert import message_to_ordereddict
from std_msgs.msg import String


def atomic_json(path,data):
    path=Path(path); temporary=path.with_suffix(path.suffix+'.tmp')
    temporary.write_text(json.dumps(data,indent=2,allow_nan=False));temporary.replace(path)


class EpisodeMonitor:
    def __init__(self,node,topics,output,provenance):
        self.node=node;self.topics=topics;self.output=output;self.provenance=provenance
        self.bridge=HostClockBridge();self.anchors=[];self.images=defaultdict(Counter)
        self.counts=Counter();self.fingerprints=defaultdict(dict);self.subscriptions={}
        self.graph={};self.devices={};self.clients={};self.pending={};self.first={};self.last={}
        self.host='host_monotonic:'+socket.gethostname()+':'+Path('/proc/sys/kernel/random/boot_id').read_text().strip()
        self.ros='host_ros_system:'+socket.gethostname()+':'+Path('/proc/sys/kernel/random/boot_id').read_text().strip()
        self.clock_pub=node.create_publisher(String,'/demonstration_timing/clock_observations',qos_profile_sensor_data)
        self.health_pub=node.create_publisher(String,'/demonstration_timing/recording_health',qos_profile_sensor_data)
        self.last_bytes=0;self.last_bytes_time=time.monotonic()
        self.clock_timer=node.create_timer(.1,self.clock)
        self.health_timer=node.create_timer(1.,self.health)
        self.clock();self.health()

    def clock(self):
        before=time.monotonic_ns();ros=self.node.get_clock().now().nanoseconds;after=time.monotonic_ns()
        anchor=self.bridge.observe(before,ros,after);self.anchors.append(anchor)
        msg=String();msg.data=json.dumps(dict(anchor,host_clock_id=self.host,ros_clock_id=self.ros,
              clock_provenance=self.provenance,camera_capture_uncertainty_ns=None));self.clock_pub.publish(msg)

    def health(self):
        if not self.node.context.ok(): return
        graph=dict(self.node.get_topic_names_and_types())
        for topic in self.topics:
            types=graph.get(topic,[])
            if types:self.graph[topic]=types
            if topic in self.subscriptions or not types or topic.startswith('/demonstration_timing/'):continue
            try:
                cls=get_message(types[0])
                qos=qos_profile_sensor_data if topic!='/tf_static' and not topic.endswith('/gripper_events') else QoSProfile(depth=100,durability=DurabilityPolicy.TRANSIENT_LOCAL)
                def receive(raw,topic=topic,msgtype=types[0]):
                    self.counts[topic]+=1
                    if msgtype!='sensor_msgs/msg/Image':return
                    header,pixels=image_view(raw);stamp=header["stamp_ns"]
                    self.images[topic][str(stamp)]+=1
                    self.first.setdefault(topic,stamp);self.last[topic]=stamp
                    # Sparse independent payload checksums; no image data retained.
                    if self.counts[topic]%15==0 and len(self.fingerprints[topic])<8:
                        self.fingerprints[topic][str(stamp)]=hashlib.sha256(pixels).hexdigest()
                self.subscriptions[topic]=self.node.create_subscription(cls,topic,receive,qos,raw=True)
            except (ImportError,ValueError,AttributeError):continue
        from realsense2_camera_msgs.srv import DeviceInfo
        services=dict(self.node.get_service_names_and_types())
        for role in ('wrist','scene'):
            if role in self.devices or role in self.pending:continue
            names=[name for name,types in services.items() if name.startswith('/cameras/'+role+'/') and 'realsense2_camera_msgs/srv/DeviceInfo' in types]
            if not names:continue
            client=self.node.create_client(DeviceInfo,names[0]);self.clients[role]=client
            if not client.service_is_ready():continue
            future=client.call_async(DeviceInfo.Request());self.pending[role]=future
            def done(future,role=role):
                try:self.devices[role]=dict(message_to_ordereddict(future.result()))
                except Exception as e:self.devices[role]={'query_error':str(e)}
            future.add_done_callback(done)
        path=Path(self.output); total=sum(p.stat().st_size for p in path.glob('*') if p.is_file()) if path.exists() else 0
        msg=String();msg.data=json.dumps({'episode':path.name,'observed_message_counts':dict(self.counts),
            'bag_bytes_so_far':total,'file_growth_bytes_per_second':(total-self.last_bytes)/max(.001,time.monotonic()-self.last_bytes_time),'disk_free_bytes':shutil.disk_usage(path.parent).free,
            'storage_drop_counter_available':False,'warning':'Humble lacks a portable recorder lost-message counter; compare bag/observer identities and logs.'})
        self.health_pub.publish(msg)
        self.last_bytes=total;self.last_bytes_time=time.monotonic()

    def snapshot(self):
        try:models=[asdict(m) for m in HostClockBridge.models(self.anchors,self.ros,self.host)]
        except ValueError:models=[]
        return {'observer_counts':dict(self.counts),'observer_image_stamps':{k:dict(v) for k,v in self.images.items()},
          'observer_payload_fingerprints':dict(self.fingerprints),'publisher_types_seen':self.graph,
          'camera_device_info':self.devices,'clock_anchors':self.anchors,'clock_models':models,
          'host_clock_id':self.host,'ros_clock_id':self.ros,'clock_provenance':self.provenance,
          'capture_uncertainty_ns':None,'extrinsic_calibration':'uncalibrated',
          'time_namespace_offsets':Path('/proc/self/timens_offsets').read_text()}

    def close(self):
        self.node.destroy_timer(self.clock_timer);self.node.destroy_timer(self.health_timer)
        for subscription in self.subscriptions.values():self.node.destroy_subscription(subscription)
        for client in self.clients.values():self.node.destroy_client(client)
        self.node.destroy_publisher(self.clock_pub);self.node.destroy_publisher(self.health_pub)
