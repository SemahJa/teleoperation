"""Exercise independent feature switches through actual mock ROS launches."""
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import time
import unittest
import uuid
import yaml
import rclpy
from rclpy.node import Node
from quest_teleop_interfaces.msg import HomingStatus, QuestControllerState, QuestButtonState

class LaunchOptionsTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls): rclpy.init()
    @classmethod
    def tearDownClass(cls): rclpy.shutdown()

    def run_features(self, enabled, expected, absent, home_disabled=False):
        prefix='/options_'+uuid.uuid4().hex
        node=Node('test_'+uuid.uuid4().hex)
        homes=[]
        node.create_subscription(HomingStatus, prefix+'/homing_status', homes.append, 10)
        raw=node.create_publisher(QuestControllerState, prefix+'/raw', 10)
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            arms=root/'arms.yaml'
            arms.write_text(yaml.safe_dump({'arms':{'arm':dict(namespace=prefix, controller='right',
                controller_state_topic=prefix+'/raw', base_frame=prefix.lstrip('/')+'/base')}}))
            with socket.socket() as sock:
                sock.bind(('127.0.0.1',0)); port=sock.getsockname()[1]
            args=['ros2','launch','teleop_bringup','teleoperation.launch.py', 'backend:=mock',
                  'arms_config:='+str(arms), 'recording_namespace:='+prefix, 'listen_port:='+str(port),
                  'recording_output_root:='+str(root/'bags')]
            switches=dict(start_controller=False,start_mapper=False,start_bridge=False,start_hand=False,
                          recording_enabled=False,mock_measured_pose=False,homing_enabled=False,cbf_enabled=False)
            switches.update(enabled)
            args += [k+':='+str(v).lower() for k,v in switches.items()]
            with (root/'launch.log').open('w') as log:
                proc=subprocess.Popen(args, stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                try:
                    deadline=time.monotonic()+12
                    names=set()
                    while time.monotonic()<deadline:
                        rclpy.spin_once(node, timeout_sec=.05)
                        names={name for name,ns in node.get_node_names_and_namespaces() if ns==prefix}
                        if set(expected)<=names: break
                        if proc.poll() is not None: break
                    self.assertTrue(set(expected)<=names,(names,(root/'launch.log').read_text()))
                    self.assertFalse(set(absent)&names,names)
                    if home_disabled:
                        end=time.monotonic()+1.8; sequence=0
                        while time.monotonic()<end:
                            sequence+=1
                            m=QuestControllerState(controller_id='right',session_id='options',sequence=sequence,
                                source_time_pc_monotonic_ns=time.monotonic_ns(),clock_uncertainty_ns=1000000,
                                reference_space='LOCAL',reference_space_generation=1)
                            for flag in ('connection_valid','session_focused','pose_action_active','position_valid',
                                         'orientation_valid','position_tracked','orientation_tracked','trigger_active',
                                         'source_time_valid','clock_sync_valid'): setattr(m,flag,True)
                            m.buttons=[QuestButtonState(name='a',active=True,pressed=sequence>10 and time.monotonic()<end-.2)]
                            raw.publish(m);rclpy.spin_once(node,timeout_sec=.01)
                        self.assertTrue(homes,(root/'launch.log').read_text())
                        self.assertTrue(all(h.request_id==0 and not h.active for h in homes))
                finally:
                    if proc.poll() is None: os.killpg(proc.pid,signal.SIGINT)
                    try: proc.wait(timeout=15)
                    except subprocess.TimeoutExpired: os.killpg(proc.pid,signal.SIGKILL);proc.wait()
                    node.destroy_node()
                self.assertEqual(proc.returncode,0,(root/'launch.log').read_text())

    def test_owner_only_homing_disabled(self):
        self.run_features({'start_controller':True}, ['panda_cartesian_impedance_controller'],
                          ['pose_mapper','franka_hand_worker','menu_recorder','mock_measured_pose'],True)

    def test_empty_selection_and_conflicting_fixture(self):
        disabled=['start_controller:=false','start_mapper:=false','start_hand:=false',
                  'start_bridge:=false','recording_enabled:=false']
        result=subprocess.run(['ros2','launch','teleop_bringup','teleoperation.launch.py',*disabled],
            capture_output=True,text=True,timeout=10)
        self.assertEqual(result.returncode,0,result.stdout+result.stderr)
        result=subprocess.run(['ros2','launch','teleop_bringup','teleoperation.launch.py',
            'mock_measured_pose:=true'],capture_output=True,text=True,timeout=10)
        self.assertNotEqual(result.returncode,0)
        self.assertIn('avoid competing pose publishers',result.stdout+result.stderr)

    def test_mapping_without_owners(self):
        self.run_features({'start_mapper':True,'mock_measured_pose':True,'start_bridge':True},
                          ['pose_mapper','mock_measured_pose'],['panda_cartesian_impedance_controller','franka_hand_worker','menu_recorder'])

    def test_gripper_and_recording_without_arm_or_mapper(self):
        self.run_features({'start_hand':True,'recording_enabled':True},['franka_hand_worker','menu_recorder'],
                          ['panda_cartesian_impedance_controller','pose_mapper','mock_measured_pose'])


class StandaloneLaunchTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls): rclpy.init()
    @classmethod
    def tearDownClass(cls): rclpy.shutdown()

    def test_standalone_feature_launches_and_parameter_loading(self):
        prefix='/standalone_'+uuid.uuid4().hex
        node=Node('test_'+uuid.uuid4().hex)
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            arms=root/'arms.yaml'
            arms.write_text(yaml.safe_dump({'arms':{'arm':dict(namespace=prefix, controller='right',
                base_frame=prefix.lstrip('/')+'/base')}}))
            with socket.socket() as sock:
                sock.bind(('127.0.0.1',0)); port=sock.getsockname()[1]
            cases=[
                ('quest_teleop','quest_usb_bridge.launch.py',['listen_port:='+str(port)],'quest_usb_bridge',prefix),
                ('quest_teleop','pose_mapper.launch.py',['command_topic:='+prefix+'/command',
                  'monitor_topic:='+prefix+'/target','measured_pose_topic:='+prefix+'/anchor',
                  'controller_state_topic:='+prefix+'/raw'],'pose_mapper',prefix),
                ('teleop_controllers','cartesian_impedance_controller.launch.py',['cbf_enabled:=false'],
                 'panda_cartesian_impedance_controller',prefix),
                ('teleop_controllers','homing.launch.py',['cbf_enabled:=false'],
                 'panda_cartesian_impedance_controller',prefix),
                ('teleop_gripper','gripper.launch.py',[],'franka_hand_worker',prefix),
                ('recording','recording.launch.py',['recording_namespace:='+prefix,
                  'recording_output_root:='+str(root/'bags')],'menu_recorder',prefix)]
            try:
                for index,(package,launch,extra,name,namespace) in enumerate(cases):
                    if namespace == prefix and package != 'recording':
                        namespace = prefix+"/case_"+str(index)
                        arms.write_text(yaml.safe_dump({"arms":{"arm":dict(namespace=namespace, controller="right",
                            base_frame=namespace.lstrip("/")+"/base")}}))
                    with self.subTest(package=package,launch=launch):
                        args=['ros2','launch',package,launch,*extra]
                        if package!='quest_teleop': args.append('arms_config:='+str(arms))
                        else: args.append('namespace:='+namespace)
                        log_path=root/(package+'-'+launch+'.log')
                        with log_path.open('w') as log:
                            proc=subprocess.Popen(args,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                            try:
                                end=time.monotonic()+10
                                while time.monotonic()<end:
                                    rclpy.spin_once(node,timeout_sec=.03)
                                    if (name,namespace) in node.get_node_names_and_namespaces(): break
                                    if proc.poll() is not None: break
                                self.assertIsNone(proc.poll(),log_path.read_text())
                                self.assertIn((name,namespace),node.get_node_names_and_namespaces(),log_path.read_text())
                            finally:
                                if proc.poll() is None: os.killpg(proc.pid,signal.SIGINT)
                                try: proc.wait(timeout=15)
                                except subprocess.TimeoutExpired: os.killpg(proc.pid,signal.SIGKILL);proc.wait()
                            self.assertEqual(proc.returncode,0,log_path.read_text())
                # A separate safety file must reach the arm owner through the scoped include.
                safety=root/'safety.yaml'
                safety.write_text(yaml.safe_dump({'cbf':{'table_height':2.0,'maximum_failures':1}}))
                result=subprocess.run(['ros2','launch','teleop_controllers','cartesian_impedance_controller.launch.py',
                    'arms_config:='+str(arms),'safety_config:='+str(safety)],capture_output=True,text=True,timeout=15)
                self.assertIn('latched CBF stop: solver_failures',result.stdout+result.stderr)
                # Its standalone entry loads parameters without opening any FCI connection.
                result=subprocess.run(['ros2','launch','safety_filter','safety_filter.launch.py',
                    'arms_config:='+str(arms),'safety_config:='+str(safety)],capture_output=True,text=True,timeout=10)
                self.assertEqual(result.returncode,0,result.stdout+result.stderr)
                self.assertIn('Loaded safety_filter parameters',result.stdout)
            finally: node.destroy_node()

if __name__=='__main__': unittest.main()
