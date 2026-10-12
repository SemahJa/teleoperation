"""Actual launch modes and Menu recording with no active left arm in single-right."""
import json
import os
from pathlib import Path
import signal
import sqlite3
import subprocess
import tempfile
import time
import unittest
import uuid
import yaml
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy
from quest_teleop_interfaces.msg import ControllerStatus, HomingStatus, QuestControllerState, QuestButtonState, GripperStatus

class TeleoperationLaunchTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls): rclpy.init()
    @classmethod
    def tearDownClass(cls): rclpy.shutdown()

    def run_mode(self, mode='single', side='right', record=False):
        prefix='/launch_'+uuid.uuid4().hex
        node=Node('test_'+uuid.uuid4().hex)
        statuses={s:[] for s in ('left','right')}; homes={s:[] for s in statuses}
        raw={s:node.create_publisher(QuestControllerState,prefix+'/'+s+'/raw',10) for s in statuses}
        for s in statuses:
            node.create_subscription(ControllerStatus,prefix+'/'+s+'/controller_status',statuses[s].append,100)
            node.create_subscription(HomingStatus,prefix+'/'+s+'/homing_status',homes[s].append,100)
        grippers=[]
        node.create_subscription(GripperStatus,prefix+'/right/gripper_events',grippers.append,100)
        recordings=[]
        # The recording status topic is remapped per test to avoid existing user's nodes.
        node.create_subscription(String,prefix+'/recording_status',
            lambda m:recordings.append(json.loads(m.data)),QoSProfile(depth=100,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        foreign=node.create_publisher(String,prefix+'/left/foreign_published',10)
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            arms={s:dict(namespace=prefix+'/'+s,controller=s,base_frame=prefix.lstrip('/')+'/'+s+'/base',
                controller_state_topic=prefix+'/'+s+'/raw') for s in statuses}
            config=root/'arms.yaml'; config.write_text(yaml.safe_dump(dict(arms=arms)))
            runtime=root/'runtime.yaml'; runtime.write_text((Path(__file__).parents[1]/'config/teleoperation_runtime.yaml').read_text())
            record_config=root/'record.yaml'; record_config.write_text(yaml.safe_dump(dict(
                include_default_topics=False,output_root=str(root/'bags'),storage_id='sqlite3',topics=[
                    prefix+'/right/controller_status',prefix+'/right/cartesian_teleop_command',
                    prefix+'/left/foreign_published',prefix+'/unpublished'])))
            args=['ros2','launch','franka_teleop_controller','cartesian_impedance_controller.launch.py',
                'backend:=mock','arms_config:='+str(config),'runtime_config:='+str(runtime),
                'start_hand:='+str(record).lower(),'recording_enabled:='+str(record).lower(),
                'left_controller_state_topic:='+prefix+'/left/raw','recording_config:='+str(record_config)]
            # Node action doesn't take global CLI remaps. Status is namespaced /quest,
            # so query only this launched recorder by namespace remapping in launch.
            args+=['recording_namespace:='+prefix]
            if mode!='single': args+=['mode:='+mode]
            if side!='right': args+=['single_controller:='+side]
            log_path=root/'launch.log'
            with log_path.open('w') as log:
                proc=subprocess.Popen(args,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                sequence=0
                def drive(duration=.1,menu=False,trigger=0.,home=False,tracked=True):
                    nonlocal sequence
                    end=time.monotonic()+duration
                    while time.monotonic()<end:
                        sequence+=1
                        for s in statuses:
                            m=QuestControllerState(); m.controller_id=s; m.session_id='launch-'+s
                            m.sequence=sequence; m.source_time_pc_monotonic_ns=time.monotonic_ns()
                            m.acquisition_time_ns=m.openxr_location_time_ns=m.source_time_pc_monotonic_ns
                            m.source_time_valid=m.clock_sync_valid=True; m.clock_uncertainty_ns=1000000
                            m.connection_valid=m.session_focused=m.pose_action_active=True
                            m.position_valid=m.orientation_valid=True
                            m.position_tracked=m.orientation_tracked=tracked
                            m.reference_space='LOCAL';m.reference_space_generation=1
                            m.trigger_active=True;m.trigger_value=trigger
                            m.grip_pose.orientation.w=1.
                            b=QuestButtonState();b.name='x' if s=='left' else 'a';b.active=True;b.pressed=(home is True or home==s)
                            m.buttons=[b]
                            if s=='left':
                                b=QuestButtonState();b.name='menu';b.active=True;b.pressed=menu
                                m.buttons.append(b)
                            raw[s].publish(m)
                        foreign.publish(String(data='must be excluded in single right'))
                        until=time.monotonic()+.01
                        while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.002)
                def wait(predicate,timeout=10,send=False,**drive_args):
                    end=time.monotonic()+timeout
                    while time.monotonic()<end:
                        if send: drive(.03,**drive_args)
                        else:rclpy.spin_once(node,timeout_sec=.01)
                        if predicate():return
                        if proc.poll() is not None:break
                    self.fail(str({s:statuses[s][-1].reason if statuses[s] else None for s in statuses})+'\n'+log_path.read_text())
                expected=('left','right') if mode=='dual' else (side,)
                try:
                    wait(lambda:all(statuses[s] and homes[s] and raw[s].get_subscription_count() >= (3 if record else 2) for s in expected))
                    drive(.15,menu=record);drive(.15,menu=record,trigger=.8)
                    wait(lambda:all(statuses[s][-1].motion_enabled for s in expected),send=True,menu=record,trigger=.8)
                    if mode=='single':
                        other='left' if side=='right' else 'right'
                        self.assertFalse(statuses[other]);self.assertFalse(homes[other])
                    drive(.15,menu=record)
                    if mode=='dual':
                        drive(1.1,home='left')
                        self.assertEqual(homes['left'][-1].request_id,0)
                        drive(.05)
                        wait(lambda:homes['left'][-1].phase=='succeeded',send=True)
                        self.assertEqual(homes['left'][-1].request_id,1)
                        self.assertEqual(homes['right'][-1].request_id,0)
                        drive(1.1,home='right');drive(.05)
                        wait(lambda:homes['right'][-1].phase=='succeeded',send=True)
                        self.assertEqual(homes['right'][-1].request_id,1)
                        self.assertEqual(homes['left'][-1].request_id,1)
                    if record:
                        wait(lambda:recordings and grippers and raw['left'].get_subscription_count())
                        self.assertFalse(grippers[-1].calibrated)
                        self.assertFalse(grippers[-1].busy)
                        self.assertEqual(grippers[-1].operations_started,0)
                        # First held Menu without release must not start a bag.
                        drive(.4,menu=True)
                        self.assertEqual(recordings[-1]['phase'],'idle')
                        drive(.1);drive(.1,menu=True)
                        wait(lambda:recordings[-1]['phase']=='recording')
                        output=Path(recordings[-1]['output'])
                        self.assertEqual(set(recordings[-1]['topics']),{
                            prefix+'/right/controller_status',prefix+'/right/cartesian_teleop_command'})
                        wait(lambda:len(list(output.glob('*.db3')))>0)
                        wait(lambda:node.count_subscribers(prefix+'/right/controller_status') >= 2 and
                             node.count_subscribers(prefix+'/right/cartesian_teleop_command') >= 2,
                             send=True,menu=True)
                        drive(.7,menu=True)
                        self.assertEqual(recordings[-1]['output'],str(output))
                        self.assertEqual(recordings[-1]['phase'],'recording')
                        # Tracking invalidity doesn't prevent valid Menu stop.
                        drive(.1);drive(.1,menu=True,tracked=False)
                        wait(lambda:recordings[-1]['phase']=='idle',send=True)
                        self.assertTrue((output/'metadata.yaml').is_file())
                        with sqlite3.connect(next(output.glob('*.db3'))) as db:
                            topics={row[0] for row in db.execute('select name from topics')}
                            self.assertEqual(topics,{prefix+'/right/controller_status',prefix+'/right/cartesian_teleop_command'}, Path(str(output)+'.recorder.log').read_text())
                            self.assertGreater(db.execute('select count(*) from messages').fetchone()[0],0)
                        # A second episode gets a unique path and finalizes on launch shutdown.
                        drive(.4);drive(.1,menu=True)
                        wait(lambda:recordings[-1]['phase']=='recording')
                        second=Path(recordings[-1]['output']);self.assertNotEqual(second,output)
                        wait(lambda:list(second.glob('*.db3')))
                        drive(.3,menu=True)
                finally:
                    if proc.poll() is None:os.killpg(proc.pid,signal.SIGINT)
                    try:proc.wait(timeout=15)
                    except subprocess.TimeoutExpired:
                        os.killpg(proc.pid,signal.SIGKILL);proc.wait()
                    node.destroy_node()
                self.assertEqual(proc.returncode,0,log_path.read_text())
                if record:self.assertTrue((second/'metadata.yaml').is_file(),log_path.read_text())

    def test_single_right_default_left_menu_active_topics_and_finalized_bags(self):self.run_mode(record=True)
    def test_single_left(self):self.run_mode(side='left')
    def test_dual(self):self.run_mode(mode='dual')

if __name__=='__main__':unittest.main()
