"""Actual single/dual owners call the CBF coordinator; no FCI operation."""
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import unittest
import uuid
import rclpy
from rclpy.node import Node
from quest_teleop_interfaces.msg import CartesianTeleopCommand, ControllerStatus, CbfStatus

class CbfRuntimeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls): rclpy.init()
    @classmethod
    def tearDownClass(cls): rclpy.shutdown()

    def test_dual_coupled_filter_and_independent_commands(self):
        prefix='/cbf_'+uuid.uuid4().hex
        node=Node('test_'+uuid.uuid4().hex)
        safety={s:[] for s in ('left','right')}; control={s:[] for s in safety}
        pubs={}
        for side in safety:
            namespace=prefix+'/'+side
            node.create_subscription(CbfStatus, namespace+'/cbf_status', safety[side].append,100)
            node.create_subscription(ControllerStatus, namespace+'/controller_status', control[side].append,100)
            pubs[side]=node.create_publisher(CartesianTeleopCommand,namespace+'/cartesian_teleop_command',10)
        with tempfile.TemporaryDirectory() as directory:
            log_path=Path(directory)/'dual.log'
            with log_path.open('w') as log:
                args=['ros2','run','franka_teleop_controller','cartesian_impedance_controller','--ros-args',
                      '-p','mode:=dual','-p','backend:=mock']
                for side in safety:
                    args+=['-p',side+'.namespace:='+prefix+'/'+side,
                           '-p',side+'.base_frame:='+prefix.lstrip('/')+'/'+side+'/base',
                           '-p',side+'.controller_state_topic:='+prefix+'/'+side+'/raw']
                proc=subprocess.Popen(args,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                seq=0
                def drive(duration=.1, right_active=False):
                    nonlocal seq
                    end=time.monotonic()+duration
                    while time.monotonic()<end:
                        seq+=1
                        for side in safety:
                            msg=CartesianTeleopCommand()
                            msg.header.frame_id=prefix.lstrip('/')+'/'+side+'/base'
                            msg.session_id='runtime-'+side; msg.sequence=seq
                            msg.source_time_pc_monotonic_ns=time.monotonic_ns()
                            msg.clock_uncertainty_ns=1000000
                            msg.source_time_valid=msg.clock_sync_valid=msg.input_valid=True
                            msg.reference_space_generation=1
                            msg.motion_enabled=side=='right' and right_active
                            msg.trigger_released=not msg.motion_enabled
                            msg.target_pose.position.x=.42 if msg.motion_enabled else .4
                            msg.target_pose.position.z=.4; msg.target_pose.orientation.w=1.
                            pubs[side].publish(msg)
                        until=time.monotonic()+.01
                        while time.monotonic()<until: rclpy.spin_once(node,timeout_sec=.002)
                def wait(predicate, timeout=10):
                    end=time.monotonic()+timeout
                    while time.monotonic()<end:
                        rclpy.spin_once(node,timeout_sec=.01)
                        if predicate(): return
                        if proc.poll() is not None: break
                    self.fail(log_path.read_text())
                try:
                    wait(lambda: all(safety[s] and control[s] and pubs[s].get_subscription_count() for s in safety))
                    drive(.15); drive(.15,True)
                    self.assertTrue(control['right'][-1].motion_enabled)
                    self.assertFalse(control['left'][-1].motion_enabled)
                    for side in safety:
                        event=safety[side][-1]
                        self.assertTrue(event.solver_succeeded,event.solver_status)
                        self.assertEqual(event.stop_reason,'none')
                        self.assertFalse(event.hardware_backend)
                        self.assertTrue(all(event.source_time_pc_monotonic_ns))
                        self.assertLess(event.state_skew_ns,50000000)
                        self.assertGreater(event.minimum_inter_arm_clearance,1.)
                        self.assertGreater(event.active_constraints,0)
                        self.assertLessEqual(event.maximum_constraint_violation,.001)
                    self.assertTrue(any(s.intervention_norm>.01 for s in safety['right']))
                    # Both diagnostic streams report identical paired cycles.
                    paired={tuple(s.source_time_pc_monotonic_ns):s for s in safety['left']}
                    matches=[s for s in safety['right'] if tuple(s.source_time_pc_monotonic_ns) in paired]
                    self.assertTrue(matches)
                    for right in matches:
                        left=paired[tuple(right.source_time_pc_monotonic_ns)]
                        self.assertEqual(left.control_cycle,right.control_cycle)
                finally:
                    if proc.poll() is None: os.killpg(proc.pid,signal.SIGINT)
                    try: proc.wait(timeout=8)
                    except subprocess.TimeoutExpired:
                        os.killpg(proc.pid,signal.SIGKILL);proc.wait()
                    node.destroy_node()
                self.assertEqual(proc.returncode,0,log_path.read_text())

    def test_repeated_solver_failure_stops_process(self):
        result=subprocess.run(['ros2','run','franka_teleop_controller','cartesian_impedance_controller',
            '--ros-args','-p','backend:=mock','-p','cbf.table_height:=2.0',
            '-p','cbf.maximum_failures:=3','-p','controller_state_topic:=/unused_cbf_fault_input'],
            capture_output=True,text=True,timeout=10)
        self.assertNotEqual(result.returncode,0)
        self.assertIn('solver_failures',result.stderr+result.stdout)

    def test_duplicate_dual_ownership_rejected_before_connection(self):
        for extra, expected in [(['-p','left.namespace:=/same','-p','right.namespace:=/same'],'unique'),
                                (['-p','mode:=bad'],'mode must')]:
            result=subprocess.run(['ros2','run','franka_teleop_controller','cartesian_impedance_controller',
                '--ros-args','-p','mode:=dual','-p','backend:=mock']+extra,
                capture_output=True,text=True,timeout=10)
            self.assertNotEqual(result.returncode,0)
            self.assertIn(expected,result.stderr+result.stdout)

if __name__=='__main__': unittest.main()
