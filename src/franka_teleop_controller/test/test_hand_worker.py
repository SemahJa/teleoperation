"""Actual non-RT Hand worker with mock backend and guarded calibration service."""
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import unittest
import uuid
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from quest_teleop_interfaces.msg import GripperStatus, QuestControllerState
from quest_teleop_interfaces.srv import CalibrateGripper


class HandWorkerTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls): rclpy.init()

    @classmethod
    def tearDownClass(cls): rclpy.shutdown()

    def test_calibration_toggle_busy_and_disconnect_completion(self):
        prefix = '/hand_' + uuid.uuid4().hex
        node = Node('test_' + uuid.uuid4().hex)
        executor = SingleThreadedExecutor(); executor.add_node(node)
        statuses = []
        qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(GripperStatus, prefix + '/gripper_events', statuses.append, qos)
        publisher = node.create_publisher(QuestControllerState, prefix + '/raw', 10)
        calibration = node.create_client(CalibrateGripper, prefix + '/calibrate_gripper')
        sequence = 0
        with tempfile.TemporaryDirectory() as directory:
            log_path = Path(directory) / 'hand.log'
            with log_path.open('w') as log:
                proc = subprocess.Popen(['ros2', 'run', 'franka_teleop_controller', 'franka_hand_worker',
                    '--ros-args', '-r', '__ns:=' + prefix, '-p', 'backend:=mock',
                    '-p', 'controller_state_topic:=' + prefix + '/raw',
                    '-p', 'grasp_width_m:=0.006', '-p', 'epsilon_inner_m:=0.001',
                    '-p', 'epsilon_outer_m:=0.002', '-p', 'mock_operation_duration_ms:=300.0'],
                    stdout=log, stderr=subprocess.STDOUT, start_new_session=True)

                def wait_for(predicate, timeout=10):
                    end = time.monotonic() + timeout
                    while time.monotonic() < end:
                        executor.spin_once(timeout_sec=.005)
                        if predicate(): return
                        if proc.poll() is not None: break
                    self.fail('Hand condition timed out:\n' + log_path.read_text())

                def send(value, tracked=True, duration=.08):
                    nonlocal sequence
                    end = time.monotonic() + duration
                    while time.monotonic() < end:
                        sequence += 1
                        s = QuestControllerState()
                        s.controller_id = 'right'; s.session_id = 'hand-session'; s.sequence = sequence
                        s.source_time_pc_monotonic_ns = time.monotonic_ns()
                        s.clock_sync_valid = s.source_time_valid = True
                        s.clock_uncertainty_ns = 1000000
                        s.reference_space = 'LOCAL'; s.reference_space_generation = 1
                        s.connection_valid = s.session_focused = s.pose_action_active = tracked
                        s.position_valid = s.orientation_valid = True
                        s.position_tracked = s.orientation_tracked = tracked
                        s.grip_active = True; s.grip_value = float(value)
                        publisher.publish(s)
                        until = time.monotonic() + .01
                        while time.monotonic() < until: executor.spin_once(timeout_sec=.002)

                def calibrate(empty, confirm):
                    request = CalibrateGripper.Request()
                    request.empty_hand_confirm = empty; request.operator_confirm = confirm
                    future = calibration.call_async(request)
                    wait_for(future.done)
                    return future.result()

                try:
                    wait_for(lambda: statuses and calibration.service_is_ready() and
                             publisher.get_subscription_count() > 0)
                    self.assertEqual(statuses[-1].state, 'unknown')
                    self.assertEqual(statuses[-1].operations_started, 0)
                    self.assertFalse(statuses[-1].calibrated)
                    send(0); send(.8)
                    self.assertEqual(statuses[-1].operations_started, 0)
                    self.assertFalse(calibrate(False, True).accepted)
                    self.assertFalse(calibrate(True, False).accepted)
                    self.assertTrue(calibrate(True, True).accepted)
                    self.assertFalse(calibrate(True, True).accepted)
                    wait_for(lambda: statuses[-1].calibrated and not statuses[-1].busy)
                    self.assertEqual(statuses[-1].state, 'unknown')
                    send(0); send(.8)
                    wait_for(lambda: statuses[-1].busy and statuses[-1].operation == 'open')
                    # Deliberate extra press while busy is ignored, not queued.
                    send(0); send(.8)
                    wait_for(lambda: statuses[-1].operations_completed == 2)
                    self.assertEqual(statuses[-1].state, 'open')
                    send(.8)
                    self.assertEqual(statuses[-1].operations_started, 2)
                    send(0); send(.8)
                    wait_for(lambda: statuses[-1].busy and statuses[-1].operation == 'grasp')
                    send(.8, tracked=False)
                    wait_for(lambda: statuses[-1].operations_completed == 3)
                    self.assertEqual(statuses[-1].state, 'grasped')
                    self.assertAlmostEqual(statuses[-1].width, .006)
                    send(.8)
                    self.assertEqual(statuses[-1].operations_started, 3)
                    send(0); send(.8)
                    wait_for(lambda: statuses[-1].operations_completed == 4)
                    self.assertEqual(statuses[-1].state, 'open')
                    self.assertAlmostEqual(statuses[-1].width, .08)
                    self.assertFalse(statuses[-1].hardware_backend)
                finally:
                    if proc.poll() is None:
                        os.killpg(proc.pid, signal.SIGINT)
                        try: proc.wait(timeout=8)
                        except subprocess.TimeoutExpired:
                            os.killpg(proc.pid, signal.SIGKILL); proc.wait()
                    executor.remove_node(node); node.destroy_node(); executor.shutdown()

    def test_hardware_guard_prevents_connection(self):
        result = subprocess.run(['ros2', 'run', 'franka_teleop_controller', 'franka_hand_worker',
            '--ros-args', '-p', 'backend:=hardware', '-p', 'robot_ip:=192.0.2.1'],
            capture_output=True, text=True, timeout=10)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('operator_confirm', result.stderr)


if __name__ == '__main__': unittest.main()
