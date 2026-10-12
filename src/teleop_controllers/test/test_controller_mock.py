"""Actual controller process + production mapper, with no hardware connection."""
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
from rclpy.parameter import Parameter
from quest_teleop.pose_mapper import PoseMapper
from quest_teleop_interfaces.msg import CartesianTeleopCommand, ControllerStatus, QuestControllerState


class ControllerMockTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_mapper_consumer_fault_hold_and_explicit_reengagement(self):
        prefix = '/consumer_' + uuid.uuid4().hex
        frame = prefix.lstrip('/') + '/panda_link0'
        overrides = {'controller_state_topic': prefix + '/raw', 'base_frame': frame,
                     'measured_pose_topic': prefix + '/teleop_anchor_pose',
                     'command_topic': prefix + '/cartesian_teleop_command',
                     'monitor_topic': prefix + '/target_pose'}
        mapper = PoseMapper(parameter_overrides=[Parameter(k, value=v) for k, v in overrides.items()],
                            node_name='mapper_' + uuid.uuid4().hex)
        node = Node('test_' + uuid.uuid4().hex)
        executor = SingleThreadedExecutor()
        executor.add_node(mapper); executor.add_node(node)
        statuses = []
        commands = []
        node.create_subscription(ControllerStatus, prefix + '/controller_status', statuses.append, 100)
        node.create_subscription(CartesianTeleopCommand, overrides['command_topic'], commands.append, 100)
        publisher = node.create_publisher(QuestControllerState, overrides['controller_state_topic'], 10)
        sequence = 0
        generation = 1
        with tempfile.TemporaryDirectory() as directory:
            log_path = Path(directory) / 'controller.log'
            with log_path.open('w') as log:
                proc = subprocess.Popen(['ros2', 'run', 'teleop_controllers',
                    'cartesian_impedance_controller', '--ros-args', '-r', '__ns:=' + prefix,
                    '-p', 'backend:=mock', '-p', 'base_frame:=' + frame,
                    '-p', 'controller_state_topic:=' + prefix + '/raw'],
                    stdout=log, stderr=subprocess.STDOUT, start_new_session=True)

                def wait_for(predicate, timeout=10):
                    deadline = time.monotonic() + timeout
                    while time.monotonic() < deadline:
                        executor.spin_once(timeout_sec=.005)
                        if predicate(): return
                        if proc.poll() is not None: break
                    self.fail('controller condition timed out:\n' + log_path.read_text())

                def send(trigger, x=0., tracked=True, duration=.06):
                    nonlocal sequence
                    end = time.monotonic() + duration
                    while time.monotonic() < end:
                        sequence += 1
                        s = QuestControllerState()
                        s.session_id = 'consumer-session'; s.controller_id = 'right'
                        s.sequence = sequence
                        s.acquisition_time_ns = s.openxr_location_time_ns = time.monotonic_ns()
                        s.source_time_pc_monotonic_ns = s.acquisition_time_ns
                        s.source_time_valid = s.clock_sync_valid = True
                        s.clock_uncertainty_ns = 1000000
                        s.reference_space = 'LOCAL'; s.reference_space_generation = generation
                        s.connection_valid = s.session_focused = s.pose_action_active = True
                        s.position_valid = s.orientation_valid = True
                        s.position_tracked = s.orientation_tracked = tracked
                        s.grip_pose.position.x = float(x); s.grip_pose.orientation.w = 1.
                        s.trigger_active = True; s.trigger_value = float(trigger)
                        s.linear_velocity_valid = s.angular_velocity_valid = True
                        publisher.publish(s)
                        until = time.monotonic() + .01
                        while time.monotonic() < until: executor.spin_once(timeout_sec=.002)

                try:
                    wait_for(lambda: bool(statuses) and mapper.measured is not None)
                    self.assertFalse(statuses[-1].hardware_backend)
                    send(.8)
                    self.assertFalse(statuses[-1].motion_enabled)
                    send(0.); send(.8)
                    wait_for(lambda: statuses[-1].motion_enabled)
                    send(.8, .02)
                    self.assertTrue(statuses[-1].motion_enabled)
                    self.assertTrue(any(abs(c.target_pose.position.x - .42) < 1e-6 for c in commands))
                    self.assertTrue(any(abs(s.nominal_torque[0]) > .1 for s in statuses))
                    send(.8, .02, tracked=False)
                    wait_for(lambda: not statuses[-1].motion_enabled)
                    send(.8, .02)
                    self.assertFalse(statuses[-1].motion_enabled)
                    send(0., .02); send(.8, .02)
                    wait_for(lambda: statuses[-1].motion_enabled)
                    generation += 1
                    send(.8, .02)
                    self.assertFalse(statuses[-1].motion_enabled)
                    send(0., .02); send(.8, .02)
                    wait_for(lambda: statuses[-1].motion_enabled)
                    # Stop the production mapper itself, rather than merely
                    # injecting its watchdog output; controller must time out.
                    executor.remove_node(mapper); mapper.destroy_node(); mapper = None
                    wait_for(lambda: statuses[-1].reason == 'stale' and not statuses[-1].motion_enabled)
                    self.assertEqual(list(statuses[-1].nominal_torque), [0.] * 7)
                    self.assertEqual(statuses[-1].command_queue_overflows, 0)
                finally:
                    if proc.poll() is None:
                        os.killpg(proc.pid, signal.SIGINT)
                        try: proc.wait(timeout=8)
                        except subprocess.TimeoutExpired:
                            os.killpg(proc.pid, signal.SIGKILL); proc.wait()
                    if mapper is not None:
                        executor.remove_node(mapper); mapper.destroy_node()
                    executor.remove_node(node); node.destroy_node(); executor.shutdown()

    def test_hardware_guard_fails_before_connecting(self):
        result = subprocess.run(['ros2', 'run', 'teleop_controllers',
            'cartesian_impedance_controller', '--ros-args', '-p', 'backend:=hardware',
            '-p', 'robot_ip:=192.0.2.1'], capture_output=True, text=True, timeout=10)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('operator_confirm', result.stderr)

    def test_early_mock_failure_keeps_original_error(self):
        # End control on its first failed solve while the ROS executor is still
        # starting. Repeat to exercise cleanup timing; never connect to FCI.
        from ament_index_python.packages import get_package_prefix
        executable = str(Path(get_package_prefix('teleop_controllers')) /
                         'lib/teleop_controllers/cartesian_impedance_controller')
        for attempt in range(5):
            with self.subTest(attempt=attempt):
                result = subprocess.run([executable, '--ros-args',
                    '-r', '__ns:=/startup_failure_' + uuid.uuid4().hex,
                    '-p', 'backend:=mock', '-p', 'cbf.table_height:=2.0',
                    '-p', 'cbf.maximum_failures:=1'],
                    capture_output=True, text=True, timeout=10)
                output = result.stdout + result.stderr
                self.assertEqual(result.returncode, 2, output)
                self.assertIn('Controller stopped: latched CBF stop: solver_failures', output)
                self.assertNotIn('failed to create guard condition', output)
                self.assertNotIn('terminate called', output)


if __name__ == '__main__':
    unittest.main()
