"""Independent X/A home gestures through two real mock controller processes."""
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
from quest_teleop_interfaces.msg import CartesianTeleopCommand, ControllerStatus, HomingStatus, QuestButtonState, QuestControllerState


class HomingMockTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls): rclpy.init()

    @classmethod
    def tearDownClass(cls): rclpy.shutdown()

    def test_independent_hold_release_cancellation_and_exclusive_nominal_owner(self):
        prefix = '/home_' + uuid.uuid4().hex
        node = Node('test_' + uuid.uuid4().hex)
        executor = SingleThreadedExecutor(); executor.add_node(node)
        sides = ('left', 'right')
        homes = {side: [] for side in sides}; controls = {side: [] for side in sides}
        buttons = {side: False for side in sides}; triggers = {side: 0. for side in sides}
        sequences = {side: 0 for side in sides}
        publishers = {}; command_publishers = {}
        for side in sides:
            namespace = prefix + '/' + side
            publishers[side] = node.create_publisher(QuestControllerState, namespace + '/raw', 10)
            command_publishers[side] = node.create_publisher(CartesianTeleopCommand, namespace + '/cartesian_teleop_command', 10)
            node.create_subscription(HomingStatus, namespace + '/homing_status', homes[side].append, 100)
            node.create_subscription(ControllerStatus, namespace + '/controller_status', controls[side].append, 100)
        with tempfile.TemporaryDirectory() as directory:
            streams = []; processes = []
            for side in sides:
                namespace = prefix + '/' + side
                stream = (Path(directory) / (side + '.log')).open('w'); streams.append(stream)
                processes.append(subprocess.Popen(['ros2', 'run', 'franka_teleop_controller',
                    'cartesian_impedance_controller', '--ros-args', '-r', '__ns:=' + namespace,
                    '-p', 'backend:=mock', '-p', 'controller_id:=' + side,
                    '-p', 'controller_state_topic:=' + namespace + '/raw',
                    '-p', 'base_frame:=' + namespace.lstrip('/') + '/panda_link0'],
                    stdout=stream, stderr=subprocess.STDOUT, start_new_session=True))

            def tick():
                for side in sides:
                    sequences[side] += 1
                    sample = QuestControllerState()
                    sample.controller_id = side; sample.session_id = side + '-session'
                    sample.sequence = sequences[side]
                    sample.source_time_pc_monotonic_ns = time.monotonic_ns()
                    sample.source_time_valid = sample.clock_sync_valid = True
                    sample.clock_uncertainty_ns = 1000000
                    sample.reference_space = 'LOCAL'; sample.reference_space_generation = 1
                    sample.connection_valid = sample.session_focused = sample.pose_action_active = True
                    sample.position_valid = sample.orientation_valid = True
                    sample.position_tracked = sample.orientation_tracked = True
                    sample.trigger_active = True; sample.trigger_value = triggers[side]
                    button = QuestButtonState(); button.name = 'x' if side == 'left' else 'a'
                    button.pressed = buttons[side]; button.active = True; sample.buttons = [button]
                    publishers[side].publish(sample)
                    command = CartesianTeleopCommand()
                    command.header.frame_id = prefix.lstrip('/') + '/' + side + '/panda_link0'
                    command.session_id = sample.session_id; command.sequence = sample.sequence
                    command.source_time_pc_monotonic_ns = sample.source_time_pc_monotonic_ns
                    command.clock_uncertainty_ns = sample.clock_uncertainty_ns
                    command.source_time_valid = command.clock_sync_valid = command.input_valid = True
                    command.reference_space_generation = 1
                    command.trigger_released = triggers[side] <= .25
                    command.motion_enabled = triggers[side] >= .75
                    command.target_pose.position.x = command.target_pose.position.z = .4
                    command.target_pose.orientation.w = 1.
                    command_publishers[side].publish(command)

            def drive(duration):
                end = time.monotonic() + duration
                while time.monotonic() < end:
                    tick()
                    until = time.monotonic() + .01
                    while time.monotonic() < until: executor.spin_once(timeout_sec=.002)

            def wait_for(predicate, timeout=8):
                end = time.monotonic() + timeout
                while time.monotonic() < end:
                    executor.spin_once(timeout_sec=.005)
                    if predicate(): return
                    if any(proc.poll() is not None for proc in processes): break
                logs = '\n'.join((Path(directory) / (side + '.log')).read_text() for side in sides)
                self.fail('homing process condition timed out:\n' + logs)

            try:
                wait_for(lambda: all(homes[s] and controls[s] and publishers[s].get_subscription_count()
                                     and command_publishers[s].get_subscription_count() for s in sides))
                buttons['left'] = True; drive(1.1)
                self.assertEqual(homes['left'][-1].request_id, 0, 'boot-held button cannot request home')
                buttons['left'] = False; drive(.05)
                buttons['left'] = True; drive(.2)
                buttons['left'] = False; drive(.05)
                self.assertEqual(homes['left'][-1].request_id, 0, 'short hold cannot request home')
                buttons['left'] = True; drive(1.1)
                self.assertEqual(homes['left'][-1].request_id, 0, 'holding does not request before release')
                buttons['left'] = False; drive(.4)
                self.assertEqual(homes['left'][-1].phase, 'succeeded')
                self.assertEqual(homes['left'][-1].request_id, 1)
                self.assertEqual(homes['right'][-1].request_id, 0, 'left X must not home the right arm')
                buttons['right'] = True; drive(1.1)
                self.assertEqual(homes['right'][-1].request_id, 0)
                buttons['right'] = False; drive(.05)
                self.assertEqual(homes['right'][-1].phase, 'homing')
                triggers['right'] = .8; drive(.9)
                end = time.monotonic() + 3
                while homes['right'][-1].phase == 'stopping' and time.monotonic() < end:
                    drive(.05)
                self.assertEqual(homes['right'][-1].phase, 'cancelled')
                self.assertFalse(controls['right'][-1].motion_enabled, 'held trigger cannot resume after cancellation')
                self.assertEqual(homes['left'][-1].request_id, 1)
                triggers['right'] = 0.; drive(.05)
                triggers['right'] = .8; drive(.05)
                self.assertTrue(controls['right'][-1].motion_enabled)
                # Match paired messages from the same control-cycle snapshot.
                for side in sides:
                    def stamp(msg): return (msg.header.stamp.sec, msg.header.stamp.nanosec)
                    by_stamp = {stamp(msg): msg for msg in controls[side]}
                    matched = [by_stamp[stamp(home)] for home in homes[side]
                               if home.active and stamp(home) in by_stamp]
                    self.assertTrue(matched)
                    self.assertTrue(all(not msg.motion_enabled for msg in matched))
                    self.assertTrue(all(not msg.hardware_backend for msg in controls[side]))
            finally:
                for proc in processes:
                    if proc.poll() is None: os.killpg(proc.pid, signal.SIGINT)
                for proc in processes:
                    try: proc.wait(timeout=8)
                    except subprocess.TimeoutExpired:
                        os.killpg(proc.pid, signal.SIGKILL); proc.wait()
                for stream in streams: stream.close()
                executor.remove_node(node); node.destroy_node(); executor.shutdown()


if __name__ == '__main__': unittest.main()
