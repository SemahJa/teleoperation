import math
import time
import unittest
import uuid

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from quest_teleop_interfaces.msg import CartesianTeleopCommand, QuestControllerState

from quest_teleop.pose_mapper import PoseMapper, qexp, qlog, qrotate


class CapturePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class PoseMapperTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = PoseMapper()
        self.commands = CapturePublisher()
        self.monitors = CapturePublisher()
        self.node.pub = self.commands
        self.node.monitor_pub = self.monitors
        measured = PoseStamped()
        measured.header.frame_id = self.node.base_frame
        measured.header.stamp = self.node.get_clock().now().to_msg()
        measured.pose.orientation.w = 1.0
        self.node.on_measured_pose(measured)
        self.sequence = 0
        self.location_ns = 1_000_000_000

    def tearDown(self):
        self.node.destroy_node()

    def state(self, trigger):
        self.sequence += 1
        self.location_ns += 10_000_000
        sample = QuestControllerState()
        sample.session_id = 'test-session'
        sample.sequence = self.sequence
        sample.acquisition_time_ns = self.sequence * 10_000_000
        sample.openxr_location_time_ns = self.location_ns
        sample.source_time_pc_monotonic_ns = time.monotonic_ns()
        sample.source_time_valid = True
        sample.clock_sync_valid = True
        sample.clock_uncertainty_ns = 1_000_000
        sample.reference_space = 'LOCAL'
        sample.reference_space_generation = 1
        sample.controller_id = 'right'
        sample.connection_valid = True
        sample.session_focused = True
        sample.pose_action_active = True
        sample.position_valid = True
        sample.orientation_valid = True
        sample.position_tracked = True
        sample.orientation_tracked = True
        sample.grip_pose.orientation.w = 1.0
        sample.trigger_value = trigger
        sample.trigger_active = True
        return sample

    def test_basis_rotation_maps_translation(self):
        half = math.pi / 4.0
        z_quarter_turn = (0.0, 0.0, math.sin(half), math.cos(half))
        mapped = qrotate(z_quarter_turn, (1.0, 0.0, 0.0))
        self.assertAlmostEqual(mapped[0], 0.0, places=6)
        self.assertAlmostEqual(mapped[1], 1.0, places=6)
        self.assertAlmostEqual(mapped[2], 0.0, places=6)

    def test_quaternion_log_exp_small_angle_and_shortest_arc(self):
        rotation = (1e-10, -2e-10, 3e-10)
        recovered = qlog(qexp(rotation))
        for actual, expected in zip(recovered, rotation):
            self.assertAlmostEqual(actual, expected, delta=1e-15)
        q = qexp((0.2, -0.1, 0.05))
        negated = tuple(-value for value in q)
        for actual, expected in zip(qlog(negated), qlog(q)):
            self.assertAlmostEqual(actual, expected, places=12)

    def test_trigger_hysteresis_and_fresh_release(self):
        self.node.on_state(self.state(0.0))
        self.assertFalse(self.node.enabled)
        self.assertFalse(self.node.need_release)

        self.node.on_state(self.state(0.5))
        self.assertFalse(self.node.enabled)
        self.node.on_state(self.state(0.8))
        self.assertTrue(self.node.enabled)

        self.node.on_state(self.state(0.5))
        self.assertTrue(self.node.enabled)
        self.node.on_state(self.state(0.2))
        self.assertFalse(self.node.enabled)
        self.assertFalse(self.node.need_release)

    def test_pose_twist_and_command_metadata_publish_atomically(self):
        self.node.on_state(self.state(0.0))
        self.node.on_state(self.state(0.8))
        first = self.commands.messages[-1]
        self.assertTrue(first.motion_enabled)
        self.assertEqual(first.target_pose.position.x, 0.0)
        self.assertEqual(first.desired_twist.linear.x, 0.0)

        moved = self.state(0.5)
        moved.grip_pose.position.x = 0.01
        self.node.on_state(moved)
        command = self.commands.messages[-1]
        self.assertTrue(command.motion_enabled)
        self.assertEqual(command.sequence, moved.sequence)
        self.assertEqual(command.session_id, moved.session_id)
        self.assertEqual(command.source_acquisition_time_ns, moved.acquisition_time_ns)
        self.assertEqual(command.source_time_pc_monotonic_ns, moved.source_time_pc_monotonic_ns)
        self.assertEqual(command.clock_uncertainty_ns, moved.clock_uncertainty_ns)
        self.assertEqual(command.reference_space_generation, moved.reference_space_generation)
        self.assertTrue(command.input_valid)
        self.assertFalse(command.trigger_released)
        self.assertAlmostEqual(command.target_pose.position.x, 0.01, places=6)
        self.assertGreater(command.desired_twist.linear.x, 0.0)
        self.assertEqual(self.monitors.messages[-1].pose, command.target_pose)

    def test_native_and_pose_twist_blend_with_component_fallback(self):
        self.node.feedforward_scale = 0.5
        self.node.on_state(self.state(0.0))
        self.node.on_state(self.state(0.8))
        moved = self.state(0.5)
        moved.grip_pose.position.x = 0.01
        moved.linear_velocity_valid = True
        moved.native_twist.linear.x = 0.2
        self.node.on_state(moved)
        # Pose derivative is 1 m/s; blended target is 0.6 m/s then 10 Hz filtered.
        expected = (1.0 - math.exp(-2.0 * math.pi * 10.0 * 0.01)) * 0.6
        self.assertAlmostEqual(self.commands.messages[-1].desired_twist.linear.x,
                               expected, places=5)

        fallback = self.state(0.5)
        fallback.grip_pose.position.x = 0.02
        fallback.linear_velocity_valid = False
        self.node.on_state(fallback)
        expected_fallback = (1.0 - math.exp(-2.0 * math.pi * 10.0 * 0.01)) * 1.0
        previous = (1.0 - math.exp(-2.0 * math.pi * 10.0 * 0.01)) * 0.6
        expected_fallback += math.exp(-2.0 * math.pi * 10.0 * 0.01) * previous
        self.assertAlmostEqual(self.commands.messages[-1].desired_twist.linear.x,
                               expected_fallback, places=5)

    def test_inactive_trigger_action_disarms_instead_of_looking_released(self):
        self.node.on_state(self.state(0.0))
        self.node.on_state(self.state(0.8))
        unavailable = self.state(0.8)
        unavailable.trigger_active = False
        self.node.on_state(unavailable)
        self.assertFalse(self.node.enabled)
        self.assertTrue(self.node.need_release)
        recovered = self.state(0.8)
        self.node.on_state(recovered)
        self.assertFalse(self.node.enabled)
        self.node.on_state(self.state(0.0))
        self.node.on_state(self.state(0.8))
        self.assertTrue(self.node.enabled)

    def test_invalid_clock_disarms(self):
        self.node.on_state(self.state(0.0))
        self.node.on_state(self.state(0.8))
        invalid = self.state(0.8)
        invalid.clock_sync_valid = False
        self.node.on_state(invalid)
        self.assertFalse(self.node.enabled)
        self.assertTrue(self.node.need_release)
        self.assertFalse(self.commands.messages[-1].motion_enabled)
        self.assertFalse(self.commands.messages[-1].input_valid)
        self.assertFalse(self.commands.messages[-1].trigger_released)

    def test_stale_source_and_tracking_loss_disarm(self):
        self.node.on_state(self.state(0.0))
        self.node.on_state(self.state(0.8))
        stale = self.state(0.8)
        now_ns = time.monotonic_ns()
        self.node.last_source_ns = now_ns - 300_000_000
        stale.source_time_pc_monotonic_ns = now_ns - 200_000_000
        self.node.on_state(stale)
        self.assertFalse(self.node.enabled)
        self.assertTrue(self.node.need_release)

        self.node.on_state(self.state(0.0))
        self.node.on_state(self.state(0.8))
        untracked = self.state(0.8)
        untracked.position_tracked = False
        self.node.on_state(untracked)
        self.assertFalse(self.node.enabled)
        self.assertTrue(self.node.need_release)

    def test_reference_change_requires_release_before_reengage(self):
        self.node.on_state(self.state(0.0))
        self.node.on_state(self.state(0.8))
        changed = self.state(0.8)
        changed.reference_space_generation += 1
        self.node.on_state(changed)
        self.assertFalse(self.node.enabled)
        self.assertTrue(self.node.need_release)
        released = self.state(0.0)
        released.reference_space_generation = changed.reference_space_generation
        self.node.on_state(released)
        engaged = self.state(0.8)
        engaged.reference_space_generation = changed.reference_space_generation
        self.node.on_state(engaged)
        self.assertTrue(self.node.enabled)

    def test_duplicate_sample_disarms(self):
        self.node.on_state(self.state(0.0))
        engaged = self.state(0.8)
        self.node.on_state(engaged)
        self.assertTrue(self.node.enabled)
        self.node.on_state(engaged)
        self.assertFalse(self.node.enabled)
        self.assertTrue(self.node.need_release)

    def test_watchdog_timeout_disarms(self):
        self.node.on_state(self.state(0.0))
        self.node.on_state(self.state(0.8))
        self.node.last_receive_ns = time.monotonic_ns() - self.node.timeout_ns - 1
        self.node.watchdog()
        self.assertFalse(self.node.enabled)
        self.assertTrue(self.node.need_release)

    def test_stale_measured_pose_requires_trigger_release_after_recovery(self):
        self.node.on_state(self.state(0.0))
        self.node.on_state(self.state(0.8))
        self.assertTrue(self.node.enabled)

        self.node.measured_received_ns = time.monotonic_ns() - self.node.timeout_ns - 1
        self.node.on_state(self.state(0.8))
        self.assertFalse(self.node.enabled)
        self.assertTrue(self.node.need_release)

        recovered = PoseStamped()
        recovered.header.frame_id = self.node.base_frame
        recovered.header.stamp = self.node.get_clock().now().to_msg()
        recovered.pose.orientation.w = 1.0
        self.node.on_measured_pose(recovered)
        self.node.on_state(self.state(0.8))
        self.assertFalse(self.node.enabled)
        self.node.on_state(self.state(0.0))
        self.node.on_state(self.state(0.8))
        self.assertTrue(self.node.enabled)

    def test_session_change_requires_release_before_reengagement(self):
        self.node.on_state(self.state(0.0))
        self.node.on_state(self.state(0.8))
        changed = self.state(0.8)
        changed.session_id = 'new-session'
        self.node.on_state(changed)
        self.assertFalse(self.node.enabled)
        self.assertTrue(self.node.need_release)
        released = self.state(0.0)
        released.session_id = 'new-session'
        self.node.on_state(released)
        engaged = self.state(0.8)
        engaged.session_id = 'new-session'
        self.node.on_state(engaged)
        self.assertTrue(self.node.enabled)

    def test_stale_or_disconnected_input_disarms(self):
        self.node.on_state(self.state(0.0))
        self.node.on_state(self.state(0.8))
        self.assertTrue(self.node.enabled)
        lost = self.state(0.8)
        lost.connection_valid = False
        self.node.on_state(lost)
        self.assertFalse(self.node.enabled)
        self.assertTrue(self.node.need_release)

    def test_left_samples_do_not_refresh_right_watchdog(self):
        self.node.last_receive_ns = 123
        left = self.state(0.0)
        left.controller_id = 'left'
        self.node.on_state(left)
        self.assertEqual(self.node.last_receive_ns, 123)

    def test_ros_topics_publish_atomic_commands_and_disarm_on_bad_clock(self):
        suffix = uuid.uuid4().hex
        base = f'/mapper_test/case_{suffix}'
        state_topic, measured_topic = f'{base}/state', f'{base}/measured'
        command_topic, monitor_topic = f'{base}/command', f'{base}/monitor'
        mapper = PoseMapper(node_name=f'pose_mapper_{suffix}', parameter_overrides=[
            Parameter('controller_state_topic', value=state_topic),
            Parameter('measured_pose_topic', value=measured_topic),
            Parameter('command_topic', value=command_topic),
            Parameter('monitor_topic', value=monitor_topic),
        ])
        test_node = Node(f'mapper_test_{suffix}')
        states = test_node.create_publisher(QuestControllerState, state_topic, 10)
        measured = test_node.create_publisher(PoseStamped, measured_topic, 10)
        commands, monitors = [], []
        test_node.create_subscription(CartesianTeleopCommand, command_topic, commands.append, 10)
        test_node.create_subscription(PoseStamped, monitor_topic, monitors.append, 10)
        executor = SingleThreadedExecutor()
        executor.add_node(mapper)
        executor.add_node(test_node)

        def spin_until(predicate, timeout=2.0):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.02)
                if predicate():
                    return True
            return predicate()

        try:
            self.assertTrue(spin_until(lambda: states.get_subscription_count() > 0 and
                                       measured.get_subscription_count() > 0))
            anchor = PoseStamped()
            anchor.header.frame_id = mapper.base_frame
            anchor.header.stamp = test_node.get_clock().now().to_msg()
            anchor.pose.orientation.w = 1.0
            measured.publish(anchor)
            self.assertTrue(spin_until(lambda: mapper.measured_pose_fresh()))

            seq = 0
            location = 2_000_000_000

            def send(trigger, *, clock_valid=True, x=0.0):
                nonlocal seq, location
                seq += 1
                location += 10_000_000
                message = QuestControllerState()
                message.session_id = 'ros-integration-session'
                message.sequence = seq
                message.acquisition_time_ns = seq
                message.openxr_location_time_ns = location
                message.source_time_pc_monotonic_ns = time.monotonic_ns()
                message.source_time_valid = True
                message.clock_sync_valid = clock_valid
                message.clock_uncertainty_ns = 1_000_000
                message.reference_space = 'LOCAL'
                message.reference_space_generation = 1
                message.controller_id = 'right'
                message.connection_valid = True
                message.session_focused = True
                message.pose_action_active = True
                message.position_valid = message.orientation_valid = True
                message.position_tracked = message.orientation_tracked = True
                message.grip_pose.position.x = x
                message.grip_pose.orientation.w = 1.0
                message.trigger_value = trigger
                message.trigger_active = True
                states.publish(message)
                return message

            send(0.0)
            self.assertTrue(spin_until(lambda: any(c.sequence == 1 and not c.motion_enabled
                                                   for c in commands)))
            send(0.8)
            self.assertTrue(spin_until(lambda: any(c.sequence == 2 and c.motion_enabled
                                                   for c in commands)))
            moved = send(0.5, x=0.01)
            self.assertTrue(spin_until(lambda: any(c.sequence == moved.sequence and c.motion_enabled
                                                   for c in commands)))
            command = next(c for c in reversed(commands) if c.sequence == moved.sequence)
            self.assertEqual(command.header.frame_id, mapper.base_frame)
            self.assertEqual(command.session_id, moved.session_id)
            self.assertAlmostEqual(command.target_pose.position.x, 0.01, places=6)
            self.assertTrue(spin_until(lambda: any(m.pose == command.target_pose for m in monitors)))

            invalid = send(0.8, clock_valid=False)
            self.assertTrue(spin_until(lambda: any(c.sequence == invalid.sequence and not c.motion_enabled
                                                   for c in commands)))
        finally:
            executor.remove_node(mapper)
            executor.remove_node(test_node)
            mapper.destroy_node()
            test_node.destroy_node()
            executor.shutdown()


if __name__ == '__main__':
    unittest.main()
