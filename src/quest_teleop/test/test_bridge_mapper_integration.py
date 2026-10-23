import json
import socket
import struct
import subprocess
import time
import unittest
import uuid

import rclpy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PoseStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from quest_teleop_interfaces.msg import CartesianTeleopCommand, QuestControllerState
from quest_teleop.pose_mapper import PoseMapper


class BridgeMapperIntegrationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.port = self.free_port()
        suffix = uuid.uuid4().hex
        prefix = f"/bridge_mapper_test/case_{suffix}"
        self.bridge_topic = f"{prefix}/quest_usb_bridge/controller_state"
        self.left_topic = f"{prefix}/quest_usb_bridge/left/controller_state"
        self.right_topic = f"{prefix}/quest_usb_bridge/right/controller_state"
        self.measured_topic = f"{prefix}/measured"
        self.command_topic = f"{prefix}/command"
        self.monitor_topic = f"{prefix}/monitor"
        self.mapper = PoseMapper(
            node_name=f"mapper_{suffix}",
            parameter_overrides=[
                Parameter("controller_state_topic", value=self.bridge_topic),
                Parameter("measured_pose_topic", value=self.measured_topic),
                Parameter("command_topic", value=self.command_topic),
                Parameter("monitor_topic", value=self.monitor_topic),
            ],
        )
        self.test_node = Node(f"test_{suffix}")
        self.measured_pub = self.test_node.create_publisher(
            PoseStamped, self.measured_topic, 10
        )
        self.commands = []
        self.states = []
        self.left_states = []
        self.right_states = []
        self.test_node.create_subscription(
            CartesianTeleopCommand, self.command_topic, self.commands.append, 50
        )
        self.test_node.create_subscription(
            QuestControllerState, self.bridge_topic, self.states.append, 50
        )
        self.test_node.create_subscription(
            QuestControllerState, self.left_topic, self.left_states.append, 50
        )
        self.test_node.create_subscription(
            QuestControllerState, self.right_topic, self.right_states.append, 50
        )
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.mapper)
        self.executor.add_node(self.test_node)
        binary = (
            f'{get_package_prefix("quest_teleop")}/lib/quest_teleop/quest_usb_bridge'
        )
        self.bridge = subprocess.Popen(
            [
                binary,
                "--ros-args",
                "-r",
                f"__ns:={prefix}",
                "-p",
                f"listen_port:={self.port}",
                "-p",
                "bind_address:=127.0.0.1",
                "-p",
                "client_receive_timeout_ms:=5000",
            ]
        )
        self.sock = None
        self.sequence = 0
        self.acquisition = time.monotonic_ns()
        self.location = 5_000_000_000
        self.session = "e2e-session"
        self.generation = 1
        self.wait_for(lambda: self.bridge.poll() is None)
        measured = PoseStamped()
        measured.header.frame_id = self.mapper.base_frame
        measured.header.stamp = self.test_node.get_clock().now().to_msg()
        measured.pose.orientation.w = 1.0
        self.measured_pub.publish(measured)
        self.wait_for(self.mapper.measured_pose_fresh)
        # TCP readiness is independent of DDS discovery. Do not drop the first
        # release sample by sending it before the bridge publisher is matched.
        self.wait_for(
            lambda: self.test_node.count_publishers(self.bridge_topic) > 0
            and self.test_node.count_publishers(self.left_topic) > 0
            and self.test_node.count_publishers(self.right_topic) > 0
        )

    def tearDown(self):
        if self.sock:
            self.sock.close()
        if self.bridge:
            self.bridge.terminate()
            try:
                self.bridge.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.bridge.kill()
                self.bridge.wait()
        self.executor.remove_node(self.mapper)
        self.executor.remove_node(self.test_node)
        self.mapper.destroy_node()
        self.test_node.destroy_node()
        self.executor.shutdown()

    @staticmethod
    def free_port():
        with socket.socket() as sock:
            sock.bind(("127.0.0.1", 0))
            return sock.getsockname()[1]

    def wait_for(self, predicate, timeout=3.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.02)
            if predicate():
                return True
        latest = self.states[-1] if self.states else None
        self.fail(
            f"timed out; mapper enabled={self.mapper.enabled} need_release={self.mapper.need_release}; "
            f"latest state={latest!r}; commands={len(self.commands)}"
        )

    def connect(self):
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            try:
                self.sock = socket.create_connection(
                    ("127.0.0.1", self.port), timeout=1
                )
                self.sock.settimeout(1)
                # Publisher discovery alone does not prove that the writer has
                # matched our readers. Invalid-clock warmup cannot arm or count
                # as trigger release. Wait for real per-hand/global receipt.
                acknowledged = set()
                while time.monotonic() < deadline:
                    warmup = self.packet(0.0, clock_valid=False)
                    acknowledged.add(int(warmup["sequence"]))
                    self.send(warmup)
                    until = time.monotonic() + 0.02
                    while time.monotonic() < until:
                        self.executor.spin_once(timeout_sec=0.002)
                    if any(m.sequence in acknowledged for m in self.states) and any(
                        m.sequence in acknowledged for m in self.right_states
                    ):
                        return
                self.fail("DDS telemetry handshake was not acknowledged")
            except OSError:
                time.sleep(0.02)
        self.fail("bridge did not accept TCP connection")

    def close_connection(self):
        if self.sock:
            self.sock.close()
            self.sock = None

    def packet(
        self,
        trigger,
        *,
        controller="right",
        session=None,
        sequence=None,
        clock_valid=True,
        tracked=True,
        generation=None,
        x=0.0,
        button=False,
        source_time_ns=None,
    ):
        self.sequence += 1
        self.location += 10_000_000
        self.acquisition = (
            max(self.acquisition + 1, time.monotonic_ns())
            if source_time_ns is None
            else source_time_ns
        )
        acquisition = self.acquisition
        if generation is None:
            generation = self.generation
        else:
            self.generation = generation
        return {
            "version": 1,
            "type": "controller_input",
            "session_id": session or self.session,
            "sequence": str(self.sequence if sequence is None else sequence),
            "acquisition_time_ns": str(acquisition),
            "openxr_location_time_ns": str(self.location),
            "clock_sync_valid": clock_valid,
            "clock_offset_pc_minus_quest_ns": "0",
            "clock_uncertainty_ns": "1000000",
            "clock_sync_age_ns": "0",
            "reference_space": "LOCAL",
            "reference_space_generation": str(generation),
            "controller_id": controller,
            "session_focused": True,
            "pose": {
                "pose_action_active": True,
                "position": [x, 0.0, 0.0],
                "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
                "position_valid": True,
                "orientation_valid": True,
                "position_tracked": tracked,
                "orientation_tracked": tracked,
            },
            "native_twist": {
                "linear": [0.1, 0.0, 0.0],
                "angular": [0.0, 0.0, 0.0],
                "linear_valid": True,
                "angular_valid": True,
            },
            "inputs": (
                {
                    "trigger": trigger,
                    "trigger_active": True,
                    "grip": 0.0,
                    "grip_active": True,
                    "button_a": button,
                    "button_a_active": True,
                    "button_b": False,
                    "button_b_active": True,
                }
                if controller == "right"
                else {
                    "trigger": trigger,
                    "trigger_active": True,
                    "grip": 0.0,
                    "grip_active": True,
                    "button_x": button,
                    "button_x_active": True,
                    "button_y": False,
                    "button_y_active": True,
                    "menu_button": False,
                    "menu_button_active": True,
                }
            ),
        }

    def send(self, packet):
        measured = PoseStamped()
        measured.header.frame_id = self.mapper.base_frame
        measured.header.stamp = self.test_node.get_clock().now().to_msg()
        measured.pose.orientation.w = 1.0
        self.measured_pub.publish(measured)
        payload = json.dumps(packet, separators=(",", ":")).encode()
        self.sock.sendall(struct.pack("!I", len(payload)) + payload)

    def start_and_arm(self):
        self.connect()
        release = self.packet(0.0)
        self.send(release)
        self.wait_for(
            lambda: self.mapper.last_sequence == int(release["sequence"])
            and not self.mapper.need_release
        )
        self.send(self.packet(0.8))
        self.wait_for(lambda: self.mapper.enabled)

    def assert_fault_needs_release(self, fault_packet):
        self.send(fault_packet)
        self.wait_for(lambda: not self.mapper.enabled)
        self.acquisition = time.monotonic_ns()
        held = self.packet(0.8)
        self.send(held)
        self.wait_for(
            lambda: any(
                c.sequence == int(held["sequence"]) and not c.motion_enabled
                for c in self.commands
            )
        )
        self.assertFalse(self.mapper.enabled)
        self.send(self.packet(0.0))
        self.send(self.packet(0.8))
        self.wait_for(lambda: self.mapper.enabled)

    def test_per_hand_buttons_preserve_pressed_and_active_separately(self):
        self.connect()
        right = self.packet(0.0, button=True)
        right["inputs"]["button_b_active"] = False
        self.send(right)
        self.wait_for(
            lambda: any(s.sequence == int(right["sequence"]) for s in self.right_states)
        )
        state = self.right_states[-1]
        self.assertEqual(
            [(b.name, b.pressed, b.active) for b in state.buttons],
            [("a", True, True), ("b", False, False)],
        )
        left = self.packet(0.0, controller="left")
        left["inputs"]["button_y"] = True
        self.send(left)
        self.wait_for(
            lambda: any(s.sequence == int(left["sequence"]) for s in self.left_states)
        )
        self.assertEqual(
            [(b.name, b.pressed, b.active) for b in self.left_states[-1].buttons],
            [("x", False, True), ("y", True, True), ("menu", False, True)],
        )
        self.close_connection()
        self.wait_for(lambda: any(not s.connection_valid for s in self.right_states))
        self.wait_for(lambda: any(not s.connection_valid for s in self.left_states))
        self.assertEqual(list(self.right_states[-1].buttons), [])
        self.assertEqual(list(self.left_states[-1].buttons), [])

    def test_joystick_click_is_forwarded_for_each_hand_and_requires_a_complete_pair(
        self,
    ):
        self.connect()
        for controller in ("right", "left"):
            packet = self.packet(0.0, controller=controller)
            packet["inputs"].update(thumbstick_click=True, thumbstick_click_active=True)
            self.send(packet)
            states = self.right_states if controller == "right" else self.left_states
            self.wait_for(
                lambda: any(s.sequence == int(packet["sequence"]) for s in states)
            )
            buttons = [(b.name, b.pressed, b.active) for b in states[-1].buttons]
            self.assertIn(("thumbstick", True, True), buttons)
            self.assertEqual(len(buttons), 3 if controller == "right" else 4)
        malformed = self.packet(0.0)
        malformed["inputs"]["thumbstick_click"] = False
        before = len(self.right_states)
        self.send(malformed)
        self.wait_for(
            lambda: any(not s.connection_valid for s in self.right_states[before:])
        )

    def test_reject_bad_decimal_strings_and_cross_hand_buttons(self):
        mutations = [
            ("negative sequence", lambda p: p.update(sequence="-1")),
            ("signed acquisition", lambda p: p.update(acquisition_time_ns="+123")),
            (
                "whitespace timestamp",
                lambda p: p.update(openxr_location_time_ns=" 123"),
            ),
            ("unsigned overflow", lambda p: p.update(sequence="18446744073709551616")),
            (
                "whitespace offset",
                lambda p: p.update(clock_offset_pc_minus_quest_ns=" 0"),
            ),
            (
                "cross-hand button",
                lambda p: p["inputs"].update(button_x=True, button_x_active=True),
            ),
            (
                "unexposed system button",
                lambda p: p["inputs"].update(system_button=False),
            ),
            ("missing button", lambda p: p["inputs"].pop("button_b_active")),
        ]
        for label, mutate in mutations:
            with self.subTest(label=label):
                self.connect()
                valid = self.packet(0.0)
                self.send(valid)
                self.wait_for(
                    lambda: any(
                        s.connection_valid and s.sequence == int(valid["sequence"])
                        for s in self.right_states
                    )
                )
                before = len(self.right_states)
                bad = self.packet(0.0)
                mutate(bad)
                self.send(bad)
                self.wait_for(
                    lambda: any(
                        not s.connection_valid for s in self.right_states[before:]
                    )
                )
                self.close_connection()

    def test_bridge_fault_reconnect_and_raw_per_controller_state(self):
        self.start_and_arm()
        self.send(self.packet(0.5, x=0.01, button=True))
        self.wait_for(
            lambda: any(
                c.motion_enabled and c.target_pose.position.x > 0.005
                for c in self.commands
            )
        )
        self.wait_for(
            lambda: any(
                s.controller_id == "right"
                and any(b.name == "a" and b.pressed and b.active for b in s.buttons)
                for s in self.right_states
            )
        )
        self.assertEqual([b.name for b in self.right_states[-1].buttons], ["a", "b"])
        self.assertFalse(hasattr(self.right_states[-1], "button_x"))

        # A valid left frame remains separately identifiable and does not drive the right mapper.
        before = len(self.right_states)
        self.send(self.packet(0.8, controller="left", button=True))
        self.wait_for(lambda: len(self.left_states) > 0)
        self.assertEqual(len(self.right_states), before)
        self.assertEqual(
            [b.name for b in self.left_states[-1].buttons], ["x", "y", "menu"]
        )
        self.assertTrue(self.left_states[-1].buttons[0].pressed)
        self.assertFalse(hasattr(self.left_states[-1], "button_a"))

        # Clock, tracking, and LOCAL reference changes each require an explicit fresh clutch.
        self.assert_fault_needs_release(self.packet(0.8, clock_valid=False))
        self.assert_fault_needs_release(self.packet(0.8, tracked=False))
        self.assert_fault_needs_release(self.packet(0.8, generation=2))

        # Delayed source time is delivered as invalid and disarms motion.
        time.sleep(0.15)
        self.send(self.packet(0.8, source_time_ns=self.acquisition + 1_000_000))
        self.wait_for(lambda: not self.mapper.enabled)
        self.acquisition = time.monotonic_ns()
        held = self.packet(0.8)
        self.send(held)
        self.wait_for(
            lambda: any(
                c.sequence == int(held["sequence"]) and not c.motion_enabled
                for c in self.commands
            )
        )
        self.assertFalse(
            self.mapper.enabled, "fresh packets must not re-arm after stale input"
        )

        # Require release, then re-engagement; malformed JSON must close TCP and invalidate both IDs.
        self.send(self.packet(0.0))
        self.send(self.packet(0.8))
        self.wait_for(lambda: self.mapper.enabled)
        self.sock.sendall(struct.pack("!I", 1) + b"{")
        self.wait_for(lambda: not self.mapper.enabled)
        self.wait_for(lambda: any(not s.connection_valid for s in self.states))
        self.close_connection()

        self.session = "reconnected-session"
        self.connect()
        self.send(self.packet(0.8, session=self.session))
        self.wait_for(lambda: not self.mapper.enabled)
        self.send(self.packet(0.0, session=self.session))
        self.send(self.packet(0.8, session=self.session))
        self.wait_for(lambda: self.mapper.enabled)

        # A duplicate sequence is a transport fault; reconnect-held trigger cannot resume.
        duplicate = self.packet(0.8, session=self.session, sequence=self.sequence)
        self.send(duplicate)
        self.wait_for(lambda: not self.mapper.enabled)
        self.close_connection()
        self.session = "third-session"
        self.connect()
        self.send(self.packet(0.8, session=self.session))
        self.wait_for(lambda: not self.mapper.enabled)
        out_of_order = self.packet(
            0.0, session=self.session, sequence=self.sequence - 1
        )
        self.send(out_of_order)
        self.wait_for(lambda: any(not s.connection_valid for s in self.states[-2:]))
        self.close_connection()

        self.session = "fourth-session"
        self.connect()
        self.send(self.packet(0.0, session=self.session))
        self.send(self.packet(0.8, session=self.session))
        self.wait_for(lambda: self.mapper.enabled)

        # Watchdog timeout on silence disarms; fresh held input alone cannot restore motion.
        self.mapper.last_receive_ns = time.monotonic_ns() - self.mapper.timeout_ns - 1
        self.mapper.watchdog()
        self.assertFalse(self.mapper.enabled)
        self.acquisition = time.monotonic_ns()
        held = self.packet(0.8, session=self.session)
        self.send(held)
        self.wait_for(
            lambda: any(
                c.sequence == int(held["sequence"]) and not c.motion_enabled
                for c in self.commands
            )
        )
        self.assertFalse(self.mapper.enabled)


if __name__ == "__main__":
    unittest.main()
