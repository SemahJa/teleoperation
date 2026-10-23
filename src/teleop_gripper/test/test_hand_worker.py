"""Actual non-RT Hand worker with mock backend and guarded calibration service."""

import os
import re
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
from quest_teleop_interfaces.msg import (
    GripperStatus,
    QuestControllerState,
    QuestButtonState,
)
from quest_teleop_interfaces.srv import CalibrateGripper


class HandWorkerTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_startup_calibration_once_without_automatic_grasp(self):
        prefix = "/auto_hand_" + uuid.uuid4().hex
        node = Node("test_" + uuid.uuid4().hex)
        statuses = []
        node.create_subscription(
            GripperStatus,
            prefix + "/gripper_events",
            statuses.append,
            QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        with tempfile.TemporaryDirectory() as directory:
            with (Path(directory) / "worker.log").open("w") as log:
                proc = subprocess.Popen(
                    [
                        "ros2",
                        "run",
                        "teleop_gripper",
                        "franka_hand_worker",
                        "--ros-args",
                        "-r",
                        "__ns:=" + prefix,
                        "-p",
                        "backend:=mock",
                        "-p",
                        "calibrate_on_start:=true",
                    ],
                    stdout=log,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                )
                try:
                    deadline = time.monotonic() + 8
                    while time.monotonic() < deadline and not (
                        statuses and statuses[-1].calibrated
                    ):
                        rclpy.spin_once(node, timeout_sec=0.05)
                    self.assertTrue(statuses and statuses[-1].calibrated)
                    deadline = time.monotonic() + 0.3
                    while time.monotonic() < deadline:
                        rclpy.spin_once(node, timeout_sec=0.05)
                    self.assertFalse(statuses[-1].busy)
                    self.assertEqual(statuses[-1].operations_started, 1)
                    self.assertEqual(statuses[-1].operations_completed, 1)
                    self.assertEqual(statuses[-1].state, "unknown")
                finally:
                    if proc.poll() is None:
                        os.killpg(proc.pid, signal.SIGINT)
                    proc.wait(timeout=8)
        node.destroy_node()

    def test_hold_close_release_stop_joystick_open_and_disconnect(self):
        prefix = "/hand_" + uuid.uuid4().hex
        node = Node("test_" + uuid.uuid4().hex)
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        statuses = []
        qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(
            GripperStatus, prefix + "/gripper_events", statuses.append, qos
        )
        publisher = node.create_publisher(QuestControllerState, prefix + "/raw", 10)
        calibration = node.create_client(
            CalibrateGripper, prefix + "/calibrate_gripper"
        )
        sequence = 0
        with tempfile.TemporaryDirectory() as directory:
            log_path = Path(directory) / "hand.log"
            with log_path.open("w") as log:
                proc = subprocess.Popen(
                    [
                        "ros2",
                        "run",
                        "teleop_gripper",
                        "franka_hand_worker",
                        "--ros-args",
                        "-r",
                        "__ns:=" + prefix,
                        "-p",
                        "backend:=mock",
                        "-p",
                        "controller_state_topic:=" + prefix + "/raw",
                        "-p",
                        "grasp_width_m:=0.006",
                        "-p",
                        "epsilon_inner_m:=0.001",
                        "-p",
                        "grasp_force_n:=30.0",
                        "-p",
                        "epsilon_outer_m:=0.002",
                        "-p",
                        "mock_operation_duration_ms:=300.0",
                    ],
                    stdout=log,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                )

                def wait_for(predicate, timeout=10):
                    end = time.monotonic() + timeout
                    while time.monotonic() < end:
                        executor.spin_once(timeout_sec=0.005)
                        if predicate():
                            return
                        if proc.poll() is not None:
                            break
                    self.fail("Hand condition timed out:\n" + log_path.read_text())

                def send(
                    value,
                    tracked=True,
                    duration=0.08,
                    controller="right",
                    pose_tracked=True,
                    joystick=False,
                ):
                    nonlocal sequence
                    end = time.monotonic() + duration
                    while time.monotonic() < end:
                        sequence += 1
                        s = QuestControllerState()
                        s.controller_id = controller
                        s.session_id = "hand-session"
                        s.sequence = sequence
                        s.source_time_pc_monotonic_ns = time.monotonic_ns()
                        s.clock_sync_valid = s.source_time_valid = True
                        s.clock_uncertainty_ns = 1000000
                        s.reference_space = "LOCAL"
                        s.reference_space_generation = 1
                        s.connection_valid = s.session_focused = (
                            s.pose_action_active
                        ) = tracked
                        s.position_valid = s.orientation_valid = True
                        s.position_tracked = s.orientation_tracked = (
                            tracked and pose_tracked
                        )
                        s.grip_active = True
                        s.grip_value = float(value)
                        button = QuestButtonState()
                        button.name = "thumbstick"
                        button.active = True
                        button.pressed = joystick
                        s.buttons = [button]
                        publisher.publish(s)
                        until = time.monotonic() + 0.01
                        while time.monotonic() < until:
                            executor.spin_once(timeout_sec=0.002)

                def calibrate(empty, confirm):
                    request = CalibrateGripper.Request()
                    request.empty_hand_confirm = empty
                    request.operator_confirm = confirm
                    future = calibration.call_async(request)
                    wait_for(future.done)
                    return future.result()

                try:
                    wait_for(
                        lambda: statuses
                        and calibration.service_is_ready()
                        and publisher.get_subscription_count() > 0
                    )
                    self.assertEqual(statuses[-1].state, "unknown")
                    self.assertEqual(statuses[-1].operations_started, 0)
                    self.assertFalse(statuses[-1].calibrated)
                    send(0)
                    send(0.8)
                    self.assertEqual(statuses[-1].operations_started, 0)
                    self.assertFalse(calibrate(False, True).accepted)
                    self.assertFalse(calibrate(True, False).accepted)
                    self.assertTrue(calibrate(True, True).accepted)
                    self.assertFalse(calibrate(True, True).accepted)
                    wait_for(lambda: statuses[-1].calibrated and not statuses[-1].busy)
                    self.assertEqual(statuses[-1].state, "unknown")
                    send(0)
                    send(0.8, duration=0.25)
                    wait_for(
                        lambda: statuses[-1].busy and statuses[-1].operation == "close"
                    )
                    send(0, duration=0.02, controller="left")
                    self.assertTrue(
                        statuses[-1].busy
                    )  # Other hand cannot stop this worker.
                    send(0, duration=0.02)
                    wait_for(
                        lambda: not statuses[-1].busy
                        and statuses[-1].state == "stopped"
                    )
                    self.assertLess(statuses[-1].width, 0.08)
                    self.assertGreater(statuses[-1].width, 0.0)
                    send(0, duration=0.4)
                    send(0, duration=0.04, joystick=True)
                    wait_for(
                        lambda: statuses[-1].operation == "open"
                        or statuses[-1].state == "open"
                    )
                    send(0, duration=1.0)
                    wait_for(
                        lambda: statuses[-1].state == "open" and not statuses[-1].busy
                    )
                    self.assertAlmostEqual(statuses[-1].width, 0.08)
                    send(0.8, duration=0.25, pose_tracked=False)
                    self.assertTrue(statuses[-1].busy)
                    send(0, duration=0.02, pose_tracked=False)
                    wait_for(
                        lambda: statuses[-1].state == "stopped"
                        and not statuses[-1].busy
                    )
                    send(0, duration=0.4)
                    send(0.8, duration=0.25)
                    send(0.8, tracked=False, duration=0.03)
                    wait_for(
                        lambda: statuses[-1].state == "stopped"
                        and not statuses[-1].busy
                    )
                    self.assertLess(statuses[-1].width, 0.08)
                    count = statuses[-1].operations_started
                    send(0.8, duration=0.15)
                    self.assertEqual(statuses[-1].operations_started, count)
                    send(0)
                    send(0.8, duration=0.25)
                    wait_for(
                        lambda: statuses[-1].state == "stopped"
                        and not statuses[-1].busy
                    )
                    self.assertFalse(statuses[-1].hardware_backend)

                finally:
                    if proc.poll() is None:
                        os.killpg(proc.pid, signal.SIGINT)
                        try:
                            proc.wait(timeout=8)
                        except subprocess.TimeoutExpired:
                            os.killpg(proc.pid, signal.SIGKILL)
                            proc.wait()
                    executor.remove_node(node)
                    node.destroy_node()
                    executor.shutdown()

    def test_hardware_guard_prevents_connection(self):
        result = subprocess.run(
            [
                "ros2",
                "run",
                "teleop_gripper",
                "franka_hand_worker",
                "--ros-args",
                "-p",
                "backend:=hardware",
                "-p",
                "robot_ip:=192.0.2.1",
            ],
            capture_output=True,
            text=True,
            timeout=10,
        )
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("operator_confirm", result.stderr)

    def test_slow_reporting_cannot_delay_release_or_joystick_open(self):
        library = Path(os.environ["HAND_TEST_SLOW_PUBLISH_LIBRARY"])
        self.assertTrue(library.is_file())
        prefix = "/fast_hand_" + uuid.uuid4().hex
        node = Node("test_" + uuid.uuid4().hex)
        statuses = []
        node.create_subscription(
            GripperStatus, prefix + "/gripper_events", statuses.append, 10
        )
        pub = node.create_publisher(QuestControllerState, prefix + "/raw", 10)
        sequence = 0

        def drive(duration, grip=0.0, joystick=False):
            nonlocal sequence
            until, next_send = time.monotonic() + duration, 0
            while time.monotonic() < until:
                if time.monotonic() >= next_send:
                    next_send = time.monotonic() + 0.01
                    sequence += 1
                    s = QuestControllerState()
                    s.controller_id, s.session_id = "right", "slow-report"
                    s.sequence = sequence
                    s.source_time_pc_monotonic_ns = time.monotonic_ns()
                    s.reference_space = "LOCAL"
                    s.reference_space_generation = 1
                    s.source_time_valid = s.clock_sync_valid = True
                    s.connection_valid = s.session_focused = s.grip_active = True
                    s.grip_value = float(grip)
                    b = QuestButtonState()
                    b.name, b.active, b.pressed = "thumbstick", True, joystick
                    s.buttons = [b]
                    pub.publish(s)
                rclpy.spin_once(node, timeout_sec=0.001)

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "hand.log"
            with path.open("w") as log:
                proc = subprocess.Popen(
                    [
                        "ros2",
                        "run",
                        "teleop_gripper",
                        "franka_hand_worker",
                        "--ros-args",
                        "-r",
                        "__ns:=" + prefix,
                        "-p",
                        "backend:=mock",
                        "-p",
                        "calibrate_on_start:=true",
                        "-p",
                        "mock_operation_duration_ms:=10.0",
                        "-p",
                        "controller_state_topic:=" + prefix + "/raw",
                    ],
                    env={**os.environ, "LD_PRELOAD": str(library)},
                    stdout=log,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                )
                try:
                    deadline = time.monotonic() + 8
                    while time.monotonic() < deadline and not (
                        statuses
                        and statuses[-1].calibrated
                        and pub.get_subscription_count()
                    ):
                        drive(0.02)
                    self.assertTrue(
                        statuses and statuses[-1].calibrated, path.read_text()
                    )
                    drive(0.15)
                    drive(0.15, 1.0)
                    drive(0.02, 0.0, True)
                    drive(1.5)
                    self.assertEqual(statuses[-1].state, "open", path.read_text())
                    text = path.read_text()
                    self.assertIn("Hand request: operation=open", text)
                    result = re.search(
                        r"Hand result: operation=close .*stopped=1 width_m=([0-9.e+-]+)",
                        text,
                    )
                    self.assertIsNotNone(result, text)
                    # 150 ms at 20 mm/s should close about 3 mm, even though
                    # every status publish is deliberately delayed by 400 ms.
                    self.assertGreater(float(result.group(1)), 0.075, text)
                    stop = re.search(
                        r"Hand request: operation=stop .*input_age_ms=([0-9.]+)", text
                    )
                    self.assertIsNotNone(stop, text)
                    self.assertLess(float(stop.group(1)), 100, text)
                finally:
                    if proc.poll() is None:
                        os.killpg(proc.pid, signal.SIGINT)
                    proc.wait(timeout=8)
                    node.destroy_node()
                self.assertEqual(proc.returncode, 0, path.read_text())


if __name__ == "__main__":
    unittest.main()
