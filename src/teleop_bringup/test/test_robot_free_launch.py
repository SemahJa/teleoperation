"""Actual launch acceptance: independent left/right mapping and selected-topic bag."""

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

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
import yaml
from quest_teleop_interfaces.msg import CartesianTeleopCommand, QuestControllerState


class RobotFreeLaunchTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_two_arm_clutch_fault_isolation_and_custom_recording(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            prefix = "/launch_test_" + uuid.uuid4().hex
            commands = {
                hand: f"{prefix}/{hand}/cartesian_teleop_command"
                for hand in ("left", "right")
            }
            inputs = {hand: f"{prefix}/{hand}/raw" for hand in ("left", "right")}
            config = {
                "arms": {
                    hand: {
                        "namespace": f"{prefix}/{hand}",
                        "controller": hand,
                        "controller_state_topic": inputs[hand],
                        "translation_scale": 0.5 if hand == "left" else 1.0,
                    }
                    for hand in ("left", "right")
                }
            }
            arms_path = root / "arms.yaml"
            arms_path.write_text(yaml.safe_dump(config))
            log_path = root / "launch.log"
            bag_root = root / "bags"
            node = Node("test_" + uuid.uuid4().hex)
            executor = SingleThreadedExecutor()
            executor.add_node(node)
            received = {hand: [] for hand in ("left", "right")}
            publishers = {}
            for hand in ("left", "right"):
                publishers[hand] = node.create_publisher(
                    QuestControllerState, inputs[hand], 10
                )
                node.create_subscription(
                    CartesianTeleopCommand, commands[hand], received[hand].append, 50
                )
            log_stream = log_path.open("w")
            proc = subprocess.Popen(
                [
                    "ros2",
                    "launch",
                    "teleop_bringup",
                    "robot_free.launch.py",
                    f"arms_config:={arms_path}",
                    "mock_measured_pose:=true",
                    "recording_enabled:=true",
                    f"recording_output_root:={bag_root}",
                    "recording_topics:=" + json.dumps([commands["left"]]),
                ],
                stdout=log_stream,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            sequences = {"left": 0, "right": 0}

            def wait_for(predicate, timeout=12):
                deadline = time.monotonic() + timeout
                while time.monotonic() < deadline:
                    executor.spin_once(timeout_sec=0.02)
                    if predicate():
                        return
                    if proc.poll() is not None:
                        break
                self.fail(
                    "launch acceptance timed out:\n" + log_path.read_text()[-5000:]
                )

            def sample(hand, trigger, x=0.0, tracked=True, controller=None):
                sequences[hand] += 1
                msg = QuestControllerState()
                msg.session_id = "launch-test"
                msg.controller_id = controller or hand
                msg.sequence = sequences[hand]
                msg.acquisition_time_ns = time.monotonic_ns()
                msg.source_time_pc_monotonic_ns = msg.acquisition_time_ns
                msg.openxr_location_time_ns = msg.acquisition_time_ns
                msg.clock_sync_valid = msg.source_time_valid = True
                msg.connection_valid = msg.session_focused = msg.pose_action_active = (
                    True
                )
                msg.reference_space = "LOCAL"
                msg.reference_space_generation = 1
                msg.position_valid = msg.orientation_valid = True
                msg.position_tracked = msg.orientation_tracked = tracked
                msg.grip_pose.position.x = x
                msg.grip_pose.orientation.w = 1.0
                msg.trigger_active = msg.grip_active = True
                msg.trigger_value = float(trigger)
                publishers[hand].publish(msg)

            def send_pair(
                left_trigger, right_trigger, left_x=0.0, right_x=0.0, left_tracked=True
            ):
                for _ in range(5):
                    sample("left", left_trigger, left_x, left_tracked)
                    sample("right", right_trigger, right_x)
                    until = time.monotonic() + 0.015
                    while time.monotonic() < until:
                        executor.spin_once(timeout_sec=0.005)

            try:
                wait_for(lambda: all(received[hand] for hand in received))
                wait_for(
                    lambda: all(
                        publishers[hand].get_subscription_count() > 0
                        for hand in publishers
                    )
                )
                wait_for(lambda: "Subscribed to topic" in log_path.read_text())
                send_pair(0.0, 0.0)
                send_pair(0.8, 0.8)
                wait_for(
                    lambda: all(
                        any(msg.motion_enabled for msg in received[hand])
                        for hand in received
                    )
                )
                send_pair(0.8, 0.8, 0.02, 0.05)
                wait_for(
                    lambda: any(
                        m.motion_enabled and abs(m.target_pose.position.x - 0.41) < 1e-6
                        for m in received["left"]
                    )
                    and any(
                        m.motion_enabled and abs(m.target_pose.position.x - 0.45) < 1e-6
                        for m in received["right"]
                    )
                )
                self.assertEqual(
                    received["left"][-1].header.frame_id,
                    prefix.lstrip("/") + "/left/panda_link0",
                )
                self.assertEqual(
                    received["right"][-1].header.frame_id,
                    prefix.lstrip("/") + "/right/panda_link0",
                )
                # A left tracking fault leaves the right arm independently engaged.
                send_pair(0.8, 0.8, 0.02, 0.05, left_tracked=False)
                self.assertFalse(received["left"][-1].motion_enabled)
                self.assertTrue(received["right"][-1].motion_enabled)
                send_pair(0.8, 0.8, 0.02, 0.05)
                self.assertFalse(received["left"][-1].motion_enabled)
                send_pair(0.0, 0.8, 0.02, 0.05)
                send_pair(0.8, 0.8, 0.02, 0.05)
                self.assertTrue(received["left"][-1].motion_enabled)
                # Wrong controller IDs are ignored even on a hand-specific input topic.
                marker = len(received["left"])
                sample("left", 0.8, 99.0, controller="right")
                wrong_sequence = sequences["left"]
                end = time.monotonic() + 0.04
                while time.monotonic() < end:
                    executor.spin_once(timeout_sec=0.005)
                self.assertFalse(
                    any(m.sequence == wrong_sequence for m in received["left"][marker:])
                )
            finally:
                if proc.poll() is None:
                    os.killpg(proc.pid, signal.SIGINT)
                    try:
                        proc.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        os.killpg(proc.pid, signal.SIGKILL)
                        proc.wait()
                log_stream.close()
                executor.remove_node(node)
                node.destroy_node()
                executor.shutdown()
            databases = list(bag_root.rglob("*.db3"))
            self.assertEqual(len(databases), 1, log_path.read_text())
            with sqlite3.connect(databases[0]) as database:
                recorded_topics = [
                    row[0] for row in database.execute("SELECT name FROM topics")
                ]
                self.assertEqual(recorded_topics, [commands["left"]])
                self.assertGreater(
                    database.execute("SELECT COUNT(*) FROM messages").fetchone()[0], 0
                )
            self.assertEqual(len(list(bag_root.rglob("metadata.yaml"))), 1)


if __name__ == "__main__":
    unittest.main()
