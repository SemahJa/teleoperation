"""Slow status publication must not block fresh anchors or input consumption."""

import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import uuid

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from quest_teleop.pose_mapper import PoseMapper
from quest_teleop_interfaces.msg import QuestControllerState, TrackingSample


def test_slow_status_publication_cannot_starve_anchors_or_input():
    library = Path(os.environ["TELEOP_TEST_SLOW_PUBLISH_LIBRARY"])
    assert library.is_file()
    rclpy.init()
    prefix = "/isolation_" + uuid.uuid4().hex
    frame = prefix[1:] + "/panda_link0"
    node = Node("observer_" + uuid.uuid4().hex)
    mapper = PoseMapper(
        parameter_overrides=[
            Parameter(k, value=v)
            for k, v in {
                "controller_state_topic": prefix + "/raw",
                "base_frame": frame,
                "measured_pose_topic": prefix + "/teleop_anchor_pose",
                "command_topic": prefix + "/cartesian_teleop_command",
            }.items()
        ],
        node_name="mapper_" + uuid.uuid4().hex,
    )
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(mapper)
    anchors, tracking = [], []
    node.create_subscription(
        PoseStamped,
        prefix + "/teleop_anchor_pose",
        anchors.append,
        qos_profile_sensor_data,
    )
    node.create_subscription(
        TrackingSample, prefix + "/tracking_samples", tracking.append, 2048
    )
    pub = node.create_publisher(QuestControllerState, prefix + "/raw", 10)
    sequence = 0

    def pump(duration, trigger=None):
        nonlocal sequence
        until, next_send = time.monotonic() + duration, 0
        while time.monotonic() < until:
            now = time.monotonic()
            if trigger is not None and now >= next_send:
                next_send = now + 0.01
                sequence += 1
                s = QuestControllerState()
                s.controller_id = "right"
                s.session_id = "isolation"
                s.sequence = sequence
                s.acquisition_time_ns = s.openxr_location_time_ns = time.monotonic_ns()
                s.source_time_pc_monotonic_ns = s.acquisition_time_ns
                s.source_time_valid = s.clock_sync_valid = True
                s.clock_uncertainty_ns = 1000000
                s.reference_space = "LOCAL"
                s.reference_space_generation = 1
                s.connection_valid = s.session_focused = s.pose_action_active = True
                s.position_valid = s.orientation_valid = True
                s.position_tracked = s.orientation_tracked = True
                s.grip_pose.orientation.w = 1.0
                s.trigger_active = True
                s.trigger_value = float(trigger)
                pub.publish(s)
            executor.spin_once(timeout_sec=0.001)

    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory) / "controller.log"
        with path.open("w") as log:
            proc = subprocess.Popen(
                [
                    "ros2",
                    "run",
                    "teleop_controllers",
                    "cartesian_impedance_controller",
                    "--ros-args",
                    "-r",
                    "__ns:=" + prefix,
                    "-p",
                    "backend:=mock",
                    "-p",
                    "cbf.enabled:=false",
                    "-p",
                    "base_frame:=" + frame,
                    "-p",
                    "controller_state_topic:=" + prefix + "/raw",
                ],
                env={**os.environ, "LD_PRELOAD": str(library)},
                stdout=log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            try:
                deadline = time.monotonic() + 10
                while time.monotonic() < deadline and not (
                    tracking and anchors and pub.get_subscription_count() >= 2
                ):
                    pump(0.02)
                assert tracking and anchors, path.read_text()
                pump(0.5, 0.0)
                tracking.clear()
                pump(1.0, 1.0)
                assert any(s.motion_enabled for s in tracking), path.read_text()
                # Require sustained engagement across several 400 ms stalls,
                # rather than one lucky command just before a slow publish.
                tracking.clear()
                pump(0.5, 1.0)
                assert tracking and all(
                    s.motion_enabled for s in tracking
                ), path.read_text()
                last_anchor = anchors[-1].header.stamp
                anchor_ns = last_anchor.sec * 1_000_000_000 + last_anchor.nanosec
                assert node.get_clock().now().nanoseconds - anchor_ns < 100_000_000
                pump(0.3, 0.0)
                tracking.clear()
                pump(0.15, 0.0)
                assert tracking and all(not s.motion_enabled for s in tracking)
                assert proc.poll() is None, path.read_text()
            finally:
                if proc.poll() is None:
                    os.killpg(proc.pid, signal.SIGINT)
                    try:
                        proc.wait(timeout=8)
                    except subprocess.TimeoutExpired:
                        os.killpg(proc.pid, signal.SIGKILL)
                        proc.wait()
                executor.shutdown()
                mapper.destroy_node()
                node.destroy_node()
                rclpy.shutdown()
            assert proc.returncode == 0, path.read_text()
