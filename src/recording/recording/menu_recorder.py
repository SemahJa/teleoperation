#!/usr/bin/env python3
"""Left Menu episode toggle, adapted from thesis_ws for ROS Humble rosbag2.

Recording runs in its own non-RT process. The topic selection is the current
configured list and active arm namespaces. Demonstration profiles retain late topics.
"""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
from datetime import datetime, timezone
import uuid
import sys
from recording.profiles import qos_overrides, selected_topics
from recording.storage_options import mcap_storage_options
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from quest_teleop_interfaces.msg import QuestControllerState
from quest_teleop.configuration import full_topic


class MenuGate:
    def __init__(self, debounce_ns=350000000):
        self.debounce_ns = debounce_ns
        self.identity = None
        self.sequence = self.source = self.last_toggle = 0
        self.release_seen = False
        self.pressed = False

    def reset(self):
        self.release_seen = self.pressed = False

    def watchdog(self, now):
        if self.source and now - self.source > 100000000:
            self.reset()

    def update(self, msg, now):
        if msg.controller_id != "left":
            return False
        buttons = [b for b in msg.buttons if b.name == "menu"]
        source = msg.source_time_pc_monotonic_ns
        valid = (
            len(buttons) == 1
            and buttons[0].active
            and msg.connection_valid
            and msg.session_focused
            and msg.clock_sync_valid
            and msg.source_time_valid
            and bool(msg.session_id)
            and msg.reference_space == "LOCAL"
            and 0 < source <= now + 10000000
            and now - source <= 100000000
            and 0 <= msg.clock_uncertainty_ns <= 10000000
        )
        if not valid:
            self.reset()
            return False
        identity = (msg.session_id, msg.reference_space_generation)
        changed = identity != self.identity
        if changed:
            self.identity = identity
            self.sequence = self.source = 0
            self.reset()
        if (
            msg.sequence <= self.sequence
            or source <= self.source
            or (self.source and source - self.source > 100000000)
        ):
            self.reset()
            # An invalid/order fault cannot itself establish physical release.
            if msg.sequence <= self.sequence or source <= self.source:
                return False
            self.sequence = msg.sequence
            self.source = source
            return False
        self.sequence = msg.sequence
        self.source = source
        pressed = buttons[0].pressed
        if not pressed:
            self.release_seen = True
            self.pressed = False
            return False
        rising = not self.pressed
        self.pressed = True
        if (
            not self.release_seen
            or not rising
            or source - self.last_toggle < self.debounce_ns
        ):
            return False
        self.last_toggle = source
        return True


class MenuRecorder(Node):
    def __init__(self, **kwargs):
        super().__init__("menu_recorder", **kwargs)
        self.declare_parameter(
            "controller_state_topic", "/quest_usb_bridge/left/controller_state"
        )
        self.declare_parameter(
            "topics_json", '["/quest_usb_bridge/left/controller_state"]'
        )
        self.declare_parameter("inactive_namespaces_json", "[]")
        self.declare_parameter("output_root", "/ws/log/quest_bags")
        self.declare_parameter("storage_id", "sqlite3")
        self.declare_parameter("menu_debounce_seconds", 0.35)
        self.declare_parameter("profile", "teleop")
        self.declare_parameter("record_late_topics", False)
        self.declare_parameter("max_cache_size", 104857600)
        self.declare_parameter("clock_provenance", "unverified")
        self.declare_parameter("mcap_compression", "None")
        self.declare_parameter("mcap_compression_level", "Fast")
        self.mcap_options = mcap_storage_options(
            self.get_parameter("mcap_compression").value,
            self.get_parameter("mcap_compression_level").value)
        self.declare_parameter("autostart", False)
        self.declare_parameter("menu_enabled", True)
        self.declare_parameter("mock_measured_pose", False)
        self.demo = self.get_parameter("profile").value == "demonstration"
        self.late = self.get_parameter("record_late_topics").value
        self.monitor = self.validation = self.validation_log = None
        self.manifest = {}
        self.last_flush = 0
        self.autostart_timer = None
        self.topics = json.loads(self.get_parameter("topics_json").value)
        self.inactive = json.loads(self.get_parameter("inactive_namespaces_json").value)
        if not isinstance(self.topics, list) or not self.topics:
            raise ValueError("topics_json needs a nonempty list")
        self.topics = list(dict.fromkeys(full_topic(topic) for topic in self.topics))
        if not isinstance(self.inactive, list):
            raise ValueError("inactive_namespaces_json needs a list")
        self.inactive = [full_topic(ns) for ns in self.inactive]
        self.root = Path(self.get_parameter("output_root").value).expanduser()
        if not self.root.is_absolute():
            raise ValueError("output_root must be absolute")
        self.storage = self.get_parameter("storage_id").value
        import rosbag2_py
        if self.storage not in ("sqlite3", "mcap") or self.storage not in rosbag2_py.get_registered_writers():
            raise ValueError(f"storage {self.storage} is unavailable; install the matching Humble storage plugin")
        debounce = self.get_parameter("menu_debounce_seconds").value
        if not math.isfinite(debounce) or debounce < 0:
            raise ValueError("invalid Menu debounce")
        self.gate = MenuGate(int(debounce * 1e9))
        self.process = self.log = None
        self.output = ""
        self.selected = []
        self.phase = "idle"
        self.message = ""
        self.events = self.create_publisher(
            String,
            "recording_status",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.create_subscription(
            QuestControllerState,
            self.get_parameter("controller_state_topic").value,
            self.on_input,
            20,
        )
        self.create_timer(0.05, self.tick)
        self.publish()
        if self.get_parameter("autostart").value:
            self.autostart_timer = self.create_timer(0.5, self.auto_start)

    def auto_start(self):
        self.autostart_timer.cancel()
        self.start()

    def flush_manifest(self):
        if not self.demo or not self.output: return
        from recording.episode_monitor import atomic_json
        if self.monitor: self.manifest.update(self.monitor.snapshot())
        self.manifest["phase"] = self.phase
        atomic_json(self.output + ".manifest.json", self.manifest)
        if Path(self.output).is_dir():
            atomic_json(Path(self.output) / "episode_manifest.json", self.manifest)

    def publish(self):
        if not self.context.ok():
            return
        msg = String()
        msg.data = json.dumps(
            dict(
                phase=self.phase,
                message=self.message,
                output=self.output,
                topics=self.selected,
                manifest=self.output + ".manifest.json" if self.demo and self.output else "",
            )
        )
        self.events.publish(msg)

    def on_input(self, msg):
        if self.get_parameter("menu_enabled").value and self.gate.update(msg, time.monotonic_ns()):
            if self.phase in ("stopping", "validating"):
                return  # No queued episode while the previous bag finalizes.
            if self.process is None:
                self.start()
            else:
                self.stop()

    def start(self):
        published = {t for t in self.topics if self.get_publishers_info_by_topic(t)}
        self.selected = selected_topics(self.topics, self.inactive, published, self.late)
        if not self.selected:
            self.phase = "idle"
            self.message = "no selected topics currently have publishers"
            self.publish()
            return
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        self.output = str(self.root / f"quest_{stamp}_{uuid.uuid4().hex[:8]}")
        try:
            self.root.mkdir(parents=True, exist_ok=True)
            self.log = Path(self.output + ".recorder.log").open("w")
            if self.demo:
                from recording.episode_monitor import EpisodeMonitor
                self.manifest = dict(schema_version=1,episode_id=Path(self.output).name,
                    status="recording",storage_id=self.storage,requested_topics=self.selected,
                    mock_measured_pose=self.get_parameter("mock_measured_pose").value,
                    available_topics_at_start=sorted(published),start_request_ros_ns=self.get_clock().now().nanoseconds,
                    start_request_monotonic_ns=time.monotonic_ns(),calibration_status="uncalibrated_extrinsics",
                    warnings=[],storage_drop_count=None)
                if self.storage == "mcap":
                    self.manifest["mcap_storage_options"] = dict(self.mcap_options)
                self.monitor = EpisodeMonitor(self,self.selected,self.output,self.get_parameter("clock_provenance").value)
                self.flush_manifest()
            command = [
                "ros2",
                "bag",
                "record",
                "--storage",
                self.storage,
                "--output",
                self.output,
                "--max-cache-size", str(self.get_parameter("max_cache_size").value),
            ]
            if self.demo:
                import yaml
                qos_path = self.output + ".qos.yaml"
                Path(qos_path).write_text(yaml.safe_dump(qos_overrides(self.selected)))
                command += ["--qos-profile-overrides-path", qos_path]
                if self.storage == "mcap":
                    options_path = self.output + ".storage.yaml"
                    Path(options_path).write_text(yaml.safe_dump(self.mcap_options))
                    command += ["--storage-config-file", options_path]
            command += [*self.selected
            ]
            self.process = subprocess.Popen(
                command,
                stdin=subprocess.DEVNULL,
                stdout=self.log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            self.phase = "recording"
            self.message = "recorder started; graph discovery remains enabled"
            if self.demo:
                self.manifest["recorder_command"]=command
                self.flush_manifest()
        except Exception as error:
            if self.log:
                self.log.close()
            self.process = self.log = None
            self.phase = "error"
            self.message = str(error)
            if self.demo:
                self.manifest.update(status="start_failed",error=str(error))
                if self.monitor:self.monitor.close();self.monitor=None
                self.flush_manifest()
        self.publish()

    def stop(self):
        if self.process is None:
            return
        try:
            os.killpg(self.process.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
        if self.demo:
            self.manifest["stop_request_monotonic_ns"] = time.monotonic_ns()
        self.phase = "stopping"
        self.message = "finalizing bag"
        self.publish()

    def finalize(self, normal, code):
        self.log.close()
        self.log = self.process = None
        if self.demo:
            self.manifest.update(status="bag_finalized" if normal else "incomplete",recorder_returncode=code)
            self.flush_manifest()
            if self.monitor:self.monitor.close();self.monitor=None
            self.validation_log=Path(self.output+".validation.log").open("w")
            self.validation=subprocess.Popen([sys.executable,"-m","recording.validate_episode",self.output],
                stdout=self.validation_log,stderr=subprocess.STDOUT)
            self.phase="validating"
        else:self.phase="idle" if normal else "error"
        self.publish()

    def tick(self):
        if self.validation is not None:
            code=self.validation.poll()
            if code is not None:
                self.validation_log.close();self.validation_log=self.validation=None
                self.phase="idle" if code==0 else "error"
                self.message="episode validation complete" if code==0 else "validation failed; raw episode preserved"
                self.publish()
        if self.demo and self.process is not None and time.monotonic()-self.last_flush>1:
            self.last_flush=time.monotonic();self.flush_manifest()
        self.gate.watchdog(time.monotonic_ns())
        if self.process is not None:
            code = self.process.poll()
            if code is not None:
                normal = (
                    self.phase == "stopping"
                    and code in (0, -signal.SIGINT)
                    and (Path(self.output) / "metadata.yaml").is_file()
                )
                self.phase = "idle" if normal else "error"
                self.message = (
                    "bag finalized"
                    if normal
                    else f"recorder exited with code {code}; see {self.output}.recorder.log"
                )
                self.finalize(normal,code)

    def close(self):
        if self.process is not None:
            self.close_process()
        if self.validation is not None:
            try:self.validation.wait(timeout=30)
            except subprocess.TimeoutExpired:
                # Keep the independent offline validator running; raw data and
                # the pending manifest are preserved if application shutdown wins.
                pass
            self.validation_log.close();self.validation_log=self.validation=None

    def close_process(self):
        self.stop()
        try:
            self.process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(self.process.pid, signal.SIGTERM)
            try:
                self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid, signal.SIGKILL)
                self.process.wait()
        normal = self.process.returncode in (0,-signal.SIGINT) and (Path(self.output)/"metadata.yaml").exists()
        self.finalize(normal,self.process.returncode)


def main(args=None):
    rclpy.init(args=args)
    node = MenuRecorder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
