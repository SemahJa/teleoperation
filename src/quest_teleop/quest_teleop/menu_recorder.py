#!/usr/bin/env python3
"""Left Menu episode toggle, adapted from thesis_ws for ROS Humble rosbag2.

Recording runs in its own non-RT process. The topic selection is the current
publisher graph intersected with the configured list and active arm namespaces.
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
        if msg.controller_id != 'left':
            return False
        buttons = [b for b in msg.buttons if b.name == 'menu']
        source = msg.source_time_pc_monotonic_ns
        valid = (len(buttons) == 1 and buttons[0].active and msg.connection_valid and
                 msg.session_focused and msg.clock_sync_valid and msg.source_time_valid and
                 bool(msg.session_id) and msg.reference_space == 'LOCAL' and
                 0 < source <= now + 10000000 and now - source <= 100000000 and
                 0 <= msg.clock_uncertainty_ns <= 10000000)
        if not valid:
            self.reset(); return False
        identity = (msg.session_id, msg.reference_space_generation)
        changed = identity != self.identity
        if changed:
            self.identity = identity
            self.sequence = self.source = 0
            self.reset()
        if (msg.sequence <= self.sequence or source <= self.source or
                (self.source and source - self.source > 100000000)):
            self.reset()
            # An invalid/order fault cannot itself establish physical release.
            if msg.sequence <= self.sequence or source <= self.source:
                return False
            self.sequence = msg.sequence; self.source = source
            return False
        self.sequence = msg.sequence; self.source = source
        pressed = buttons[0].pressed
        if not pressed:
            self.release_seen = True; self.pressed = False; return False
        rising = not self.pressed
        self.pressed = True
        if not self.release_seen or not rising or source - self.last_toggle < self.debounce_ns:
            return False
        self.last_toggle = source
        return True


class MenuRecorder(Node):
    def __init__(self, **kwargs):
        super().__init__('menu_recorder', **kwargs)
        self.declare_parameter('controller_state_topic', '/quest_usb_bridge/left/controller_state')
        self.declare_parameter('topics_json', '["/quest_usb_bridge/left/controller_state"]')
        self.declare_parameter('inactive_namespaces_json', '[]')
        self.declare_parameter('output_root', '/ws/log/quest_bags')
        self.declare_parameter('storage_id', 'sqlite3')
        self.declare_parameter('menu_debounce_seconds', .35)
        self.topics = json.loads(self.get_parameter('topics_json').value)
        self.inactive = json.loads(self.get_parameter('inactive_namespaces_json').value)
        if not isinstance(self.topics, list) or not self.topics:
            raise ValueError('topics_json needs a nonempty list')
        self.topics = list(dict.fromkeys(full_topic(topic) for topic in self.topics))
        if not isinstance(self.inactive, list): raise ValueError('inactive_namespaces_json needs a list')
        self.inactive = [full_topic(ns) for ns in self.inactive]
        self.root = Path(self.get_parameter('output_root').value).expanduser()
        if not self.root.is_absolute(): raise ValueError('output_root must be absolute')
        self.storage = self.get_parameter('storage_id').value
        if self.storage != 'sqlite3': raise ValueError('this Humble environment provides sqlite3 recording')
        debounce = self.get_parameter('menu_debounce_seconds').value
        if not math.isfinite(debounce) or debounce < 0: raise ValueError('invalid Menu debounce')
        self.gate = MenuGate(int(debounce * 1e9))
        self.process = self.log = None
        self.output = ''; self.selected = []; self.phase = 'idle'; self.message = ''
        self.events = self.create_publisher(String, 'recording_status',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(QuestControllerState, self.get_parameter('controller_state_topic').value,
                                 self.on_input, 20)
        self.create_timer(.05, self.tick)
        self.publish()

    def publish(self):
        if not self.context.ok(): return
        msg = String()
        msg.data = json.dumps(dict(phase=self.phase, message=self.message,
                                  output=self.output, topics=self.selected))
        self.events.publish(msg)

    def on_input(self, msg):
        if self.gate.update(msg, time.monotonic_ns()):
            if self.phase == 'stopping':
                return  # No queued episode while the previous bag finalizes.
            if self.process is None: self.start()
            else: self.stop()

    def start(self):
        self.selected = [t for t in self.topics
            if not any(t == ns or t.startswith(ns + '/') for ns in self.inactive)
            and self.get_publishers_info_by_topic(t)]
        if not self.selected:
            self.phase = 'idle'; self.message = 'no selected topics currently have publishers'; self.publish(); return
        stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
        self.output = str(self.root / f'quest_{stamp}_{uuid.uuid4().hex[:8]}')
        try:
            self.root.mkdir(parents=True, exist_ok=True)
            self.log = Path(self.output + '.recorder.log').open('w')
            command = ['ros2', 'bag', 'record', '--storage', self.storage, '--output', self.output,
                       '--node-name', 'quest_episode_recorder_' + uuid.uuid4().hex[:8], *self.selected]
            self.process = subprocess.Popen(command, stdin=subprocess.DEVNULL,
                stdout=self.log, stderr=subprocess.STDOUT, start_new_session=True)
            self.phase = 'recording'; self.message = 'started by left Menu'
        except Exception as error:
            if self.log: self.log.close()
            self.process = self.log = None; self.phase = 'error'; self.message = str(error)
        self.publish()

    def stop(self):
        if self.process is None: return
        try: os.killpg(self.process.pid, signal.SIGINT)
        except ProcessLookupError: pass
        self.phase = 'stopping'; self.message = 'finalizing bag'; self.publish()

    def tick(self):
        self.gate.watchdog(time.monotonic_ns())
        if self.process is not None:
            code = self.process.poll()
            if code is not None:
                normal = (self.phase == 'stopping' and code in (0, -signal.SIGINT)
                          and (Path(self.output) / 'metadata.yaml').is_file())
                self.phase = 'idle' if normal else 'error'
                self.message = 'bag finalized' if normal else f'recorder exited with code {code}; see {self.output}.recorder.log'
                self.log.close(); self.log = self.process = None
                self.publish()

    def close(self):
        if self.process is None: return
        self.stop()
        try: self.process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(self.process.pid, signal.SIGTERM)
            try: self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid, signal.SIGKILL); self.process.wait()
        self.log.close(); self.log = self.process = None


def main(args=None):
    rclpy.init(args=args)
    node = MenuRecorder()
    try: rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException): pass
    finally:
        node.close(); node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__': main()
