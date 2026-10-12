#!/usr/bin/env python3
"""Robot-independent Quest pose mapper; publishes commands but never connects to FCI."""

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from quest_teleop_interfaces.msg import CartesianTeleopCommand, QuestControllerState


def qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def qconj(q):
    return (-q[0], -q[1], -q[2], q[3])


def qrotate(q, v):
    r = qmul(qmul(q, (v[0], v[1], v[2], 0.0)), qconj(q))
    return r[:3]


def qnormalize(q):
    norm = math.sqrt(sum(x * x for x in q))
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError('invalid quaternion')
    return tuple(x / norm for x in q)


def qlog(q):
    """Shortest-arc quaternion logarithm as a rotation vector."""
    q = qnormalize(q)
    if q[3] < 0.0:
        q = tuple(-x for x in q)
    vector_norm = math.sqrt(sum(x * x for x in q[:3]))
    if vector_norm < 1e-8:
        return tuple(2.0 * x for x in q[:3])
    angle = 2.0 * math.atan2(vector_norm, max(-1.0, min(1.0, q[3])))
    return tuple(x * angle / vector_norm for x in q[:3])


def qexp(rotation_vector):
    angle = math.sqrt(sum(x * x for x in rotation_vector))
    half = 0.5 * angle
    if angle < 1e-8:
        return qnormalize((0.5 * rotation_vector[0], 0.5 * rotation_vector[1],
                           0.5 * rotation_vector[2], 1.0))
    scale = math.sin(half) / angle
    return (rotation_vector[0] * scale, rotation_vector[1] * scale,
            rotation_vector[2] * scale, math.cos(half))


def pose_values(pose):
    p = pose.position
    o = pose.orientation
    return (p.x, p.y, p.z), qnormalize((o.x, o.y, o.z, o.w))


def set_pose(msg, position, orientation):
    msg.position.x, msg.position.y, msg.position.z = position
    msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = orientation


def vec_add(a, b):
    return tuple(x + y for x, y in zip(a, b))


def vec_sub(a, b):
    return tuple(x - y for x, y in zip(a, b))


def vec_scale(a, s):
    return tuple(x * s for x in a)


class PoseMapper(Node):
    def __init__(self, parameter_overrides=None, node_name='quest_pose_mapper'):
        super().__init__(node_name, parameter_overrides=parameter_overrides)
        self.declare_parameter('controller_id', 'right')
        self.declare_parameter('controller_state_topic', '/quest_usb_bridge/controller_state')
        self.declare_parameter('measured_pose_topic', '/panda_1/teleop_anchor_pose')
        self.declare_parameter('command_topic', '/panda_1/cartesian_teleop_command')
        self.declare_parameter('monitor_topic', '/panda_1/target_pose')
        self.declare_parameter('base_frame', 'panda_1/panda_link0')
        self.declare_parameter('reference_to_robot_xyzw', [0.0, 0.0, 0.0, 1.0])
        self.declare_parameter('translation_scale', 1.0)
        self.declare_parameter('rotation_scale', 1.0)
        self.declare_parameter('feedforward_scale', 1.0)
        self.declare_parameter('twist_filter_cutoff_hz', 10.0)
        self.declare_parameter('command_timeout_ms', 100)
        self.declare_parameter('max_clock_uncertainty_ms', 10.0)
        self.declare_parameter('max_source_future_ms', 10.0)
        self.declare_parameter('min_derivative_dt_ms', 1.0)
        self.declare_parameter('max_derivative_dt_ms', 100.0)
        self.declare_parameter('trigger_engage', 0.75)
        self.declare_parameter('trigger_release', 0.25)
        self.state_topic = self.get_parameter('controller_state_topic').value
        self.controller_id = self.get_parameter('controller_id').value
        if self.controller_id not in ('left', 'right'):
            raise ValueError('controller_id must be left or right')
        self.pose_topic = self.get_parameter('measured_pose_topic').value
        self.command_topic = self.get_parameter('command_topic').value
        self.monitor_topic = self.get_parameter('monitor_topic').value
        self.base_frame = self.get_parameter('base_frame').value
        self.basis = qnormalize(tuple(self.get_parameter('reference_to_robot_xyzw').value))
        self.translation_scale = float(self.get_parameter('translation_scale').value)
        self.rotation_scale = float(self.get_parameter('rotation_scale').value)
        self.feedforward_scale = float(self.get_parameter('feedforward_scale').value)
        self.cutoff = float(self.get_parameter('twist_filter_cutoff_hz').value)
        self.timeout_ns = int(self.get_parameter('command_timeout_ms').value * 1e6)
        self.max_uncertainty_ns = int(self.get_parameter('max_clock_uncertainty_ms').value * 1e6)
        self.max_future_ns = int(self.get_parameter('max_source_future_ms').value * 1e6)
        self.min_dt = self.get_parameter('min_derivative_dt_ms').value * 1e6
        self.max_dt = self.get_parameter('max_derivative_dt_ms').value * 1e6
        self.engage = self.get_parameter('trigger_engage').value
        self.release = self.get_parameter('trigger_release').value
        if not (0.0 <= self.release < self.engage <= 1.0):
            raise ValueError('trigger thresholds must satisfy 0 <= release < engage <= 1')
        if not (0.0 <= self.feedforward_scale <= 1.0) or self.cutoff <= 0.0:
            raise ValueError('feedforward_scale must be in [0,1] and cutoff must be positive')
        if self.timeout_ns <= 0 or self.min_dt <= 0 or self.max_dt < self.min_dt:
            raise ValueError('invalid freshness or derivative interval limits')
        self.pub = self.create_publisher(CartesianTeleopCommand, self.command_topic, 10)
        self.monitor_pub = self.create_publisher(PoseStamped, self.monitor_topic, 10)
        self.create_subscription(QuestControllerState, self.state_topic, self.on_state, 20)
        self.create_subscription(PoseStamped, self.pose_topic, self.on_measured_pose, 10)
        self.create_timer(0.02, self.watchdog)
        self.measured = None
        self.measured_received_ns = 0
        self.measured_stamp_ros_ns = 0
        self.last_receive_ns = 0
        self.last_state = None
        self.anchor_controller = None
        self.anchor_robot = None
        self.previous_sample = None
        self.filtered_twist = [0.0] * 6
        self.previous_filter_time = None
        self.enabled = False
        self.trigger_latched = False
        self.need_release = True
        self.session = ''
        self.reference_generation = None
        self.last_sequence = None
        self.last_source_ns = None
        self.get_logger().info('Quest mapper active; hardware ownership belongs to the separately launched controller')

    def reset_motion(self):
        self.enabled = False
        self.trigger_latched = False
        self.anchor_controller = None
        self.anchor_robot = None
        self.previous_sample = None
        self.filtered_twist = [0.0] * 6
        self.previous_filter_time = None

    def on_measured_pose(self, msg):
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        now_ros_ns = self.get_clock().now().nanoseconds
        if (msg.header.frame_id != self.base_frame or stamp_ns <= 0 or
                now_ros_ns < stamp_ns or now_ros_ns - stamp_ns > self.timeout_ns):
            self.measured = None
            self.measured_received_ns = 0
            return
        try:
            self.measured = pose_values(msg.pose)
            self.measured_received_ns = time.monotonic_ns()
            self.measured_stamp_ros_ns = stamp_ns
        except ValueError:
            self.measured = None
            self.measured_received_ns = 0

    def measured_pose_fresh(self):
        if self.measured is None or self.measured_received_ns == 0:
            return False
        now_ros_ns = self.get_clock().now().nanoseconds
        now_mono_ns = time.monotonic_ns()
        return (now_mono_ns >= self.measured_received_ns and
                now_mono_ns - self.measured_received_ns <= self.timeout_ns and
                now_ros_ns >= self.measured_stamp_ros_ns and
                now_ros_ns - self.measured_stamp_ros_ns <= self.timeout_ns)

    def copy_source_metadata(self, cmd, state):
        cmd.session_id = state.session_id
        cmd.sequence = state.sequence
        cmd.source_acquisition_time_ns = state.acquisition_time_ns
        cmd.openxr_location_time_ns = state.openxr_location_time_ns
        cmd.source_time_valid = state.source_time_valid
        cmd.source_time_pc_monotonic_ns = state.source_time_pc_monotonic_ns
        cmd.clock_uncertainty_ns = state.clock_uncertainty_ns
        cmd.clock_sync_valid = state.clock_sync_valid
        cmd.reference_space_generation = state.reference_space_generation

    def publish_disarmed(self, state=None, input_valid=False, trigger_released=False):
        cmd = CartesianTeleopCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = self.base_frame
        cmd.motion_enabled = False
        cmd.input_valid = input_valid
        cmd.trigger_released = trigger_released
        if state is not None:
            self.copy_source_metadata(cmd, state)
        self.pub.publish(cmd)

    def on_state(self, s):
        if s.controller_id != self.controller_id:
            return
        now_ns = time.monotonic_ns()
        self.last_receive_ns = now_ns
        if not s.connection_valid or not s.session_focused or not s.pose_action_active:
            self.reset_motion()
            self.need_release = True
            self.last_state = s
            self.publish_disarmed(s)
            return
        if self.session and s.session_id != self.session:
            self.reset_motion()
            self.need_release = True
            self.last_sequence = None
            self.last_source_ns = None
        if self.reference_generation is not None and s.reference_space_generation != self.reference_generation:
            self.reset_motion()
            self.need_release = True
        if (self.last_sequence is not None and s.sequence <= self.last_sequence) or \
                (self.last_source_ns is not None and s.source_time_pc_monotonic_ns <= self.last_source_ns):
            self.reset_motion()
            self.need_release = True
            self.publish_disarmed(s)
            return
        self.session = s.session_id
        self.reference_generation = s.reference_space_generation
        self.last_sequence = s.sequence
        self.last_source_ns = s.source_time_pc_monotonic_ns
        self.last_state = s
        valid = (s.reference_space == 'LOCAL' and s.trigger_active and
                 s.position_valid and s.orientation_valid and
                 s.position_tracked and s.orientation_tracked and s.clock_sync_valid and
                 s.clock_uncertainty_ns <= self.max_uncertainty_ns and s.source_time_valid and
                 s.source_time_pc_monotonic_ns > 0 and
                 ((now_ns >= s.source_time_pc_monotonic_ns and
                   now_ns - s.source_time_pc_monotonic_ns <= self.timeout_ns) or
                  (s.source_time_pc_monotonic_ns > now_ns and
                   s.source_time_pc_monotonic_ns - now_ns <= self.max_future_ns)))
        if not valid:
            self.reset_motion()
            self.need_release = True
            self.publish_disarmed(s)
            return
        if s.trigger_value <= self.release:
            self.reset_motion()
            self.need_release = False
            self.publish_disarmed(s, input_valid=True, trigger_released=True)
            return
        if not self.trigger_latched:
            if self.need_release or s.trigger_value < self.engage:
                self.publish_disarmed(s, input_valid=True)
                return
            self.trigger_latched = True
        if not self.measured_pose_fresh():
            self.reset_motion()
            self.need_release = True
            self.publish_disarmed(s)
            return
        try:
            ctrl_p, ctrl_q = pose_values(s.grip_pose)
        except ValueError:
            self.reset_motion()
            self.need_release = True
            self.publish_disarmed(s)
            return
        if not self.enabled:
            self.anchor_controller = (ctrl_p, ctrl_q)
            self.anchor_robot = self.measured
            self.previous_sample = (s.openxr_location_time_ns, ctrl_p, ctrl_q)
            self.filtered_twist = [0.0] * 6
            self.previous_filter_time = s.openxr_location_time_ns
            self.enabled = True
            self.publish_command(s, *self.measured, [0.0] * 6)
            return
        target_p, target_q = self.map_pose(ctrl_p, ctrl_q)
        twist = self.estimate_twist(s, ctrl_p, ctrl_q, target_p, target_q)
        if twist is None:
            self.reset_motion()
            self.need_release = True
            self.publish_disarmed(s)
            return
        cmd_twist = self.filter_twist(twist, s.openxr_location_time_ns)
        self.publish_command(s, target_p, target_q, cmd_twist)

    def map_pose(self, p, q):
        ap, aq = self.anchor_controller
        rp, rq = self.anchor_robot
        dp = qrotate(self.basis, vec_sub(p, ap))
        target_p = vec_add(rp, vec_scale(dp, self.translation_scale))
        relative = qmul(q, qconj(aq))
        mapped = qmul(qmul(self.basis, relative), qconj(self.basis))
        rotation_vector = vec_scale(qlog(mapped), self.rotation_scale)
        delta = qexp(rotation_vector)
        return target_p, qnormalize(qmul(delta, rq))

    def estimate_twist(self, s, cp, cq, target_p, target_q):
        old_t, old_p, old_q = self.previous_sample
        dt_ns = s.openxr_location_time_ns - old_t
        if dt_ns <= 0 or dt_ns < self.min_dt or dt_ns > self.max_dt:
            return None
        dt = dt_ns * 1e-9
        old_target_p, old_target_q = self.map_pose(old_p, old_q)
        pose_linear = vec_scale(vec_sub(target_p, old_target_p), 1.0 / dt)
        dq = qmul(target_q, qconj(old_target_q))
        pose_angular = vec_scale(qlog(dq), 1.0 / dt)
        native_l = qrotate(self.basis, (s.native_twist.linear.x, s.native_twist.linear.y,
                                         s.native_twist.linear.z))
        native_a = qrotate(self.basis, (s.native_twist.angular.x, s.native_twist.angular.y,
                                         s.native_twist.angular.z))
        native_l = vec_scale(native_l, self.translation_scale)
        native_a = vec_scale(native_a, self.rotation_scale)
        blend = self.feedforward_scale
        linear = (vec_add(vec_scale(pose_linear, 1.0 - blend), vec_scale(native_l, blend))
                  if s.linear_velocity_valid else pose_linear)
        angular = (vec_add(vec_scale(pose_angular, 1.0 - blend), vec_scale(native_a, blend))
                   if s.angular_velocity_valid else pose_angular)
        self.previous_sample = (s.openxr_location_time_ns, cp, cq)
        return list(linear + angular)

    def filter_twist(self, twist, timestamp):
        previous = self.previous_filter_time
        self.previous_filter_time = timestamp
        dt = 0.0 if previous is None else (timestamp - previous) * 1e-9
        if dt <= 0.0:
            self.filtered_twist = [0.0] * 6
        else:
            a = math.exp(-2.0 * math.pi * self.cutoff * dt)
            self.filtered_twist = [a * old + (1.0 - a) * new
                                   for old, new in zip(self.filtered_twist, twist)]
        return self.filtered_twist[:]

    def publish_command(self, s, p, q, twist):
        cmd = CartesianTeleopCommand()
        ros_now_ns = self.get_clock().now().nanoseconds
        source_age_ns = max(0, time.monotonic_ns() - s.source_time_pc_monotonic_ns)
        source_ros_ns = max(0, ros_now_ns - source_age_ns)
        cmd.header.stamp.sec = source_ros_ns // 1_000_000_000
        cmd.header.stamp.nanosec = source_ros_ns % 1_000_000_000
        cmd.header.frame_id = self.base_frame
        set_pose(cmd.target_pose, p, q)
        cmd.desired_twist.linear.x, cmd.desired_twist.linear.y, cmd.desired_twist.linear.z = twist[:3]
        cmd.desired_twist.angular.x, cmd.desired_twist.angular.y, cmd.desired_twist.angular.z = twist[3:]
        cmd.motion_enabled = True
        self.copy_source_metadata(cmd, s)
        cmd.input_valid = True
        self.pub.publish(cmd)
        monitor = PoseStamped()
        monitor.header = cmd.header
        monitor.pose = cmd.target_pose
        self.monitor_pub.publish(monitor)

    def watchdog(self):
        now_ns = time.monotonic_ns()
        if self.enabled and (not self.last_receive_ns or now_ns - self.last_receive_ns > self.timeout_ns):
            self.reset_motion()
            self.need_release = True
            self.publish_disarmed(self.last_state)
        elif not self.enabled:
            # Repeated release messages preserve permission only while the actual
            # source release remains fresh; faults never masquerade as releases.
            valid_input = (not self.need_release and self.last_state is not None and
                           now_ns - self.last_receive_ns <= self.timeout_ns)
            released = valid_input and self.last_state.trigger_value <= self.release
            self.publish_disarmed(self.last_state, input_valid=valid_input,
                                  trigger_released=released)


def main(args=None):
    rclpy.init(args=args)
    node = PoseMapper()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
