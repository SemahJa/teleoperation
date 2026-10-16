#pragma once

#include <algorithm>
#include <limits>
#include <quest_teleop_interfaces/msg/tracking_sample.hpp>

#include "teleop_controllers/cartesian_impedance_controller.hpp"
#include "teleop_controllers/tracking.hpp"

namespace teleop_controllers {
// Non-realtime conversion; all ROS strings/messages are constructed here.
inline quest_teleop_interfaces::msg::TrackingSample tracking_message(
    const TrackingSnapshot& s, const std::string& frame, const builtin_interfaces::msg::Time& stamp,
    const Vector7& limits, double maximum_rate, bool cbf_enabled, uint64_t drops,
    const CartesianImpedanceParameters& impedance) {
  quest_teleop_interfaces::msg::TrackingSample msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame;
  msg.control_cycle = s.cycle;
  msg.state_time_pc_monotonic_ns = s.state_ns;
  msg.control_time_pc_monotonic_ns = s.control_ns;
  msg.command_issue_time_pc_monotonic_ns = s.issued_ns;
  msg.command_source_time_pc_monotonic_ns = s.requested.source_ns;
  msg.command_receipt_time_pc_monotonic_ns = s.requested.receipt_ns;
  msg.openxr_location_time_ns = s.requested.location_ns;
  msg.sequence = s.requested.sequence;
  msg.reference_space_generation = s.requested.generation;
  msg.session_id = s.requested.session.data();
  msg.reason = reason_name(s.control.reason);
  msg.hardware_backend = s.hardware;
  msg.motion_enabled = s.control.motion_enabled;
  msg.requested_pose_valid = s.control.motion_enabled;
  msg.requested_twist_valid = msg.requested_pose_valid && s.requested.requested_twist_valid;
  msg.cartesian_reference_valid = s.control.motion_enabled && !s.homing.active;
  msg.homing_active = s.homing.active;
  msg.period_seconds = s.period;
  msg.lambda_translation = impedance.lambda_translation;
  msg.lambda_rotation = impedance.lambda_rotation;
  msg.gain_translation = impedance.gain_translation;
  msg.gain_rotation = impedance.gain_rotation;
  msg.command_age_ms =
      s.requested.source_ns
          ? (static_cast<double>(s.control_ns) - static_cast<double>(s.requested.source_ns)) / 1e6
          : std::numeric_limits<double>::quiet_NaN();
  const auto pose = [](auto& output, const auto& p, const auto& q) {
    output.position.x = p.x();
    output.position.y = p.y();
    output.position.z = p.z();
    output.orientation.x = q.x();
    output.orientation.y = q.y();
    output.orientation.z = q.z();
    output.orientation.w = q.w();
  };
  const auto twist = [](auto& output, const Vector6& value) {
    output.linear.x = value[0];
    output.linear.y = value[1];
    output.linear.z = value[2];
    output.angular.x = value[3];
    output.angular.y = value[4];
    output.angular.z = value[5];
  };
  pose(msg.requested_pose, s.requested.position, s.requested.orientation);
  pose(msg.reference_pose, s.control.target_position, s.control.target_orientation);
  pose(msg.measured_pose, s.state.position, s.state.orientation);
  twist(msg.requested_twist, s.requested.requested_twist);
  twist(msg.reference_twist, s.control.desired_twist);
  const Vector6 measured_twist = s.state.jacobian * s.state.dq;
  twist(msg.measured_twist, measured_twist);
  const Eigen::Vector3d position = s.state.position - s.control.target_position;
  const Eigen::Vector3d rotation =
      rotation_error(s.state.orientation, s.control.target_orientation);
  std::copy_n(position.data(), 3, msg.position_error.begin());
  std::copy_n(rotation.data(), 3, msg.rotation_error.begin());
  msg.position_error_norm = position.norm();
  msg.rotation_error_angle = rotation.norm();
  msg.linear_velocity_error_norm =
      (measured_twist.head<3>() - s.control.desired_twist.head<3>()).norm();
  msg.angular_velocity_error_norm =
      (measured_twist.tail<3>() - s.control.desired_twist.tail<3>()).norm();
  const auto copy = [](const Vector7& input, auto& output) {
    std::copy_n(input.data(), 7, output.begin());
  };
  copy(s.state.q, msg.joint_position);
  copy(s.state.dq, msg.joint_velocity);
  copy(s.homing.desired_q, msg.homing_joint_position);
  copy(s.homing.desired_dq, msg.homing_joint_velocity);
  copy(s.control.nominal, msg.nominal_torque);
  copy(s.control.torque, msg.commanded_torque);
  msg.robot_torque_valid =
      s.hardware && s.robot_desired.allFinite() && s.measured_torque.allFinite();
  msg.gravity_valid = s.hardware && s.gravity.allFinite();
  copy(s.robot_desired, msg.robot_desired_torque);
  copy(s.measured_torque, msg.measured_torque);
  copy(s.gravity, msg.gravity_torque);
  msg.previous_command_valid = s.previous_cycle != 0;
  msg.previous_control_cycle = s.previous_cycle;
  msg.previous_command_issue_time_pc_monotonic_ns = s.previous_issued_ns;
  copy(s.previous_nominal, msg.previous_nominal_torque);
  copy(s.previous_commanded, msg.previous_commanded_torque);
  copy(limits, msg.torque_limits);
  msg.maximum_torque_rate = maximum_rate;
  msg.torque_rate_limited = s.control.rate_limited;
  msg.torque_magnitude_limited = s.control.magnitude_limited;
  msg.cbf_enabled = cbf_enabled;
  msg.safety_fallback = s.fallback;
  msg.solver_succeeded = s.solver_succeeded;
  msg.normalized_torque_intervention =
      ((s.control.torque - s.control.nominal).array() / limits.array()).matrix().norm();
  msg.monitor_queue_drops = drops;
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const auto invalid_pose = [&](auto& p) {
    p.position.x = p.position.y = p.position.z = nan;
    p.orientation.x = p.orientation.y = p.orientation.z = p.orientation.w = nan;
  };
  const auto invalid_twist = [&](auto& t) {
    t.linear.x = t.linear.y = t.linear.z = nan;
    t.angular.x = t.angular.y = t.angular.z = nan;
  };
  if (!msg.requested_pose_valid)
    invalid_pose(msg.requested_pose);
  if (!msg.requested_twist_valid)
    invalid_twist(msg.requested_twist);
  if (!msg.cartesian_reference_valid) {
    invalid_pose(msg.reference_pose);
    invalid_twist(msg.reference_twist);
    msg.position_error.fill(nan);
    msg.rotation_error.fill(nan);
    msg.position_error_norm = msg.rotation_error_angle = nan;
    msg.linear_velocity_error_norm = msg.angular_velocity_error_norm = nan;
  }
  if (!msg.robot_torque_valid) {
    msg.robot_desired_torque.fill(nan);
    msg.measured_torque.fill(nan);
  }
  if (!msg.gravity_valid)
    msg.gravity_torque.fill(nan);
  return msg;
}
}  // namespace teleop_controllers
