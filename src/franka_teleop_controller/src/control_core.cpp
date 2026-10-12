#include "franka_teleop_controller/control_core.hpp"
#include <algorithm>
#include <cmath>

namespace franka_teleop_controller {
const char* reason_name(Reason reason) noexcept {
  switch (reason) {
    case Reason::startup: return "startup";
    case Reason::released: return "released";
    case Reason::active: return "active";
    case Reason::invalid_command: return "invalid_command";
    case Reason::stale: return "stale";
    case Reason::order: return "duplicate_or_out_of_order";
    case Reason::session_change: return "session_or_reference_change";
    case Reason::envelope: return "outer_envelope";
    case Reason::queue_overflow: return "command_queue_overflow";
    case Reason::invalid_state: return "invalid_robot_state";
    case Reason::shutdown: return "shutdown";
    case Reason::safety_filter: return "safety_filter";
    case Reason::homing: return "homing_owns_nominal_control";
  }
  return "unknown";
}

bool Parameters::valid() const noexcept {
  const double numbers[] = {lambda_translation, lambda_rotation, gain_translation,
      gain_rotation, chain_radius, torque_rate};
  for (double value : numbers) if (!std::isfinite(value) || value <= 0) return false;
  if (!envelope_center.allFinite() || !std::isfinite(tool_offset) || tool_offset < 0 ||
      !std::isfinite(envelope_margin) || envelope_margin < 0 ||
      torque_rate > 1000 || timeout_ns == 0 || uncertainty_ns == 0) return false;
  const Vector7 ceilings = (Vector7() << 87, 87, 87, 87, 12, 12, 12).finished();
  return torque_limits.allFinite() && (torque_limits.array() > 0).all() &&
         (torque_limits.array() <= ceilings.array()).all();
}

bool ControlCore::fresh(const Command& c, uint64_t now) const noexcept {
  return c.source_ns > 0 && c.receipt_ns > 0 && now >= c.receipt_ns &&
      now - c.receipt_ns <= p_.timeout_ns && c.uncertainty_ns <= p_.uncertainty_ns &&
      (c.source_ns <= now ? now - c.source_ns <= p_.timeout_ns :
                           c.source_ns - now <= p_.future_ns);
}

void ControlCore::fault(Reason reason) noexcept {
  enabled_ = false;
  release_seen_ = false;
  reason_ = reason;
}

void ControlCore::accept(const Command& c, uint64_t now) noexcept {
  if (!c.valid || c.session[0] == '\0') { fault(Reason::invalid_command); return; }
  if (!fresh(c, now)) { fault(Reason::stale); return; }
  if (!identity_ || c.session != session_ || c.generation != generation_) {
    fault(Reason::session_change);
    session_ = c.session;
    generation_ = c.generation;
    identity_ = true;
    ordered_ = false;
  }
  if (ordered_ && (c.sequence <= sequence_ || c.source_ns <= source_ns_)) {
    // Mapper watchdog repeats a valid release. It cannot refresh freshness or
    // authorize a new release; enabled/reordered commands always fault.
    if (!c.enabled && !last_.enabled && c.released == last_.released && c.sequence == sequence_ &&
        c.source_ns == source_ns_) return;
    fault(Reason::order);
    return;
  }
  ordered_ = true;
  sequence_ = c.sequence;
  source_ns_ = c.source_ns;
  last_ = c;
  if (!c.enabled) {
    enabled_ = false;
    if (c.released) { release_seen_ = true; reason_ = Reason::released; }
    else if (release_seen_) reason_ = Reason::released;
    return;
  }
  if (!c.position.allFinite() || !c.orientation.coeffs().allFinite() ||
      std::abs(c.orientation.norm() - 1) > 0.01 || !c.twist.allFinite() || c.released) {
    fault(Reason::invalid_command); return;
  }
  if ((c.position - p_.envelope_center).norm() >
      p_.chain_radius + p_.tool_offset + p_.envelope_margin) {
    fault(Reason::envelope); return;
  }
  if (!release_seen_) { enabled_ = false; return; }
  enabled_ = true;
  reason_ = Reason::active;
}

Output ControlCore::update(const ArmState& s, uint64_t now, double dt) noexcept {
  Output out;
  if (identity_ && !fresh(last_, now)) fault(Reason::stale);
  if (!s.position.allFinite() || !s.orientation.coeffs().allFinite() ||
      std::abs(s.orientation.norm() - 1) > 0.01 || !s.q.allFinite() || !s.dq.allFinite() ||
      !s.coriolis.allFinite() || !s.jacobian.allFinite() || !s.previous_torque.allFinite() ||
      !std::isfinite(dt) || dt < 0 || dt > 0.02) {
    fault(Reason::invalid_state);
    out.reason = reason_;
    return out;  // Adapter must terminate, never send this placeholder torque.
  }
  // Disarm immediately removes target error and feedforward. The final torque
  // approaches damping/coriolis through actuator slew bounds; stale targets
  // are never pursued during that transition.
  out.target_position = enabled_ ? last_.position : s.position;
  out.target_orientation = enabled_ ? last_.orientation : s.orientation;
  out.desired_twist = enabled_ ? last_.twist : Vector6::Zero();
  Vector6 error = Vector6::Zero();
  error.head<3>() = s.position - out.target_position;
  Eigen::Quaterniond rotation(out.target_orientation.conjugate() * s.orientation);
  if (rotation.w() < 0) rotation.coeffs() *= -1;
  const Eigen::AngleAxisd angle(rotation);
  error.tail<3>() = out.target_orientation * (angle.axis() * angle.angle());
  Vector6 lambda, gain;
  lambda << p_.lambda_translation, p_.lambda_translation, p_.lambda_translation,
      p_.lambda_rotation, p_.lambda_rotation, p_.lambda_rotation;
  gain << p_.gain_translation, p_.gain_translation, p_.gain_translation,
      p_.gain_rotation, p_.gain_rotation, p_.gain_rotation;
  const Vector6 r = s.jacobian * s.dq - out.desired_twist + lambda.cwiseProduct(error);
  out.nominal = -s.jacobian.transpose() * gain.cwiseProduct(r) + s.coriolis;
  out.motion_enabled = enabled_;
  out.reason = reason_;
  out.source_ns = last_.source_ns;
  out.sequence = last_.sequence;
  return bound_nominal(s, out, dt);
}

Output ControlCore::bound_nominal(const ArmState& s, Output out, double dt) noexcept {
  if (!std::isfinite(dt) || dt < 0 || dt > .02 || !s.previous_torque.allFinite()) {
    fault(Reason::invalid_state); out.reason = reason_; out.motion_enabled = false; return out;
  }
  if (!out.nominal.allFinite()) { fault(Reason::invalid_state); out.reason = reason_; out.motion_enabled = false; return out; }
  const double slew = p_.torque_rate * std::min(dt, 0.001);
  for (int j = 0; j < 7; ++j) {
    const double low = std::max(-p_.torque_limits[j], s.previous_torque[j] - slew);
    const double high = std::min(p_.torque_limits[j], s.previous_torque[j] + slew);
    if (low > high) { fault(Reason::invalid_state); out.reason = reason_; out.motion_enabled = false; return out; }
    out.torque[j] = std::clamp(out.nominal[j], low, high);
    out.rate_limited |= std::abs(out.nominal[j] - s.previous_torque[j]) > slew;
    out.magnitude_limited |= std::abs(out.nominal[j]) > p_.torque_limits[j];
  }
  return out;
}
}  // namespace franka_teleop_controller
