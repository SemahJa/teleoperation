#include "teleop_controllers/controller_manager.hpp"

#include <algorithm>
#include <cmath>

namespace teleop_controllers {
const char* reason_name(Reason reason) noexcept {
  switch (reason) {
    case Reason::startup:
      return "startup";
    case Reason::released:
      return "released";
    case Reason::active:
      return "active";
    case Reason::invalid_command:
      return "invalid_command";
    case Reason::stale:
      return "stale";
    case Reason::order:
      return "duplicate_or_out_of_order";
    case Reason::session_change:
      return "session_or_reference_change";
    case Reason::envelope:
      return "outer_envelope";
    case Reason::queue_overflow:
      return "command_queue_overflow";
    case Reason::invalid_state:
      return "invalid_robot_state";
    case Reason::shutdown:
      return "shutdown";
    case Reason::safety_filter:
      return "safety_filter";
    case Reason::homing:
      return "homing_owns_nominal_control";
    case Reason::position_hold:
      return "position_hold";
  }
  return "unknown";
}

const char* state_fault_name(StateFault fault) noexcept {
  switch (fault) {
    case StateFault::none:
      return "none";
    case StateFault::nonfinite_state:
      return "nonfinite_robot_state";
    case StateFault::orientation:
      return "invalid_robot_orientation";
    case StateFault::period:
      return "invalid_callback_period";
    case StateFault::nonfinite_nominal:
      return "nonfinite_nominal_torque";
    case StateFault::actuator_interval:
      return "empty_actuator_interval";
    case StateFault::invalid_reference:
      return "invalid_controller_reference";
  }
  return "unknown";
}

bool ManagerParameters::valid() const noexcept {
  const double numbers[] = {chain_radius, torque_rate};
  for (double value : numbers)
    if (!std::isfinite(value) || value <= 0)
      return false;
  if (!envelope_center.allFinite() || !std::isfinite(tool_offset) || tool_offset < 0 ||
      !std::isfinite(envelope_margin) || envelope_margin < 0 || torque_rate > 1000 ||
      timeout_ns == 0 || uncertainty_ns == 0)
    return false;
  const Vector7 ceilings = (Vector7() << 87, 87, 87, 87, 12, 12, 12).finished();
  return torque_limits.allFinite() && (torque_limits.array() > 0).all() &&
         (torque_limits.array() <= ceilings.array()).all();
}

bool ControllerManager::fresh(const Command& c, uint64_t now) const noexcept {
  return c.source_ns > 0 && c.receipt_ns > 0 && now >= c.receipt_ns &&
         now - c.receipt_ns <= p_.timeout_ns && c.uncertainty_ns <= p_.uncertainty_ns &&
         (c.source_ns <= now ? now - c.source_ns <= p_.timeout_ns
                             : c.source_ns - now <= p_.future_ns);
}

void ControllerManager::fault(Reason reason) noexcept {
  teleoperation_reset_pending_ = true;
  hold_reset_pending_ = true;
  // Homing arbitration must not turn a preceding input fault into permission
  // to hold. Only a fresh, explicit release restores that permission.
  if (reason != Reason::homing)
    hold_permitted_ = false;
  enabled_ = false;
  release_seen_ = false;
  reason_ = reason;
}

void ControllerManager::accept(const Command& c, uint64_t now) noexcept {
  if (shutdown_)
    return;
  if (!c.valid || c.session[0] == '\0') {
    fault(Reason::invalid_command);
    return;
  }
  if (!fresh(c, now)) {
    fault(Reason::stale);
    return;
  }
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
        c.source_ns == source_ns_)
      return;
    fault(Reason::order);
    return;
  }
  ordered_ = true;
  sequence_ = c.sequence;
  source_ns_ = c.source_ns;
  last_ = c;
  if (!c.enabled) {
    if (enabled_)
      teleoperation_reset_pending_ = true;
    enabled_ = false;
    if (c.released) {
      release_seen_ = true;
      hold_permitted_ = true;
      reason_ = Reason::released;
    } else if (release_seen_)
      reason_ = Reason::released;
    return;
  }
  if (!c.position.allFinite() || !c.orientation.coeffs().allFinite() ||
      std::abs(c.orientation.norm() - 1) > 0.01 || !c.twist.allFinite() || c.released) {
    fault(Reason::invalid_command);
    return;
  }
  if ((c.position - p_.envelope_center).norm() >
      p_.chain_radius + p_.tool_offset + p_.envelope_margin) {
    fault(Reason::envelope);
    return;
  }
  if (!release_seen_) {
    enabled_ = false;
    return;
  }
  enabled_ = true;
  hold_permitted_ = false;
  hold_reset_pending_ = true;
  reason_ = Reason::active;
}

void ControllerManager::shutdown() noexcept {
  shutdown_ = true;
  fault(Reason::shutdown);
  homing_.cancel("shutdown");
}

void ControllerManager::safety_fallback() noexcept {
  fault(Reason::safety_filter);
  homing_.cancel("CBF fallback");
}

ManagerOutput ControllerManager::update(const ArmState& s, uint64_t now, double dt) noexcept {
  ManagerOutput result;
  auto& out = result.control;
  if (identity_ && !fresh(last_, now))
    fault(Reason::stale);
  if (!s.position.allFinite() || !s.orientation.coeffs().allFinite() || !s.q.allFinite() ||
      !s.dq.allFinite() || !s.coriolis.allFinite() || !s.jacobian.allFinite() ||
      !s.previous_torque.allFinite())
    out.state_fault = StateFault::nonfinite_state;
  else if (std::abs(s.orientation.norm() - 1) > 0.01)
    out.state_fault = StateFault::orientation;
  else if (!std::isfinite(dt) || dt <= 0 || dt > 0.02)
    out.state_fault = StateFault::period;
  if (out.state_fault == StateFault::none) {
    const double slew = p_.torque_rate * std::min(dt, 0.001);
    for (int j = 0; j < 7; ++j)
      if (std::max(-p_.torque_limits[j], s.previous_torque[j] - slew) >
          std::min(p_.torque_limits[j], s.previous_torque[j] + slew))
        out.state_fault = StateFault::actuator_interval;
  }
  if (out.state_fault != StateFault::none) {
    fault(Reason::invalid_state);
    out.reason = reason_;
    return result;  // Runtime must terminate; never send placeholder torques.
  }

  const bool was_homing = homing_.active();
  result.homing = homing_.update(s, now, dt);
  if (was_homing || result.homing.active)
    fault(Reason::homing);
  if (shutdown_)
    fault(Reason::shutdown);

  out.target_position = enabled_ ? last_.position : s.position;
  out.target_orientation = enabled_ ? last_.orientation : s.orientation;
  out.desired_twist = enabled_ ? last_.twist : Vector6::Zero();
  out.motion_enabled = enabled_;
  out.reason = reason_;
  out.source_ns = last_.source_ns;
  out.sequence = last_.sequence;

  if (result.homing.stop_failed)
    return result;  // Fatal controlled-stop failure.
  TorqueController* law = enabled_ ? &teleoperation_ : &damping_;
  ControlReference reference =
      CartesianReference{out.target_position, out.target_orientation, out.desired_twist};
  if (result.homing.active) {
    law = &homing_;
    reference = JointReference{result.homing.desired_q, result.homing.desired_dq};
  } else if (!enabled_ && !shutdown_ && hold_permitted_ &&
             (reason_ == Reason::startup || reason_ == Reason::released ||
              (reason_ == Reason::homing && last_.valid && last_.released && !last_.enabled &&
               fresh(last_, now)))) {
    // A deliberate release holds the measured arm configuration, never an old
    // Quest goal. Startup also holds locally without authorizing teleoperation.
    // Faults, stale input, safety fallback and shutdown retain damping behavior.
    law = &position_hold_;
    if (selected_ != law || hold_reset_pending_) {
      hold_reference_ = JointReference{s.q, Vector7::Zero()};
      hold_position_ = s.position;
      hold_orientation_ = s.orientation;
    }
    reference = hold_reference_;
    out.target_position = hold_position_;
    out.target_orientation = hold_orientation_;
    out.position_hold_active = true;
    out.reason = Reason::position_hold;
  }
  if (law != selected_ || (law == &teleoperation_ && teleoperation_reset_pending_) ||
      (law == &position_hold_ && hold_reset_pending_)) {
    law->reset(s);
    selected_ = law;
    if (law == &teleoperation_)
      teleoperation_reset_pending_ = false;
    if (law == &position_hold_)
      hold_reset_pending_ = false;
  }
  const auto nominal = law->compute(s, reference, dt);
  out.nominal = nominal.torque;
  out.state_fault = nominal.fault;
  if (out.state_fault == StateFault::none && !out.nominal.allFinite())
    out.state_fault = StateFault::nonfinite_nominal;
  if (out.state_fault != StateFault::none) {
    fault(Reason::invalid_state);
    out.reason = reason_;
    out.motion_enabled = false;
    out.position_hold_active = false;
  }
  // No clamping here: every selected nominal goes through the same final
  // safety coordinator, including its actuator-only path when CBF is disabled.
  return result;
}
}  // namespace teleop_controllers
