// Copyright (c) 2026 Jiachun. Licensed under Apache-2.0.
// Adapted from thesis_ws HomingCore; fixed storage, per-arm guarded gesture,
// FER limits and controlled cancellation added for direct-libfranka Panda.
#include "franka_teleop_controller/homing_core.hpp"
#include <algorithm>
#include <cmath>
namespace franka_teleop_controller {
const char* home_phase_name(HomePhase phase) noexcept {
  switch (phase) {
    case HomePhase::homing: return "homing";
    case HomePhase::stopping: return "stopping";
    case HomePhase::succeeded: return "succeeded";
    case HomePhase::cancelled: return "cancelled";
    case HomePhase::failed: return "failed";
    default: return "idle";
  }
}
bool HomingParameters::valid() const noexcept {
  const double values[] = {speed_factor, position_tolerance, velocity_tolerance,
      completion_timeout, stop_duration, stop_acceleration, stop_timeout};
  for (auto v : values) if (!std::isfinite(v) || v <= 0) return false;
  return speed_factor <= 1 && stop_timeout > std::max(stop_duration, 1.5707963267948966 *
      2.5 * speed_factor / stop_acceleration) && std::isfinite(velocity_filter_tau) && velocity_filter_tau >= 0 &&
      std::isfinite(settle_duration) && settle_duration >= 0 && stop_timeout > stop_duration &&
      stiffness.allFinite() && damping.allFinite() && (stiffness.array() > 0).all() &&
      (damping.array() > 0).all() && stop_acceleration <= 7.5 && hold_ns >= 1000000000 &&
      timeout_ns > 0 && timeout_ns <= 100000000 && uncertainty_ns > 0 && uncertainty_ns <= 10000000 &&
      future_ns <= 10000000;
}
bool HomingCore::fresh(const HomingInput& c, uint64_t now) const noexcept {
  return c.source_ns > 0 && c.receipt_ns > 0 && now >= c.receipt_ns &&
      now - c.receipt_ns <= p_.timeout_ns && c.uncertainty_ns <= p_.uncertainty_ns &&
      (c.source_ns <= now ? now - c.source_ns <= p_.timeout_ns : c.source_ns - now <= p_.future_ns);
}
void HomingCore::cancel(const char* reason) noexcept {
  reset_gesture();
  if (phase_ == HomePhase::homing) { cancel_requested_ = true; message_ = reason; }
}
void HomingCore::input(const HomingInput& c, uint64_t now) noexcept {
  if (!c.valid || c.session[0] == '\0' || !fresh(c, now)) { cancel("invalid/stale controller input"); return; }
  if (!identity_ || c.session != last_.session || c.generation != last_.generation) {
    cancel("session/reference change"); identity_ = true; last_ = HomingInput{};
  }
  if (last_.source_ns && (c.sequence <= last_.sequence || c.source_ns <= last_.source_ns)) {
    cancel("out-of-order controller input"); return;
  }
  if (last_.source_ns && c.source_ns - last_.source_ns > p_.timeout_ns)
    cancel("controller source gap");
  last_ = c;
  if (!c.trigger_released) { cancel("trigger pressed"); return; }
  if (active()) { reset_gesture(); return; }  // No queued home gesture.
  if (!c.button_pressed) {
    const bool held = !require_button_release_ && hold_start_ns_ &&
                      c.source_ns - hold_start_ns_ >= p_.hold_ns;
    require_button_release_ = false; hold_start_ns_ = 0;
    if (held) { request_ = true; ++request_id_; }
  } else if (!require_button_release_ && !hold_start_ns_) {
    hold_start_ns_ = c.source_ns;
  }
}
void HomingCore::start_stop(HomePhase terminal, const char* message) noexcept {
  stop_q_ = desired_q_; stop_dq_ = desired_dq_;
  stop_duration_ = std::max(p_.stop_duration,
      1.5707963267948966 * stop_dq_.cwiseAbs().maxCoeff() / p_.stop_acceleration);
  stop_elapsed_ = settled_ = 0;
  terminal_ = terminal; phase_ = HomePhase::stopping; message_ = message;
  cancel_requested_ = false; reset_gesture();
}
HomingOutput HomingCore::update(const ArmState& s, uint64_t now, double dt) noexcept {
  if (identity_ && !fresh(last_, now)) cancel("controller watchdog");
  if (!std::isfinite(dt) || dt < 0 || dt > .02 || !s.q.allFinite() ||
      !s.dq.allFinite() || !s.coriolis.allFinite()) {
    phase_ = HomePhase::failed; stop_failed_ = true; reset_gesture(); message_ = "invalid homing robot state";
  }
  if (request_ && !active() && !stop_failed_) {
    request_ = false;
    bool within_limits = true;
    for (int j = 0; j < 7; ++j) within_limits &= s.q[j] >= kJointLower[j] && s.q[j] <= kJointUpper[j];
    if (within_limits) {
      trajectory_.emplace(p_.speed_factor, s.q, Eigen::Map<const Vector7>(kHome.data()));
      desired_q_ = s.q; desired_dq_.setZero(); filtered_dq_ = s.dq;
      phase_ = HomePhase::homing; elapsed_ = settled_ = 0; message_ = "homing";
    } else { phase_ = HomePhase::failed; message_ = "start joints outside FER limits"; }
  }
  const Vector7 home = Eigen::Map<const Vector7>(kHome.data());
  bool trajectory_complete = false;
  if (phase_ == HomePhase::homing) {
    if (cancel_requested_) start_stop(HomePhase::cancelled, message_);
    else {
      elapsed_ += dt;
      const auto trajectory = trajectory_->getDesiredJointPositions(elapsed_);
      desired_q_ = trajectory.first;
      desired_dq_ = trajectory_->getDesiredJointVelocities(elapsed_);
      trajectory_complete = trajectory.second;
      if (elapsed_ >= p_.completion_timeout && !trajectory_complete)
        start_stop(HomePhase::failed, "homing timeout");
    }
  }
  if (phase_ == HomePhase::stopping) {
    stop_elapsed_ += dt;
    const double t = std::min(stop_elapsed_, stop_duration_);
    const double phase = 3.141592653589793 * t / stop_duration_;
    desired_q_ = stop_q_ + stop_dq_ * .5 * (t + stop_duration_ / 3.141592653589793 * std::sin(phase));
    desired_dq_ = stop_dq_ * .5 * (1 + std::cos(phase));
    const bool settled = stop_elapsed_ >= stop_duration_ &&
        s.dq.cwiseAbs().maxCoeff() <= p_.velocity_tolerance &&
        (desired_q_ - s.q).cwiseAbs().maxCoeff() <= p_.position_tolerance;
    settled_ = settled ? settled_ + dt : 0;
    if (settled_ >= p_.settle_duration && settled) phase_ = terminal_;
    if (stop_elapsed_ >= p_.stop_timeout && phase_ == HomePhase::stopping) {
      phase_ = HomePhase::failed; stop_failed_ = true; message_ = "controlled stop did not settle; terminate control";
    }
  } else if (phase_ == HomePhase::homing) {
    const bool settled = trajectory_complete && (home - s.q).cwiseAbs().maxCoeff() <= p_.position_tolerance &&
                         s.dq.cwiseAbs().maxCoeff() <= p_.velocity_tolerance;
    settled_ = settled ? settled_ + dt : 0;
    if (settled && settled_ >= p_.settle_duration) { phase_ = HomePhase::succeeded; message_ = "home reached; fresh trigger release/press required"; }
    else if (elapsed_ >= p_.completion_timeout) start_stop(HomePhase::failed, "homing did not settle before timeout");
  }
  const double alpha = p_.velocity_filter_tau <= 0 ? 1 : 1 - std::exp(-dt / p_.velocity_filter_tau);
  filtered_dq_ += alpha * (s.dq - filtered_dq_);
  HomingOutput out;
  out.active = active(); out.phase = phase_; out.message = message_; out.request_id = request_id_;
  out.desired_q = desired_q_; out.desired_dq = desired_dq_; out.elapsed = elapsed_;
  out.position_error = (home - s.q).cwiseAbs().maxCoeff(); out.velocity = s.dq.cwiseAbs().maxCoeff();
  out.stop_failed = stop_failed_;
  if (out.active) out.nominal = p_.stiffness.cwiseProduct(desired_q_ - s.q) -
                                p_.damping.cwiseProduct(filtered_dq_) + s.coriolis;
  return out;
}
}
