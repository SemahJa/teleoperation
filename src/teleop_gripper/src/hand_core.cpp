#include "teleop_gripper/hand_core.hpp"

#include <algorithm>
#include <cmath>

namespace teleop_gripper {
bool valid_hand_feedback(double width, double maximum) noexcept {
  return std::isfinite(width) && std::isfinite(maximum) && maximum > 0 &&
         width >= -kWidthReadbackTolerance && width <= maximum + kWidthReadbackTolerance;
}
bool HandParameters::valid() const noexcept {
  return std::isfinite(press) && std::isfinite(release) && release >= 0 && press > release &&
         press <= 1 && std::isfinite(speed) && speed > 0 && std::isfinite(open_speed) &&
         open_speed > 0 && std::isfinite(open_width) && (open_width == -1 || open_width > 0) &&
         std::isfinite(close_width) && close_width >= 0 &&
         (open_width == -1 || close_width < open_width) && debounce_ns <= 100000000 &&
         timeout_ns > 0 && timeout_ns <= 100000000 && uncertainty_ns > 0 &&
         uncertainty_ns <= 10000000 &&
         (close_retry_ns == 0 || (close_retry_ns >= 50000000 && close_retry_ns <= 10000000000ULL));
}
void HandCore::reset_input() noexcept {
  need_release_ = true;
  pressed_ = consumed_ = false;
  press_start_ns_ = 0;
  open_down_ = false;
  open_release_required_ = true;
  open_after_stop_ = false;
  close_after_stop_ = false;
  retry_due_ns_ = 0;
}
HandRequest HandCore::interrupt() noexcept {
  if (!busy() || stopping_)
    return {};
  stopping_ = true;
  message_ = "stop requested";
  return {HandOperation::stop};
}
HandRequest HandCore::start(HandOperation operation, uint64_t now) noexcept {
  if (!calibrated_ || busy())
    return {};
  const double open = p_.open_width == -1 ? maximum_width_ : p_.open_width;
  if (open > maximum_width_ || p_.close_width >= open) {
    message_ = "configured widths exceed calibrated travel or close width is not below open width";
    return {};
  }
  // width_ is a completion snapshot, not live feedback. The backend must read
  // current width and clamp a close target so stale feedback cannot block a
  // fresh press or accidentally command opening.
  operation_ = operation;
  // Held grip is a persistent goal, not a failure-triggered retry. Arm its
  // next refresh when a close starts. A still-running move keeps that goal
  // active without interruption; the overdue refresh runs once it completes.
  retry_due_ns_ =
      operation == HandOperation::close && p_.close_retry_ns ? now + p_.close_retry_ns : 0;
  message_ =
      operation == HandOperation::close ? "closing while grip held" : "joystick-click opening";
  return {operation, operation == HandOperation::close ? p_.close_width : open,
          operation == HandOperation::close ? p_.speed : p_.open_speed};
}
HandRequest HandCore::input(const GripInput& c, uint64_t now) noexcept {
  if (operation_ == HandOperation::calibrate) {
    reset_input();
    return {};  // Empty-hand calibration is independent of Quest input.
  }
  const bool fresh =
      c.source_ns > 0 &&
      (c.source_ns <= now ? now - c.source_ns <= p_.timeout_ns : c.source_ns - now <= 10000000) &&
      c.uncertainty_ns <= p_.uncertainty_ns;
  if (!c.valid || !fresh || !std::isfinite(c.value) || c.value < 0 || c.value > 1 ||
      c.session[0] == '\0') {
    reset_input();
    const auto stop = interrupt();
    message_ = "invalid/stale grip input; stop requested if moving; release required";
    return stop;
  }
  if (!identity_ || c.session != last_.session || c.generation != last_.generation) {
    reset_input();
    identity_ = true;
    last_ = GripInput{};
    const auto stop = interrupt();
    if (stop.operation != HandOperation::none) {
      last_ = c;
      return stop;
    }
  }
  if (last_.source_ns && (c.sequence <= last_.sequence || c.source_ns <= last_.source_ns ||
                          c.source_ns - last_.source_ns > p_.timeout_ns)) {
    reset_input();
    const auto stop = interrupt();
    message_ = "grip order/gap fault; release required";
    // Keep the watermark on out-of-order input, advance on a forward gap.
    if (c.sequence > last_.sequence && c.source_ns > last_.source_ns)
      last_ = c;
    return stop;
  }
  last_ = c;
  // Each available joystick needs a fresh released sample after startup/fault.
  // The press edge opens immediately; no click-duration or release wait.
  const bool open_edge = c.open_active && c.open_pressed && !open_down_ && !open_release_required_;
  if (!c.open_active) {
    open_release_required_ = true;
    open_down_ = false;
  } else {
    if (!c.open_pressed)
      open_release_required_ = false;
    open_down_ = c.open_pressed;
  }
  if (open_edge && calibrated_) {
    retry_due_ns_ = 0;
    need_release_ = true;  // A held side grip cannot reverse this opening.
    pressed_ = consumed_ = close_after_stop_ = false;
    if (operation_ == HandOperation::open)
      return {};
    if (busy()) {
      open_after_stop_ = true;
      return interrupt();
    }
    return start(HandOperation::open, now);
  }
  if ((!c.grip_active && operation_ == HandOperation::close) ||
      (!c.open_active && operation_ == HandOperation::open)) {
    reset_input();
    return interrupt();
  }
  if (!c.grip_active) {
    need_release_ = true;
    pressed_ = consumed_ = close_after_stop_ = false;
    retry_due_ns_ = 0;
    return {};
  }
  if (c.value <= p_.release) {
    retry_due_ns_ = 0;
    close_after_stop_ = false;
    need_release_ = false;
    pressed_ = consumed_ = false;
    press_start_ns_ = 0;
    if (operation_ == HandOperation::close)
      return interrupt();
    return {};
  }
  if (need_release_)
    return {};
  if (!pressed_) {
    if (c.value < p_.press)
      return {};
    pressed_ = true;
    press_start_ns_ = c.source_ns;
    consumed_ = false;
  }
  if (!consumed_ && c.source_ns - press_start_ns_ >= p_.debounce_ns) {
    consumed_ = true;
    if (busy()) {
      // Keep only the current, fresh held intent. Release/fault/watchdog clears
      // it; historical clicks are never replayed as later close commands.
      close_after_stop_ = true;
      open_after_stop_ = false;
      return interrupt();
    }
    return start(HandOperation::close, now);
  }
  return {};
}
HandRequest HandCore::calibrate(bool empty_hand, bool confirmed) noexcept {
  if (!empty_hand || !confirmed || busy()) {
    message_ = "calibration requires both confirmations and an idle worker";
    return {};
  }
  reset_input();
  calibrated_ = false;
  state_ = HandState::unknown;
  operation_ = HandOperation::calibrate;
  message_ = "calibration started";
  return {operation_};
}
HandRequest HandCore::complete(HandOperation operation, bool success, bool communication_ok,
                               double width, double maximum, uint64_t now,
                               bool retryable) noexcept {
  const bool stopped = stopping_;
  const bool reopen = open_after_stop_;
  const bool reclose = close_after_stop_ && pressed_ && !need_release_;
  operation_ = HandOperation::none;
  stopping_ = open_after_stop_ = false;
  close_after_stop_ = false;
  if (!communication_ok || !valid_hand_feedback(width, maximum)) {
    calibrated_ = false;
    state_ = HandState::unknown;
    reset_input();
    message_ = "Hand communication/state fault; explicit calibration required";
    return {};
  }
  width_ = std::clamp(width, 0., maximum);
  maximum_width_ = maximum;
  const bool repeat_close = operation == HandOperation::close && !stopped &&
                            (success || retryable) && calibrated_ && pressed_ && !need_release_ &&
                            p_.close_retry_ns;
  if (repeat_close) {
    // Keep the existing cadence after success, failure, or reaching the lower
    // bound. Periodic live readback can detect a later reopened finger gap.
    state_ = success ? HandState::closed : HandState::unknown;
    message_ = "grip held; closing demand remains active";
    return {};
  }
  if (!success || (stopped && operation == HandOperation::calibrate)) {
    if (operation == HandOperation::calibrate)
      calibrated_ = false;
    state_ = HandState::unknown;
    reset_input();
    message_ = "operation failed or calibration stopped; no automatic retry";
    return {};
  }
  if (operation == HandOperation::calibrate) {
    calibrated_ = true;
    reset_input();
    state_ = HandState::unknown;
    message_ = "calibrated; release grip before closing";
  } else {
    state_ = stopped ? HandState::stopped
                     : (operation == HandOperation::open ? HandState::open : HandState::closed);
    message_ = stopped ? "motion stopped"
                       : (operation == HandOperation::open ? "open completed" : "close completed");
  }
  if (reopen)
    return start(HandOperation::open, now);
  if (reclose)
    return start(HandOperation::close, now);
  return {};
}
HandRequest HandCore::watchdog(uint64_t now) noexcept {
  if (operation_ == HandOperation::calibrate)
    return {};
  if (last_.source_ns && (now >= last_.source_ns ? now - last_.source_ns > p_.timeout_ns
                                                 : last_.source_ns - now > 10000000)) {
    reset_input();
    return interrupt();
  }
  if (retry_due_ns_ && now >= retry_due_ns_ && !busy() && calibrated_ && pressed_ && !need_release_)
    return start(HandOperation::close, now);
  return {};
}
HandRequest HandCore::shutdown() noexcept {
  reset_input();
  return interrupt();
}
}  // namespace teleop_gripper
