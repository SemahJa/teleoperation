#include "franka_teleop_controller/hand_core.hpp"

namespace franka_teleop_controller {
bool HandParameters::valid() const noexcept {
  const double values[] = {press, release, speed, force, open_width, grasp_width,
                           epsilon_inner, epsilon_outer};
  for (auto v : values) if (!std::isfinite(v)) return false;
  return release >= 0 && press > release && press <= 1 && speed > 0 && force > 0 &&
         (open_width == -1 || open_width >= 0) && (grasp_width == -1 || grasp_width >= 0) &&
         (epsilon_inner == -1 || epsilon_inner >= 0) && (epsilon_outer == -1 || epsilon_outer >= 0) &&
         debounce_ns > 0 && timeout_ns > 0 && timeout_ns <= 100000000 &&
         uncertainty_ns > 0 && uncertainty_ns <= 10000000;
}
HandRequest HandCore::input(const GripInput& c, uint64_t now) noexcept {
  HandRequest request;
  const bool fresh = c.source_ns > 0 &&
      (c.source_ns <= now ? now - c.source_ns <= p_.timeout_ns : c.source_ns - now <= 10000000) &&
      c.uncertainty_ns <= p_.uncertainty_ns;
  if (!c.valid || !fresh || !std::isfinite(c.value) || c.value < 0 || c.value > 1 ||
      c.session[0] == '\0') {
    reset_input(); message_ = "invalid/stale grip input; release required"; return request;
  }
  if (!identity_ || c.session != last_.session || c.generation != last_.generation) {
    reset_input(); identity_ = true; last_ = GripInput{};
  }
  if (last_.source_ns && (c.sequence <= last_.sequence || c.source_ns <= last_.source_ns)) {
    reset_input(); message_ = "out-of-order grip input; release required"; return request;
  }
  if (last_.source_ns && c.source_ns - last_.source_ns > p_.timeout_ns) reset_input();
  last_ = c;
  if (c.value <= p_.release) {
    need_release_ = false; press_start_ns_ = 0; return request;
  }
  if (need_release_) return request;
  if (c.value < p_.press) { press_start_ns_ = 0; return request; }
  if (!press_start_ns_) { press_start_ns_ = c.source_ns; return request; }
  if (c.source_ns - press_start_ns_ < p_.debounce_ns) return request;
  // Consume every qualifying gesture, even when disabled or busy. Never queue
  // it or fire it later when calibration/another operation finishes.
  reset_input();
  if (busy_) { message_ = "grip press ignored while busy"; return request; }
  if (!calibrated_) { message_ = "grip press rejected: calibration required"; return request; }
  request.speed = p_.speed; request.force = p_.force;
  if (state_ == HandState::open) {
    if (p_.grasp_width < 0 || p_.epsilon_inner < 0 || p_.epsilon_outer < 0 ||
        p_.grasp_width > maximum_width_) {
      message_ = "configure object width and grasp tolerances before grasping"; return request;
    }
    request.operation = HandOperation::grasp; request.width = p_.grasp_width;
    request.epsilon_inner = p_.epsilon_inner; request.epsilon_outer = p_.epsilon_outer;
  } else {
    request.operation = HandOperation::open;
    request.width = p_.open_width == -1 ? maximum_width_ : p_.open_width;
    if (request.width > maximum_width_) {
      message_ = "configured open width exceeds calibrated maximum"; return HandRequest{};
    }
  }
  busy_ = true; message_ = "operation started";
  return request;
}
HandRequest HandCore::calibrate(bool empty_hand, bool confirmed) noexcept {
  if (!empty_hand || !confirmed) {
    message_ = "calibration requires explicit empty-hand and operator confirmation"; return {};
  }
  if (busy_) { message_ = "calibration rejected while busy"; return {}; }
  busy_ = true; calibrated_ = false; state_ = HandState::unknown; reset_input();
  message_ = "calibration started";
  HandRequest request; request.operation = HandOperation::calibrate; return request;
}
void HandCore::complete(HandOperation operation, bool success, bool communication_ok,
                        double width, double maximum) noexcept {
  busy_ = false;
  const bool valid = communication_ok && std::isfinite(width) && width >= 0 &&
                     std::isfinite(maximum) && maximum > 0 && width <= maximum + 1e-6;
  if (!valid) {
    calibrated_ = false; state_ = HandState::unknown; reset_input();
    message_ = "Hand communication/state fault; no retry, explicit calibration required"; return;
  }
  width_ = width; maximum_width_ = maximum;
  if (!success) {
    state_ = HandState::unknown;
    if (operation == HandOperation::calibrate) calibrated_ = false;
    message_ = "operation failed; no retry or force increase; next deliberate press opens"; return;
  }
  if (operation == HandOperation::calibrate) {
    calibrated_ = true; state_ = HandState::unknown;
    message_ = "calibrated; next deliberate grip press opens";
  } else {
    state_ = operation == HandOperation::open ? HandState::open : HandState::grasped;
    message_ = operation == HandOperation::open ? "open completed" : "grasp completed";
  }
}
void HandCore::watchdog(uint64_t now) noexcept {
  if (last_.source_ns && (now >= last_.source_ns ? now - last_.source_ns > p_.timeout_ns :
                                              last_.source_ns - now > 10000000))
    reset_input();  // Does not cancel/alter any already-issued Hand operation.
}
}  // namespace franka_teleop_controller
