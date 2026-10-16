#include "teleop_controllers/position_hold_controller.hpp"

#include <algorithm>
#include <cmath>

namespace teleop_controllers {
bool PositionHoldParameters::valid() const noexcept {
  return stiffness.allFinite() && damping.allFinite() && (stiffness.array() > 0).all() &&
         (damping.array() > 0).all() && std::isfinite(stiffness_ramp_seconds) &&
         stiffness_ramp_seconds > 0;
}

NominalTorque PositionHoldController::compute(const ArmState& s, const ControlReference& reference,
                                              double period) noexcept {
  const auto* target = std::get_if<JointReference>(&reference);
  if (!target || !target->position.allFinite() || !target->velocity.allFinite())
    return {Vector7::Zero(), StateFault::invalid_reference};
  if (!std::isfinite(period) || period <= 0 || period > .02)
    return {Vector7::Zero(), StateFault::period};
  elapsed_ = std::min(p_.stiffness_ramp_seconds, elapsed_ + period);
  const double u = elapsed_ / p_.stiffness_ramp_seconds;
  const double scale = u * u * (3 - 2 * u);
  return {scale * p_.stiffness.cwiseProduct(target->position - s.q) +
              p_.damping.cwiseProduct(target->velocity - s.dq) + s.coriolis,
          StateFault::none};
}
}  // namespace teleop_controllers
