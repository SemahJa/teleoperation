#include "teleop_controllers/orientation_only_controller.hpp"

namespace teleop_controllers {
NominalTorque OrientationOnlyController::compute(const ArmState& state,
                                                 const ControlReference& reference,
                                                 double period) noexcept {
  const auto* target = std::get_if<CartesianReference>(&reference);
  if (!target)
    return {Vector7::Zero(), StateFault::invalid_reference};
  CartesianReference held = *target;
  held.twist.head<3>().setZero();
  return impedance_.compute(state, held, period);
}
}  // namespace teleop_controllers
