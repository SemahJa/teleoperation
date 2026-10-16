#include "teleop_controllers/cartesian_impedance_controller.hpp"

#include <cmath>

namespace teleop_controllers {
bool CartesianImpedanceParameters::valid() const noexcept {
  for (double value : {lambda_translation, lambda_rotation, gain_translation, gain_rotation})
    if (!std::isfinite(value) || value <= 0)
      return false;
  return true;
}

NominalTorque CartesianImpedanceController::compute(const ArmState& s,
                                                    const ControlReference& reference,
                                                    double) noexcept {
  const auto* target = std::get_if<CartesianReference>(&reference);
  if (!target)
    return {Vector7::Zero(), StateFault::invalid_reference};
  Vector6 error = Vector6::Zero();
  error.head<3>() = s.position - target->position;
  Eigen::Quaterniond rotation(target->orientation.conjugate() * s.orientation);
  if (rotation.w() < 0)
    rotation.coeffs() *= -1;
  const Eigen::AngleAxisd angle(rotation);
  error.tail<3>() = target->orientation * (angle.axis() * angle.angle());
  const Vector6 r = s.jacobian * s.dq - target->twist + lambda_.cwiseProduct(error);
  return {-s.jacobian.transpose() * gain_.cwiseProduct(r) + s.coriolis, StateFault::none};
}
}  // namespace teleop_controllers
