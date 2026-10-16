#include "teleop_controllers/damping_controller.hpp"

namespace teleop_controllers {
NominalTorque DampingController::compute(const ArmState& s, const ControlReference&,
                                         double) noexcept {
  return {-s.jacobian.transpose() * gain_.cwiseProduct(s.jacobian * s.dq) + s.coriolis,
          StateFault::none};
}
}  // namespace teleop_controllers
