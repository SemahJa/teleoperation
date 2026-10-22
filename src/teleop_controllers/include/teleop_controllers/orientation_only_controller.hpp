#pragma once

#include "teleop_controllers/cartesian_impedance_controller.hpp"

namespace teleop_controllers {
// Cartesian impedance about a position captured when orientation mode begins.
// The manager supplies that fixed position and removes translational feedforward.
class OrientationOnlyController final : public TorqueController {
 public:
  explicit OrientationOnlyController(const CartesianImpedanceParameters& parameters)
      : impedance_(parameters) {}
  NominalTorque compute(const ArmState& state, const ControlReference& reference,
                        double period) noexcept override;

 private:
  CartesianImpedanceController impedance_;
};
}  // namespace teleop_controllers
