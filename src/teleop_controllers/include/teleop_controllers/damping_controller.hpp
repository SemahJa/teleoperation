#pragma once

#include "teleop_controllers/torque_controller.hpp"

namespace teleop_controllers {
// Fault/shutdown behavior. Normal trigger release selects position holding.
class DampingController final : public TorqueController {
 public:
  DampingController(double translation_gain, double rotation_gain) {
    gain_ << translation_gain, translation_gain, translation_gain, rotation_gain, rotation_gain,
        rotation_gain;
  }
  NominalTorque compute(const ArmState& state, const ControlReference& reference,
                        double period) noexcept override;

 private:
  Vector6 gain_;
};
}  // namespace teleop_controllers
