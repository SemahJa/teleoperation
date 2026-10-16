#pragma once

#include "teleop_controllers/torque_controller.hpp"

namespace teleop_controllers {
struct CartesianImpedanceParameters {
  double lambda_translation{50}, lambda_rotation{40};
  double gain_translation{20}, gain_rotation{2.5};
  bool valid() const noexcept;
};

class CartesianImpedanceController final : public TorqueController {
 public:
  explicit CartesianImpedanceController(const CartesianImpedanceParameters& parameters)
      : p_(parameters) {
    lambda_ << p_.lambda_translation, p_.lambda_translation, p_.lambda_translation,
        p_.lambda_rotation, p_.lambda_rotation, p_.lambda_rotation;
    gain_ << p_.gain_translation, p_.gain_translation, p_.gain_translation, p_.gain_rotation,
        p_.gain_rotation, p_.gain_rotation;
  }
  NominalTorque compute(const ArmState& state, const ControlReference& reference,
                        double period) noexcept override;

 private:
  CartesianImpedanceParameters p_;
  Vector6 lambda_, gain_;
};
}  // namespace teleop_controllers
