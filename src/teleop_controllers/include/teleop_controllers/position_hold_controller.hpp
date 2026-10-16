#pragma once

#include "teleop_controllers/torque_controller.hpp"

namespace teleop_controllers {
struct PositionHoldParameters {
  // Joint stiffness [Nm/rad] and damping [Nm s/rad], in Panda joint order.
  Vector7 stiffness{(Vector7() << 600, 600, 600, 600, 250, 150, 75).finished()};
  Vector7 damping{(Vector7() << 50, 50, 50, 50, 20, 15, 10).finished()};
  double stiffness_ramp_seconds{.25};
  bool valid() const noexcept;
};

// The manager captures a JointReference once on entry. This law never follows
// new measured positions: a disturbance must produce restoring joint torque.
class PositionHoldController final : public TorqueController {
 public:
  explicit PositionHoldController(const PositionHoldParameters& parameters) : p_(parameters) {}
  void reset(const ArmState&) noexcept override { elapsed_ = 0; }
  NominalTorque compute(const ArmState& state, const ControlReference& reference,
                        double period) noexcept override;

 private:
  PositionHoldParameters p_;
  double elapsed_{0};
};
}  // namespace teleop_controllers
