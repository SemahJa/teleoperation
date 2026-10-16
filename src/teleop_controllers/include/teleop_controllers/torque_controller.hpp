#pragma once

#include <variant>

#include "teleop_controllers/control_types.hpp"

namespace teleop_controllers {
struct CartesianReference {
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation{1, 0, 0, 0};
  Vector6 twist{Vector6::Zero()};
};
struct JointReference {
  Vector7 position{Vector7::Zero()}, velocity{Vector7::Zero()};
};
using ControlReference = std::variant<CartesianReference, JointReference>;

struct NominalTorque {
  Vector7 torque{Vector7::Zero()};
  StateFault fault{StateFault::none};
};

// All laws produce seven joint torques in Nm, in Panda joint order, with the
// backend's Coriolis compensation convention. The shared safety layer owns
// final actuator/CBF bounds. No ROS, allocation, I/O or blocking in these calls.
class TorqueController {
 public:
  virtual ~TorqueController() = default;
  virtual void reset(const ArmState&) noexcept {}
  virtual NominalTorque compute(const ArmState& state, const ControlReference& reference,
                                double period) noexcept = 0;
};
}  // namespace teleop_controllers
