#pragma once

#include "teleop_controllers/controller_manager.hpp"

namespace teleop_controllers {
// Fixed-size handoff only. Construction/copying on the owner thread performs
// no ROS operations, allocation, numerical differentiation, or file I/O.
struct TrackingSnapshot {
  ArmState state;
  Output control;
  Command requested;
  HomingOutput homing;
  Vector7 robot_desired{Vector7::Zero()}, measured_torque{Vector7::Zero()};
  Vector7 gravity{Vector7::Zero()};
  Vector7 previous_nominal{Vector7::Zero()}, previous_commanded{Vector7::Zero()};
  uint64_t cycle{0}, state_ns{0}, control_ns{0}, issued_ns{0};
  uint64_t previous_cycle{0}, previous_issued_ns{0};
  double period{0};
  bool hardware{false}, fallback{false}, solver_succeeded{false};
};

inline Eigen::Vector3d rotation_error(const Eigen::Quaterniond& measured,
                                      const Eigen::Quaterniond& reference) noexcept {
  Eigen::Quaterniond relative = reference.normalized().conjugate() * measured.normalized();
  if (relative.w() < 0)
    relative.coeffs() *= -1;
  const Eigen::AngleAxisd angle(relative);
  return reference.normalized() * (angle.axis() * angle.angle());
}
}  // namespace teleop_controllers
