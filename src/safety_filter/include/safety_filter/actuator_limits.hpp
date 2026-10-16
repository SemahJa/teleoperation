#pragma once

#include <cmath>

#include "safety_filter/cbf_types.hpp"
#include "safety_filter/panda_limits.hpp"

namespace safety_filter {
inline constexpr double kDefaultActuatorLimitScale = .35;

struct ActuatorLimitScales {
  double torque{kDefaultActuatorLimitScale};
  double torque_rate{kDefaultActuatorLimitScale};
  bool valid() const noexcept {
    return std::isfinite(torque) && torque > 0 && torque <= 1 && std::isfinite(torque_rate) &&
           torque_rate > 0 && torque_rate <= 1;
  }
};

// Scale the published FCI ceilings once, not already derated software limits.
// Configuration validates scales before calling these helpers; no ROS or I/O.
inline cbf::Vector7d panda_torque_limits(double scale = kDefaultActuatorLimitScale) noexcept {
  return scale * Eigen::Map<const cbf::Vector7d>(kJointTorque.data());
}
inline double panda_torque_rate_limit(double scale = kDefaultActuatorLimitScale) noexcept {
  return scale * kJointTorqueRate;
}
}  // namespace safety_filter
