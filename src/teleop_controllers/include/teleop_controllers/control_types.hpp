#pragma once

#include <Eigen/Dense>
#include <array>
#include <cstdint>

namespace teleop_controllers {
using Vector6 = Eigen::Matrix<double, 6, 1>;
using Vector7 = Eigen::Matrix<double, 7, 1>;
using Jacobian = Eigen::Matrix<double, 6, 7>;

struct Command {
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation{1, 0, 0, 0};
  Vector6 twist{Vector6::Zero()};
  Vector6 requested_twist{Vector6::Zero()};
  bool requested_twist_valid{false};
  std::array<char, 65> session{};
  uint64_t sequence{0}, generation{0}, source_ns{0}, receipt_ns{0}, uncertainty_ns{0},
      location_ns{0};
  bool valid{false}, enabled{false}, released{false};
};

struct ArmState {
  Vector7 q{Vector7::Zero()}, dq{Vector7::Zero()}, coriolis{Vector7::Zero()};
  Vector7 previous_torque{Vector7::Zero()};  // libfranka tau_J_d, not measured tau_J
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation{1, 0, 0, 0};
  Jacobian jacobian{Jacobian::Zero()};
};

enum class Reason : uint8_t {
  startup,
  released,
  active,
  invalid_command,
  stale,
  order,
  session_change,
  envelope,
  queue_overflow,
  invalid_state,
  shutdown,
  homing,
  safety_filter,
  position_hold
};
const char* reason_name(Reason reason) noexcept;

enum class StateFault : uint8_t {
  none,
  nonfinite_state,
  orientation,
  period,
  nonfinite_nominal,
  actuator_interval,
  invalid_reference
};
const char* state_fault_name(StateFault fault) noexcept;

struct Output {
  Vector7 nominal{Vector7::Zero()};
  // Filled only by the runtime after the shared safety coordinator accepts this cycle.
  Vector7 torque{Vector7::Zero()};
  Eigen::Vector3d target_position{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond target_orientation{1, 0, 0, 0};
  Vector6 desired_twist{Vector6::Zero()};
  uint64_t source_ns{0}, sequence{0};
  Reason reason{Reason::startup};
  StateFault state_fault{StateFault::none};
  bool motion_enabled{false}, rate_limited{false}, magnitude_limited{false};
  bool position_hold_active{false};
};

}  // namespace teleop_controllers
