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
  std::array<char, 65> session{};
  uint64_t sequence{0}, generation{0}, source_ns{0}, receipt_ns{0}, uncertainty_ns{0};
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
  startup, released, active, invalid_command, stale, order, session_change,
  envelope, queue_overflow, invalid_state, shutdown, homing, safety_filter
};
const char* reason_name(Reason reason) noexcept;

struct Parameters {
  double lambda_translation{50}, lambda_rotation{40};
  double gain_translation{20}, gain_rotation{2.5};
  uint64_t timeout_ns{100000000}, uncertainty_ns{10000000}, future_ns{10000000};
  // Pinned Panda URDF joint-origin chain, including flange. Center at joint1.
  // This is an outer bound only; it is not an IK or collision check.
  Eigen::Vector3d envelope_center{0, 0, 0.333};
  double chain_radius{0.316 + 0.0825 + std::hypot(0.0825, 0.384) + 0.088 + 0.107};
  double tool_offset{0.1034}, envelope_margin{0.001};
  double torque_rate{900};
  Vector7 torque_limits{(Vector7() << 87, 87, 87, 87, 12, 12, 12).finished()};
  bool valid() const noexcept;
};

enum class StateFault : uint8_t { none, nonfinite_state, orientation, period,
  nonfinite_nominal, actuator_interval };
const char* state_fault_name(StateFault fault) noexcept;

struct Output {
  Vector7 nominal{Vector7::Zero()}, torque{Vector7::Zero()};
  Eigen::Vector3d target_position{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond target_orientation{1, 0, 0, 0};
  Vector6 desired_twist{Vector6::Zero()};
  uint64_t source_ns{0}, sequence{0};
  Reason reason{Reason::startup};
  StateFault state_fault{StateFault::none};
  bool motion_enabled{false}, rate_limited{false}, magnitude_limited{false};
};

// Pure, fixed-size control core shared by the mock and direct-libfranka adapters.
// accept(), fault(), update() run on the same control thread. No ROS, heap,
// mutex, exceptions, socket operations, or callbacks occur here.
class ControlCore {
 public:
  explicit ControlCore(const Parameters& parameters) : p_(parameters) {}
  void accept(const Command& command, uint64_t now_ns) noexcept;
  void fault(Reason reason) noexcept;
  Output update(const ArmState& state, uint64_t now_ns, double period) noexcept;
  Output bound_nominal(const ArmState& state, Output output, double period) noexcept;
  bool fresh(const Command& command, uint64_t now_ns) const noexcept;
 private:
  Parameters p_;
  Command last_{};
  std::array<char, 65> session_{};
  uint64_t generation_{0}, sequence_{0}, source_ns_{0};
  bool identity_{false}, ordered_{false}, release_seen_{false}, enabled_{false};
  Reason reason_{Reason::startup};
};
}  // namespace teleop_controllers
