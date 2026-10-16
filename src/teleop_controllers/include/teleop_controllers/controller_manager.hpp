#pragma once

#include "safety_filter/actuator_limits.hpp"
#include "teleop_controllers/homing_controller.hpp"

namespace teleop_controllers {
struct ManagerParameters {
  uint64_t timeout_ns{100000000}, uncertainty_ns{10000000}, future_ns{10000000};
  // Pinned Panda URDF joint-origin chain, including flange. Center at joint1.
  // This is an outer bound only; it is not an IK or collision check.
  Eigen::Vector3d envelope_center{0, 0, 0.333};
  double chain_radius{0.316 + 0.0825 + std::hypot(0.0825, 0.384) + 0.088 + 0.107};
  double tool_offset{0.1034}, envelope_margin{0.001};
  double torque_rate{safety_filter::panda_torque_rate_limit()};
  Vector7 torque_limits{safety_filter::panda_torque_limits()};
  bool valid() const noexcept;
};

struct ManagerOutput {
  Output control;
  HomingOutput homing;
};

// One per arm, called only by its permanent control thread. Owns input gates,
// mode transitions and selection of exactly one nominal torque law per cycle.
// Laws are constructed before realtime control and must outlive the manager.
class ControllerManager {
 public:
  ControllerManager(const ManagerParameters& parameters, const HomingParameters& homing_parameters,
                    TorqueController& teleoperation, TorqueController& position_hold,
                    TorqueController& damping)
      : p_(parameters),
        homing_(homing_parameters),
        teleoperation_(teleoperation),
        position_hold_(position_hold),
        damping_(damping) {}
  void accept(const Command& command, uint64_t now_ns) noexcept;
  const Command& requested_command() const noexcept { return last_; }
  void homing_input(const HomingInput& input, uint64_t now_ns) noexcept {
    if (!shutdown_)
      homing_.input(input, now_ns);
  }
  void cancel_homing(const char* reason) noexcept { homing_.cancel(reason); }
  void fault(Reason reason) noexcept;
  void shutdown() noexcept;
  void safety_fallback() noexcept;
  ManagerOutput update(const ArmState& state, uint64_t now_ns, double period) noexcept;

 private:
  bool fresh(const Command& command, uint64_t now_ns) const noexcept;
  ManagerParameters p_;
  HomingController homing_;
  TorqueController& teleoperation_;
  TorqueController& position_hold_;
  TorqueController& damping_;
  TorqueController* selected_{nullptr};
  JointReference hold_reference_;
  Eigen::Vector3d hold_position_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond hold_orientation_{1, 0, 0, 0};
  Command last_{};
  std::array<char, 65> session_{};
  uint64_t generation_{0}, sequence_{0}, source_ns_{0};
  bool identity_{false}, ordered_{false}, release_seen_{false}, enabled_{false};
  bool shutdown_{false}, teleoperation_reset_pending_{true};
  bool hold_reset_pending_{true};
  bool hold_permitted_{true};
  Reason reason_{Reason::startup};
};
}  // namespace teleop_controllers
