// Copyright (c) 2026 Jiachun. Licensed under Apache-2.0.
#pragma once
#include <optional>
#include "teleop_controllers/control_core.hpp"
#include "teleop_controllers/joint_motion_generator.hpp"
#include "safety_filter/panda_limits.hpp"
namespace teleop_controllers {
enum class HomePhase { idle, homing, stopping, succeeded, cancelled, failed };
const char* home_phase_name(HomePhase phase) noexcept;
struct HomingParameters {
  double speed_factor{.15}, velocity_filter_tau{.01};
  double position_tolerance{.01}, velocity_tolerance{.02}, settle_duration{.25};
  double completion_timeout{30}, stop_duration{.5}, stop_acceleration{.75}, stop_timeout{5};
  Vector7 stiffness{(Vector7() << 300, 300, 300, 300, 120, 100, 50).finished()};
  Vector7 damping{(Vector7() << 30, 30, 30, 30, 10, 10, 5).finished()};
  uint64_t hold_ns{1000000000}, timeout_ns{100000000}, uncertainty_ns{10000000}, future_ns{10000000};
  bool valid() const noexcept;
};
struct HomingInput {
  std::array<char, 65> session{};
  uint64_t sequence{0}, generation{0}, source_ns{0}, receipt_ns{0}, uncertainty_ns{0};
  bool valid{false}, button_pressed{false}, trigger_released{false};
};
struct HomingOutput {
  Vector7 nominal{Vector7::Zero()}, desired_q{Vector7::Zero()}, desired_dq{Vector7::Zero()};
  HomePhase phase{HomePhase::idle};
  const char* message{"idle"};
  uint64_t request_id{0};
  double elapsed{0}, position_error{0}, velocity{0};
  bool active{false}, stop_failed{false};
};
// Independent nominal homing controller. It never owns a socket/FCI session;
// the permanent arm owner selects exactly one nominal law, then applies the
// same actuator and CBF filter to it.
class HomingCore {
 public:
  explicit HomingCore(const HomingParameters& p) : p_(p) {}
  void input(const HomingInput& input, uint64_t now) noexcept;
  void cancel(const char* reason) noexcept;
  HomingOutput update(const ArmState& state, uint64_t now, double dt) noexcept;
  bool active() const noexcept { return phase_ == HomePhase::homing || phase_ == HomePhase::stopping; }
 private:
  bool fresh(const HomingInput& input, uint64_t now) const noexcept;
  void reset_gesture() noexcept { require_button_release_ = true; hold_start_ns_ = 0; request_ = false; }
  void start_stop(HomePhase terminal, const char* message) noexcept;
  HomingParameters p_;
  HomingInput last_{};
  std::optional<JointMotionGenerator> trajectory_;
  Vector7 filtered_dq_{Vector7::Zero()}, desired_q_{Vector7::Zero()}, desired_dq_{Vector7::Zero()};
  Vector7 stop_q_{Vector7::Zero()}, stop_dq_{Vector7::Zero()};
  HomePhase phase_{HomePhase::idle}, terminal_{HomePhase::cancelled};
  const char* message_{"idle"};
  bool identity_{false}, require_button_release_{true}, request_{false}, cancel_requested_{false};
  bool stop_failed_{false};
  bool stop_hold_captured_{false};
  uint64_t hold_start_ns_{0}, request_id_{0};
  double elapsed_{0}, settled_{0}, stop_elapsed_{0}, stop_duration_{0};
};
}
