// Copyright (c) 2026 Jiachun. Licensed under Apache-2.0.
#pragma once
#include <atomic>
#include "franka_teleop_controller/control_core.hpp"
#include "franka_teleop_controller/cbf_safety_filter.hpp"
#include "franka_teleop_controller/handoff.hpp"
namespace franka_teleop_controller {
enum class SafetyStop : unsigned { none, result_deadline, state_age, state_skew,
  invalid_model, solver_failures, filter_deadline, owner_failure, scheduling };
const char* safety_stop_name(SafetyStop reason) noexcept;
struct ModelRequest {
  cbf::ArmState state;
  Vector7 nominal{Vector7::Zero()};
  uint64_t cycle{0}, stamp_ns{0};
  double period{.001};
  bool motion_active{false}, finish_requested{false};
};
struct SafetyReply {
  cbf::SafetyResult result;
  std::array<uint64_t, 2> cycles{}, source_ns{};
  uint64_t completed_ns{0};
  unsigned consecutive_failures{0};
  bool finish{false};
};
struct CoordinatorParameters {
  std::array<bool, 2> active{true, false};
  uint64_t maximum_age_ns{2000000}, maximum_skew_ns{1000000};
  uint64_t result_deadline_ns{900000}, filter_deadline_ns{800000};
  unsigned maximum_failures{20};
  bool valid() const noexcept;
};
// One producer/consumer per arm lane. Arm callbacks submit once and await only
// their own cycle. A single coordinator owns the coupled solver. No ROS/mutexes.
class SafetyCoordinator {
 public:
  bool configure(const CoordinatorParameters& limits, const cbf::SafetyParameters& safety,
      const std::array<cbf::ArmIdentity, 2>& identities, std::string* error);
  void submit(int arm, const ModelRequest& request) noexcept { requests_[arm].publish(request); }
  bool receive(int arm, SafetyReply& reply) noexcept { return replies_[arm].read(reply); }
  bool pump(uint64_t now_ns = 0);
  bool accepts(int arm, uint64_t cycle, uint64_t started_ns,
               uint64_t now_ns, const SafetyReply& reply) noexcept;
  void stop(SafetyStop reason) noexcept;
  SafetyStop stopped() const noexcept { return stop_.load(std::memory_order_acquire); }
  const CoordinatorParameters& limits() const noexcept { return limits_; }
 private:
  cbf::CbfSafetyFilter filter_;
  CoordinatorParameters limits_;
  std::array<LatestSnapshot<ModelRequest>, 2> requests_;
  std::array<LatestSnapshot<SafetyReply>, 2> replies_;
  std::array<ModelRequest, 2> pending_;
  std::array<bool, 2> new_{};
  unsigned failures_{0};
  std::atomic<SafetyStop> stop_{SafetyStop::none};
};
}  // namespace franka_teleop_controller
