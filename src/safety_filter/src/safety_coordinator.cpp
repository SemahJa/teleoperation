// Copyright (c) 2026 Jiachun. Licensed under Apache-2.0.
#include "safety_filter/safety_coordinator.hpp"

#include <algorithm>
#include <chrono>
namespace safety_filter {
const char* safety_stop_name(SafetyStop r) noexcept {
  switch (r) {
    case SafetyStop::none:
      return "none";
    case SafetyStop::result_deadline:
      return "result_deadline";
    case SafetyStop::state_age:
      return "state_age";
    case SafetyStop::state_skew:
      return "state_skew";
    case SafetyStop::invalid_model:
      return "invalid_model";
    case SafetyStop::solver_failures:
      return "solver_failures";
    case SafetyStop::filter_deadline:
      return "filter_deadline";
    case SafetyStop::owner_failure:
      return "owner_failure";
    case SafetyStop::scheduling:
      return "scheduling";
  }
  return "unknown";
}
bool CoordinatorParameters::valid() const noexcept {
  return (active[0] || active[1]) && maximum_age_ns > 0 && maximum_skew_ns > 0 &&
         maximum_skew_ns <= maximum_age_ns && result_deadline_ns > 0 && filter_deadline_ns > 0 &&
         filter_deadline_ns <= result_deadline_ns && maximum_failures > 0 &&
         maximum_failures <= 100;
}
bool SafetyCoordinator::configure(const CoordinatorParameters& p, const cbf::SafetyParameters& s,
                                  const std::array<cbf::ArmIdentity, 2>& ids, std::string* error) {
  if (!p.valid() || p.active != s.active_arms) {
    if (error)
      *error = "invalid coordinator limits or arm mask";
    return false;
  }
  limits_ = p;
  return filter_.configure(s, ids, error);
}
void SafetyCoordinator::stop(SafetyStop r) noexcept {
  auto expected = SafetyStop::none;
  stop_.compare_exchange_strong(expected, r, std::memory_order_acq_rel);
}
bool SafetyCoordinator::pump(uint64_t now) {
  if (stopped() != SafetyStop::none)
    return false;
  for (int i = 0; i < 2; ++i)
    if (limits_.active[i]) {
      ModelRequest request;
      if (requests_[i].read(request)) {
        pending_[i] = request;
        new_[i] = true;
      }
    }
  for (int i = 0; i < 2; ++i)
    if (limits_.active[i] && !new_[i])
      return false;
  // In production read the clock after consuming snapshots. A producer can
  // publish between an earlier clock read and the acquire, without being future-dated.
  if (!now)
    now = std::chrono::duration_cast<std::chrono::nanoseconds>(
              std::chrono::steady_clock::now().time_since_epoch())
              .count();
  uint64_t oldest = now, newest = 0;
  cbf::DualArmState state;
  cbf::Vector14d nominal = cbf::Vector14d::Zero();
  std::array<bool, 2> activity{};
  double period = .001;
  SafetyReply reply;
  reply.started_ns = now;
  reply.finish = true;
  for (int i = 0; i < 2; ++i)
    if (limits_.active[i]) {
      const auto& p = pending_[i];
      if (!std::isfinite(p.period) || p.period <= 0 || p.period > .02) {
        stop(SafetyStop::invalid_model);
        return false;
      }
      if (!p.cycle || p.stamp_ns > now || now - p.stamp_ns > limits_.maximum_age_ns) {
        stop(SafetyStop::state_age);
        return false;
      }
      oldest = std::min(oldest, p.stamp_ns);
      newest = std::max(newest, p.stamp_ns);
      state.arm[i] = p.state;
      nominal.segment<7>(i * 7) = p.nominal;
      activity[i] = p.motion_active;
      period = std::min(period, p.period);
      reply.finish = reply.finish && p.finish_requested;
      reply.cycles[i] = p.cycle;
      reply.source_ns[i] = p.stamp_ns;
    }
  if (newest - oldest > limits_.maximum_skew_ns) {
    stop(SafetyStop::state_skew);
    return false;
  }
  reply.result = filter_.filter(nominal, state, period, activity);
  filter_status_ = reply.result.solver_status;
  if (reply.result.fatal_input_fault)
    stop(SafetyStop::invalid_model);
  if (reply.result.filter_time_seconds * 1e9 > limits_.filter_deadline_ns)
    stop(SafetyStop::filter_deadline);
  failures_ = reply.result.solver_succeeded ? 0 : failures_ + 1;
  if (failures_ >= limits_.maximum_failures)
    stop(SafetyStop::solver_failures);
  reply.consecutive_failures = failures_;
  reply.completed_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                           std::chrono::steady_clock::now().time_since_epoch())
                           .count();
  diagnostic_reply_ = reply;
  for (int i = 0; i < 2; ++i)
    if (limits_.active[i]) {
      replies_[i].publish(reply);
      new_[i] = false;
    }
  return true;
}
bool SafetyCoordinator::accepts(int arm, uint64_t cycle, uint64_t started, uint64_t now,
                                const SafetyReply& reply) noexcept {
  if (stopped() != SafetyStop::none)
    return false;
  if (reply.cycles[arm] != cycle || reply.source_ns[arm] != started)
    return false;
  if (now < started || now - started > limits_.result_deadline_ns || reply.completed_ns < started ||
      reply.completed_ns > now) {
    stop(SafetyStop::result_deadline);
    return false;
  }
  return true;
}
}  // namespace safety_filter
