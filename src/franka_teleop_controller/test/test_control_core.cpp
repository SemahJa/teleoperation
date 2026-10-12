#include <gtest/gtest.h>
#include <atomic>
#include <cstring>
#include <limits>
#include <thread>
#include "franka_teleop_controller/control_core.hpp"
#include "franka_teleop_controller/handoff.hpp"
namespace ft = franka_teleop_controller;
namespace {
constexpr uint64_t now = 1000000000;
ft::Command command(uint64_t seq, bool enabled, bool released = false) {
  ft::Command c;
  std::strcpy(c.session.data(), "first");
  c.sequence = seq; c.source_ns = now + seq; c.receipt_ns = now + seq;
  c.valid = true; c.enabled = enabled; c.released = released;
  c.position << 0.4, 0, 0.4;
  return c;
}
ft::ArmState state() {
  ft::ArmState s;
  s.position << 0.4, 0, 0.4;
  s.jacobian.leftCols<6>().setIdentity();
  return s;
}
void arm(ft::ControlCore& core) {
  core.accept(command(1, false, true), now + 1);
  core.accept(command(2, true), now + 2);
}
}
TEST(Core, ReleaseIsRequiredAndNominalLawIsPreserved) {
  ft::ControlCore core(ft::Parameters{});
  auto s = state();
  core.accept(command(1, true), now + 1);
  EXPECT_FALSE(core.update(s, now + 1, .001).motion_enabled);
  core.accept(command(2, false, true), now + 2);
  auto c = command(3, true);
  c.position.x() += .01;
  c.twist.x() = .1;
  core.accept(c, now + 3);
  s.dq[0] = .02;
  s.coriolis[0] = .5;
  const auto out = core.update(s, now + 3, .001);
  ASSERT_TRUE(out.motion_enabled);
  // -20 * (.02 - .1 + 50 * -.01) + .5 = 12.1
  EXPECT_NEAR(out.nominal[0], 12.1, 1e-10);
  EXPECT_NEAR(out.torque[0], .9, 1e-10);
  EXPECT_TRUE(out.rate_limited);
}
TEST(Core, StaleTargetIsRemovedAndPacketsCannotResume) {
  ft::ControlCore core(ft::Parameters{});
  arm(core);
  auto s = state(); s.previous_torque[0] = 5;
  const auto out = core.update(s, now + 100000003, .001);
  EXPECT_FALSE(out.motion_enabled);
  EXPECT_EQ(out.reason, ft::Reason::stale);
  EXPECT_EQ(out.target_position, s.position);
  EXPECT_EQ(out.desired_twist, ft::Vector6::Zero());
  EXPECT_NEAR(out.torque[0], 4.1, 1e-10);
  auto c = command(3, true);
  c.source_ns = c.receipt_ns = now + 100000004;
  core.accept(c, c.receipt_ns);
  EXPECT_FALSE(core.update(s, c.receipt_ns, .001).motion_enabled);
  c.sequence++; c.source_ns++; c.receipt_ns++; c.enabled = false; c.released = true;
  core.accept(c, c.receipt_ns);
  c.sequence++; c.source_ns++; c.receipt_ns++; c.enabled = true; c.released = false;
  core.accept(c, c.receipt_ns);
  EXPECT_TRUE(core.update(s, c.receipt_ns, .001).motion_enabled);
}
TEST(Core, FaultDisarmIsNeverARelease) {
  ft::ControlCore core(ft::Parameters{});
  arm(core);
  auto c = command(3, false, true); c.valid = false;
  core.accept(c, now + 3);
  core.accept(command(4, true), now + 4);
  EXPECT_FALSE(core.update(state(), now + 4, .001).motion_enabled);
}
TEST(Core, SessionReferenceOrderingAndClockFaultsRequireRelease) {
  for (int scenario = 0; scenario < 7; ++scenario) {
    ft::ControlCore core(ft::Parameters{}); arm(core);
    auto c = command(3, true);
    if (scenario == 0) std::strcpy(c.session.data(), "second");
    if (scenario == 1) c.generation++;
    if (scenario == 2) c.sequence = 2;
    if (scenario == 3) c.source_ns = now + 2;
    if (scenario == 4) c.uncertainty_ns = 10000001;
    if (scenario == 5) c.source_ns = now - 100000001;
    if (scenario == 6) c.source_ns = now + 20000000;
    core.accept(c, now + 3);
    EXPECT_FALSE(core.update(state(), now + 3, .001).motion_enabled) << scenario;
  }
}
TEST(Core, ReleaseRepeatsDoNotRefreshSourceAge) {
  ft::ControlCore core(ft::Parameters{});
  auto c = command(1, false, true);
  core.accept(c, now + 1);
  c.receipt_ns = now + 90000000;
  core.accept(c, c.receipt_ns);
  auto enabled = command(2, true);
  enabled.source_ns = enabled.receipt_ns = now + 100000002;
  core.update(state(), enabled.receipt_ns, .001);
  core.accept(enabled, enabled.receipt_ns);
  EXPECT_FALSE(core.update(state(), enabled.receipt_ns, .001).motion_enabled);
}
TEST(Core, OuterEnvelopeAndNonFiniteTargetsDisarm) {
  for (bool nan : {false, true}) {
    ft::ControlCore core(ft::Parameters{}); arm(core);
    auto c = command(3, true);
    c.position.x() = nan ? std::numeric_limits<double>::quiet_NaN() : 10;
    core.accept(c, now + 3);
    auto out = core.update(state(), now + 3, .001);
    EXPECT_FALSE(out.motion_enabled);
    EXPECT_EQ(out.desired_twist, ft::Vector6::Zero());
  }
}
TEST(Core, TorqueCeilingsAndInvalidActuatorIntersection) {
  ft::ControlCore core(ft::Parameters{}); arm(core);
  auto s = state(); s.coriolis.setConstant(10000);
  s.previous_torque << 86.8, 86.8, 86.8, 86.8, 11.8, 11.8, 11.8;
  auto out = core.update(s, now + 2, .001);
  EXPECT_LE(out.torque[0], 87); EXPECT_LE(out.torque[6], 12);
  EXPECT_TRUE(out.magnitude_limited);
  s.previous_torque[0] = 100;
  EXPECT_EQ(core.update(s, now + 2, .001).reason, ft::Reason::invalid_state);
}
TEST(Core, QuaternionSignDoesNotChangeTorque) {
  ft::ControlCore first(ft::Parameters{}), second(ft::Parameters{});
  arm(first); arm(second);
  auto c = command(3, true);
  c.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(.1, Eigen::Vector3d::UnitZ()));
  first.accept(c, now + 3);
  c.orientation.coeffs() *= -1;
  second.accept(c, now + 3);
  EXPECT_TRUE(first.update(state(), now + 3, .001).nominal.isApprox(
      second.update(state(), now + 3, .001).nominal));
}
TEST(Core, ParametersRejectUnapprovedCeilingChangesAndNaNs) {
  ft::Parameters p; ASSERT_TRUE(p.valid());
  p.torque_rate = 1001; EXPECT_FALSE(p.valid());
  p = ft::Parameters{}; p.torque_limits[6] = 13; EXPECT_FALSE(p.valid());
  p = ft::Parameters{}; p.tool_offset = std::numeric_limits<double>::infinity(); EXPECT_FALSE(p.valid());
}
TEST(Core, NonFiniteNominalReportsDisarmedBeforeTermination) {
  ft::ControlCore core(ft::Parameters{}); arm(core);
  auto c = command(3, true); c.twist[0] = 1e308;
  core.accept(c, now + 3);
  const auto out = core.update(state(), now + 3, .001);
  EXPECT_EQ(out.reason, ft::Reason::invalid_state);
  EXPECT_FALSE(out.motion_enabled);
}
TEST(Handoff, QueueIsBoundedAndPreservesOrder) {
  ft::SpscQueue<int, 4> queue;
  ASSERT_TRUE(queue.push(1)); ASSERT_TRUE(queue.push(2)); ASSERT_TRUE(queue.push(3));
  EXPECT_FALSE(queue.push(4));
  int out;
  for (int expected = 1; expected < 4; ++expected) { ASSERT_TRUE(queue.pop(out)); EXPECT_EQ(out, expected); }
  EXPECT_FALSE(queue.pop(out));
}
TEST(Handoff, ConcurrentSamplesRemainAtomic) {
  using Sample = std::array<uint64_t, 16>;
  ft::LatestSnapshot<Sample> snapshot;
  std::atomic<bool> done{false};
  std::thread producer([&] {
    for (uint64_t n = 1; n <= 100000; ++n) { Sample s; s.fill(n); snapshot.publish(s); }
    done.store(true, std::memory_order_release);
  });
  uint64_t previous = 0;
  while (!done.load(std::memory_order_acquire)) {
    Sample s;
    if (snapshot.read(s)) {
      EXPECT_GT(s[0], previous); previous = s[0];
      for (auto value : s) EXPECT_EQ(value, s[0]);
    }
  }
  producer.join();
  Sample s;
  if (snapshot.read(s)) { for (auto value : s) EXPECT_EQ(value, uint64_t{100000}); }
}
