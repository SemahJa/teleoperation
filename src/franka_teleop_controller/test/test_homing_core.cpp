#include <gtest/gtest.h>
#include <cstring>
#include "franka_teleop_controller/homing_core.hpp"
namespace ft = franka_teleop_controller;
namespace {
struct Fixture {
  ft::HomingParameters p;
  ft::HomingCore core;
  ft::ArmState s;
  uint64_t now{1000000000}, sequence{0};
  ft::HomingInput last;
  explicit Fixture(ft::HomingParameters parameters = {}) : p(parameters), core(p) {
    s.q = Eigen::Map<const ft::Vector7>(ft::kHome.data());
    s.q[0] += .2; s.q[1] -= .1; s.q[5] += .2;
    std::strcpy(last.session.data(), "home-test");
  }
  void input(bool pressed, bool released = true, bool valid = true) {
    last.sequence = ++sequence; last.source_ns = last.receipt_ns = now;
    last.valid = valid; last.button_pressed = pressed; last.trigger_released = released;
    core.input(last, now);
  }
  ft::HomingOutput tick(bool follow = false) {
    now += 1000000;
    if (now % 10000000 == 0) input(false);
    const auto out = core.update(s, now, .001);
    if (follow && out.active) { s.q = out.desired_q; s.dq = out.desired_dq; }
    return out;
  }
  void request() {
    input(false);
    for (int n = 0; n <= 100; ++n) { now += 10000000; input(true); core.update(s, now, .01); }
    now += 10000000; input(false);
  }
};
}
TEST(Homing, StartupHeldAndShortHoldDoNotHome) {
  Fixture f;
  for (int n = 0; n < 110; ++n) { f.now += 10000000; f.input(true); EXPECT_FALSE(f.core.update(f.s, f.now, .01).active); }
  f.now += 10000000; f.input(false);
  for (int n = 0; n < 50; ++n) { f.now += 10000000; f.input(true); f.core.update(f.s, f.now, .01); }
  f.now += 10000000; f.input(false);
  EXPECT_EQ(f.core.update(f.s, f.now, .01).request_id, uint64_t{0});
}
TEST(Homing, LongHoldRequestsOnlyOnReleaseAndUsesFixedHome) {
  Fixture f; f.request();
  auto out = f.tick(true);
  ASSERT_EQ(out.phase, ft::HomePhase::homing);
  EXPECT_EQ(out.request_id, uint64_t{1});
  for (int n = 0; n < 10000 && out.active; ++n) out = f.tick(true);
  EXPECT_EQ(out.phase, ft::HomePhase::succeeded);
  EXPECT_TRUE(out.desired_q.isApprox(Eigen::Map<const ft::Vector7>(ft::kHome.data()), 1e-9));
  EXPECT_LT(out.desired_dq.norm(), 1e-9);
}
TEST(Homing, InvalidTrackingResetsGestureAndCannotResumeOnHeldButton) {
  Fixture f; f.input(false);
  for (int n = 0; n < 80; ++n) { f.now += 10000000; f.input(true); }
  f.now += 10000000; f.input(true, true, false);
  for (int n = 0; n < 150; ++n) { f.now += 10000000; f.input(true); }
  f.now += 10000000; f.input(false);
  EXPECT_FALSE(f.core.update(f.s, f.now, .01).active);
  EXPECT_EQ(f.core.update(f.s, f.now, .01).request_id, uint64_t{0});
}
TEST(Homing, TriggerSessionAndSourceFaultsCancelWithoutModeJump) {
  for (int fault = 0; fault < 4; ++fault) {
    Fixture f; f.request();
    auto previous = f.tick(true);
    for (int n = 0; n < 500; ++n) previous = f.tick(true);
    f.now += 1000000;
    if (fault == 0) f.input(false, false);
    if (fault == 1) { std::strcpy(f.last.session.data(), "second"); f.input(false); }
    if (fault == 2) { f.last.generation++; f.input(false); }
    if (fault == 3) { f.now += 101000000; }
    auto out = f.core.update(f.s, f.now, .001);
    EXPECT_EQ(out.phase, ft::HomePhase::stopping);
    EXPECT_LT((out.desired_q - previous.desired_q).norm(), .001);
    EXPECT_LT((out.desired_dq - previous.desired_dq).norm(), .01);
    ft::Vector7 last_velocity = out.desired_dq;
    f.s.q = out.desired_q; f.s.dq = out.desired_dq;
    for (int n = 0; n < 5000 && out.active; ++n) {
      out = f.tick(true);
      EXPECT_LE(((out.desired_dq - last_velocity) / .001).cwiseAbs().maxCoeff(), f.p.stop_acceleration + 1e-6);
      last_velocity = out.desired_dq;
    }
    EXPECT_EQ(out.phase, ft::HomePhase::cancelled);
    EXPECT_FALSE(out.stop_failed);
    EXPECT_LT(out.desired_dq.norm(), 1e-8);
  }
}
TEST(Homing, CancelTimeoutTerminatesAndCannotAutomaticallyRestart) {
  ft::HomingParameters p; p.stop_timeout = 1;
  Fixture f(p); ASSERT_TRUE(p.valid()); f.request();
  for (int n = 0; n < 1000; ++n) f.tick();
  f.core.cancel("test cancellation");
  ft::HomingOutput out;
  for (int n = 0; n < 1500; ++n) out = f.tick();
  EXPECT_EQ(out.phase, ft::HomePhase::failed);
  EXPECT_TRUE(out.stop_failed);
  f.request(); EXPECT_FALSE(f.tick().active);
}
TEST(Homing, CompletionTimeoutUsesControlledStop) {
  ft::HomingParameters p; p.completion_timeout = .2;
  Fixture f(p); f.request();
  ft::HomingOutput out;
  for (int n = 0; n < 250; ++n) out = f.tick(true);
  EXPECT_EQ(out.phase, ft::HomePhase::stopping);
  for (int n = 0; n < 1500 && out.active; ++n) out = f.tick(true);
  EXPECT_EQ(out.phase, ft::HomePhase::failed);
  EXPECT_FALSE(out.stop_failed);
}
TEST(Homing, NominalJointLawAndStartLimits) {
  Fixture f; f.s.dq[0] = .05; f.s.coriolis[0] = .5; f.request();
  const auto out = f.tick();
  EXPECT_NEAR(out.nominal[0], 300 * (out.desired_q[0] - f.s.q[0]) - 30 * .05 + .5, 1e-10);
  Fixture bad; bad.s.q[3] = 0; bad.request();
  EXPECT_EQ(bad.tick().phase, ft::HomePhase::failed);
}
TEST(Trajectory, AnalyticalVelocityMatchesPositionsAndFERBounds) {
  const ft::Vector7 home = Eigen::Map<const ft::Vector7>(ft::kHome.data());
  ft::Vector7 start = home; start[0] += .2; start[1] -= .1; start[5] += .2;
  ft::JointMotionGenerator trajectory(.15, start, home);
  ft::Vector7 previous = ft::Vector7::Zero();
  for (int n = 1; n < 10000; ++n) {
    const double t = n * .001;
    const auto velocity = trajectory.getDesiredJointVelocities(t);
    const ft::Vector7 numeric = (trajectory.getDesiredJointPositions(t + 1e-6).first -
                          trajectory.getDesiredJointPositions(t - 1e-6).first) / 2e-6;
    EXPECT_TRUE(velocity.isApprox(numeric, 1e-5) || (velocity - numeric).norm() < 1e-8);
    for (int j = 0; j < 7; ++j) EXPECT_LT(std::abs(velocity[j]), ft::kJointVelocity[j]);
    EXPECT_LE(((velocity - previous) / .001).cwiseAbs().maxCoeff(), .75 + 1e-6);
    previous = velocity;
    if (trajectory.getDesiredJointPositions(t).second) break;
  }
}
