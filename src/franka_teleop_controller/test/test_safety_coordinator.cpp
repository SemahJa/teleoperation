#include <gtest/gtest.h>
#include <chrono>
#include "franka_teleop_controller/safety_coordinator.hpp"
#include "franka_teleop_controller/panda_model_fixture.hpp"
#include "franka_teleop_controller/panda_limits.hpp"
namespace ft = franka_teleop_controller;
namespace {
uint64_t now() { return std::chrono::duration_cast<std::chrono::nanoseconds>(
  std::chrono::steady_clock::now().time_since_epoch()).count(); }
std::array<ft::cbf::ArmIdentity, 2> ids() {
  std::array<ft::cbf::ArmIdentity, 2> out;
  out[0].arm_id="left"; out[0].base_frame="left/base";
  out[1].arm_id="right"; out[1].base_frame="right/base";
  out[1].world_from_base.translation().x()=2;
  return out;
}
ft::ModelRequest request(uint64_t stamp, uint64_t cycle=1) {
  ft::ModelRequest r; r.cycle=cycle; r.stamp_ns=stamp;
  r.state=ft::panda_model_fixture(Eigen::Map<const ft::Vector7>(ft::kHome.data()));
  return r;
}
void setup(ft::SafetyCoordinator& c, ft::CoordinatorParameters p={}, bool infeasible=false) {
  ft::cbf::SafetyParameters safety; safety.active_arms=p.active;
  if (infeasible) safety.table_height=2;
  std::string error; ASSERT_TRUE(c.configure(p,safety,ids(),&error))<<error;
}
}
TEST(Coordinator, ActualPandaFixtureFiltersAndOwnCycleOnly) {
  ft::SafetyCoordinator c; ft::CoordinatorParameters p;
  p.filter_deadline_ns=100000000; p.result_deadline_ns=200000000;
  setup(c,p); auto r=request(now()); r.nominal[0]=2; c.submit(0,r);
  ASSERT_TRUE(c.pump(now())); ft::SafetyReply reply; ASSERT_TRUE(c.receive(0,reply));
  ASSERT_TRUE(reply.result.solver_succeeded)<<reply.result.solver_status;
  EXPECT_GT(reply.result.self_collision_constraints,0);
  EXPECT_EQ(c.stopped(),ft::SafetyStop::none);
  EXPECT_FALSE(c.accepts(0,2,r.stamp_ns,now(),reply));
  EXPECT_FALSE(c.accepts(0,1,r.stamp_ns+1,now(),reply));
  EXPECT_TRUE(c.accepts(0,1,r.stamp_ns,now(),reply));
  EXPECT_LE(std::abs(reply.result.safe_torque[0]),.9);
}
TEST(Coordinator, DualRequiresBothFreshAndProducesOneCoupledResult) {
  ft::SafetyCoordinator c; ft::CoordinatorParameters p; p.active={true,true};
  p.filter_deadline_ns=100000000; p.result_deadline_ns=200000000;
  setup(c,p); const auto stamp=now(); c.submit(0,request(stamp));
  EXPECT_FALSE(c.pump(now())); ft::SafetyReply left,right; EXPECT_FALSE(c.receive(0,left));
  c.submit(1,request(stamp)); ASSERT_TRUE(c.pump(now()));
  ASSERT_TRUE(c.receive(0,left)); ASSERT_TRUE(c.receive(1,right));
  EXPECT_EQ(left.cycles,right.cycles); EXPECT_EQ(left.completed_ns,right.completed_ns);
  EXPECT_EQ(left.source_ns,right.source_ns);
  EXPECT_TRUE(left.result.solver_succeeded)<<left.result.solver_status;
  EXPECT_GT(left.result.minimum_inter_arm_clearance,1.);
  // A new left state alone cannot reuse the old right state or finish a pair.
  auto next=request(now(),2); next.finish_requested=true; c.submit(0,next);
  EXPECT_FALSE(c.pump(now())); EXPECT_FALSE(c.receive(0,left));
  next.finish_requested=false; c.submit(1,next); ASSERT_TRUE(c.pump(now()));
  ASSERT_TRUE(c.receive(0,left)); EXPECT_FALSE(left.finish);
  next=request(now(),3); next.finish_requested=true; c.submit(0,next); c.submit(1,next);
  ASSERT_TRUE(c.pump(now())); ASSERT_TRUE(c.receive(0,left)); ASSERT_TRUE(c.receive(1,right));
  EXPECT_TRUE(left.finish); EXPECT_TRUE(right.finish);
}
TEST(Coordinator, AgeSkewAndResultDeadlineLatchNoAutomaticRecovery) {
  for (int scenario=0; scenario<3; ++scenario) {
    ft::SafetyCoordinator c; ft::CoordinatorParameters p; p.active={true,true};
    p.filter_deadline_ns=100000000; p.result_deadline_ns=200000000; setup(c,p);
    auto stamp=now(); c.submit(0,request(stamp));
    c.submit(1,request(scenario==1?stamp-1500000:stamp));
    if (scenario==0) EXPECT_FALSE(c.pump(stamp+3000000));
    else if (scenario==1) EXPECT_FALSE(c.pump(stamp));
    else {
      ASSERT_TRUE(c.pump(now())); ft::SafetyReply reply; ASSERT_TRUE(c.receive(0,reply));
      EXPECT_FALSE(c.accepts(0,1,stamp,stamp+p.result_deadline_ns+1,reply));
    }
    EXPECT_EQ(c.stopped(),scenario==0?ft::SafetyStop::state_age:
      scenario==1?ft::SafetyStop::state_skew:ft::SafetyStop::result_deadline);
    c.submit(0,request(now(),2)); c.submit(1,request(now(),2)); EXPECT_FALSE(c.pump(now()));
    c.stop(ft::SafetyStop::owner_failure); // Preserve first cause.
    EXPECT_NE(c.stopped(),ft::SafetyStop::owner_failure);
  }
}
TEST(Coordinator, InvalidModelAndRepeatedInfeasibilityTerminate) {
  for (int scenario=0;scenario<2;++scenario) {
    ft::SafetyCoordinator c; ft::CoordinatorParameters p; p.maximum_failures=3;
    p.filter_deadline_ns=100000000; p.result_deadline_ns=200000000; setup(c,p,scenario==1);
    for (int n=1;n<=3;++n) {
      auto r=request(now(),n); if (scenario==0) r.state.mass(0,0)=-1;
      c.submit(0,r); ASSERT_TRUE(c.pump(now())); ft::SafetyReply reply; ASSERT_TRUE(c.receive(0,reply));
      if (scenario==0) { EXPECT_TRUE(reply.result.fatal_input_fault); break; }
      EXPECT_TRUE(reply.result.fallback_active); EXPECT_EQ(reply.consecutive_failures,unsigned(n));
      EXPECT_EQ(c.stopped(),n<3?ft::SafetyStop::none:ft::SafetyStop::solver_failures);
    }
    EXPECT_EQ(c.stopped(),scenario==0?ft::SafetyStop::invalid_model:ft::SafetyStop::solver_failures);
    EXPECT_FALSE(c.pump(now()));
  }
}
TEST(Coordinator, FilterDeadlineAndConfigurationGuard) {
  ft::SafetyCoordinator c; ft::CoordinatorParameters p; p.filter_deadline_ns=1;
  setup(c,p); c.submit(0,request(now())); ASSERT_TRUE(c.pump(now()));
  EXPECT_EQ(c.stopped(),ft::SafetyStop::filter_deadline);
  ft::SafetyCoordinator other; ft::cbf::SafetyParameters safety; std::string error;
  EXPECT_FALSE(other.configure(p,safety,ids(),&error)); // Inconsistent active mask.
  safety.active_arms=p.active; safety.enabled=false;
  EXPECT_FALSE(other.configure(p,safety,ids(),&error));
}
