#include <gtest/gtest.h>
#include <chrono>
#include "safety_filter/safety_coordinator.hpp"
#include "safety_filter/panda_model_fixture.hpp"
#include "safety_filter/panda_limits.hpp"
namespace ft = safety_filter;
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
  r.state=ft::panda_model_fixture(Eigen::Map<const ft::cbf::Vector7d>(ft::kHome.data()));
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
  const auto& diagnostic = c.diagnostic_reply();
  EXPECT_EQ(diagnostic.cycles[0],1u);
  EXPECT_GT(diagnostic.started_ns,0u);
  EXPECT_GE(diagnostic.completed_ns,diagnostic.started_ns);
  EXPECT_GT(diagnostic.result.filter_time_seconds,0.);
  ft::SafetyCoordinator other; ft::cbf::SafetyParameters safety; std::string error;
  EXPECT_FALSE(other.configure(p,safety,ids(),&error)); // Inconsistent active mask.
  safety.active_arms=p.active; safety.enabled=false;
  EXPECT_TRUE(other.configure(p,safety,ids(),&error)); // Explicit diagnostic mode.
}

TEST(Coordinator, DisabledCbfRetainsActuatorBoundsAndInvalidModelStop) {
  ft::SafetyCoordinator c; ft::CoordinatorParameters p;
  p.filter_deadline_ns=100000000; p.result_deadline_ns=200000000;
  ft::cbf::SafetyParameters s; s.active_arms=p.active; s.enabled=false;
  s.table_height=2.; // Infeasible geometry is intentionally not a barrier.
  std::string error; ASSERT_TRUE(c.configure(p,s,ids(),&error))<<error;
  auto r=request(now()); r.nominal.setConstant(100.); c.submit(0,r);
  ASSERT_TRUE(c.pump()); ft::SafetyReply reply; ASSERT_TRUE(c.receive(0,reply));
  EXPECT_STREQ(reply.result.solver_status,"disabled_actuator_bounds_only");
  EXPECT_TRUE(reply.result.solver_succeeded); EXPECT_FALSE(reply.result.fallback_active);
  EXPECT_EQ(reply.result.solver_iterations,0);
  EXPECT_LE(reply.result.safe_torque.head<7>().cwiseAbs().maxCoeff(),.9);
  EXPECT_TRUE(reply.result.safe_torque.tail<7>().isZero());
  EXPECT_TRUE(c.accepts(0,1,r.stamp_ns,now(),reply));
  r=request(now(),2); r.state.mass(0,0)=-1.; c.submit(0,r);
  ASSERT_TRUE(c.pump()); EXPECT_EQ(c.stopped(),ft::SafetyStop::invalid_model);
}

TEST(Coordinator, LateResultTimingIsPreservedWithoutRelaxingDeadline) {
  ft::SafetyCoordinator c; ft::CoordinatorParameters p;
  p.filter_deadline_ns=100000000; p.result_deadline_ns=200000000;
  setup(c,p);
  const auto r=request(now(),42); c.submit(0,r);
  ASSERT_TRUE(c.pump()); ft::SafetyReply reply; ASSERT_TRUE(c.receive(0,reply));
  ASSERT_GE(reply.started_ns,r.stamp_ns);
  ASSERT_GE(reply.completed_ns,reply.started_ns);
  EXPECT_FALSE(c.accepts(0,42,r.stamp_ns,r.stamp_ns+p.result_deadline_ns+1,reply));
  ASSERT_EQ(c.stopped(),ft::SafetyStop::result_deadline);
  const auto& diagnostic=c.diagnostic_reply();
  EXPECT_EQ(diagnostic.cycles[0],42u);
  EXPECT_EQ(diagnostic.source_ns[0],r.stamp_ns);
  EXPECT_EQ(diagnostic.started_ns,reply.started_ns);
  EXPECT_EQ(diagnostic.completed_ns,reply.completed_ns);
  c.submit(0,request(now(),43)); EXPECT_FALSE(c.pump());
  EXPECT_EQ(c.diagnostic_reply().cycles[0],42u);
}

TEST(Coordinator, InvalidModelSnapshotRemainsAvailableForPostmortem) {
  ft::SafetyCoordinator c; ft::CoordinatorParameters p;
  p.filter_deadline_ns=100000000; p.result_deadline_ns=200000000;
  setup(c,p);
  auto r=request(now(),42); r.state.mass(0,0)=-1;
  c.submit(0,r); ASSERT_TRUE(c.pump(now()));
  ASSERT_EQ(c.stopped(),ft::SafetyStop::invalid_model);
  // No pump thread is running in this test, so postmortem reads are safe.
  EXPECT_STREQ(c.diagnostic_filter_status(), "invalid_model_state_or_actuator_bounds");
  EXPECT_EQ(c.diagnostic_request(0).cycle,42u);
  EXPECT_EQ(c.diagnostic_request(0).state.mass(0,0),-1);
  c.submit(0,request(now(),43)); EXPECT_FALSE(c.pump(now()));
  EXPECT_EQ(c.diagnostic_request(0).cycle,42u);
}
