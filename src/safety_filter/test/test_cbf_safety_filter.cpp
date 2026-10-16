#include <gtest/gtest.h>

#include <cstdlib>
#include <limits>
#include <new>

#include "safety_filter/cbf_safety_filter.hpp"
#include "safety_filter/legacy_pose.hpp"
#include "safety_filter/panda_limits.hpp"
#include "safety_filter/panda_model_fixture.hpp"
namespace cbf = safety_filter::cbf;
namespace {
thread_local bool track_allocations = false;
thread_local size_t allocations = 0;
}  // namespace
extern "C" void* __real_malloc(size_t);
extern "C" void* __real_calloc(size_t, size_t);
extern "C" void* __real_realloc(void*, size_t);
extern "C" void* __wrap_malloc(size_t n) {
  if (track_allocations)
    ++allocations;
  return __real_malloc(n);
}
extern "C" void* __wrap_calloc(size_t n, size_t s) {
  if (track_allocations)
    ++allocations;
  return __real_calloc(n, s);
}
extern "C" void* __wrap_realloc(void* p, size_t n) {
  if (track_allocations)
    ++allocations;
  return __real_realloc(p, n);
}
void* operator new(size_t n) {
  if (track_allocations)
    ++allocations;
  if (auto p = std::malloc(n))
    return p;
  throw std::bad_alloc();
}
void operator delete(void* p) noexcept { std::free(p); }
void operator delete(void* p, size_t) noexcept { std::free(p); }

namespace {
std::array<cbf::ArmIdentity, 2> identities() {
  std::array<cbf::ArmIdentity, 2> ids;
  ids[0].arm_id = "left";
  ids[0].base_frame = "left/base";
  ids[1].arm_id = "right";
  ids[1].base_frame = "right/base";
  ids[1].world_from_base.translation().x() = 2;
  return ids;
}
cbf::SafetyParameters parameters() {
  cbf::SafetyParameters p;
  p.table_enabled = p.self_collision_enabled = p.inter_arm_collision_enabled = false;
  // Explicit historical bounds for the numerical barrier fixtures below.
  // Separate tests cover the new 35%-of-FCI operational defaults.
  p.maximum_joint_torque << 80, 80, 80, 80, 10, 10, 10;
  p.maximum_torque_rate = 900;
  return p;
}
cbf::DualArmState state() {
  cbf::DualArmState s;
  for (auto& arm : s.arm)
    arm.q = Eigen::Map<const cbf::Vector7d>(safety_filter::kHome.data());
  return s;
}
void configure(cbf::CbfSafetyFilter& filter, const cbf::SafetyParameters& p,
               const std::array<cbf::ArmIdentity, 2>& ids = identities()) {
  std::string error;
  ASSERT_TRUE(filter.configure(p, ids, &error)) << error;
}
}  // namespace
TEST(CBF, PandaDefaultsAndLimitsValidate) {
  cbf::CbfSafetyFilter filter;
  std::string error;
  ASSERT_TRUE(filter.configure(cbf::SafetyParameters{}, identities(), &error)) << error;
  auto p = parameters();
  p.maximum_joint_velocity[0] = 2.62;
  EXPECT_FALSE(filter.configure(p, identities(), &error));
  p = parameters();
  p.maximum_joint_acceleration[1] = 8;
  EXPECT_FALSE(filter.configure(p, identities(), &error));
  p = parameters();
  p.maximum_joint_torque[6] = 13;
  EXPECT_FALSE(filter.configure(p, identities(), &error));
  p = parameters();
  p.minimum_joint_position[0] = -2.9;
  EXPECT_FALSE(filter.configure(p, identities(), &error));
  auto ids = identities();
  ids[0].world_from_base.linear()(0, 0) = 2;
  EXPECT_FALSE(filter.configure(parameters(), ids, &error));
}
TEST(ActuatorLimits, DefaultsUsePublishedFciCeilingsAndScalesAreValidated) {
  const cbf::SafetyParameters defaults;
  EXPECT_TRUE(defaults.maximum_joint_torque.isApprox(
      (cbf::Vector7d() << 30.45, 30.45, 30.45, 30.45, 4.2, 4.2, 4.2).finished()));
  EXPECT_DOUBLE_EQ(defaults.maximum_torque_rate, 350);
  EXPECT_TRUE(safety_filter::panda_torque_limits(.5).isApprox(
      (cbf::Vector7d() << 43.5, 43.5, 43.5, 43.5, 6, 6, 6).finished()));
  EXPECT_DOUBLE_EQ(safety_filter::panda_torque_rate_limit(.2), 200);
  safety_filter::ActuatorLimitScales scales;
  EXPECT_TRUE(scales.valid());
  for (double invalid : {0., -1., 1.01, std::numeric_limits<double>::infinity(),
                         std::numeric_limits<double>::quiet_NaN()}) {
    scales = {};
    scales.torque = invalid;
    EXPECT_FALSE(scales.valid());
    scales = {};
    scales.torque_rate = invalid;
    EXPECT_FALSE(scales.valid());
  }
}

TEST(ActuatorLimits, ThirtyFivePercentMagnitudeAndRateBoundsApplyWithCbfOnAndOff) {
  for (bool enabled : {false, true}) {
    cbf::SafetyParameters p;
    p.enabled = enabled;
    p.joint_position_enabled = p.joint_velocity_enabled = p.joint_acceleration_enabled = false;
    p.table_enabled = p.self_collision_enabled = p.inter_arm_collision_enabled = false;
    p.solver_max_iterations = 1000;
    cbf::CbfSafetyFilter filter;
    configure(filter, p);
    auto s = state();
    for (double sign : {-1., 1.}) {
      for (auto& arm : s.arm)
        arm.previous_torque.setZero();
      auto out = filter.filter(cbf::Vector14d::Constant(sign * 100), s, .001, {true, true});
      ASSERT_TRUE(out.solver_succeeded) << out.solver_status;
      EXPECT_FALSE(out.fallback_active);
      EXPECT_LE(out.safe_torque.cwiseAbs().maxCoeff(), .35 + 1e-12);
      EXPECT_NEAR(out.safe_torque[0], sign * .35, 1e-3);
      for (auto& arm : s.arm)
        arm.previous_torque = sign * (p.maximum_joint_torque.array() - .1).matrix();
      out = filter.filter(cbf::Vector14d::Constant(sign * 100), s, .001, {true, true});
      ASSERT_TRUE(out.solver_succeeded) << out.solver_status;
      for (int arm = 0; arm < 2; ++arm)
        for (int j = 0; j < 7; ++j) {
          EXPECT_LE(std::abs(out.safe_torque[7 * arm + j]), p.maximum_joint_torque[j] + 1e-12);
          EXPECT_NEAR(out.safe_torque[7 * arm + j], sign * p.maximum_joint_torque[j], 1e-3);
        }
    }
  }
}
TEST(CBF, InteriorNominalAndActualTorqueRateAnchor) {
  cbf::CbfSafetyFilter filter;
  configure(filter, parameters());
  auto s = state();
  cbf::Vector14d nominal = cbf::Vector14d::Zero();
  auto out = filter.filter(nominal, s, .001, {false, false});
  ASSERT_TRUE(out.solver_succeeded) << out.solver_status;
  EXPECT_LT(out.safe_torque.norm(), 1e-8);
  nominal[0] = .5;
  out = filter.filter(nominal, s, .001, {true, false});
  ASSERT_TRUE(out.solver_succeeded) << out.solver_status;
  EXPECT_NEAR(out.safe_torque[0], .5, 1e-3);
  s.arm[0].previous_torque[0] = 4;
  nominal[0] = 10;
  out = filter.filter(nominal, s, .001, {true, false});
  ASSERT_TRUE(out.solver_succeeded) << out.solver_status;
  EXPECT_LE(out.safe_torque[0], 4.9);
  EXPECT_GE(out.safe_torque[0], 3.1);
  EXPECT_LE(out.maximum_constraint_violation, 1e-3);
}
TEST(CBF, JointPositionBarrierRejectsOutwardNominal) {
  cbf::CbfSafetyFilter filter;
  configure(filter, parameters());
  auto s = state();
  s.arm[0].q[0] = 2.8973 - .1 + .01;
  cbf::Vector14d nominal = cbf::Vector14d::Zero();
  nominal[0] = 2;
  auto out = filter.filter(nominal, s, .001, {true, false});
  ASSERT_TRUE(out.solver_succeeded) << out.solver_status;
  EXPECT_LE(out.safe_torque[0], -.64 + 1e-3);
  EXPECT_EQ(out.joint_position_constraints, 28);
}
TEST(CBF, VelocityAndAccelerationRowsFilterNominal) {
  cbf::CbfSafetyFilter filter;
  configure(filter, parameters());
  auto s = state();
  s.arm[0].dq[0] = 2.175 * .9 + .01;
  cbf::Vector14d nominal = cbf::Vector14d::Zero();
  nominal[0] = .5;
  auto out = filter.filter(nominal, s, .001, {true, false});
  ASSERT_TRUE(out.solver_succeeded) << out.solver_status;
  EXPECT_LE(out.safe_torque[0], -.1 + 1e-3);
  EXPECT_EQ(out.joint_velocity_constraints, 28);
  s.arm[0].dq.setZero();
  s.arm[0].previous_torque[0] = 6.8;
  nominal[0] = 20;
  out = filter.filter(nominal, s, .001, {true, false});
  ASSERT_TRUE(out.solver_succeeded) << out.solver_status;
  EXPECT_LE(out.safe_torque[0], 7 + 1e-3);
  EXPECT_EQ(out.joint_acceleration_constraints, 14);
}
TEST(CBF, SingleArmHasNoDummyArmBarriersOrTorques) {
  cbf::CbfSafetyFilter filter;
  auto p = parameters();
  p.active_arms = {true, false};
  p.inter_arm_collision_enabled = true;
  configure(filter, p);
  auto s = state();
  s.arm[1].mass.setConstant(std::numeric_limits<double>::quiet_NaN());
  cbf::Vector14d nominal = cbf::Vector14d::Zero();
  nominal.tail<7>().setConstant(99);
  const auto out = filter.filter(nominal, s, .001, {false, false});
  ASSERT_TRUE(out.solver_succeeded) << out.solver_status;
  EXPECT_EQ(out.joint_position_constraints, 14);
  EXPECT_EQ(out.inter_arm_constraints, 0);
  EXPECT_LT(out.safe_torque.tail<7>().norm(), 1e-8);
}
TEST(CBF, InvalidModelAndEmptyActuatorIntervalsAreFatal) {
  for (int scenario = 0; scenario < 5; ++scenario) {
    cbf::CbfSafetyFilter filter;
    configure(filter, parameters());
    auto s = state();
    if (scenario == 0)
      s.arm[0].mass(0, 0) = -1;
    if (scenario == 1)
      s.arm[0].q[0] = std::numeric_limits<double>::quiet_NaN();
    if (scenario == 2)
      s.arm[0].previous_torque[0] = 100;
    if (scenario == 3)
      s.arm[0].frame_jacobian[0](0, 0) = std::numeric_limits<double>::infinity();
    if (scenario == 4)
      s.arm[0].frame_pose[0].matrix()(3, 3) = 2;
    const auto out = filter.filter(cbf::Vector14d::Zero(), s, .001, {false, false});
    EXPECT_TRUE(out.fatal_input_fault);
    EXPECT_FALSE(out.solver_succeeded);
  }
}
TEST(CBF, InfeasibleTableHasBoundedFallbackAndReportsViolation) {
  cbf::CbfSafetyFilter filter;
  auto p = parameters();
  p.table_enabled = true;
  p.active_arms = {true, false};
  p.table_height = 2;
  configure(filter, p);
  auto s = state();
  s.arm[0].previous_torque.setConstant(.2);
  const auto out = filter.filter(cbf::Vector14d::Constant(.5), s, .001, {false, false});
  EXPECT_TRUE(out.fallback_active);
  EXPECT_FALSE(out.solver_succeeded);
  EXPECT_GT(out.table_constraints, 0);
  EXPECT_GT(out.maximum_constraint_violation, 0);
  EXPECT_TRUE(out.safe_torque.allFinite());
  EXPECT_LE((out.safe_torque.head<7>() - s.arm[0].previous_torque).cwiseAbs().maxCoeff(), .9);
}
TEST(CBF, DualArmCollisionConstraintCouplesBothTorques) {
  cbf::CbfSafetyFilter filter;
  auto p = parameters();
  p.inter_arm_collision_enabled = true;
  p.solver_max_iterations = 1000;
  auto ids = identities();
  ids[1].world_from_base.translation().x() = .155;
  configure(filter, p, ids);
  auto s = state();
  for (int arm = 0; arm < 2; ++arm)
    for (int frame = 0; frame < 9; ++frame) {
      s.arm[arm].frame_pose[frame].translation() << arm * 5., frame * 3., 3.;
      if (frame == 2) {
        s.arm[arm].frame_pose[frame].translation().x() = 0;
        s.arm[arm].frame_jacobian[frame](0, 0) = 1;
      }
    }
  const auto out = filter.filter(cbf::Vector14d::Zero(), s, .001, {false, false});
  ASSERT_TRUE(out.solver_succeeded) << out.solver_status;
  EXPECT_GT(out.inter_arm_constraints, 0);
  EXPECT_LT(out.safe_torque[0], -.4);
  EXPECT_GT(out.safe_torque[7], .4);
  EXPECT_LE(out.maximum_constraint_violation, 1e-3);
}
TEST(CBF, FilterAndSolverDoNotAllocateAfterConfigure) {
  cbf::CbfSafetyFilter filter;
  configure(filter, parameters());
  auto s = state();
  cbf::SafetyResult out;
  allocations = 0;
  track_allocations = true;
  for (int n = 0; n < 20; ++n)
    out = filter.filter(cbf::Vector14d::Zero(), s, .001, {false, false});
  track_allocations = false;
  EXPECT_EQ(allocations, size_t{0});
  EXPECT_TRUE(out.solver_succeeded);
  EXPECT_GT(out.filter_time_seconds, 0);
}

TEST(CBF, LegacyRoundedEndEffectorRotationPassesOnlyAfterBoundedConversion) {
  cbf::CbfSafetyFilter filter;
  auto p = parameters();
  p.active_arms = {true, false};
  configure(filter, p);
  auto s = state();
  Eigen::Affine3d rounded = Eigen::Affine3d::Identity();
  const double c = static_cast<float>(0.7071);
  rounded.linear() << c, c, 0, -c, c, 0, 0, 0, 1;
  rounded.translation() << .1, .2, .1034;
  // Exercise an arbitrary base/flange rotation as in the hardware report.
  rounded = Eigen::AngleAxisd(.7, Eigen::Vector3d(1, 2, 3).normalized()) * rounded;
  s.arm[0].frame_pose[8] = rounded;
  auto out = filter.filter(cbf::Vector14d::Zero(), s, .001, {false, false});
  ASSERT_TRUE(out.fatal_input_fault);
  const Eigen::Vector3d position = rounded.translation();
  ASSERT_TRUE(safety_filter::orthonormalize_legacy_pose(rounded));
  EXPECT_TRUE(rounded.translation().isApprox(position, 1e-14));
  EXPECT_NEAR(rounded.linear().determinant(), 1., 1e-12);
  s.arm[0].frame_pose[8] = rounded;
  out = filter.filter(cbf::Vector14d::Zero(), s, .001, {false, false});
  EXPECT_FALSE(out.fatal_input_fault);
  EXPECT_TRUE(out.solver_succeeded) << out.solver_status;
}

TEST(CBF, LegacyPoseConversionRejectsMalformedAndDistortedTransforms) {
  for (int scenario = 0; scenario < 7; ++scenario) {
    Eigen::Affine3d pose = Eigen::Affine3d::Identity();
    if (scenario == 0)
      pose.linear()(0, 0) = 1.001;
    if (scenario == 1)
      pose.linear()(0, 0) = -1.;
    if (scenario == 2)
      pose.linear()(0, 1) = .001;
    if (scenario == 3)
      pose.matrix()(3, 3) = 2.;
    if (scenario == 4)
      pose.translation().x() = std::numeric_limits<double>::quiet_NaN();
    if (scenario == 5)
      pose.linear()(0, 0) = 0.;
    if (scenario == 6)
      pose.linear()(0, 0) = std::numeric_limits<double>::infinity();
    EXPECT_FALSE(safety_filter::orthonormalize_legacy_pose(pose)) << scenario;
    cbf::CbfSafetyFilter filter;
    auto p = parameters();
    p.active_arms = {true, false};
    configure(filter, p);
    auto s = state();
    s.arm[0].frame_pose[8] = pose;
    EXPECT_TRUE(filter.filter(cbf::Vector14d::Zero(), s, .001, {false, false}).fatal_input_fault)
        << scenario;
  }
}

TEST(CBF, LegacyPoseConversionDoesNotAllocate) {
  Eigen::Affine3d input = Eigen::Affine3d::Identity();
  input.linear() << .7071, .7071, 0, -.7071, .7071, 0, 0, 0, 1;
  bool valid = true;
  allocations = 0;
  track_allocations = true;
  for (int n = 0; n < 20; ++n) {
    Eigen::Affine3d pose = input;
    valid &= safety_filter::orthonormalize_legacy_pose(pose);
  }
  track_allocations = false;
  EXPECT_TRUE(valid);
  EXPECT_EQ(allocations, size_t{0});
}

TEST(CBF, SparsePatternRetainsEitherSingleArmConstraintsWithoutAllocating) {
  for (int active = 0; active < 2; ++active) {
    cbf::SafetyParameters p;
    p.active_arms = {active == 0, active == 1};
    p.table_height = 2.;  // Deliberately infeasible: retain table rows and fallback.
    p.obstacle_collision_enabled = true;
    p.obstacle_centers = {0., 0., .4};
    p.obstacle_radii = {.2};
    cbf::CbfSafetyFilter filter;
    auto ids = identities();
    ids[1].world_from_base.setIdentity();
    configure(filter, p, ids);
    cbf::DualArmState s;
    s.arm[active] = safety_filter::panda_model_fixture(
        Eigen::Map<const cbf::Vector7d>(safety_filter::kHome.data()));
    auto out = filter.filter(cbf::Vector14d::Constant(20), s, .001, {false, false});
    allocations = 0;
    track_allocations = true;
    for (int n = 0; n < 20; ++n)
      out = filter.filter(cbf::Vector14d::Constant(20), s, .001, {false, false});
    track_allocations = false;
    EXPECT_EQ(allocations, size_t{0});
    EXPECT_FALSE(out.fatal_input_fault);
    EXPECT_TRUE(out.fallback_active);
    EXPECT_EQ(out.joint_position_constraints, 14);
    EXPECT_EQ(out.joint_velocity_constraints, 14);
    EXPECT_EQ(out.joint_acceleration_constraints, 7);
    EXPECT_GT(out.table_constraints, 0);
    EXPECT_GT(out.obstacle_constraints, 0);
    EXPECT_EQ(out.inter_arm_constraints, 0);
    EXPECT_TRUE(out.safe_torque.segment<7>((1 - active) * 7).isZero(0));
  }
}

TEST(CBF, CompactRowsRemoveOnlyPermanentlyDisabledConstraints) {
  cbf::CbfSafetyFilter filter;
  cbf::SafetyParameters p;
  configure(filter, p);
  const auto dual_rows = filter.solverConstraintCount();
  p.active_arms = {true, false};
  configure(filter, p);
  const auto single_rows = filter.solverConstraintCount();
  EXPECT_LT(single_rows, dual_rows);
  p.joint_position_enabled = p.joint_velocity_enabled = p.joint_acceleration_enabled = false;
  p.table_enabled = p.self_collision_enabled = p.inter_arm_collision_enabled = false;
  configure(filter, p);
  EXPECT_EQ(filter.solverConstraintCount(), 14u);  // Both actuator lanes remain.

  // Collision activation changes numeric bounds, never solver row mapping.
  p.table_enabled = true;
  p.table_height = 0.;
  configure(filter, p);
  const auto table_rows = filter.solverConstraintCount();
  EXPECT_GT(table_rows, 14u);
  auto s = state();
  for (auto& pose : s.arm[0].frame_pose)
    pose.translation().z() = 5.;
  auto clear = filter.filter(cbf::Vector14d::Zero(), s, .001, {false, false});
  EXPECT_EQ(clear.table_constraints, 0);
  EXPECT_TRUE(clear.solver_succeeded);
  for (auto& pose : s.arm[0].frame_pose)
    pose.translation().z() = -5.;
  auto blocked = filter.filter(cbf::Vector14d::Zero(), s, .001, {false, false});
  EXPECT_GT(blocked.table_constraints, 0);
  EXPECT_TRUE(blocked.fallback_active);
  EXPECT_GT(blocked.maximum_constraint_violation, 0.);
  EXPECT_EQ(filter.solverConstraintCount(), table_rows);
}

TEST(CBF, FeasibleNominalIsExactOptimumAndViolatingNominalStillUsesSolver) {
  cbf::CbfSafetyFilter filter;
  auto p = parameters();
  p.active_arms = {true, false};
  configure(filter, p);
  auto s = state();
  cbf::Vector14d nominal = cbf::Vector14d::Zero();
  nominal[0] = .2;
  auto out = filter.filter(nominal, s, .001, {false, false});
  ASSERT_TRUE(out.solver_succeeded);
  EXPECT_STREQ(out.solver_status, "feasible_nominal");
  EXPECT_EQ(out.solver_iterations, 0);
  EXPECT_EQ(out.maximum_constraint_violation, 0.);
  EXPECT_TRUE(out.safe_torque.isApprox(nominal, 0.));
  // Even a violation below solver feasibility tolerance must not take the
  // exact-optimum path. The normal solver retains its independent acceptance.
  nominal[0] = .900001;
  out = filter.filter(nominal, s, .001, {false, false});
  EXPECT_STRNE(out.solver_status, "feasible_nominal");
  EXPECT_GT(out.solver_iterations, 0);
  EXPECT_LE(out.safe_torque[0], .9);
  nominal.setZero();
  nominal[6] = 10.;
  out = filter.filter(nominal, s, .001, {false, false});
  EXPECT_STRNE(out.solver_status, "feasible_nominal");
  EXPECT_LE(out.safe_torque[6], .9);
  nominal.setZero();
  allocations = 0;
  track_allocations = true;
  for (int n = 0; n < 20; ++n)
    out = filter.filter(nominal, s, .001, {false, false});
  track_allocations = false;
  EXPECT_EQ(allocations, size_t{0});
  EXPECT_STREQ(out.solver_status, "feasible_nominal");
}
