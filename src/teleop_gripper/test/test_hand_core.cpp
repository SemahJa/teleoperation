#include <gtest/gtest.h>

#include <cstring>

#include "teleop_gripper/hand_core.hpp"
namespace ft = teleop_gripper;
namespace {
struct Fixture {
  ft::HandParameters p;
  ft::HandCore core;
  uint64_t now{1000000000}, sequence{0};
  std::string session{"first"};
  uint64_t generation{1};
  Fixture(bool object_configured = true) : p(make_parameters(object_configured)), core(p) {}
  static ft::HandParameters make_parameters(bool configured) {
    ft::HandParameters p;
    if (configured) {
      p.grasp_width = .006;
      p.epsilon_inner = .001;
      p.epsilon_outer = .002;
    }
    return p;
  }
  ft::HandRequest input(double value, bool valid = true, uint64_t dt = 10000000) {
    now += dt;
    ft::GripInput c;
    std::strcpy(c.session.data(), session.c_str());
    c.source_ns = now;
    c.sequence = ++sequence;
    c.generation = generation;
    c.valid = valid;
    c.value = value;
    return core.input(c, now);
  }
  void calibrate() {
    ASSERT_EQ(core.calibrate(true, true).operation, ft::HandOperation::calibrate);
    core.complete(ft::HandOperation::calibrate, true, true, .08, .08);
  }
  ft::HandRequest press() {
    input(0);
    for (int n = 0; n < 5; ++n)
      EXPECT_EQ(input(.8).operation, ft::HandOperation::none);
    return input(.8);
  }
};
}  // namespace
TEST(Hand, ExplicitCalibrationAndUnknownFirstOpen) {
  Fixture f;
  EXPECT_EQ(f.core.state(), ft::HandState::unknown);
  EXPECT_FALSE(f.core.calibrated());
  EXPECT_EQ(f.core.calibrate(false, true).operation, ft::HandOperation::none);
  EXPECT_EQ(f.core.calibrate(true, false).operation, ft::HandOperation::none);
  EXPECT_EQ(f.press().operation, ft::HandOperation::none);
  f.calibrate();
  ASSERT_TRUE(f.core.calibrated());
  EXPECT_EQ(f.core.state(), ft::HandState::unknown);
  auto request = f.press();
  EXPECT_EQ(request.operation, ft::HandOperation::open);
  EXPECT_DOUBLE_EQ(request.width, .08);
}
TEST(Hand, ToggleIsDiscreteAndBusyPressesAreNotQueued) {
  Fixture f;
  f.calibrate();
  ASSERT_EQ(f.press().operation, ft::HandOperation::open);
  EXPECT_EQ(f.core.calibrate(true, true).operation, ft::HandOperation::none);
  // Another deliberate press while busy is consumed and never replayed.
  EXPECT_EQ(f.press().operation, ft::HandOperation::none);
  f.core.complete(ft::HandOperation::open, true, true, .08, .08);
  for (int n = 0; n < 20; ++n)
    EXPECT_EQ(f.input(.8).operation, ft::HandOperation::none);
  auto request = f.press();
  ASSERT_EQ(request.operation, ft::HandOperation::grasp);
  EXPECT_DOUBLE_EQ(request.width, .006);
  EXPECT_DOUBLE_EQ(request.force, 5);
  EXPECT_DOUBLE_EQ(request.speed, .02);
  EXPECT_DOUBLE_EQ(request.epsilon_inner, .001);
  f.core.complete(ft::HandOperation::grasp, true, true, .006, .08);
  EXPECT_EQ(f.input(0).operation, ft::HandOperation::none);
  EXPECT_EQ(f.core.state(), ft::HandState::grasped);  // release does not open
  EXPECT_EQ(f.press().operation, ft::HandOperation::open);
}
TEST(Hand, DebounceRequiresContinuouslyHighFreshSource) {
  Fixture f;
  f.calibrate();
  f.input(0);
  f.input(.8);
  f.input(.8);
  f.input(.5);
  for (int n = 0; n < 5; ++n)
    EXPECT_EQ(f.input(.8).operation, ft::HandOperation::none);
  EXPECT_EQ(f.input(.8).operation, ft::HandOperation::open);
}
TEST(Hand, InvalidTrackingCannotCancelIssuedOperationOrResumeGrip) {
  Fixture f;
  f.calibrate();
  ASSERT_EQ(f.press().operation, ft::HandOperation::open);
  f.input(.8, false);
  EXPECT_TRUE(f.core.busy());
  f.core.complete(ft::HandOperation::open, true, true, .08, .08);
  for (int n = 0; n < 10; ++n)
    EXPECT_EQ(f.input(.8).operation, ft::HandOperation::none);
  EXPECT_EQ(f.press().operation, ft::HandOperation::grasp);
}
TEST(Hand, SessionReferenceAndMissingSourceResetPendingPress) {
  for (int fault = 0; fault < 3; ++fault) {
    Fixture f;
    f.calibrate();
    f.input(0);
    f.input(.8);
    f.input(.8);
    if (fault == 0)
      f.session = "second";
    if (fault == 1)
      f.generation++;
    if (fault == 2) {
      f.now += 200000000;
      f.core.watchdog(f.now);
    }
    for (int n = 0; n < 10; ++n)
      EXPECT_EQ(f.input(.8).operation, ft::HandOperation::none);
    EXPECT_EQ(f.press().operation, ft::HandOperation::open);
  }
}
TEST(Hand, FailedGraspIsUnknownThenDeliberateOpenWithoutRetry) {
  Fixture f;
  f.calibrate();
  f.press();
  f.core.complete(ft::HandOperation::open, true, true, .08, .08);
  ASSERT_EQ(f.press().operation, ft::HandOperation::grasp);
  f.core.complete(ft::HandOperation::grasp, false, true, .012, .08);
  EXPECT_EQ(f.core.state(), ft::HandState::unknown);
  for (int n = 0; n < 10; ++n)
    EXPECT_EQ(f.input(.8).operation, ft::HandOperation::none);
  EXPECT_EQ(f.press().operation, ft::HandOperation::open);
}
TEST(Hand, CommunicationFaultRequiresExplicitCalibration) {
  Fixture f;
  f.calibrate();
  f.press();
  f.core.complete(ft::HandOperation::open, false, false, 0, 0);
  EXPECT_FALSE(f.core.calibrated());
  EXPECT_EQ(f.core.state(), ft::HandState::unknown);
  EXPECT_EQ(f.press().operation, ft::HandOperation::none);
}
TEST(Hand, MissingObjectSettingsDoNotPreventOpenButRejectGrasp) {
  Fixture f(false);
  f.calibrate();
  EXPECT_EQ(f.press().operation, ft::HandOperation::open);
  f.core.complete(ft::HandOperation::open, true, true, .08, .08);
  EXPECT_EQ(f.press().operation, ft::HandOperation::none);
  EXPECT_FALSE(f.core.busy());
}
TEST(Hand, SourceOrderingAndUncertaintyFaultsRequireRelease) {
  for (int fault = 0; fault < 4; ++fault) {
    Fixture f;
    f.calibrate();
    f.input(0);
    ft::GripInput c;
    std::strcpy(c.session.data(), "first");
    c.valid = true;
    c.value = .8;
    c.generation = 1;
    c.sequence = 2;
    c.source_ns = f.now + 1;
    if (fault == 0)
      c.sequence = 1;
    if (fault == 1)
      c.source_ns = f.now;
    if (fault == 2)
      c.uncertainty_ns = 10000001;
    if (fault == 3)
      c.source_ns = f.now - 200000000;
    EXPECT_EQ(f.core.input(c, f.now + 1).operation, ft::HandOperation::none);
    for (int n = 0; n < 10; ++n)
      EXPECT_EQ(f.input(.8).operation, ft::HandOperation::none);
    EXPECT_EQ(f.press().operation, ft::HandOperation::open);
  }
}
