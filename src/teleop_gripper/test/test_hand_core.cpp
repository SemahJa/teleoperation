#include <gtest/gtest.h>

#include <cstring>
#include <limits>

#include "teleop_gripper/hand_core.hpp"
namespace ft = teleop_gripper;
namespace {
struct Fixture {
  static ft::HandParameters parameters() {
    ft::HandParameters p;
    p.debounce_ns = 50000000;  // Optional debounce is exercised by this fixture.
    return p;
  }
  ft::HandCore core{parameters()};
  uint64_t now{1000000000}, sequence{0}, generation{1};
  const char* session{"first"};
  ft::HandRequest input(double value, bool valid = true, uint64_t dt = 10000000, bool open = false,
                        bool open_active = true) {
    now += dt;
    ft::GripInput c;
    std::strcpy(c.session.data(), session);
    c.source_ns = now;
    c.sequence = ++sequence;
    c.generation = generation;
    c.valid = valid;
    c.value = value;
    c.open_active = open_active;
    c.open_pressed = open;
    return core.input(c, now);
  }
  void calibrate() {
    ASSERT_EQ(core.calibrate(true, true).operation, ft::HandOperation::calibrate);
    core.complete(ft::HandOperation::calibrate, true, true, .08, .08, now);
  }
  ft::HandRequest press() {
    const auto first = input(.8);
    if (first.operation != ft::HandOperation::none)
      return first;
    for (int n = 0; n < 4; ++n)
      EXPECT_EQ(input(.8).operation, ft::HandOperation::none);
    return input(.8);
  }
  void stopped() { core.complete(ft::HandOperation::close, true, true, .075, .08, now); }
  void click() {
    press();
    EXPECT_EQ(input(0).operation, ft::HandOperation::stop);
    stopped();
  }
};
}  // namespace
TEST(Hand, CalibrationAndFreshReleaseRequired) {
  Fixture f;
  EXPECT_EQ(f.core.calibrate(false, true).operation, ft::HandOperation::none);
  EXPECT_EQ(f.core.calibrate(true, false).operation, ft::HandOperation::none);
  EXPECT_EQ(f.press().operation, ft::HandOperation::none);
  f.calibrate();
  EXPECT_EQ(f.press().operation, ft::HandOperation::none);
  f.input(0);
  auto r = f.press();
  EXPECT_EQ(r.operation, ft::HandOperation::close);
  EXPECT_DOUBLE_EQ(r.width, 0);
  EXPECT_DOUBLE_EQ(r.speed, .02);
}
TEST(Hand, CalibrationAllowsBoundedEndpointReadbackNoiseWithoutExpandingTravel) {
  for (double width : {-.000005, .080005}) {
    Fixture f;
    f.core.calibrate(true, true);
    f.core.complete(ft::HandOperation::calibrate, true, true, width, .08, f.now);
    ASSERT_TRUE(f.core.calibrated());
    EXPECT_DOUBLE_EQ(f.core.maximum_width(), .08);
    EXPECT_DOUBLE_EQ(f.core.width(), width < 0 ? 0 : .08);
    EXPECT_EQ(f.press().operation, ft::HandOperation::none);
    f.input(0);
    EXPECT_EQ(f.press().operation, ft::HandOperation::close);
  }
}
TEST(Hand, InvalidEndpointReadbackAndCommunicationStillRejectCalibration) {
  for (double width : {-.000011, .080011, std::numeric_limits<double>::quiet_NaN(),
                       std::numeric_limits<double>::infinity()}) {
    Fixture f;
    f.core.calibrate(true, true);
    f.core.complete(ft::HandOperation::calibrate, true, true, width, .08, f.now);
    EXPECT_FALSE(f.core.calibrated());
    EXPECT_EQ(f.input(0).operation, ft::HandOperation::none);
    EXPECT_EQ(f.press().operation, ft::HandOperation::none);
  }
  Fixture f;
  f.core.calibrate(true, true);
  f.core.complete(ft::HandOperation::calibrate, true, false, .08, .08, f.now);
  EXPECT_FALSE(f.core.calibrated());
}
TEST(Hand, HoldIssuesOneCloseAndReleaseStopsImmediately) {
  Fixture f;
  f.calibrate();
  f.input(0);
  ASSERT_EQ(f.press().operation, ft::HandOperation::close);
  for (int n = 0; n < 100; ++n)
    EXPECT_EQ(f.input(.8).operation, ft::HandOperation::none);
  EXPECT_EQ(f.input(0).operation, ft::HandOperation::stop);
  EXPECT_EQ(f.input(0).operation, ft::HandOperation::none);
  EXPECT_TRUE(f.core.busy());
  f.stopped();
  EXPECT_FALSE(f.core.busy());
  EXPECT_EQ(f.core.state(), ft::HandState::stopped);
}
TEST(Hand, JoystickPressOpensAfterCloseStopWithoutWaitingForButtonRelease) {
  Fixture f;
  f.calibrate();
  f.input(0);
  f.press();
  EXPECT_EQ(f.input(.8, true, 10000000, true).operation, ft::HandOperation::stop);
  const auto next = f.core.complete(ft::HandOperation::close, true, true, .07, .08, f.now);
  EXPECT_EQ(next.operation, ft::HandOperation::open);
  EXPECT_DOUBLE_EQ(next.width, .08);
  EXPECT_DOUBLE_EQ(next.speed, .1);
  for (int i = 0; i < 10; ++i)
    EXPECT_EQ(f.input(.8, true, 10000000, true).operation, ft::HandOperation::none);
  f.core.complete(ft::HandOperation::open, true, true, .08, .08, f.now);
  EXPECT_EQ(f.input(.8, true, 10000000, true).operation, ft::HandOperation::none);
  EXPECT_FALSE(f.core.busy());  // Held grip cannot immediately reverse opening.
}

TEST(Hand, JoystickOpeningCanReplaceAClosingIntentDuringPendingStop) {
  Fixture f;
  f.calibrate();
  f.input(0);
  f.press();
  EXPECT_EQ(f.input(0).operation, ft::HandOperation::stop);
  EXPECT_EQ(f.input(0, true, 10000000, true).operation, ft::HandOperation::none);
  EXPECT_EQ(f.core.complete(ft::HandOperation::close, true, true, .075, .08, f.now).operation,
            ft::HandOperation::open);
}

TEST(Hand, SideGripClicksNeverOpen) {
  Fixture f;
  f.calibrate();
  f.input(0);
  for (int n = 0; n < 4; ++n) {
    EXPECT_EQ(f.press().operation, ft::HandOperation::close);
    EXPECT_EQ(f.input(0).operation, ft::HandOperation::stop);
    EXPECT_EQ(f.core.complete(ft::HandOperation::close, true, true, .07, .08, f.now).operation,
              ft::HandOperation::none);
  }
}

TEST(Hand, InputFaultsStopAndPreventHeldResumption) {
  for (int scenario = 0; scenario < 5; ++scenario) {
    Fixture f;
    f.calibrate();
    f.input(0);
    f.press();
    ft::HandRequest r;
    if (scenario == 0)
      r = f.input(.8, false);
    if (scenario == 1) {
      f.session = "second";
      r = f.input(.8);
    }
    if (scenario == 2) {
      f.generation++;
      r = f.input(.8);
    }
    if (scenario == 3) {
      f.now += 200000000;
      r = f.core.watchdog(f.now);
    }
    if (scenario == 4)
      r = f.input(.8, true, 200000000);
    EXPECT_EQ(r.operation, ft::HandOperation::stop);
    f.stopped();
    for (int n = 0; n < 10; ++n)
      EXPECT_EQ(f.input(.8).operation, ft::HandOperation::none);
    f.input(0);
    EXPECT_EQ(f.press().operation, ft::HandOperation::close);
  }
}
TEST(Hand, StaleDuplicateFutureAndUncertainInputsStop) {
  for (int scenario = 0; scenario < 4; ++scenario) {
    Fixture f;
    f.calibrate();
    f.input(0);
    f.press();
    ft::GripInput c;
    std::strcpy(c.session.data(), "first");
    c.valid = true;
    c.value = .8;
    c.generation = 1;
    c.sequence = f.sequence + 1;
    c.source_ns = f.now + 1;
    if (scenario == 0)
      c.sequence = f.sequence;
    if (scenario == 1)
      c.source_ns = f.now - 200000000;
    if (scenario == 2)
      c.source_ns = f.now + 20000000;
    if (scenario == 3)
      c.uncertainty_ns = 10000001;
    EXPECT_EQ(f.core.input(c, f.now + 1).operation, ft::HandOperation::stop);
  }
}
TEST(Hand, FailureClearsDeferredOpen) {
  Fixture f;
  f.calibrate();
  f.input(0);
  f.press();
  f.input(0, true, 10000000, true);
  EXPECT_EQ(f.core.complete(ft::HandOperation::close, false, true, .07, .08, f.now).operation,
            ft::HandOperation::none);
  EXPECT_EQ(f.core.state(), ft::HandState::unknown);
  EXPECT_EQ(f.press().operation, ft::HandOperation::none);
}

TEST(Hand, CommunicationFailureRequiresCalibration) {
  Fixture f;
  f.calibrate();
  f.input(0);
  f.press();
  f.core.complete(ft::HandOperation::close, false, false, 0, 0, f.now);
  EXPECT_FALSE(f.core.calibrated());
  f.input(0);
  EXPECT_EQ(f.press().operation, ft::HandOperation::none);
}
TEST(Hand, ShutdownStopsOpeningAndClearsGestures) {
  Fixture f;
  f.calibrate();
  f.input(0);
  ASSERT_EQ(f.input(0, true, 10000000, true).operation, ft::HandOperation::open);
  EXPECT_EQ(f.core.shutdown().operation, ft::HandOperation::stop);
  EXPECT_EQ(f.core.shutdown().operation, ft::HandOperation::none);
}

TEST(Hand, SuccessfulZeroWidthStillRefreshesTheHeldClosingDemand) {
  Fixture f;
  f.calibrate();
  f.input(0);
  f.press();
  f.core.complete(ft::HandOperation::close, true, true, 0, .08, f.now);
  for (int n = 0; n < 40; ++n) {
    EXPECT_EQ(f.input(.8).operation, ft::HandOperation::none);
    const auto r = f.core.watchdog(f.now);
    if ((n + 1) % 20 == 0) {
      EXPECT_EQ(r.operation, ft::HandOperation::close);
      EXPECT_DOUBLE_EQ(r.width, 0);
      f.core.complete(ft::HandOperation::close, true, true, 0, .08, f.now);
    } else
      EXPECT_EQ(r.operation, ft::HandOperation::none);
  }
  f.input(0);
  for (int n = 0; n < 40; ++n) {
    f.input(0);
    EXPECT_EQ(f.core.watchdog(f.now).operation, ft::HandOperation::none);
  }
}

TEST(Hand, SlowCloseKeepsDemandActiveAndRefreshesWhenAvailableWithoutFailureDelay) {
  for (bool success : {false, true}) {
    Fixture f;
    f.calibrate();
    f.input(0);
    ASSERT_EQ(f.press().operation, ft::HandOperation::close);
    for (int attempt = 0; attempt < 3; ++attempt) {
      // A running move is not interrupted or reissued by a retry timer.
      for (int n = 0; n < 25; ++n) {
        f.input(.8);
        EXPECT_EQ(f.core.watchdog(f.now).operation, ft::HandOperation::none);
      }
      f.core.complete(ft::HandOperation::close, success, true, .012, .08, f.now, true);
      EXPECT_FALSE(f.core.busy());
      f.input(.8);
      const auto next = f.core.watchdog(f.now);
      EXPECT_EQ(next.operation, ft::HandOperation::close);
      EXPECT_DOUBLE_EQ(next.width, 0);
      EXPECT_DOUBLE_EQ(next.speed, .02);
    }
    EXPECT_EQ(f.input(0).operation, ft::HandOperation::stop);
    f.stopped();
    EXPECT_EQ(f.press().operation, ft::HandOperation::close);
  }
}

TEST(Hand, HeldRefreshCadenceStartsAtDispatchRatherThanAtFailure) {
  Fixture f;
  f.calibrate();
  f.input(0);
  f.press();
  for (int n = 0; n < 15; ++n) {
    f.input(.8);
    EXPECT_EQ(f.core.watchdog(f.now).operation, ft::HandOperation::none);
  }
  f.core.complete(ft::HandOperation::close, false, true, .012, .08, f.now, true);
  for (int n = 0; n < 4; ++n) {
    f.input(.8);
    EXPECT_EQ(f.core.watchdog(f.now).operation, ft::HandOperation::none);
  }
  f.input(.8);
  EXPECT_EQ(f.core.watchdog(f.now).operation, ft::HandOperation::close);
}

TEST(Hand, ReleaseFaultAndShutdownCancelPendingRetries) {
  for (int scenario = 0; scenario < 6; ++scenario) {
    Fixture f;
    f.calibrate();
    f.input(0);
    f.press();
    f.core.complete(ft::HandOperation::close, false, true, .012, .08, f.now, true);
    if (scenario == 0)
      f.input(0);
    if (scenario == 1)
      f.input(.8, false);
    if (scenario == 2) {
      f.now += 200000000;
      f.core.watchdog(f.now);
    }
    if (scenario == 3) {
      f.session = "replacement";
      f.input(.8);
    }
    if (scenario == 4) {
      ++f.generation;
      f.input(.8);
    }
    if (scenario == 5)
      f.core.shutdown();
    for (int n = 0; n < 30; ++n) {
      EXPECT_EQ(f.input(scenario == 0 ? 0 : .8).operation, ft::HandOperation::none);
      EXPECT_EQ(f.core.watchdog(f.now).operation, ft::HandOperation::none);
    }
    EXPECT_FALSE(f.core.busy());
  }
}

TEST(Hand, DisabledRetriesAndNonretryableFailuresRequireANewPress) {
  for (bool disabled : {false, true}) {
    Fixture f;
    ft::HandParameters p;
    if (disabled)
      p.close_retry_ns = 0;
    f.core = ft::HandCore(p);
    f.calibrate();
    f.input(0);
    f.press();
    f.core.complete(ft::HandOperation::close, false, true, .012, .08, f.now, disabled);
    for (int n = 0; n < 30; ++n) {
      EXPECT_EQ(f.input(.8).operation, ft::HandOperation::none);
      EXPECT_EQ(f.core.watchdog(f.now).operation, ft::HandOperation::none);
    }
    f.input(0);
    EXPECT_EQ(f.press().operation, ft::HandOperation::close);
  }
}
TEST(Hand, WidthsAndTimingValidated) {
  ft::HandParameters p;
  EXPECT_TRUE(p.valid());
  p.open_width = .025;
  p.close_width = .03;
  EXPECT_FALSE(p.valid());
  p.close_width = 0;
  p.debounce_ns = 100000001;
  EXPECT_FALSE(p.valid());
  p.debounce_ns = 50000000;
  p.open_speed = 0;
  EXPECT_FALSE(p.valid());
  p.open_speed = .1;
  p.close_retry_ns = 1;
  EXPECT_FALSE(p.valid());
  p.close_retry_ns = 10000000001ULL;
  EXPECT_FALSE(p.valid());
  p.close_retry_ns = 0;
  EXPECT_TRUE(p.valid());
  p.close_retry_ns = 200000000;
  p.open_width = .09;
  ft::HandCore core(p);
  core.calibrate(true, true);
  core.complete(ft::HandOperation::calibrate, true, true, .08, .08, 0);
  ft::GripInput c;
  std::strcpy(c.session.data(), "x");
  c.valid = true;
  c.source_ns = 1000000000;
  c.sequence = 1;
  core.input(c, c.source_ns);
  c.value = .8;
  c.source_ns += 10000000;
  ++c.sequence;
  core.input(c, c.source_ns);
  c.source_ns += 50000000;
  ++c.sequence;
  EXPECT_EQ(core.input(c, c.source_ns).operation, ft::HandOperation::none);
}
TEST(Hand, QuestLossCannotInterruptEmptyHandCalibration) {
  Fixture f;
  f.core.calibrate(true, true);
  EXPECT_EQ(f.input(.8, false).operation, ft::HandOperation::none);
  f.now += 200000000;
  EXPECT_EQ(f.core.watchdog(f.now).operation, ft::HandOperation::none);
  f.core.complete(ft::HandOperation::calibrate, true, true, .08, .08, f.now);
  EXPECT_TRUE(f.core.calibrated());
  EXPECT_EQ(f.press().operation, ft::HandOperation::none);
}
TEST(Hand, FaultClearsPendingJoystickOpen) {
  Fixture f;
  f.calibrate();
  f.input(0);
  f.press();
  f.input(0, true, 10000000, true);
  f.input(0, false);
  EXPECT_EQ(f.core.complete(ft::HandOperation::close, true, true, .07, .08, f.now).operation,
            ft::HandOperation::none);
}

TEST(Hand, CloseWidthSafetyIsDelegatedToLiveBackendFeedback) {
  ft::HandParameters p;
  p.debounce_ns = 50000000;
  p.close_width = .007;
  p.open_width = .025;
  ft::HandCore core(p);
  core.calibrate(true, true);
  core.complete(ft::HandOperation::calibrate, true, true, .005, .08, 0);
  ft::GripInput c;
  std::strcpy(c.session.data(), "x");
  c.valid = true;
  c.source_ns = 1000000000;
  c.sequence = 1;
  core.input(c, c.source_ns);
  c.value = .8;
  c.source_ns += 10000000;
  ++c.sequence;
  core.input(c, c.source_ns);
  c.source_ns += 50000000;
  ++c.sequence;
  EXPECT_EQ(core.input(c, c.source_ns).operation, ft::HandOperation::close);
}
TEST(Hand, JoystickFromClosedCalibrationOpensToConfiguredWidth) {
  Fixture f;
  auto p = Fixture::parameters();
  p.open_width = .025;
  p.close_width = .005;
  f.core = ft::HandCore(p);
  f.calibrate();
  f.core.complete(ft::HandOperation::calibrate, true, true, 0, .08, f.now);
  f.input(0);
  const auto r = f.input(0, true, 10000000, true);
  EXPECT_EQ(r.operation, ft::HandOperation::open);
  EXPECT_DOUBLE_EQ(r.width, .025);
}

TEST(Hand, FreshHeldPressDuringStopResumesAfterAcknowledgement) {
  Fixture f;
  f.calibrate();
  f.input(0);
  f.press();
  ASSERT_EQ(f.input(0).operation, ft::HandOperation::stop);
  EXPECT_EQ(f.press().operation, ft::HandOperation::none);
  const auto next = f.core.complete(ft::HandOperation::close, true, true, .05, .08, f.now);
  EXPECT_EQ(next.operation, ft::HandOperation::close);
}
TEST(Hand, OldZeroWidthCannotRejectANewCloseRequest) {
  Fixture f;
  f.calibrate();
  f.input(0);
  f.press();
  f.core.complete(ft::HandOperation::close, true, true, 0, .08, f.now);
  // Fingers may subsequently be moved externally; core snapshots are not live.
  f.input(0);
  EXPECT_EQ(f.press().operation, ft::HandOperation::close);
}
TEST(Hand, ReleaseCancelsHeldIntentDuringAStop) {
  Fixture f;
  f.calibrate();
  f.input(0);
  f.press();
  f.input(0);
  f.press();
  f.input(0);
  const auto r = f.core.complete(ft::HandOperation::close, true, true, .04, .08, f.now);
  EXPECT_NE(r.operation, ft::HandOperation::close);
}
TEST(Hand, FreshGripPressInterruptsAutomaticOpening) {
  Fixture f;
  f.calibrate();
  f.input(0);
  ASSERT_EQ(f.input(0, true, 10000000, true).operation, ft::HandOperation::open);
  f.input(0);  // Release side grip after the opening request.
  EXPECT_EQ(f.press().operation, ft::HandOperation::stop);
  EXPECT_EQ(f.core.complete(ft::HandOperation::open, true, true, .05, .08, f.now).operation,
            ft::HandOperation::close);
}

TEST(Hand, DefaultClosingAndOpeningHaveNoGestureDelay) {
  Fixture f;
  f.core = ft::HandCore(ft::HandParameters{});
  f.calibrate();
  f.input(0);
  EXPECT_EQ(f.input(.8).operation, ft::HandOperation::close);
  EXPECT_EQ(f.input(0).operation, ft::HandOperation::stop);
  f.stopped();
  EXPECT_EQ(f.input(0, true, 10000000, true).operation, ft::HandOperation::open);
}
TEST(Hand, StartupHeldJoystickAndUnavailableButtonCannotOpen) {
  Fixture f;
  f.calibrate();
  for (int n = 0; n < 5; ++n)
    EXPECT_EQ(f.input(0, true, 10000000, true).operation, ft::HandOperation::none);
  f.input(0);
  EXPECT_EQ(f.input(0, true, 10000000, true, false).operation, ft::HandOperation::none);
  EXPECT_EQ(f.input(0, true, 10000000, true).operation, ft::HandOperation::none);
  f.input(0);
  EXPECT_EQ(f.input(0, true, 10000000, true).operation, ft::HandOperation::open);
}
TEST(Hand, MissingJoystickDoesNotPreventClosingButLostOpenActionStopsOpening) {
  Fixture f;
  f.calibrate();
  f.input(0, true, 10000000, false, false);
  EXPECT_EQ(f.press().operation, ft::HandOperation::close);
  f.input(0);
  f.stopped();
  EXPECT_EQ(f.input(0, true, 10000000, true).operation, ft::HandOperation::open);
  EXPECT_EQ(f.input(0, true, 10000000, false, false).operation, ft::HandOperation::stop);
}
TEST(Hand, HeldJoystickCannotRepeatOrReverseAnOpening) {
  Fixture f;
  f.calibrate();
  f.input(0);
  EXPECT_EQ(f.input(0, true, 10000000, true).operation, ft::HandOperation::open);
  f.core.complete(ft::HandOperation::open, true, true, .08, .08, f.now);
  for (int n = 0; n < 10; ++n)
    EXPECT_EQ(f.input(0, true, 10000000, true).operation, ft::HandOperation::none);
  f.input(0);
  EXPECT_EQ(f.input(0, true, 10000000, true).operation, ft::HandOperation::open);
}
TEST(Hand, UnavailableGripCannotAuthorizeHeldClosingWhenItReturns) {
  Fixture f;
  f.calibrate();
  f.input(0);
  ft::GripInput c;
  std::strcpy(c.session.data(), "first");
  c.valid = true;
  c.sequence = ++f.sequence;
  c.generation = 1;
  c.source_ns = f.now += 10000000;
  c.grip_active = false;
  f.core.input(c, f.now);
  EXPECT_EQ(f.press().operation, ft::HandOperation::none);
  f.input(0);
  EXPECT_EQ(f.press().operation, ft::HandOperation::close);
}
