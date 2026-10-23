#include <franka/exception.h>
#include <gtest/gtest.h>

#include <chrono>
#include <cstring>
#include <thread>

#include "teleop_gripper/hand_worker.hpp"
namespace ft = teleop_gripper;
namespace {
std::optional<ft::HandResult> collect(ft::HandWorker& worker,
                                      std::chrono::seconds timeout = std::chrono::seconds(2)) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (auto result = worker.collect())
      return result;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return {};
}
}  // namespace
TEST(HandBackend, ReleaseCancelsPartialClosingAndNextOpenIsSerialized) {
  ft::HandWorker worker(false, "", 10);
  worker.issue({ft::HandOperation::close, 0, .02}, .08);
  std::this_thread::sleep_for(std::chrono::milliseconds(60));
  worker.issue({ft::HandOperation::stop}, 0);
  auto r = collect(worker);
  ASSERT_TRUE(r);
  EXPECT_TRUE(r->stopped);
  EXPECT_TRUE(r->success);
  EXPECT_LT(r->width, .08);
  EXPECT_GT(r->width, .07);
  worker.issue({ft::HandOperation::open, .08, .1}, r->width);
  r = collect(worker);
  ASSERT_TRUE(r);
  EXPECT_TRUE(r->success);
  EXPECT_DOUBLE_EQ(r->width, .08);
}
TEST(HandBackend, ImmediateRepeatedStopCannotLeaveMotionRunning) {
  ft::HandWorker worker(false, "", 10);
  worker.issue({ft::HandOperation::close, 0, .02}, .08);
  for (int i = 0; i < 4; ++i)
    worker.issue({ft::HandOperation::stop}, 0);
  auto r = collect(worker);
  ASSERT_TRUE(r);
  EXPECT_TRUE(r->stopped);
  EXPECT_TRUE(r->success);
  EXPECT_GT(r->width, .075);
}
TEST(HandBackend, RejectsOverlappingMotion) {
  ft::HandWorker worker(false, "", 10);
  worker.issue({ft::HandOperation::close, 0, .02}, .08);
  EXPECT_THROW(worker.issue({ft::HandOperation::open, .08, .1}, .07), std::logic_error);
  worker.issue({ft::HandOperation::stop}, 0);
  ASSERT_TRUE(collect(worker));
}
TEST(HandBackend, DestructorStopsOutstandingMotion) {
  const auto begin = std::chrono::steady_clock::now();
  {
    ft::HandWorker worker(false, "", 10);
    worker.issue({ft::HandOperation::close, 0, .001}, .08);
  }
  EXPECT_LT(std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count(), 1.0);
}
namespace {
struct ScriptedHand final : ft::HandDevice {
  std::atomic<double> width{.012}, target{-1};
  std::atomic<int> stops{0};
  std::atomic<int> moves{0};
  std::atomic<int> reads{0};
  bool slow_final_read{false};
  int failure{0};
  int stop_failure{0};
  bool transport_fault_after_ack{false};
  bool delayed{false}, reject_all_stops{false};
  bool homing() override { return true; }
  ft::HandFeedback read() override {
    if (++reads >= 2 && slow_final_read)
      std::this_thread::sleep_for(std::chrono::milliseconds(150));
    return {width.load(), .08};
  }
  bool move(double value, double) override {
    target = value;
    ++moves;
    if (failure == 1)
      return false;
    if (failure == 2)
      throw franka::CommandException("move ended without reaching width");
    if (failure == 3)
      throw franka::NetworkException("move transport failed");
    if (failure == 4)
      throw std::runtime_error("unknown device failure");
    if (delayed) {
      std::this_thread::sleep_for(std::chrono::milliseconds(70));
      return false;
    }
    width = value;
    return true;
  }
  bool stop() override {
    ++stops;
    if (transport_fault_after_ack && stops > 1)
      throw franka::NetworkException("stop transport lost after acknowledgement");
    if (stop_failure == 1)
      return false;
    if (stop_failure == 2)
      throw franka::CommandException("stop command rejected");
    if (stop_failure == 3)
      throw franka::NetworkException("stop connection lost");
    if (stop_failure == 4 || reject_all_stops)
      throw std::runtime_error("already stopped or communication fault");
    if (stops > 1)
      throw franka::CommandException("redundant stop rejected");
    return true;
  }
};
}  // namespace
TEST(HandBackend, CloseReadsLiveWidthInsteadOfTheOldZeroSnapshot) {
  auto device = std::make_shared<ScriptedHand>();
  ft::HandWorker worker(device);
  worker.issue({ft::HandOperation::close, 0, .02}, 0);
  const auto r = collect(worker);
  ASSERT_TRUE(r);
  EXPECT_TRUE(r->success);
  EXPECT_DOUBLE_EQ(device->target, 0);
  EXPECT_DOUBLE_EQ(r->width, 0);
}
TEST(HandBackend, BoundedEndpointNoiseDoesNotPreventClosingOrCommandOpening) {
  for (double width : {-.000005, .080005}) {
    auto device = std::make_shared<ScriptedHand>();
    device->width = width;
    ft::HandWorker worker(device);
    worker.issue({ft::HandOperation::close, 0, .02}, .08);
    const auto r = collect(worker);
    ASSERT_TRUE(r);
    EXPECT_TRUE(r->success);
    EXPECT_TRUE(r->communication_ok);
    EXPECT_EQ(device->moves.load(), width < 0 ? 0 : 1);
    if (width > 0)
      EXPECT_DOUBLE_EQ(device->target, 0);
  }
}
TEST(HandBackend, OutOfBoundsLiveFeedbackNeverMovesOrRetries) {
  auto device = std::make_shared<ScriptedHand>();
  device->width = .080011;
  ft::HandWorker worker(device);
  worker.issue({ft::HandOperation::close, 0, .02}, .08);
  const auto r = collect(worker);
  ASSERT_TRUE(r);
  EXPECT_FALSE(r->success);
  EXPECT_FALSE(r->communication_ok);
  EXPECT_FALSE(r->retryable);
  EXPECT_EQ(device->moves.load(), 0);
}
TEST(HandBackend, CloseCannotOpenWhenLiveWidthIsBelowConfiguredLimit) {
  auto device = std::make_shared<ScriptedHand>();
  device->width = .003;
  ft::HandWorker worker(device);
  worker.issue({ft::HandOperation::close, .007, .02}, .04);
  const auto r = collect(worker);
  ASSERT_TRUE(r);
  EXPECT_TRUE(r->success);
  EXPECT_DOUBLE_EQ(device->target, -1);  // Already below the bound: no move.
  EXPECT_DOUBLE_EQ(r->width, .003);
}
TEST(HandBackend, RedundantStopExceptionCannotDiscardSuccessfulCancellation) {
  auto device = std::make_shared<ScriptedHand>();
  device->delayed = true;
  ft::HandWorker worker(device);
  worker.issue({ft::HandOperation::close, 0, .02}, .012);
  for (int n = 0; n < 1000 && device->target.load() < 0; ++n)
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  ASSERT_GE(device->target.load(), 0);
  worker.issue({ft::HandOperation::stop}, 0);
  const auto r = collect(worker);
  ASSERT_TRUE(r);
  EXPECT_TRUE(r->success);
  EXPECT_TRUE(r->communication_ok);
  EXPECT_TRUE(r->stopped);
  EXPECT_FALSE(r->retryable);
  EXPECT_GT(device->stops.load(), 1);
}
TEST(HandBackend, UnfinishedMovesAreRetryableButTransportAndUnknownErrorsAreNot) {
  for (int failure = 1; failure <= 4; ++failure) {
    auto device = std::make_shared<ScriptedHand>();
    device->failure = failure;
    ft::HandWorker worker(device);
    worker.issue({ft::HandOperation::close, 0, .02}, .012);
    const auto r = collect(worker);
    ASSERT_TRUE(r);
    EXPECT_FALSE(r->success);
    EXPECT_EQ(r->communication_ok, failure <= 2);
    EXPECT_EQ(r->retryable, failure <= 2);
    EXPECT_DOUBLE_EQ(r->width, .012);
  }
}

TEST(HandBackend, PersistentCloseHandlesFailureAndReopenedGapThenStopsOnRelease) {
  for (bool blocked : {false, true}) {
    auto device = std::make_shared<ScriptedHand>();
    device->failure = blocked ? 1 : 0;
    ft::HandWorker worker(device);
    ft::HandParameters p;
    p.close_retry_ns = 50000000;
    ft::HandCore core(p);
    core.calibrate(true, true);
    auto now_ns = [] {
      return std::chrono::duration_cast<std::chrono::nanoseconds>(
                 std::chrono::steady_clock::now().time_since_epoch())
          .count();
    };
    core.complete(ft::HandOperation::calibrate, true, true, .012, .08, now_ns());
    uint64_t sequence = 0;
    int completions = 0;
    auto step = [&](double value) {
      ft::GripInput c;
      std::strcpy(c.session.data(), "held-grip");
      c.valid = true;
      c.sequence = ++sequence;
      c.source_ns = now_ns();
      c.value = value;
      worker.issue(core.input(c, c.source_ns), core.width());
      worker.issue(core.watchdog(c.source_ns), core.width());
      if (auto r = worker.collect()) {
        ++completions;
        worker.issue(core.complete(r->operation, r->success, r->communication_ok, r->width,
                                   r->maximum_width, now_ns(), r->retryable),
                     core.width());
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    };
    step(0);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (completions < 3 && std::chrono::steady_clock::now() < deadline)
      step(.8);
    ASSERT_GE(completions, 3);
    ASSERT_FALSE(core.busy());  // Paused between retry attempts.
    if (!blocked) {
      EXPECT_EQ(device->moves.load(), 1);  // At zero: readback refresh, no no-op move.
      device->width = .006;                // The gap reopens during the same hold.
      const auto reopen_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
      while (device->moves.load() == 1 && std::chrono::steady_clock::now() < reopen_deadline)
        step(.8);
      EXPECT_GT(device->moves.load(), 1);
      EXPECT_DOUBLE_EQ(device->target, 0);
      while (core.busy() && std::chrono::steady_clock::now() < reopen_deadline)
        step(.8);
      ASSERT_FALSE(core.busy());
    }
    const int moves = device->moves.load();
    device->width = .004;
    for (int n = 0; n < 100; ++n)
      step(0);
    EXPECT_EQ(device->moves.load(), moves);
    const auto restart_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
    while (device->moves.load() == moves && std::chrono::steady_clock::now() < restart_deadline)
      step(.8);
    EXPECT_GT(device->moves.load(), moves);
    EXPECT_DOUBLE_EQ(device->target, 0);
    step(0);
  }
}
TEST(HandBackend, MissingStopAcknowledgementStillFailsClosed) {
  auto device = std::make_shared<ScriptedHand>();
  device->delayed = true;
  device->reject_all_stops = true;
  ft::HandWorker worker(device);
  worker.issue({ft::HandOperation::close, 0, .02}, .012);
  for (int n = 0; n < 1000 && device->target.load() < 0; ++n)
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  ASSERT_GE(device->target.load(), 0);
  worker.issue({ft::HandOperation::stop}, 0);
  const auto r = collect(worker);
  ASSERT_TRUE(r);
  EXPECT_FALSE(r->success);
  EXPECT_FALSE(r->communication_ok);
  EXPECT_EQ(device->stops.load(), 1);
  EXPECT_NE(r->error.find("already stopped or communication fault"), std::string::npos);
}
TEST(HandBackend, FinalReadbackCannotGenerateRedundantStopTraffic) {
  auto device = std::make_shared<ScriptedHand>();
  device->slow_final_read = true;
  ft::HandWorker worker(device);
  worker.issue({ft::HandOperation::close, 0, .02}, .012);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (device->reads.load() < 2 && std::chrono::steady_clock::now() < deadline)
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  ASSERT_GE(device->reads.load(), 2);
  worker.issue({ft::HandOperation::stop}, 0);
  const auto r = collect(worker);
  ASSERT_TRUE(r);
  EXPECT_TRUE(r->success);
  EXPECT_TRUE(r->stopped);
  EXPECT_EQ(device->stops.load(), 0);
  EXPECT_EQ(r->stop_calls, 0u);
  EXPECT_GT(r->readback_ms, 100);
  EXPECT_LT(r->command_ms, 50);
}
namespace {
// Reproduce a blocking SDK stop acknowledgement that outlasts the retry
// window, with move completion delivered just after the acknowledgement.
struct LateStopHand final : ft::HandDevice {
  std::atomic<bool> moving{false}, acknowledged{false};
  std::atomic<int> moves{0};
  double width{.012};
  int completion_delay_ms{5};
  bool homing() override { return true; }
  ft::HandFeedback read() override { return {width, .08}; }
  bool move(double value, double) override {
    if (++moves == 1) {
      moving = true;
      while (!acknowledged.load())
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      std::this_thread::sleep_for(std::chrono::milliseconds(completion_delay_ms));
      return false;
    }
    width = value;
    return true;
  }
  bool stop() override {
    std::this_thread::sleep_for(std::chrono::milliseconds(2050));
    acknowledged = true;
    return true;
  }
};
}  // namespace
TEST(HandBackend, LateStopAcknowledgementAndCompletionPermitOpeningThenClosing) {
  auto device = std::make_shared<LateStopHand>();
  ft::HandWorker worker(device);
  worker.issue({ft::HandOperation::close, 0, .05}, .012);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (!device->moving.load() && std::chrono::steady_clock::now() < deadline)
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  ASSERT_TRUE(device->moving.load());
  worker.issue({ft::HandOperation::stop}, 0);
  auto r = collect(worker, std::chrono::seconds(4));
  ASSERT_TRUE(r);
  ASSERT_TRUE(r->success);
  ASSERT_TRUE(r->communication_ok);
  EXPECT_TRUE(r->stopped);
  EXPECT_TRUE(r->stop_acknowledged);
  EXPECT_FALSE(r->stop_retry_exhausted);
  EXPECT_GT(r->stop_call_ms, 2000);
  worker.issue({ft::HandOperation::open, .025, .1}, r->width);
  r = collect(worker);
  ASSERT_TRUE(r);
  ASSERT_TRUE(r->success);
  EXPECT_DOUBLE_EQ(r->width, .025);
  worker.issue({ft::HandOperation::close, 0, .05}, r->width);
  r = collect(worker);
  ASSERT_TRUE(r);
  EXPECT_TRUE(r->success);
  EXPECT_DOUBLE_EQ(r->width, 0);
}
TEST(HandBackend, StopFailuresRetainTheirCauseAndNeverRetry) {
  for (int failure = 1; failure <= 4; ++failure) {
    auto device = std::make_shared<ScriptedHand>();
    device->delayed = true;
    device->stop_failure = failure;
    ft::HandWorker worker(device);
    worker.issue({ft::HandOperation::close, 0, .05}, .012);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
    while (device->target.load() < 0 && std::chrono::steady_clock::now() < deadline)
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    ASSERT_GE(device->target.load(), 0);
    worker.issue({ft::HandOperation::stop}, 0);
    const auto r = collect(worker);
    ASSERT_TRUE(r);
    EXPECT_FALSE(r->success);
    EXPECT_FALSE(r->communication_ok);
    EXPECT_FALSE(r->stop_acknowledged);
    EXPECT_FALSE(r->stop_retry_exhausted);
    EXPECT_FALSE(r->retryable);
    EXPECT_EQ(device->stops.load(), 1);
    const char* causes[] = {"unsuccessful", "stop command rejected", "stop connection lost",
                            "already stopped or communication fault"};
    EXPECT_NE(r->error.find(causes[failure - 1]), std::string::npos);
    ft::HandCore core(ft::HandParameters{});
    core.calibrate(true, true);
    core.complete(ft::HandOperation::calibrate, true, true, .012, .08, 1);
    core.complete(r->operation, r->success, r->communication_ok, r->width, r->maximum_width, 2);
    EXPECT_FALSE(core.calibrated());
    // Faults still require explicit calibration; ordinary moves cannot reconnect.
    worker.issue({ft::HandOperation::open, .025, .1}, r->width);
    const auto next = collect(worker);
    ASSERT_TRUE(next);
    EXPECT_FALSE(next->communication_ok);
    EXPECT_EQ(device->moves.load(), 1);
  }
}
TEST(HandBackend, TransportFaultAfterAcknowledgementStillFailsClosed) {
  auto device = std::make_shared<ScriptedHand>();
  device->delayed = true;
  device->transport_fault_after_ack = true;
  ft::HandWorker worker(device);
  worker.issue({ft::HandOperation::close, 0, .05}, .012);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (device->target.load() < 0 && std::chrono::steady_clock::now() < deadline)
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  ASSERT_GE(device->target.load(), 0);
  worker.issue({ft::HandOperation::stop}, 0);
  const auto r = collect(worker);
  ASSERT_TRUE(r);
  EXPECT_TRUE(r->stop_acknowledged);
  EXPECT_FALSE(r->success);
  EXPECT_FALSE(r->communication_ok);
  EXPECT_NE(r->error.find("stop transport lost after acknowledgement"), std::string::npos);
}
TEST(HandBackend, AcknowledgementWithoutCommandCompletionStillExhaustsRetryWindow) {
  auto device = std::make_shared<LateStopHand>();
  device->completion_delay_ms = 100;
  ft::HandWorker worker(device);
  worker.issue({ft::HandOperation::close, 0, .05}, .012);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (!device->moving.load() && std::chrono::steady_clock::now() < deadline)
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  ASSERT_TRUE(device->moving.load());
  worker.issue({ft::HandOperation::stop}, 0);
  const auto r = collect(worker, std::chrono::seconds(4));
  ASSERT_TRUE(r);
  EXPECT_TRUE(r->stop_acknowledged);
  EXPECT_TRUE(r->stop_retry_exhausted);
  EXPECT_FALSE(r->success);
  EXPECT_FALSE(r->communication_ok);
  EXPECT_NE(r->error.find("retry window elapsed before command completion"), std::string::npos);
}
