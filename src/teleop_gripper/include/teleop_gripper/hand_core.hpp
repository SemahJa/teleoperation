#pragma once
#include <array>
#include <cstdint>

namespace teleop_gripper {
enum class HandOperation { none, calibrate, close, open, stop };
enum class HandState { unknown, open, stopped, closed };
// Software readback allowance, not extra commanded travel or a force tolerance.
// Homing estimates the maximum; endpoint measurements may differ by a few um.
inline constexpr double kWidthReadbackTolerance = 1e-5;
bool valid_hand_feedback(double width, double maximum_width) noexcept;
struct HandParameters {
  double press{0.75}, release{0.25}, speed{0.02}, open_speed{0.1};
  double open_width{-1}, close_width{0};
  uint64_t debounce_ns{0}, timeout_ns{100000000}, uncertainty_ns{10000000};
  uint64_t close_retry_ns{200000000};  // Held-close refresh period; zero disables repetition.
  bool valid() const noexcept;
};
struct GripInput {
  std::array<char, 65> session{};
  uint64_t sequence{0}, source_ns{0}, uncertainty_ns{0}, generation{0};
  double value{0};
  bool valid{false};
  bool grip_active{true}, open_active{false}, open_pressed{false};
};
struct HandRequest {
  HandOperation operation{HandOperation::none};
  double width{0}, speed{0.02};
};
class HandCore {
 public:
  explicit HandCore(const HandParameters& parameters) : p_(parameters) {}
  HandRequest input(const GripInput& input, uint64_t now_ns) noexcept;
  HandRequest calibrate(bool empty_hand, bool operator_confirm) noexcept;
  HandRequest complete(HandOperation operation, bool success, bool communication_ok, double width,
                       double max_width, uint64_t now_ns, bool retryable = false) noexcept;
  HandRequest watchdog(uint64_t now_ns) noexcept;
  HandRequest shutdown() noexcept;
  bool busy() const noexcept { return operation_ != HandOperation::none; }
  bool calibrated() const noexcept { return calibrated_; }
  bool release_required() const noexcept { return need_release_; }
  HandState state() const noexcept { return state_; }
  double width() const noexcept { return width_; }
  double maximum_width() const noexcept { return maximum_width_; }
  uint64_t input_source_ns() const noexcept { return last_.source_ns; }
  const char* message() const noexcept { return message_; }

 private:
  void reset_input() noexcept;
  HandRequest interrupt() noexcept;
  HandRequest start(HandOperation operation, uint64_t now_ns) noexcept;
  HandParameters p_;
  GripInput last_{};
  bool identity_{false}, need_release_{true}, calibrated_{false};
  bool pressed_{false}, consumed_{false}, stopping_{false}, open_after_stop_{false};
  bool open_down_{false}, open_release_required_{true};
  bool close_after_stop_{false};
  uint64_t press_start_ns_{0}, retry_due_ns_{0};
  double width_{0}, maximum_width_{0};
  HandOperation operation_{HandOperation::none};
  HandState state_{HandState::unknown};
  const char* message_{"uncalibrated; explicit empty-hand calibration required"};
};
}  // namespace teleop_gripper
