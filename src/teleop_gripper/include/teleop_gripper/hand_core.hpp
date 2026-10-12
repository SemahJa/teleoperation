#pragma once
#include <cmath>
#include <array>
#include <cstdint>

namespace teleop_gripper {
enum class HandOperation { none, calibrate, open, grasp };
enum class HandState { unknown, open, grasped };
struct HandParameters {
  double press{0.75}, release{0.25}, speed{0.02}, force{5};
  double open_width{-1}, grasp_width{-1}, epsilon_inner{-1}, epsilon_outer{-1};
  uint64_t debounce_ns{50000000}, timeout_ns{100000000}, uncertainty_ns{10000000};
  bool valid() const noexcept;
};
struct GripInput {
  std::array<char, 65> session{};
  uint64_t sequence{0}, source_ns{0}, uncertainty_ns{0}, generation{0};
  double value{0};
  bool valid{false};
};
struct HandRequest {
  HandOperation operation{HandOperation::none};
  double width{0}, speed{0.02}, force{5}, epsilon_inner{0}, epsilon_outer{0};
};
class HandCore {
 public:
  explicit HandCore(const HandParameters& parameters) : p_(parameters) {}
  HandRequest input(const GripInput& input, uint64_t now_ns) noexcept;
  HandRequest calibrate(bool empty_hand, bool operator_confirm) noexcept;
  void complete(HandOperation operation, bool success, bool communication_ok,
                double width, double max_width) noexcept;
  void watchdog(uint64_t now_ns) noexcept;
  bool busy() const noexcept { return busy_; }
  bool calibrated() const noexcept { return calibrated_; }
  HandState state() const noexcept { return state_; }
  double width() const noexcept { return width_; }
  double maximum_width() const noexcept { return maximum_width_; }
  const char* message() const noexcept { return message_; }
 private:
  void reset_input() noexcept { need_release_ = true; press_start_ns_ = 0; }
  HandParameters p_;
  GripInput last_{};
  bool identity_{false}, need_release_{true}, calibrated_{false}, busy_{false};
  uint64_t press_start_ns_{0};
  double width_{0}, maximum_width_{0};
  HandState state_{HandState::unknown};
  const char* message_{"uncalibrated; explicit empty-hand calibration required"};
};
}  // namespace teleop_gripper
