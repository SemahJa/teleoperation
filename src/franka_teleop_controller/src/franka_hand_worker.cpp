#include <algorithm>
#include <chrono>
#include <future>
#include <memory>
#include <stdexcept>
#include <thread>
#include <franka/gripper.h>
#include <franka/exception.h>
#include <quest_teleop_interfaces/msg/gripper_status.hpp>
#include <quest_teleop_interfaces/msg/quest_controller_state.hpp>
#include <quest_teleop_interfaces/srv/calibrate_gripper.hpp>
#include <rclcpp/rclcpp.hpp>
#include "franka_teleop_controller/hand_core.hpp"

namespace ft = franka_teleop_controller;
using Clock = std::chrono::steady_clock;
uint64_t monotonic_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count();
}
struct Result {
  ft::HandOperation operation{ft::HandOperation::none};
  bool success{false}, communication_ok{false};
  double width{0}, maximum_width{0};
  std::string error;
};
const char* operation_name(ft::HandOperation operation) {
  switch (operation) {
    case ft::HandOperation::calibrate: return "calibrate";
    case ft::HandOperation::open: return "open";
    case ft::HandOperation::grasp: return "grasp";
    default: return "idle";
  }
}
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("franka_hand_worker");
  int exit_code = 0;
  try {
    const auto backend = node->declare_parameter<std::string>("backend", "mock");
    const auto ip = node->declare_parameter<std::string>("robot_ip", "");
    const auto confirmed = node->declare_parameter<bool>("operator_confirm", false);
    const auto controller = node->declare_parameter<std::string>("controller_id", "right");
    const auto topic = node->declare_parameter<std::string>("controller_state_topic",
        "/quest_usb_bridge/" + controller + "/controller_state");
    if ((backend != "mock" && backend != "hardware") ||
        (controller != "right" && controller != "left")) throw std::invalid_argument("invalid backend/controller ID");
    const bool hardware = backend == "hardware";
    if (hardware && (!confirmed || ip.empty()))
      throw std::invalid_argument("hardware Hand requires robot_ip and operator_confirm:=true");
    ft::HandParameters p;
    p.speed = node->declare_parameter("speed_mps", p.speed);
    p.force = node->declare_parameter("grasp_force_n", p.force);
    p.grasp_width = node->declare_parameter("grasp_width_m", p.grasp_width);
    p.epsilon_inner = node->declare_parameter("epsilon_inner_m", p.epsilon_inner);
    p.epsilon_outer = node->declare_parameter("epsilon_outer_m", p.epsilon_outer);
    p.open_width = node->declare_parameter("open_width_m", p.open_width);
    p.press = node->declare_parameter("grip_press", p.press);
    p.release = node->declare_parameter("grip_release", p.release);
    const double debounce = node->declare_parameter("grip_debounce_ms", 50.0);
    const double timeout = node->declare_parameter("command_timeout_ms", 100.0);
    const double uncertainty = node->declare_parameter("max_clock_uncertainty_ms", 10.0);
    const double mock_duration = node->declare_parameter("mock_operation_duration_ms", 100.0);
    if (!std::isfinite(debounce) || debounce <= 0 || debounce > 10000 ||
        !std::isfinite(timeout) || timeout <= 0 || timeout > 100 ||
        !std::isfinite(uncertainty) || uncertainty <= 0 || uncertainty > 10 ||
        !std::isfinite(mock_duration) || mock_duration < 0 || mock_duration > 10000)
      throw std::invalid_argument("invalid Hand debounce/freshness/mock duration");
    p.debounce_ns = static_cast<uint64_t>(debounce * 1e6);
    p.timeout_ns = static_cast<uint64_t>(timeout * 1e6);
    p.uncertainty_ns = static_cast<uint64_t>(uncertainty * 1e6);
    if (!p.valid()) throw std::invalid_argument("invalid Hand settings");
    ft::HandCore core(p);
    std::unique_ptr<franka::Gripper> hand;
    std::future<Result> pending;
    ft::HandOperation operation = ft::HandOperation::none;
    uint64_t started = 0, completed = 0;
    auto events = node->create_publisher<quest_teleop_interfaces::msg::GripperStatus>(
        "gripper_events", rclcpp::QoS(1).transient_local());
    auto publish = [&](const std::string& detail = "") {
      quest_teleop_interfaces::msg::GripperStatus status;
      status.header.stamp = node->now();
      status.state = core.state() == ft::HandState::open ? "open" :
          (core.state() == ft::HandState::grasped ? "grasped" : "unknown");
      status.operation = operation_name(operation);
      status.message = detail.empty() ? core.message() : detail;
      status.calibrated = core.calibrated(); status.busy = core.busy();
      status.hardware_backend = hardware; status.width = core.width();
      status.maximum_width = core.maximum_width();
      status.operations_started = started; status.operations_completed = completed;
      events->publish(status);
    };
    auto issue = [&](const ft::HandRequest& request) {
      if (request.operation == ft::HandOperation::none) return;
      operation = request.operation; ++started;
      // This worker thread owns all blocking Hand API calls. The ROS executor
      // stays responsive; there is no arm-torque callback or press queue here.
      pending = std::async(std::launch::async, [&, request] {
        Result result; result.operation = request.operation;
        try {
          if (!hardware) {
            std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(mock_duration));
            result.success = result.communication_ok = true;
            result.maximum_width = .08;
            result.width = request.operation == ft::HandOperation::calibrate ? .08 : request.width;
          } else {
            // Construct only on a deliberate operation (initially explicit
            // calibration). Communication faults discard the connection; only
            // a later explicit calibration may reconnect it.
            if (!hand) hand = std::make_unique<franka::Gripper>(ip);
            switch (request.operation) {
              case ft::HandOperation::calibrate: result.success = hand->homing(); break;
              case ft::HandOperation::open: result.success = hand->move(request.width, request.speed); break;
              case ft::HandOperation::grasp:
                result.success = hand->grasp(request.width, request.speed, request.force,
                                             request.epsilon_inner, request.epsilon_outer); break;
              default: break;
            }
            const auto state = hand->readOnce();
            result.width = state.width; result.maximum_width = state.max_width;
            if (request.operation == ft::HandOperation::grasp) result.success &= state.is_grasped;
            result.communication_ok = true;
          }
        } catch (const franka::CommandException& e) {
          result.error = e.what();
          try {
            const auto state = hand->readOnce();
            result.width = state.width; result.maximum_width = state.max_width;
            result.communication_ok = true;
          } catch (const std::exception&) { hand.reset(); }
        } catch (const std::exception& e) { result.error = e.what(); hand.reset(); }
        return result;
      });
      publish();
    };
    auto calibration = node->create_service<quest_teleop_interfaces::srv::CalibrateGripper>(
        "calibrate_gripper", [&](const std::shared_ptr<quest_teleop_interfaces::srv::CalibrateGripper::Request> request,
                                  std::shared_ptr<quest_teleop_interfaces::srv::CalibrateGripper::Response> response) {
          const auto action = core.calibrate(request->empty_hand_confirm, request->operator_confirm);
          response->accepted = action.operation == ft::HandOperation::calibrate;
          response->message = core.message();
          issue(action); publish();
        });
    auto input = node->create_subscription<quest_teleop_interfaces::msg::QuestControllerState>(
        topic, rclcpp::QoS(20), [&](quest_teleop_interfaces::msg::QuestControllerState::ConstSharedPtr state) {
          if (state->controller_id != controller) return;
          ft::GripInput c;
          c.sequence = state->sequence; c.source_ns = state->source_time_pc_monotonic_ns;
          c.uncertainty_ns = state->clock_uncertainty_ns;
          c.generation = state->reference_space_generation; c.value = state->grip_value;
          c.valid = state->connection_valid && state->session_focused && state->pose_action_active &&
              state->position_valid && state->orientation_valid && state->position_tracked &&
              state->orientation_tracked && state->grip_active && state->source_time_valid &&
              state->clock_sync_valid && state->reference_space == "LOCAL" &&
              !state->session_id.empty() && state->session_id.size() <= 64 &&
              state->session_id.find('\0') == std::string::npos;
          if (c.valid) std::copy(state->session_id.begin(), state->session_id.end(), c.session.begin());
          const auto previous_message = core.message();
          issue(core.input(c, monotonic_ns()));
          if (previous_message != core.message()) publish();
        });
    auto timer = node->create_wall_timer(std::chrono::milliseconds(10), [&] {
      core.watchdog(monotonic_ns());
      if (pending.valid() && pending.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
        const auto result = pending.get();
        core.complete(result.operation, result.success, result.communication_ok,
                      result.width, result.maximum_width);
        operation = ft::HandOperation::none; ++completed;
        publish(result.error.empty() ? "" : std::string(core.message()) + ": " + result.error);
      }
    });
    publish();
    RCLCPP_INFO(node->get_logger(), "%s Hand worker; no startup motion; explicit empty-hand calibration required", backend.c_str());
    rclcpp::spin(node);
    // Loss of Quest/ROS never opens, reverses or retries an issued operation.
    if (pending.valid()) pending.wait();
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node->get_logger(), "Hand worker stopped: %s", e.what()); exit_code = 2;
  }
  rclcpp::shutdown();
  return exit_code;
}
