#include <franka/control_types.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <pthread.h>
#include <sys/mman.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <franka_msgs/msg/franka_state.hpp>
#include <fstream>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <quest_teleop_interfaces/msg/cartesian_teleop_command.hpp>
#include <quest_teleop_interfaces/msg/cbf_status.hpp>
#include <quest_teleop_interfaces/msg/controller_status.hpp>
#include <quest_teleop_interfaces/msg/homing_status.hpp>
#include <quest_teleop_interfaces/msg/quest_controller_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sstream>
#include <stdexcept>
#include <thread>

#include "safety_filter/handoff.hpp"
#include "safety_filter/legacy_pose.hpp"
#include "safety_filter/panda_model_fixture.hpp"
#include "safety_filter/safety_coordinator.hpp"
#include "teleop_controllers/cartesian_impedance_controller.hpp"
#include "teleop_controllers/controller_manager.hpp"
#include "teleop_controllers/damping_controller.hpp"
#include "teleop_controllers/position_hold_controller.hpp"
#include "teleop_controllers/orientation_only_controller.hpp"
#include "tracking_message.hpp"

namespace ft = teleop_controllers;
namespace sf = safety_filter;
using Clock = std::chrono::steady_clock;
uint64_t monotonic_ns() noexcept {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch())
      .count();
}
struct Telemetry {
  ft::ArmState state;
  ft::Output output;
  ft::HomingOutput homing;
  franka::RobotState native;
  sf::cbf::ArmState model;
  sf::SafetyReply safety;
  ft::Vector7 gravity{ft::Vector7::Zero()};
  bool finish_requested{false};
  uint64_t stamp_ns{0}, cycle{0};
};
// Written only by the owning callback, read after all realtime threads join.
struct OwnerTiming {
  uint64_t cycle{0}, source_ns{0}, submitted_ns{0}, finished_ns{0};
  bool accepted{false};
};
void write_pose(geometry_msgs::msg::Pose& p, const Eigen::Vector3d& v,
                const Eigen::Quaterniond& q) {
  p.position.x = v.x();
  p.position.y = v.y();
  p.position.z = v.z();
  p.orientation.x = q.x();
  p.orientation.y = q.y();
  p.orientation.z = q.z();
  p.orientation.w = q.w();
}
template <class T>
T parameter(const std::shared_ptr<rclcpp::Node>& node, const std::string& name, const T& value) {
  if (node->has_parameter(name))
    return node->get_parameter(name).get_value<T>();
  return node->declare_parameter<T>(name, value);
}
int run_arm(const std::shared_ptr<rclcpp::Node>& node, int arm, sf::SafetyCoordinator& coordinator,
            const sf::cbf::SafetyParameters& safety_parameters, std::atomic<unsigned>& prepared,
            std::atomic<unsigned>& connected, unsigned count, OwnerTiming& timing) {
  int result = 0;
  bool owner_fault = false;
  double fault_period = 0;
  Telemetry fault_sample;
  try {
    const auto backend = parameter<std::string>(node, "backend", "mock");
    const auto ip = parameter<std::string>(node, "robot_ip", "");
    const auto confirmed = parameter<bool>(node, "operator_confirm", false);
    const auto frame = parameter<std::string>(node, "base_frame", "panda_1/panda_link0");
    const auto command_topic =
        parameter<std::string>(node, "command_topic", "cartesian_teleop_command");
    const auto pose_topic =
        parameter<std::string>(node, "measured_pose_topic", "teleop_anchor_pose");
    const auto state_topic = parameter<std::string>(node, "state_topic", "robot_state");
    const auto controller_id = parameter<std::string>(node, "controller_id", "right");
    const auto raw_topic = parameter<std::string>(
        node, "controller_state_topic", "/quest_usb_bridge/" + controller_id + "/controller_state");
    const auto home_button = controller_id == "left" ? "x" : "a";
    const double trigger_release = parameter(node, "trigger_release", .25);
    ft::ManagerParameters p;
    ft::CartesianImpedanceParameters impedance;
    impedance.lambda_translation =
        parameter(node, "lambda_translation", impedance.lambda_translation);
    impedance.lambda_rotation = parameter(node, "lambda_rotation", impedance.lambda_rotation);
    impedance.gain_translation = parameter(node, "gain_translation", impedance.gain_translation);
    impedance.gain_rotation = parameter(node, "gain_rotation", impedance.gain_rotation);
    const double timeout = parameter(node, "command_timeout_ms", 100.0);
    const double uncertainty = parameter(node, "max_clock_uncertainty_ms", 10.0);
    const double future = parameter(node, "max_source_future_ms", 10.0);
    p.torque_rate = parameter(node, "maximum_torque_rate", p.torque_rate);
    p.tool_offset = parameter(node, "tool_offset_m", p.tool_offset);
    p.envelope_margin = parameter(node, "envelope_margin_m", p.envelope_margin);
    const auto center = parameter<std::vector<double>>(node, "envelope_center", {0, 0, 0.333});
    const auto limits =
        parameter<std::vector<double>>(node, "torque_limits", {87, 87, 87, 87, 12, 12, 12});
    if (!std::isfinite(timeout) || timeout <= 0 || timeout > 100 || !std::isfinite(uncertainty) ||
        uncertainty <= 0 || uncertainty > 10 || !std::isfinite(future) || future < 0 ||
        future > 10 || center.size() != 3 || limits.size() != 7)
      throw std::invalid_argument("invalid watchdog/clock/envelope/torque settings");
    p.timeout_ns = static_cast<uint64_t>(timeout * 1e6);
    p.uncertainty_ns = static_cast<uint64_t>(uncertainty * 1e6);
    p.future_ns = static_cast<uint64_t>(future * 1e6);
    p.envelope_center = Eigen::Map<const Eigen::Vector3d>(center.data());
    p.torque_limits = Eigen::Map<const ft::Vector7>(limits.data());
    if (!p.valid() || !impedance.valid() || frame.empty() ||
        (backend != "mock" && backend != "hardware") ||
        (controller_id != "left" && controller_id != "right") || !std::isfinite(trigger_release) ||
        trigger_release < 0 || trigger_release >= .75)
      throw std::invalid_argument("invalid controller parameters or backend");
    const bool hardware = backend == "hardware";
    if (hardware && (ip.empty() || !confirmed))
      throw std::invalid_argument("hardware backend requires robot_ip and operator_confirm:=true");
    // Match manager preflight to the final shared actuator limits, including
    // scaling and any stricter per-arm caps. Never apply the scale twice.
    p.torque_limits = p.torque_limits.cwiseMin(safety_parameters.maximum_joint_torque);
    p.torque_rate = std::min(p.torque_rate, safety_parameters.maximum_torque_rate);
    ft::HomingParameters hp;
    hp.speed_factor = parameter(node, "homing_speed_factor", hp.speed_factor);
    hp.completion_timeout = parameter(node, "homing_timeout_seconds", hp.completion_timeout);
    hp.stop_duration = parameter(node, "homing_stop_duration_seconds", hp.stop_duration);
    hp.stop_acceleration = parameter(node, "homing_stop_acceleration", hp.stop_acceleration);
    hp.stop_timeout = parameter(node, "homing_stop_timeout_seconds", hp.stop_timeout);
    const auto hk = parameter<std::vector<double>>(
        node, "homing_stiffness",
        std::vector<double>(hp.stiffness.data(), hp.stiffness.data() + 7));
    const auto hd = parameter<std::vector<double>>(
        node, "homing_damping", std::vector<double>(hp.damping.data(), hp.damping.data() + 7));
    if (hk.size() != 7 || hd.size() != 7)
      throw std::invalid_argument("homing gains require seven values");
    hp.stiffness = Eigen::Map<const ft::Vector7>(hk.data());
    hp.damping = Eigen::Map<const ft::Vector7>(hd.data());
    hp.timeout_ns = p.timeout_ns;
    hp.uncertainty_ns = p.uncertainty_ns;
    hp.future_ns = p.future_ns;
    if (!hp.valid())
      throw std::invalid_argument("invalid homing parameters");
    ft::CartesianImpedanceController cartesian(impedance);
    ft::OrientationOnlyController orientation_only(impedance);
    ft::PositionHoldParameters hold;
    const auto hold_k = parameter<std::vector<double>>(
        node, "hold_stiffness",
        std::vector<double>(hold.stiffness.data(), hold.stiffness.data() + 7));
    const auto hold_d = parameter<std::vector<double>>(
        node, "hold_damping", std::vector<double>(hold.damping.data(), hold.damping.data() + 7));
    hold.stiffness_ramp_seconds =
        parameter(node, "hold_stiffness_ramp_seconds", hold.stiffness_ramp_seconds);
    if (hold_k.size() != 7 || hold_d.size() != 7)
      throw std::invalid_argument("hold_stiffness and hold_damping require seven values");
    hold.stiffness = Eigen::Map<const ft::Vector7>(hold_k.data());
    hold.damping = Eigen::Map<const ft::Vector7>(hold_d.data());
    if (!hold.valid())
      throw std::invalid_argument("invalid position hold gains or stiffness ramp");
    ft::PositionHoldController position_hold(hold);
    ft::DampingController damping(impedance.gain_translation, impedance.gain_rotation);
    ft::ControllerManager manager(p, hp, cartesian, orientation_only, position_hold, damping);
    const bool homing_enabled = parameter<bool>(node, "homing_enabled", true);
    sf::SpscQueue<ft::HomingInput> home_inputs;
    sf::SpscQueue<ft::Command> commands;
    sf::LatestSnapshot<Telemetry> snapshots;
    auto tracking_queue = std::make_unique<sf::SpscQueue<ft::TrackingSnapshot, 1024>>();
    sf::LatestSnapshot<ft::TrackingSnapshot> live_tracking;
    std::atomic<uint64_t> tracking_drops{0};
    std::atomic<uint64_t> overflows{0};
    std::atomic<uint64_t> home_overflows{0};
    auto raw_input = node->create_subscription<quest_teleop_interfaces::msg::QuestControllerState>(
        raw_topic, rclcpp::QoS(20),
        [&](quest_teleop_interfaces::msg::QuestControllerState::ConstSharedPtr msg) {
          if (!homing_enabled || msg->controller_id != controller_id)
            return;
          ft::HomingInput c;
          c.receipt_ns = monotonic_ns();
          c.source_ns = msg->source_time_pc_monotonic_ns;
          c.sequence = msg->sequence;
          c.generation = msg->reference_space_generation;
          c.uncertainty_ns = msg->clock_uncertainty_ns;
          int matches = 0;
          bool button_active = false;
          for (const auto& button : msg->buttons)
            if (button.name == home_button) {
              ++matches;
              c.button_pressed = button.pressed;
              button_active = button.active;
            }
          c.valid = matches == 1 && button_active && msg->connection_valid &&
                    msg->session_focused && msg->pose_action_active && msg->position_valid &&
                    msg->orientation_valid && msg->position_tracked && msg->orientation_tracked &&
                    msg->trigger_active && std::isfinite(msg->trigger_value) &&
                    msg->trigger_value >= 0 && msg->trigger_value <= 1 && msg->source_time_valid &&
                    msg->clock_sync_valid && msg->reference_space == "LOCAL" &&
                    !msg->session_id.empty() && msg->session_id.size() <= 64 &&
                    msg->session_id.find('\0') == std::string::npos;
          c.trigger_released = msg->trigger_value <= trigger_release;
          if (c.valid)
            std::copy(msg->session_id.begin(), msg->session_id.end(), c.session.begin());
          if (!home_inputs.push(c))
            home_overflows.fetch_add(1, std::memory_order_release);
        });
    auto input = node->create_subscription<quest_teleop_interfaces::msg::CartesianTeleopCommand>(
        command_topic, rclcpp::QoS(10),
        [&](quest_teleop_interfaces::msg::CartesianTeleopCommand::ConstSharedPtr msg) {
          ft::Command c;
          c.receipt_ns = monotonic_ns();
          c.source_ns = msg->source_time_pc_monotonic_ns;
          c.uncertainty_ns = msg->clock_uncertainty_ns;
          c.sequence = msg->sequence;
          c.generation = msg->reference_space_generation;
          c.location_ns = msg->openxr_location_time_ns;
          c.enabled = msg->motion_enabled;
          c.orientation_only = msg->orientation_only;
          c.released = msg->trigger_released;
          c.valid = msg->header.frame_id == frame && msg->input_valid && msg->source_time_valid &&
                    msg->clock_sync_valid && !msg->session_id.empty() &&
                    msg->session_id.size() <= 64 && msg->session_id.find('\0') == std::string::npos;
          if (c.valid)
            std::copy(msg->session_id.begin(), msg->session_id.end(), c.session.begin());
          const auto& position = msg->target_pose.position;
          const auto& q = msg->target_pose.orientation;
          c.position << position.x, position.y, position.z;
          c.orientation = Eigen::Quaterniond(q.w, q.x, q.y, q.z);
          const auto& t = msg->desired_twist;
          c.twist << t.linear.x, t.linear.y, t.linear.z, t.angular.x, t.angular.y, t.angular.z;
          const auto& requested = msg->requested_twist;
          c.requested_twist << requested.linear.x, requested.linear.y, requested.linear.z,
              requested.angular.x, requested.angular.y, requested.angular.z;
          c.requested_twist_valid = msg->requested_twist_valid && c.requested_twist.allFinite();
          if (!commands.push(c))
            overflows.fetch_add(1, std::memory_order_release);
        });
    auto measured = node->create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic, 1);
    auto joints = node->create_publisher<sensor_msgs::msg::JointState>("joint_states", 1);
    auto robot_state = node->create_publisher<franka_msgs::msg::FrankaState>(state_topic, 1);
    auto status = node->create_publisher<quest_teleop_interfaces::msg::ControllerStatus>(
        "controller_status", rclcpp::QoS(1).transient_local());
    auto home_status = node->create_publisher<quest_teleop_interfaces::msg::HomingStatus>(
        "homing_status", rclcpp::QoS(1).transient_local());
    auto safety_status = node->create_publisher<quest_teleop_interfaces::msg::CbfStatus>(
        "cbf_status", rclcpp::QoS(1).transient_local());
    auto tracking_samples = node->create_publisher<quest_teleop_interfaces::msg::TrackingSample>(
        "tracking_samples", rclcpp::QoS(2048));
    auto tracking_status = node->create_publisher<quest_teleop_interfaces::msg::TrackingSample>(
        "tracking_status", rclcpp::QoS(10));
    auto publish_tracking = [&](const ft::TrackingSnapshot& sample, const auto& publisher) {
      const auto now = monotonic_ns();
      const auto stamp = node->now() - rclcpp::Duration::from_nanoseconds(static_cast<int64_t>(
                                           now >= sample.state_ns ? now - sample.state_ns : 0));
      publisher->publish(
          ft::tracking_message(sample, frame, stamp, safety_parameters.maximum_joint_torque,
                               safety_parameters.maximum_torque_rate, safety_parameters.enabled,
                               tracking_drops.load(std::memory_order_relaxed), impedance));
    };
    auto timer = node->create_wall_timer(std::chrono::milliseconds(10), [&] {
      ft::TrackingSnapshot tracking;
      if (live_tracking.read(tracking))
        publish_tracking(tracking, tracking_status);
      Telemetry sample;
      if (!snapshots.read(sample))
        return;
      const auto now = monotonic_ns();
      const auto stamp = node->now() - rclcpp::Duration::from_nanoseconds(static_cast<int64_t>(
                                           now >= sample.stamp_ns ? now - sample.stamp_ns : 0));
      geometry_msgs::msg::PoseStamped pose;
      pose.header.stamp = stamp;
      pose.header.frame_id = frame;
      write_pose(pose.pose, sample.state.position, sample.state.orientation);
      measured->publish(pose);
      sensor_msgs::msg::JointState joint;
      joint.header = pose.header;
      for (int j = 0; j < 7; ++j)
        joint.name.push_back("panda_joint" + std::to_string(j + 1));
      joint.position.assign(sample.state.q.data(), sample.state.q.data() + 7);
      joint.velocity.assign(sample.state.dq.data(), sample.state.dq.data() + 7);
      joints->publish(joint);
      franka_msgs::msg::FrankaState state;
      state.header = pose.header;
      std::copy_n(sample.state.q.data(), 7, state.q.begin());
      std::copy_n(sample.state.dq.data(), 7, state.dq.begin());
      state.tau_j = sample.native.tau_J;
      state.tau_j_d = sample.native.tau_J_d;
      state.dtau_j = sample.native.dtau_J;
      Eigen::Affine3d actual =
          Eigen::Translation3d(sample.state.position) * sample.state.orientation;
      Eigen::Affine3d target =
          Eigen::Translation3d(sample.output.target_position) * sample.output.target_orientation;
      Eigen::Map<Eigen::Matrix4d>(state.o_t_ee.data()) = actual.matrix();
      Eigen::Map<Eigen::Matrix4d>(state.o_t_ee_d.data()) = target.matrix();
      Eigen::Map<Eigen::Matrix4d>(state.o_t_ee_c.data()) = target.matrix();
      std::copy_n(sample.output.desired_twist.data(), 6, state.o_dp_ee_d.begin());
      state.time = sample.native.time.toSec();
      state.robot_mode = static_cast<uint8_t>(sample.native.robot_mode);
      robot_state->publish(state);
      quest_teleop_interfaces::msg::ControllerStatus event;
      event.header = pose.header;
      event.hardware_backend = hardware;
      event.motion_enabled = sample.output.motion_enabled;
      event.reason = ft::reason_name(sample.output.reason);
      event.source_time_pc_monotonic_ns = sample.output.source_ns;
      event.sequence = sample.output.sequence;
      event.command_queue_overflows = overflows.load(std::memory_order_acquire);
      event.torque_rate_limited = sample.output.rate_limited;
      event.torque_magnitude_limited = sample.output.magnitude_limited;
      std::copy_n(sample.output.nominal.data(), 7, event.nominal_torque.begin());
      std::copy_n(sample.output.torque.data(), 7, event.commanded_torque.begin());
      status->publish(event);
      quest_teleop_interfaces::msg::HomingStatus home;
      home.header = pose.header;
      home.controller_id = controller_id;
      home.phase = ft::home_phase_name(sample.homing.phase);
      home.message = sample.homing.message;
      home.request_id = sample.homing.request_id;
      home.active = sample.homing.active;
      home.stop_failed = sample.homing.stop_failed;
      home.elapsed_seconds = sample.homing.elapsed;
      home.maximum_joint_position_error = sample.homing.position_error;
      home.maximum_joint_velocity = sample.homing.velocity;
      std::copy_n(sample.homing.desired_q.data(), 7, home.desired_joint_position.begin());
      std::copy_n(sample.homing.desired_dq.data(), 7, home.desired_joint_velocity.begin());
      home_status->publish(home);
      quest_teleop_interfaces::msg::CbfStatus safety;
      safety.header = pose.header;
      safety.hardware_backend = hardware;
      const auto& filtered = sample.safety.result;
      safety.solver_succeeded = filtered.solver_succeeded;
      safety.fallback_active = filtered.fallback_active;
      safety.fatal_input_fault = filtered.fatal_input_fault;
      safety.solver_status = filtered.solver_status;
      safety.stop_reason = sf::safety_stop_name(coordinator.stopped());
      safety.control_cycle = sample.cycle;
      safety.source_time_pc_monotonic_ns = sample.safety.source_ns;
      if (sample.safety.source_ns[0] && sample.safety.source_ns[1])
        safety.state_skew_ns = std::max(sample.safety.source_ns[0], sample.safety.source_ns[1]) -
                               std::min(sample.safety.source_ns[0], sample.safety.source_ns[1]);
      safety.consecutive_failures = sample.safety.consecutive_failures;
      safety.filter_time_seconds = filtered.filter_time_seconds;
      safety.solver_time_seconds = filtered.solver_time_seconds;
      safety.maximum_constraint_violation = filtered.maximum_constraint_violation;
      safety.minimum_table_clearance = filtered.minimum_table_clearance;
      safety.minimum_self_clearance = filtered.minimum_self_clearance;
      safety.minimum_inter_arm_clearance = filtered.minimum_inter_arm_clearance;
      safety.intervention_norm = filtered.arm_intervention_norm[arm];
      safety.active_constraints = filtered.active_constraints;
      safety.inter_arm_constraints = filtered.inter_arm_constraints;
      std::copy_n(sample.output.torque.data(), 7, safety.safe_torque.begin());
      safety_status->publish(safety);
    });
    prepared.fetch_add(1, std::memory_order_acq_rel);
    while (prepared.load(std::memory_order_acquire) < count &&
           coordinator.stopped() == sf::SafetyStop::none)
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    if (coordinator.stopped() != sf::SafetyStop::none)
      throw std::runtime_error("peer setup failed");
    std::atomic<bool> running{true};
    // Create ROS guard conditions before hardware startup can fail. Constructing
    // rclcpp::spin's executor in the worker raced with shutdown during cleanup.
    rclcpp::executors::SingleThreadedExecutor ros_executor;
    ros_executor.add_node(node);
    std::thread executor([&] {
      try {
        while (running.load(std::memory_order_acquire) && rclcpp::ok())
          ros_executor.spin_once(std::chrono::milliseconds(10));
      } catch (const std::exception& e) {
        if (rclcpp::ok()) {
          RCLCPP_ERROR(node->get_logger(), "ROS executor stopped: %s", e.what());
          coordinator.stop(sf::SafetyStop::owner_failure);
        }
      }
      running.store(false, std::memory_order_release);
    });
    // Dedicated consumer: full-rate ROS serialization cannot delay the command
    // executor or torque callback. A slow consumer loses samples with a counter.
    std::atomic<bool> monitoring_running{true};
    std::thread monitoring;
    try {
      monitoring = std::thread([&] {
        try {
          while (true) {
            ft::TrackingSnapshot sample;
            const bool available = tracking_queue->pop(sample);
            if (available) {
              live_tracking.publish(sample);
              if (tracking_samples->get_subscription_count())
                publish_tracking(sample, tracking_samples);
            } else {
              if (!monitoring_running.load(std::memory_order_acquire))
                break;
              std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
          }
        } catch (const std::exception& e) {
          RCLCPP_ERROR(node->get_logger(), "Monitoring publisher stopped: %s", e.what());
        }
      });
    } catch (...) {
      running.store(false, std::memory_order_release);
      ros_executor.cancel();
      executor.join();
      throw;
    }
    auto join_executor = [&] {
      monitoring_running.store(false, std::memory_order_release);
      if (monitoring.joinable())
        monitoring.join();
      running.store(false, std::memory_order_release);
      ros_executor.cancel();
      if (executor.joinable())
        executor.join();
    };
    uint64_t seen_overflows = 0;
    uint64_t seen_home_overflows = 0;
    uint64_t cycle = 0;
    ft::TrackingSnapshot previous_tracking;
    double shutdown_seconds = 0;
    auto step = [&](Telemetry& telemetry, double period) {
      ft::Command c;
      const auto count = overflows.load(std::memory_order_acquire);
      const bool overflow = count != seen_overflows;
      seen_overflows = count;
      for (int n = 0; n < 64 && commands.pop(c); ++n)
        if (!overflow && running.load(std::memory_order_acquire))
          manager.accept(c, monotonic_ns());
      if (overflow)
        manager.fault(ft::Reason::queue_overflow);
      const auto home_count = home_overflows.load(std::memory_order_acquire);
      const bool home_overflow = home_count != seen_home_overflows;
      seen_home_overflows = home_count;
      ft::HomingInput hi;
      for (int n = 0; n < 64 && home_inputs.pop(hi); ++n)
        if (!home_overflow)
          manager.homing_input(hi, monotonic_ns());
      if (home_overflow || overflow)
        manager.cancel_homing("command queue overflow");
      if (!running.load(std::memory_order_acquire)) {
        manager.shutdown();
      }
      telemetry.cycle = ++cycle;
      // ROS can enqueue input after the RobotState snapshot was timestamped.
      // Evaluate input watchdogs after draining the queues, so newly received
      // input cannot appear to arrive in the future. Keep the original model
      // timestamp for CBF age/deadline checks.
      const auto input_check_ns = monotonic_ns();
      const auto managed = manager.update(telemetry.state, input_check_ns, period);
      telemetry.homing = managed.homing;
      telemetry.output = managed.control;
      if (telemetry.output.reason == ft::Reason::invalid_state || telemetry.homing.stop_failed) {
        owner_fault = true;
        fault_period = period;
        fault_sample = telemetry;
        coordinator.stop(sf::SafetyStop::invalid_model);
        throw std::runtime_error(telemetry.homing.stop_failed
                                     ? "homing controlled stop failed"
                                     : "invalid arm state or actuator interval");
      }
      telemetry.model.q = telemetry.state.q;
      telemetry.model.dq = telemetry.state.dq;
      telemetry.model.coriolis = telemetry.state.coriolis;
      telemetry.model.previous_torque = telemetry.state.previous_torque;
      sf::ModelRequest request;
      request.state = telemetry.model;
      request.nominal = telemetry.output.nominal;
      request.cycle = telemetry.cycle;
      request.stamp_ns = telemetry.stamp_ns;
      request.period = period;
      // This flag distinguishes a moving nominal source from a parked arm for
      // inter-arm priority; joint position holding remains parked. All sources
      // still pass through the same complete safety filter.
      request.motion_active = telemetry.output.motion_enabled || telemetry.homing.active;
      if (!running.load(std::memory_order_acquire))
        shutdown_seconds += period;
      // Homing completion is a mode transition, never a connection finish.
      telemetry.finish_requested = !running.load(std::memory_order_acquire) &&
                                   shutdown_seconds >= .5 && !telemetry.homing.active;
      request.finish_requested = telemetry.finish_requested;
      timing = {telemetry.cycle, telemetry.stamp_ns, monotonic_ns(), 0, false};
      coordinator.submit(arm, request);
      sf::SafetyReply reply;
      bool accepted = false;
      while (coordinator.stopped() == sf::SafetyStop::none) {
        const auto now = monotonic_ns();
        if (coordinator.receive(arm, reply) &&
            coordinator.accepts(arm, telemetry.cycle, telemetry.stamp_ns, monotonic_ns(), reply)) {
          accepted = true;
          break;
        }
        if (now < telemetry.stamp_ns ||
            now - telemetry.stamp_ns > coordinator.limits().result_deadline_ns) {
          coordinator.stop(sf::SafetyStop::result_deadline);
          break;
        }
        // A short bounded sleep lets the equal-priority coordinator run even on
        // one CPU. No unbounded lock/spin or previous-cycle torque is consumed.
        std::this_thread::sleep_for(std::chrono::microseconds(25));
      }
      timing.finished_ns = monotonic_ns();
      timing.accepted = accepted;
      if (!accepted || coordinator.stopped() != sf::SafetyStop::none)
        throw std::runtime_error(std::string("latched CBF stop: ") +
                                 sf::safety_stop_name(coordinator.stopped()));
      telemetry.safety = reply;
      telemetry.output.torque = reply.result.safe_torque.segment<7>(arm * 7);
      // Report actuator pressure against the limits used by the final filter
      // (dual mode may use a stricter shared limit than this arm's settings).
      const double slew = safety_parameters.maximum_torque_rate * std::min(period, .001);
      telemetry.output.rate_limited =
          ((telemetry.output.nominal - telemetry.state.previous_torque).array().abs() > slew).any();
      telemetry.output.magnitude_limited =
          (telemetry.output.nominal.array().abs() > safety_parameters.maximum_joint_torque.array())
              .any();
      if (reply.result.fallback_active) {
        manager.safety_fallback();
        telemetry.output.motion_enabled = false;
        telemetry.output.reason = ft::Reason::safety_filter;
        telemetry.output.position_hold_active = false;
        telemetry.output.desired_twist.setZero();
        telemetry.output.target_position = telemetry.state.position;
        telemetry.output.target_orientation = telemetry.state.orientation;
      }
      snapshots.publish(telemetry);
      ft::TrackingSnapshot tracking;
      tracking.state = telemetry.state;
      tracking.control = telemetry.output;
      tracking.requested = manager.requested_command();
      tracking.homing = telemetry.homing;
      tracking.robot_desired = telemetry.state.previous_torque;
      tracking.measured_torque = Eigen::Map<const ft::Vector7>(telemetry.native.tau_J.data());
      tracking.gravity = telemetry.gravity;
      tracking.cycle = telemetry.cycle;
      tracking.state_ns = telemetry.stamp_ns;
      tracking.control_ns = input_check_ns;
      tracking.issued_ns = monotonic_ns();
      tracking.period = period;
      tracking.hardware = hardware;
      tracking.fallback = reply.result.fallback_active;
      tracking.solver_succeeded = reply.result.solver_succeeded;
      tracking.previous_cycle = previous_tracking.cycle;
      tracking.previous_issued_ns = previous_tracking.issued_ns;
      tracking.previous_nominal = previous_tracking.control.nominal;
      tracking.previous_commanded = previous_tracking.control.torque;
      if (!tracking_queue->push(tracking))
        tracking_drops.fetch_add(1, std::memory_order_relaxed);
      previous_tracking = tracking;
    };
    try {
      RCLCPP_INFO(node->get_logger(), "%s backend; atomic input %s; starts disarmed",
                  backend.c_str(), command_topic.c_str());
      if (!hardware) {
        // Fixed fixture, not a dynamics, reachability, or collision model.
        Telemetry sample;
        sample.state.position << 0.4, 0, 0.4;
        sample.state.q << 0, -0.7854, 0, -2.3562, 0, 1.5708, 0.7854;
        sample.state.jacobian.leftCols<6>().setIdentity();
        sample.model = sf::panda_model_fixture(sample.state.q);
        auto deadline = Clock::now();
        while (!(sample.finish_requested && sample.safety.finish)) {
          sample.stamp_ns = monotonic_ns();
          step(sample, 0.001);
          sample.state.previous_torque = sample.output.torque;
          deadline += std::chrono::milliseconds(1);
          std::this_thread::sleep_until(deadline);
        }
      } else {
        franka::Robot robot(ip);  // default RealtimeConfig::kEnforce
        franka::Model model = robot.loadModel();
        connected.fetch_add(1, std::memory_order_acq_rel);
        while (connected.load(std::memory_order_acquire) < count &&
               coordinator.stopped() == sf::SafetyStop::none)
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
        if (coordinator.stopped() != sf::SafetyStop::none)
          throw std::runtime_error("peer connection failed");
        // Read and report the robot's own state immediately before requesting
        // control. Desk readiness alone cannot diagnose a rejected move.
        // This runs outside the realtime callback and never clears errors.
        const auto startup_state = robot.readOnce();
        std::ostringstream startup;
        startup << "FCI startup robot_ip=" << ip << " mode=" << startup_state.robot_mode
                << " current_errors=" << startup_state.current_errors
                << " last_motion_errors=" << startup_state.last_motion_errors;
        RCLCPP_INFO(node->get_logger(), "%s", startup.str().c_str());
        robot.control(
            [&](const franka::RobotState& state, franka::Duration period) -> franka::Torques {
              Telemetry sample;
              sample.stamp_ns = monotonic_ns();
              sample.native = state;
              sample.state.q = Eigen::Map<const ft::Vector7>(state.q.data());
              sample.state.dq = Eigen::Map<const ft::Vector7>(state.dq.data());
              sample.state.previous_torque = Eigen::Map<const ft::Vector7>(state.tau_J_d.data());
              Eigen::Affine3d pose(Eigen::Map<const Eigen::Matrix4d>(state.O_T_EE.data()));
              if (!sf::orthonormalize_legacy_pose(pose))
                throw std::runtime_error(
                    "invalid measured end-effector transform; exceeds legacy rounding allowance");
              sample.state.position = pose.translation();
              sample.state.orientation = Eigen::Quaterniond(pose.linear());
              const auto j = model.zeroJacobian(franka::Frame::kEndEffector, state);
              const auto c = model.coriolis(state);
              const auto gravity = model.gravity(state);
              sample.gravity = Eigen::Map<const ft::Vector7>(gravity.data());
              sample.state.jacobian = Eigen::Map<const ft::Jacobian>(j.data());
              sample.state.coriolis = Eigen::Map<const ft::Vector7>(c.data());
              const auto mass = model.mass(state);
              sample.model.mass = Eigen::Map<const sf::cbf::Matrix7d>(mass.data());
              for (int frame_index = 0; frame_index < sf::cbf::kModelFrames; ++frame_index) {
                const auto model_frame = static_cast<franka::Frame>(frame_index);
                const auto transform = model.pose(model_frame, state);
                const auto jacobian = model.zeroJacobian(model_frame, state);
                sample.model.frame_pose[frame_index] =
                    Eigen::Affine3d(Eigen::Map<const Eigen::Matrix4d>(transform.data()));
                if (model_frame == franka::Frame::kEndEffector)
                  // On rejection leave the raw matrix for the strict CBF validator
                  // and postmortem diagnostics. Joint/flange frames are untouched.
                  sf::orthonormalize_legacy_pose(sample.model.frame_pose[frame_index]);
                sample.model.frame_jacobian[frame_index] =
                    Eigen::Map<const sf::cbf::Jacobian>(jacobian.data());
              }
              step(sample, period.toSec() == 0 ? .001 : period.toSec());
              if (sample.output.reason == ft::Reason::invalid_state || sample.homing.stop_failed)
                throw std::runtime_error(
                    "invalid robot state or infeasible actuator bounds; terminating FCI ownership");
              std::array<double, 7> torque{};
              std::copy_n(sample.output.torque.data(), 7, torque.begin());
              if (sample.finish_requested && sample.safety.finish)
                return franka::MotionFinished(franka::Torques(torque));
              return franka::Torques(torque);
            },
            true, franka::kMaxCutoffFrequency);
      }
    } catch (...) {
      join_executor();
      throw;
    }
    join_executor();
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node->get_logger(), "Controller stopped: %s", e.what());
    if (owner_fault) {
      // The executor has been joined before this catch; no logging in FCI callback.
      RCLCPP_ERROR(node->get_logger(),
                   "Owner fault cycle %llu: cause=%s period=%.9g homing_phase=%s homing_message=%s "
                   "request_id=%llu elapsed=%.6f home_error=%.9g reference_error=%.9g "
                   "velocity=%.9g stop_failed=%d",
                   static_cast<unsigned long long>(fault_sample.cycle),
                   ft::state_fault_name(fault_sample.output.state_fault), fault_period,
                   ft::home_phase_name(fault_sample.homing.phase), fault_sample.homing.message,
                   static_cast<unsigned long long>(fault_sample.homing.request_id),
                   fault_sample.homing.elapsed, fault_sample.homing.position_error,
                   (fault_sample.homing.desired_q - fault_sample.state.q).cwiseAbs().maxCoeff(),
                   fault_sample.homing.velocity, fault_sample.homing.stop_failed);
    }
    coordinator.stop(sf::SafetyStop::owner_failure);
    result = 2;
  }
  return result;
}

// Only called after all owners and the coordinator have joined: no logging or
// formatting in the torque callback or the real-time solver thread.
void log_model_diagnostics(const rclcpp::Logger& logger, int arm, const sf::ModelRequest& request,
                           const sf::cbf::SafetyParameters& safety) {
  if (!request.cycle) {
    RCLCPP_ERROR(logger, "Model diagnostic arm %d: no submitted model snapshot", arm);
    return;
  }
  const auto& s = request.state;
  const Eigen::LLT<sf::cbf::Matrix7d> factor(s.mass);
  RCLCPP_ERROR(logger,
               "Model diagnostic arm %d cycle %llu: period=%.9g, finite "
               "q/dq/coriolis/torque/mass=%d/%d/%d/%d/%d, mass symmetric=%d positive_definite=%d "
               "symmetry_error=%.9g",
               arm, static_cast<unsigned long long>(request.cycle), request.period, s.q.allFinite(),
               s.dq.allFinite(), s.coriolis.allFinite(), s.previous_torque.allFinite(),
               s.mass.allFinite(), s.mass.isApprox(s.mass.transpose(), 1e-9),
               factor.info() == Eigen::Success,
               (s.mass - s.mass.transpose()).cwiseAbs().maxCoeff());
  for (int frame = 0; frame < sf::cbf::kModelFrames; ++frame) {
    const auto& pose = s.frame_pose[frame];
    const Eigen::Matrix3d rotation = pose.linear();
    const Eigen::Matrix3d gram = rotation.transpose() * rotation;
    if (!pose.matrix().allFinite() || !s.frame_jacobian[frame].allFinite() ||
        !pose.matrix().row(3).isApprox(Eigen::RowVector4d(0, 0, 0, 1), 1e-8) ||
        !gram.isApprox(Eigen::Matrix3d::Identity(), 1e-8) ||
        !(std::abs(rotation.determinant() - 1) < 1e-8)) {
      std::ostringstream matrix;
      matrix << pose.matrix();
      RCLCPP_ERROR(logger,
                   "Invalid model frame %d (0..6 joints, 7 flange, 8 end effector): finite "
                   "pose/jacobian=%d/%d determinant=%.12g orthogonality_error=%.12g transform:\n%s",
                   frame, pose.matrix().allFinite(), s.frame_jacobian[frame].allFinite(),
                   rotation.determinant(),
                   (gram - Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff(),
                   matrix.str().c_str());
    }
  }
  const double slew = safety.maximum_torque_rate * std::clamp(request.period, 0., .001);
  for (int joint = 0; joint < 7; ++joint) {
    const double magnitude = safety.maximum_joint_torque[joint];
    if (std::max(-magnitude, s.previous_torque[joint] - slew) >
        std::min(magnitude, s.previous_torque[joint] + slew))
      RCLCPP_ERROR(logger,
                   "Empty actuator interval joint %d: tau_J_d=%.9g magnitude_limit=%.9g slew=%.9g",
                   joint + 1, s.previous_torque[joint], magnitude, slew);
  }
}

Eigen::Affine3d world_transform(const std::vector<double>& values) {
  if (values.size() != 7)
    throw std::invalid_argument("world_from_base_xyz_xyzw needs seven values");
  for (auto v : values)
    if (!std::isfinite(v))
      throw std::invalid_argument("nonfinite base transform");
  Eigen::Quaterniond q(values[6], values[3], values[4], values[5]);
  if (std::abs(q.norm() - 1) > 1e-6)
    throw std::invalid_argument("base transform quaternion must be unit length");
  return Eigen::Translation3d(values[0], values[1], values[2]) * q;
}
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto root = std::make_shared<rclcpp::Node>("panda_cartesian_impedance_controller");
  int result = 2;
  try {
    const auto mode = parameter<std::string>(root, "mode", "single");
    const auto backend = parameter<std::string>(root, "backend", "mock");
    const bool hardware = backend == "hardware";
    if ((mode != "single" && mode != "dual") || (backend != "mock" && !hardware))
      throw std::invalid_argument("mode must be single/dual and backend mock/hardware");
    const bool confirmed = parameter(root, "operator_confirm", false);
    if (hardware && !confirmed)
      throw std::invalid_argument("hardware backend requires operator_confirm:=true");
    std::array<std::shared_ptr<rclcpp::Node>, 2> nodes;
    std::array<sf::cbf::ArmIdentity, 2> ids;
    sf::cbf::SafetyParameters safety;
    sf::CoordinatorParameters limits;
    sf::ActuatorLimitScales scales;
    const auto& parameter_overrides =
        root->get_node_parameters_interface()->get_parameter_overrides();
    if (parameter_overrides.count("safety.torque_limit_scale") ||
        parameter_overrides.count("safety.torque_rate_limit_scale"))
      throw std::invalid_argument(
          "actuator scales moved to cbf.torque_limit_scale and cbf.torque_rate_limit_scale");
    scales.torque = parameter(root, "cbf.torque_limit_scale", scales.torque);
    scales.torque_rate = parameter(root, "cbf.torque_rate_limit_scale", scales.torque_rate);
    if (!scales.valid())
      throw std::invalid_argument(
          "cbf torque_limit_scale and torque_rate_limit_scale must be finite and in (0, 1]");
    safety.maximum_joint_torque = sf::panda_torque_limits(scales.torque);
    safety.maximum_torque_rate = sf::panda_torque_rate_limit(scales.torque_rate);
    safety.enabled = parameter<bool>(root, "cbf.enabled", true);
    if (!safety.enabled)
      RCLCPP_WARN(
          root->get_logger(),
          "CBF constraints and QP disabled for diagnostic operation; torque magnitude/rate, input "
          "gates, model validity and realtime deadlines remain enforced");
    limits.active = safety.active_arms = {true, mode == "dual"};
    const int count = mode == "dual" ? 2 : 1;
    std::vector<std::string> ips, namespaces, frames;
    // Validate ownership identities and configure the solver before any FCI connection.
    for (int i = 0; i < count; ++i) {
      const std::string side = i == 0 ? "left" : "right";
      const std::string prefix = count == 2 ? side + "." : "";
      const std::string ns =
          count == 2 ? parameter<std::string>(root, prefix + "namespace", "/panda_" + side)
                     : root->get_namespace();
      const std::string id =
          count == 2 ? side : parameter<std::string>(root, "controller_id", "right");
      const std::string frame = parameter<std::string>(
          root, prefix + "base_frame",
          count == 2 ? "panda_" + side + "/panda_link0" : "panda_1/panda_link0");
      const std::string ip = parameter<std::string>(root, prefix + "robot_ip", "");
      if (frame.empty() || ns.empty() || ns.front() != '/' || (id != "left" && id != "right") ||
          (hardware && ip.empty()))
        throw std::invalid_argument(
            "invalid arm namespace, base frame, controller assignment or robot_ip");
      if (i && (ns == namespaces[0] || frame == frames[0] || (hardware && ip == ips[0])))
        throw std::invalid_argument("dual arm namespaces, frames and hardware IPs must be unique");
      namespaces.push_back(ns);
      frames.push_back(frame);
      ips.push_back(ip);
      ids[i].arm_id = count == 2 ? side : "single";
      ids[i].base_frame = frame;
      const std::vector<double> default_base = {i * 2., 0, 0, 0, 0, 0, 1};
      // Two-metre mock separation is only a fixture. Hardware dual bases must
      // be supplied explicitly; deployment must survey them before operation.
      if (hardware && count == 2 &&
          !root->get_node_parameters_interface()->get_parameter_overrides().count(
              prefix + "world_from_base_xyz_xyzw"))
        throw std::invalid_argument(
            "hardware dual mode requires explicit world_from_base_xyz_xyzw for both arms");
      ids[i].world_from_base =
          world_transform(parameter(root, prefix + "world_from_base_xyz_xyzw", default_base));
      std::vector<rclcpp::Parameter> overrides = {
          {"backend", backend},  {"operator_confirm", confirmed},
          {"robot_ip", ip},      {"base_frame", frame},
          {"controller_id", id}, {"homing_enabled", parameter<bool>(root, "homing_enabled", true)}};
      for (const auto& entry : std::vector<std::pair<std::string, std::string>>{
               {"command_topic", "cartesian_teleop_command"},
               {"measured_pose_topic", "teleop_anchor_pose"},
               {"state_topic", "robot_state"},
               {"controller_state_topic", "/quest_usb_bridge/" + id + "/controller_state"}})
        overrides.emplace_back(entry.first,
                               parameter<std::string>(root, prefix + entry.first, entry.second));
      for (const auto& entry :
           std::vector<std::pair<std::string, double>>{{"lambda_translation", 50},
                                                       {"lambda_rotation", 40},
                                                       {"gain_translation", 20},
                                                       {"gain_rotation", 2.5},
                                                       {"command_timeout_ms", 100},
                                                       {"max_clock_uncertainty_ms", 10},
                                                       {"max_source_future_ms", 10},
                                                       {"maximum_torque_rate", 900},
                                                       {"tool_offset_m", .1034},
                                                       {"envelope_margin_m", .001},
                                                       {"trigger_release", .25},
                                                       {"homing_speed_factor", .15},
                                                       {"homing_timeout_seconds", 30},
                                                       {"homing_stop_duration_seconds", .5},
                                                       {"homing_stop_acceleration", .75},
                                                       {"homing_stop_timeout_seconds", 5},
                                                       {"hold_stiffness_ramp_seconds", .25}}) {
        const double value = parameter(root, prefix + entry.first, entry.second);
        overrides.emplace_back(entry.first, value);
        if (entry.first == "maximum_torque_rate")
          safety.maximum_torque_rate = std::min(safety.maximum_torque_rate, value);
      }
      for (const auto& entry : std::vector<std::pair<std::string, std::vector<double>>>{
               {"envelope_center", {0, 0, .333}},
               {"torque_limits", {87, 87, 87, 87, 12, 12, 12}},
               {"homing_stiffness", {300, 300, 300, 300, 120, 100, 50}},
               {"homing_damping", {30, 30, 30, 30, 10, 10, 5}},
               {"hold_stiffness", {600, 600, 600, 600, 250, 150, 75}},
               {"hold_damping", {50, 50, 50, 50, 20, 15, 10}}}) {
        const auto values = parameter(root, prefix + entry.first, entry.second);
        overrides.emplace_back(entry.first, values);
        if (entry.first == "torque_limits") {
          if (values.size() != 7)
            throw std::invalid_argument("torque_limits requires seven values");
          for (int j = 0; j < 7; ++j)
            safety.maximum_joint_torque[j] = std::min(safety.maximum_joint_torque[j], values[j]);
        }
      }
      nodes[i] = count == 1
                     ? root
                     : std::make_shared<rclcpp::Node>(
                           "panda_cartesian_impedance_controller", ns,
                           rclcpp::NodeOptions().use_global_arguments(false).parameter_overrides(
                               overrides));
    }
    if (count == 1) {
      ids[1].arm_id = "inactive";
      ids[1].base_frame = "inactive/base";
    }
    std::ostringstream actuator_limits;
    actuator_limits << "Effective actuator limits: torque_scale=" << scales.torque
                    << " torque_rate_scale=" << scales.torque_rate << " torque_limits_Nm=["
                    << safety.maximum_joint_torque.transpose()
                    << "] maximum_torque_rate_Nm_s=" << safety.maximum_torque_rate
                    << " (enforced with CBF enabled or disabled)";
    RCLCPP_INFO(root->get_logger(), "%s", actuator_limits.str().c_str());
    safety.table_height = parameter(root, "cbf.table_height", safety.table_height);
    safety.table_minimum_clearance =
        parameter(root, "cbf.table_minimum_clearance", safety.table_minimum_clearance);
    safety.self_minimum_clearance =
        parameter(root, "cbf.self_minimum_clearance", safety.self_minimum_clearance);
    safety.inter_arm_minimum_clearance =
        parameter(root, "cbf.inter_arm_minimum_clearance", safety.inter_arm_minimum_clearance);
    safety.obstacle_centers = parameter<std::vector<double>>(root, "cbf.obstacle_centers", {});
    safety.obstacle_radii = parameter<std::vector<double>>(root, "cbf.obstacle_radii", {});
    safety.obstacle_collision_enabled =
        !safety.obstacle_radii.empty() || !safety.obstacle_centers.empty();
    const auto max_failures = parameter<int64_t>(root, "cbf.maximum_failures", 20);
    if (max_failures < 1 || max_failures > 100)
      throw std::invalid_argument("cbf.maximum_failures must be 1..100");
    limits.maximum_failures = static_cast<unsigned>(max_failures);
    // Mock processes share a non-RT development host. These larger deadlines
    // exercise handoff semantics and never apply to the hardware backend.
    if (!hardware) {
      limits.maximum_age_ns = 100000000;
      limits.maximum_skew_ns = 50000000;
      limits.result_deadline_ns = 100000000;
      limits.filter_deadline_ns = 50000000;
    }
    sf::SafetyCoordinator coordinator;
    std::string error;
    if (!coordinator.configure(limits, safety, ids, &error))
      throw std::invalid_argument(error);
    std::atomic<bool> coordinating{true}, ready{false};
    std::thread coordinator_thread([&] {
      if (hardware) {
        sched_param scheduling{};
        scheduling.sched_priority = sched_get_priority_max(SCHED_FIFO);
        if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &scheduling) != 0 ||
            mlockall(MCL_CURRENT | MCL_FUTURE) != 0)
          coordinator.stop(sf::SafetyStop::scheduling);
      }
      ready.store(true, std::memory_order_release);
      while (coordinating.load(std::memory_order_acquire)) {
        coordinator.pump();
        std::this_thread::sleep_for(std::chrono::microseconds(25));
      }
    });
    while (!ready.load(std::memory_order_acquire))
      std::this_thread::yield();
    std::atomic<unsigned> prepared{0}, connected{0};
    std::array<int, 2> results{};
    std::array<OwnerTiming, 2> timings{};
    std::array<std::thread, 2> owners;
    try {
      if (coordinator.stopped() != sf::SafetyStop::none)
        throw std::runtime_error(
            "CBF coordinator requires real-time scheduling and memory locking");
      for (int i = 0; i < count; ++i)
        owners[i] = std::thread([&, i] {
          results[i] =
              run_arm(nodes[i], i, coordinator, safety, prepared, connected, count, timings[i]);
        });
      for (int i = 0; i < count; ++i)
        owners[i].join();
    } catch (...) {
      coordinator.stop(sf::SafetyStop::owner_failure);
      for (auto& owner : owners)
        if (owner.joinable())
          owner.join();
      coordinating.store(false, std::memory_order_release);
      coordinator_thread.join();
      throw;
    }
    coordinating.store(false, std::memory_order_release);
    coordinator_thread.join();
    result = results[0] || results[1] ? 2 : 0;
    if (coordinator.stopped() != sf::SafetyStop::none) {
      RCLCPP_ERROR(root->get_logger(), "Latched CBF stop: %s",
                   sf::safety_stop_name(coordinator.stopped()));
      result = 2;
      if (coordinator.stopped() == sf::SafetyStop::result_deadline ||
          coordinator.stopped() == sf::SafetyStop::filter_deadline) {
        const auto& reply = coordinator.diagnostic_reply();
        for (int i = 0; i < count; ++i) {
          const auto& t = timings[i];
          const bool matching =
              t.cycle && reply.cycles[i] == t.cycle && reply.source_ns[i] == t.source_ns;
          // -1 denotes an unavailable matching result; never mix cycles.
          const auto us = [](uint64_t end, uint64_t start) {
            return end >= start ? static_cast<double>(end - start) / 1000. : -1.;
          };
          RCLCPP_ERROR(
              root->get_logger(),
              "CBF timing arm %d cycle %llu: preparation_us=%.3f wait_us=%.3f total_us=%.3f "
              "deadline_us=%.3f accepted=%d matching_result=%d coordinator_delay_us=%.3f "
              "coordinator_work_us=%.3f filter_us=%.3f solver_us=%.3f iterations=%d solver=%s",
              i, static_cast<unsigned long long>(t.cycle),
              t.cycle ? us(t.submitted_ns, t.source_ns) : -1.,
              t.cycle ? us(t.finished_ns, t.submitted_ns) : -1.,
              t.cycle ? us(t.finished_ns, t.source_ns) : -1., limits.result_deadline_ns / 1000.,
              t.accepted, matching, matching ? us(reply.started_ns, t.submitted_ns) : -1.,
              matching ? us(reply.completed_ns, reply.started_ns) : -1.,
              matching ? reply.result.filter_time_seconds * 1e6 : -1.,
              matching ? reply.result.solver_time_seconds * 1e6 : -1.,
              matching ? reply.result.solver_iterations : -1,
              matching ? reply.result.solver_status : "unavailable");
          // Preserve the actual failing model for deterministic robot-free
          // reproduction, rather than tuning against identity-inertia fixtures.
          const auto& request = coordinator.diagnostic_request(i);
          if (request.cycle == t.cycle && request.stamp_ns == t.source_ns && t.cycle) {
            const auto path = "/tmp/franka-cbf-model-arm-" + std::to_string(i) + ".txt";
            std::ofstream model_file(path);
            model_file.precision(17);
            model_file << "CBF_MODEL_V1 " << i << ' ' << request.cycle << ' ' << request.period
                       << '\n';
            const auto write = [&](const auto& values) {
              for (Eigen::Index k = 0; k < values.size(); ++k)
                model_file << values.data()[k] << ' ';
              model_file << '\n';
            };
            write(request.state.q);
            write(request.state.dq);
            write(request.state.coriolis);
            write(request.state.previous_torque);
            write(request.nominal);
            write(request.state.mass);
            for (int frame = 0; frame < sf::cbf::kModelFrames; ++frame) {
              write(request.state.frame_pose[frame].matrix());
              write(request.state.frame_jacobian[frame]);
            }
            model_file.close();
            if (model_file)
              RCLCPP_ERROR(root->get_logger(),
                           "CBF failing model saved to %s (column-major matrices)", path.c_str());
            else
              RCLCPP_ERROR(root->get_logger(), "Could not save CBF failing model to %s",
                           path.c_str());
          }
        }
      }
      if (coordinator.stopped() == sf::SafetyStop::invalid_model) {
        RCLCPP_ERROR(root->get_logger(), "CBF filter diagnostic: %s",
                     coordinator.diagnostic_filter_status());
        for (int i = 0; i < count; ++i)
          log_model_diagnostics(root->get_logger(), i, coordinator.diagnostic_request(i), safety);
      }
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(root->get_logger(), "Controller setup stopped: %s", e.what());
  }
  rclcpp::shutdown();
  return result;
}
