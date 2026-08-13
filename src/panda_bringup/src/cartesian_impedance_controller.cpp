#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Dense>
#include <franka/control_types.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <franka_msgs/msg/franka_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace {
Eigen::Affine3d pose_to_eigen(const geometry_msgs::msg::Pose& pose) {
  Eigen::Quaterniond orientation(pose.orientation.w, pose.orientation.x, pose.orientation.y,
                                 pose.orientation.z);
  return Eigen::Translation3d(pose.position.x, pose.position.y, pose.position.z) *
         orientation.normalized();
}

Eigen::Vector3d so3_log(const Eigen::Matrix3d& rotation) {
  const Eigen::AngleAxisd angle_axis(rotation);
  return angle_axis.axis() * angle_axis.angle();
}
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("panda_cartesian_impedance_controller");
  const auto robot_ip = node->declare_parameter<std::string>("robot_ip", "");
  const auto operator_confirm = node->declare_parameter<bool>("operator_confirm", false);
  const auto lambda_translation = node->declare_parameter<double>("lambda_translation", 50.0);
  const auto lambda_rotation = node->declare_parameter<double>("lambda_rotation", 40.0);
  const auto gain_translation = node->declare_parameter<double>("gain_translation", 20.0);
  const auto gain_rotation = node->declare_parameter<double>("gain_rotation", 2.5);
  if (robot_ip.empty() || !operator_confirm || lambda_translation <= 0.0 ||
      lambda_rotation <= 0.0 || gain_translation <= 0.0 || gain_rotation <= 0.0) {
    RCLCPP_ERROR(node->get_logger(), "Set robot_ip, operator_confirm:=true, and positive gains.");
    rclcpp::shutdown();
    return 2;
  }

  std::mutex target_mutex;
  Eigen::Affine3d desired_pose = Eigen::Affine3d::Identity();
  Eigen::Matrix<double, 6, 1> desired_twist = Eigen::Matrix<double, 6, 1>::Zero();
  bool target_received = false;
  rclcpp::Time previous_target_time(0, 0, RCL_ROS_TIME);
  Eigen::Affine3d previous_target = Eigen::Affine3d::Identity();

  auto initial_pose = node->create_publisher<geometry_msgs::msg::PoseStamped>(
      "~/initial_pose", rclcpp::QoS(1).transient_local());
  auto joint_states = node->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 1);
  auto franka_state = node->create_publisher<franka_msgs::msg::FrankaState>(
      "/franka_robot_state_broadcaster/robot_state", rclcpp::QoS(1));
  auto target_subscription = node->create_subscription<geometry_msgs::msg::PoseStamped>(
      "~/target_pose", 1, [&](geometry_msgs::msg::PoseStamped::SharedPtr message) {
        const auto& p = message->pose.position;
        const auto& q = message->pose.orientation;
        const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
            !std::isfinite(norm) || norm < 0.99 || norm > 1.01) {
          return;
        }
        const Eigen::Affine3d next_target = pose_to_eigen(message->pose);
        const auto now = node->now();
        std::lock_guard<std::mutex> lock(target_mutex);
        if (target_received) {
          const double dt = (now - previous_target_time).seconds();
          if (dt > 1e-4) {
            desired_twist.head<3>() =
                (next_target.translation() - previous_target.translation()) / dt;
            Eigen::Quaterniond first(previous_target.rotation());
            Eigen::Quaterniond second(next_target.rotation());
            if (first.coeffs().dot(second.coeffs()) < 0.0) second.coeffs() *= -1.0;
            const Eigen::AngleAxisd rotation(first.inverse() * second);
            desired_twist.tail<3>() =
                first.toRotationMatrix() * rotation.axis() * rotation.angle() / dt;
          }
        }
        desired_pose = next_target;
        previous_target = next_target;
        previous_target_time = now;
        target_received = true;
      });

  std::thread executor([&] { rclcpp::spin(node); });
  try {
    franka::Robot robot(robot_ip);
    franka::Model model = robot.loadModel();
    bool initialized = false;
    RCLCPP_INFO(node->get_logger(),
                "Spong filtered-error controller: Kp=Kr*Lambda: translation %.1f N/m, "
                "rotation %.1f Nm/rad.",
                gain_translation * lambda_translation, gain_rotation * lambda_rotation);
    robot.control([&](const franka::RobotState& state, franka::Duration) -> franka::Torques {
      const Eigen::Affine3d current(Eigen::Matrix4d::Map(state.O_T_EE.data()));
      if (!initialized) {
        std::lock_guard<std::mutex> lock(target_mutex);
        desired_pose = current;
        previous_target = current;
        previous_target_time = node->now();
        desired_twist.setZero();
        target_received = true;
        initialized = true;
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = node->now();
        pose.header.frame_id = "panda_link0";
        pose.pose.position.x = current.translation().x();
        pose.pose.position.y = current.translation().y();
        pose.pose.position.z = current.translation().z();
        const Eigen::Quaterniond orientation(current.rotation());
        pose.pose.orientation.x = orientation.x();
        pose.pose.orientation.y = orientation.y();
        pose.pose.orientation.z = orientation.z();
        pose.pose.orientation.w = orientation.w();
        initial_pose->publish(pose);
      }
      Eigen::Affine3d target;
      Eigen::Matrix<double, 6, 1> target_twist;
      {
        std::lock_guard<std::mutex> lock(target_mutex);
        target = desired_pose;
        target_twist = desired_twist;
      }
      Eigen::Matrix<double, 6, 1> error;
      error.head<3>() = current.translation() - target.translation();
      error.tail<3>() = target.rotation() * so3_log(target.rotation().transpose() * current.rotation());
      const auto jacobian_array = model.zeroJacobian(franka::Frame::kEndEffector, state);
      const auto coriolis_array = model.coriolis(state);
      const Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
      const Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(state.dq.data());
      const Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
      Eigen::Matrix<double, 6, 1> lambda;
      lambda << lambda_translation, lambda_translation, lambda_translation, lambda_rotation,
          lambda_rotation, lambda_rotation;
      Eigen::Matrix<double, 6, 1> gain;
      gain << gain_translation, gain_translation, gain_translation, gain_rotation, gain_rotation,
          gain_rotation;
      const Eigen::Matrix<double, 6, 1> filtered_error =
          jacobian * dq - target_twist + lambda.cwiseProduct(error);
      std::array<double, 7> torque{};
      Eigen::Map<Eigen::Matrix<double, 7, 1>>(torque.data()) =
          jacobian.transpose() * (-gain.cwiseProduct(filtered_error)) + coriolis;
      sensor_msgs::msg::JointState joint_state;
      joint_state.header.stamp = node->now();
      joint_state.name = {"panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
                          "panda_joint5", "panda_joint6", "panda_joint7", "panda_finger_joint1",
                          "panda_finger_joint2"};
      joint_state.position.assign(state.q.begin(), state.q.end());
      joint_state.position.push_back(0.02);
      joint_state.position.push_back(0.02);
      joint_states->publish(joint_state);
      franka_msgs::msg::FrankaState state_message;
      state_message.header.stamp = node->now();
      state_message.q = state.q;
      state_message.dq = state.dq;
      state_message.tau_j = state.tau_J;
      state_message.o_t_ee = state.O_T_EE;
      Eigen::Map<Eigen::Matrix4d>(state_message.o_t_ee_d.data()) = target.matrix();
      Eigen::Map<Eigen::Matrix4d>(state_message.o_t_ee_c.data()) = target.matrix();
      for (size_t index = 0; index < state_message.o_dp_ee_d.size(); ++index) {
        state_message.o_dp_ee_d[index] = target_twist[index];
      }
      state_message.time = state.time.toSec();
      state_message.robot_mode = static_cast<uint8_t>(state.robot_mode);
      franka_state->publish(state_message);
      return franka::Torques(torque);
    }, true, franka::kMaxCutoffFrequency);
  } catch (const franka::Exception& exception) {
    RCLCPP_ERROR(node->get_logger(), "Controller failed: %s", exception.what());
  }
  rclcpp::shutdown();
  executor.join();
  return 0;
}
