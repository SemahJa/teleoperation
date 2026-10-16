// Copyright (c) 2026 Jiachun. Licensed under Apache-2.0.
// Fixed model snapshots adapted from thesis_ws's dual-arm control types.
#pragma once
#include <Eigen/Dense>
#include <array>
#include <cstdint>
#include <string>
namespace safety_filter::cbf {
constexpr int kJointsPerArm = 7, kArmCount = 2, kDualArmDof = 14, kModelFrames = 9;
using Vector7d = Eigen::Matrix<double, 7, 1>;
using Vector14d = Eigen::Matrix<double, 14, 1>;
using Matrix7d = Eigen::Matrix<double, 7, 7>;
using Matrix14d = Eigen::Matrix<double, 14, 14>;
using Jacobian = Eigen::Matrix<double, 6, 7>;
using PointJacobian = Eigen::Matrix<double, 3, 14>;
struct ArmIdentity {
  std::string arm_id, base_frame;
  Eigen::Affine3d world_from_base{Eigen::Affine3d::Identity()};
};
struct ArmState {
  Vector7d q{Vector7d::Zero()}, dq{Vector7d::Zero()}, coriolis{Vector7d::Zero()};
  Vector7d previous_torque{Vector7d::Zero()};
  Matrix7d mass{Matrix7d::Identity()};
  std::array<Eigen::Affine3d, kModelFrames> frame_pose{};
  std::array<Jacobian, kModelFrames> frame_jacobian{};
  ArmState() {
    for (auto& pose : frame_pose)
      pose.setIdentity();
    for (auto& j : frame_jacobian)
      j.setZero();
  }
};
struct DualArmState {
  std::array<ArmState, kArmCount> arm{};
};
}  // namespace safety_filter::cbf
