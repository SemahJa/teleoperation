// Copyright (c) 2026 Jiachun. Licensed under Apache-2.0.
#pragma once
#include <cmath>
#include "safety_filter/cbf_types.hpp"
namespace safety_filter {
// Pinned URDF kinematics only. Identity inertia/zero coriolis and fixed joints
// are a software fixture; this is not a simulator or physical validation.
inline cbf::ArmState panda_model_fixture(const cbf::Vector7d& q) {
  cbf::ArmState s; s.q = q;
  const std::array<Eigen::Vector3d, 7> translation = {
    Eigen::Vector3d{0,0,.333}, {0,0,0}, {0,-.316,0}, {.0825,0,0},
    {-.0825,0,.384}, {0,0,0}, {.088,0,0}};
  const std::array<double, 7> roll = {0,-M_PI/2,M_PI/2,M_PI/2,-M_PI/2,M_PI/2,M_PI/2};
  std::array<Eigen::Vector3d, 7> origins, axes;
  Eigen::Affine3d pose = Eigen::Affine3d::Identity();
  for (int j = 0; j < 7; ++j) {
    pose = pose * Eigen::Translation3d(translation[j]) * Eigen::AngleAxisd(roll[j], Eigen::Vector3d::UnitX());
    origins[j] = pose.translation(); axes[j] = pose.linear().col(2);
    pose = pose * Eigen::AngleAxisd(q[j], Eigen::Vector3d::UnitZ());
    s.frame_pose[j] = pose;
  }
  s.frame_pose[7] = pose * Eigen::Translation3d(0,0,.107);
  s.frame_pose[8] = s.frame_pose[7] * Eigen::AngleAxisd(-M_PI/4, Eigen::Vector3d::UnitZ()) *
      Eigen::Translation3d(0,0,.1034);
  for (int frame = 0; frame < 9; ++frame) for (int j = 0; j <= std::min(frame, 6); ++j) {
    s.frame_jacobian[frame].col(j).head<3>() = axes[j].cross(s.frame_pose[frame].translation() - origins[j]);
    s.frame_jacobian[frame].col(j).tail<3>() = axes[j];
  }
  return s;
}
}
