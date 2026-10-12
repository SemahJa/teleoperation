// Copyright (c) 2026 Jiachun. Licensed under Apache-2.0.
#pragma once
#include <cmath>
#include <Eigen/Geometry>
#include <Eigen/SVD>

namespace safety_filter {
// Legacy tool rotations can contain float-rounded 0.7071 rather than sqrt(.5).
// Match libfranka 0.9.1+'s orthonormalized rotation convention, but only admit
// near-rigid transforms. This is not a repair path for arbitrary scale/shear.
// Fixed-size Eigen storage; no logging or allocations on the successful path.
inline bool orthonormalize_legacy_pose(Eigen::Affine3d& pose) noexcept {
  constexpr double rounding_limit = 1e-4;
  if (!pose.matrix().allFinite() ||
      !pose.matrix().row(3).isApprox(Eigen::RowVector4d(0,0,0,1), 1e-8)) return false;
  const Eigen::Matrix3d raw = pose.linear();
  const double determinant = raw.determinant();
  const Eigen::Matrix3d gram = raw.transpose() * raw;
  if (determinant <= 0 || std::abs(determinant - 1) > rounding_limit ||
      (gram - Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff() > rounding_limit)
    return false;
  const Eigen::Matrix3d rotation = pose.rotation();
  if (!rotation.allFinite() ||
      !((rotation.transpose() * rotation).isApprox(Eigen::Matrix3d::Identity(), 1e-8)) ||
      std::abs(rotation.determinant() - 1) >= 1e-8 ||
      (rotation - raw).cwiseAbs().maxCoeff() > rounding_limit) return false;
  pose.linear() = rotation;  // Keep the measured translation and homogeneous row.
  return true;
}
}
