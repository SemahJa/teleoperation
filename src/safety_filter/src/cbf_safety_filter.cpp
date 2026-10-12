// Copyright (c) 2026 Jiachun
// Licensed under the Apache License, Version 2.0.

#include "safety_filter/cbf_safety_filter.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <sstream>
#include "safety_filter/panda_limits.hpp"

namespace safety_filter::cbf {
namespace {

constexpr double kLargeClearance = 1.0e9;
constexpr std::array<double, kJointsPerArm> kPublishedMaximumTorque{
    87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0};
constexpr int kCapsuleCount = 14;
const char* solver_status_name(c_int status) noexcept {
  switch (status) {
    case OSQP_SOLVED: return "solved";
    case OSQP_SOLVED_INACCURATE: return "solved_inaccurate";
    case OSQP_MAX_ITER_REACHED: return "iteration_limit";
    case OSQP_PRIMAL_INFEASIBLE: return "primal_infeasible";
    case OSQP_DUAL_INFEASIBLE: return "dual_infeasible";
    default: return "solver_failure";
  }
}

Eigen::Matrix3d skew(const Eigen::Vector3d& value) {
  Eigen::Matrix3d result;
  result << 0.0, -value.z(), value.y(), value.z(), 0.0, -value.x(),
      -value.y(), value.x(), 0.0;
  return result;
}

bool finitePositive(const Vector7d& value) {
  return value.allFinite() && (value.array() > 0.0).all();
}

}  // namespace

CbfSafetyFilter::~CbfSafetyFilter() { cleanupSolver(); }

void CbfSafetyFilter::cleanupSolver() {
  if (workspace_ != nullptr) {
    osqp_cleanup(workspace_);
    workspace_ = nullptr;
  }
}

bool CbfSafetyFilter::configure(
    const SafetyParameters& parameters,
    const std::array<ArmIdentity, kArmCount>& identities,
    std::string* error) {
  cleanupSolver();
  parameters_ = parameters;
  identities_ = identities;

  const bool invalid =
      (!parameters_.active_arms[0] && !parameters_.active_arms[1]) ||
      !parameters_.minimum_joint_position.allFinite() ||
      !parameters_.maximum_joint_position.allFinite() ||
      !finitePositive(parameters_.maximum_joint_velocity) ||
      !finitePositive(parameters_.maximum_joint_acceleration) ||
      !finitePositive(parameters_.maximum_joint_torque) ||
      !finitePositive(parameters_.objective_weight) ||
      !std::isfinite(parameters_.parked_arm_weight_multiplier) ||
      parameters_.parked_arm_weight_multiplier < 1.0 ||
      parameters_.parked_arm_weight_multiplier > 100.0 ||
      !std::isfinite(parameters_.priority_release_clearance) ||
      parameters_.priority_release_clearance < 0.0 ||
      (parameters_.asymmetric_inter_arm_priority_enabled &&
       parameters_.priority_release_clearance <
           parameters_.collision_activation_distance) ||
      !std::isfinite(parameters_.maximum_torque_rate) ||
      parameters_.maximum_torque_rate <= 0.0 ||
      parameters_.maximum_torque_rate > 1000.0 ||
      !std::isfinite(parameters_.joint_position_margin) ||
      parameters_.joint_position_margin < 0.0 ||
      !std::isfinite(parameters_.joint_velocity_scale) ||
      parameters_.joint_velocity_scale <= 0.0 ||
      parameters_.joint_velocity_scale > 1.0 ||
      !std::isfinite(parameters_.joint_cbf_frequency) ||
      parameters_.joint_cbf_frequency <= 0.0 ||
      !std::isfinite(parameters_.joint_cbf_damping_ratio) ||
      parameters_.joint_cbf_damping_ratio <= 0.0 ||
      !std::isfinite(parameters_.velocity_cbf_gain) ||
      parameters_.velocity_cbf_gain <= 0.0 ||
      !std::isfinite(parameters_.table_height) ||
      parameters_.table_minimum_link_index < 0 ||
      parameters_.table_minimum_link_index > 9 ||
      !std::isfinite(parameters_.table_minimum_clearance) ||
      parameters_.table_minimum_clearance < 0.0 ||
      !std::isfinite(parameters_.self_minimum_clearance) ||
      parameters_.self_minimum_clearance < 0.0 ||
      !std::isfinite(parameters_.inter_arm_minimum_clearance) ||
      parameters_.inter_arm_minimum_clearance < 0.0 ||
      !std::isfinite(parameters_.obstacle_minimum_clearance) ||
      parameters_.obstacle_minimum_clearance < 0.0 ||
      !std::isfinite(parameters_.collision_activation_distance) ||
      parameters_.collision_activation_distance <= 0.0 ||
      !std::isfinite(parameters_.collision_cbf_frequency) ||
      parameters_.collision_cbf_frequency <= 0.0 ||
      !std::isfinite(parameters_.collision_cbf_damping_ratio) ||
      parameters_.collision_cbf_damping_ratio <= 0.0 ||
      !std::isfinite(parameters_.fallback_damping) ||
      parameters_.fallback_damping <= 0.0 ||
      parameters_.solver_max_iterations <= 0 ||
      !std::isfinite(parameters_.solver_rho) ||
      parameters_.solver_rho <= 0.0 ||
      parameters_.solver_rho > 100.0 ||
      !std::isfinite(parameters_.solver_absolute_tolerance) ||
      parameters_.solver_absolute_tolerance <= 0.0 ||
      !std::isfinite(parameters_.solver_relative_tolerance) ||
      parameters_.solver_relative_tolerance <= 0.0 ||
      !std::isfinite(parameters_.solver_feasibility_tolerance) ||
      parameters_.solver_feasibility_tolerance <= 0.0 ||
      parameters_.solver_feasibility_tolerance > 1.0e-2 ||
      parameters_.obstacle_centers.size() !=
          3 * parameters_.obstacle_radii.size();
  if (invalid) {
    if (error != nullptr) {
      *error = "invalid CBF-QP parameter";
    }
    return false;
  }
  for (int joint = 0; joint < kJointsPerArm; ++joint) {
    if (parameters_.minimum_joint_position[joint] < kJointLower[joint] ||
        parameters_.maximum_joint_position[joint] > kJointUpper[joint] ||
        parameters_.maximum_joint_velocity[joint] > kJointVelocity[joint]) {
      if (error) *error = "configured joint bounds exceed FER/Panda limits";
      return false;
    }
    if (parameters_.minimum_joint_position(joint) +
            parameters_.joint_position_margin >=
        parameters_.maximum_joint_position(joint) -
            parameters_.joint_position_margin) {
      if (error != nullptr) {
        *error = "joint position margin removes the admissible interval";
      }
      return false;
    }
    if (parameters_.maximum_joint_torque(joint) >
        kPublishedMaximumTorque[joint]) {
      if (error != nullptr) {
        *error = "configured torque exceeds the published FER/Panda limit";
      }
      return false;
    }
    if (parameters_.maximum_joint_acceleration(joint) >
        kJointAcceleration[joint]) {
      if (error != nullptr) {
        *error = "configured acceleration exceeds the published FER/Panda limit";
      }
      return false;
    }
  }
  for (const auto& identity : identities_) {
    if (identity.arm_id.empty() || identity.base_frame.empty() ||
        !identity.world_from_base.matrix().allFinite() ||
        !identity.world_from_base.matrix().row(3).isApprox(Eigen::RowVector4d(0,0,0,1), 1e-9) ||
        !(identity.world_from_base.linear().transpose() * identity.world_from_base.linear()).isApprox(Eigen::Matrix3d::Identity(), 1e-9) ||
        std::abs(identity.world_from_base.linear().determinant() - 1) > 1e-9) {
      if (error != nullptr) {
        *error = "invalid dual-arm identity or base transform";
      }
      return false;
    }
  }

  // These capsules transcribe Franka's safety_distance=0 coarse collision
  // model. Collision margins are added once in the CBF pair policies below.
  // The stock Panda hand is fixed to link8/flange with a -pi/4 yaw. Use the
  // flange model frame and explicitly rotate the hand-local capsule centers
  // and axes; kEndEffector can have a configurable hardware F_T_EE offset and
  // is not necessarily the hand mounting frame.
  const Eigen::Matrix3d flange_from_hand =
      Eigen::AngleAxisd(-M_PI_4, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  capsules_ = {
      {0, -1, {-0.075, 0.0, 0.06}, Eigen::Vector3d::UnitX(), 0.03, 0.06},
      {1, 0, {0.0, 0.0, -0.1915}, Eigen::Vector3d::UnitZ(), 0.283, 0.06},
      {2, 1, {0.0, 0.0, 0.0}, Eigen::Vector3d::UnitZ(), 0.12, 0.06},
      {3, 2, {0.0, 0.0, -0.145}, Eigen::Vector3d::UnitZ(), 0.15, 0.06},
      {4, 3, {0.0, 0.0, 0.0}, Eigen::Vector3d::UnitZ(), 0.12, 0.06},
      {5, 4, {0.0, 0.0, -0.26}, Eigen::Vector3d::UnitZ(), 0.10, 0.06},
      {5, 4, {0.0, 0.08, -0.13}, Eigen::Vector3d::UnitZ(), 0.14, 0.025},
      {6, 5, {0.0, 0.0, -0.03}, Eigen::Vector3d::UnitZ(), 0.08, 0.05},
      {7, 6, {0.0, 0.0, 0.01}, Eigen::Vector3d::UnitZ(), 0.14, 0.04},
      // Pinned Panda link8 flange cylinders/spheres, not the FR3 approximation.
      {8, 7, {0.0424, 0.0424, -0.025}, -Eigen::Vector3d::UnitY(), 0.01, 0.03},
      {8, 7, {0.0424, 0.0424, -0.02}, Eigen::Vector3d::UnitZ(), 0.0, 0.03},
      {8, 7, {0.0424, 0.0424, -0.03}, Eigen::Vector3d::UnitZ(), 0.0, 0.03},
      {9, 7, flange_from_hand * Eigen::Vector3d{0.0, 0.0, 0.04},
       flange_from_hand * Eigen::Vector3d::UnitY(), 0.10, 0.04},
      {9, 7, flange_from_hand * Eigen::Vector3d{0.0, 0.0, 0.10},
       flange_from_hand * Eigen::Vector3d::UnitY(), 0.10, 0.02},
  };

  self_pairs_.clear();
  for (int first = 0; first < static_cast<int>(capsules_.size()); ++first) {
    for (int second = first + 1;
         second < static_cast<int>(capsules_.size()); ++second) {
      if (capsules_[first].link_index != capsules_[second].link_index &&
          !selfPairExcluded(capsules_[first].link_index,
                            capsules_[second].link_index)) {
        self_pairs_.emplace_back(first, second);
      }
    }
  }
  inter_arm_pairs_.clear();
  for (int left = 0; left < static_cast<int>(capsules_.size()); ++left) {
    for (int right = 0; right < static_cast<int>(capsules_.size()); ++right) {
      if (!(capsules_[left].frame_index < 0 &&
            capsules_[right].frame_index < 0)) {
        inter_arm_pairs_.emplace_back(left, right);
      }
    }
  }

  obstacles_.clear();
  for (size_t index = 0; index < parameters_.obstacle_radii.size(); ++index) {
    const double radius = parameters_.obstacle_radii[index];
    const Eigen::Vector3d center{
        parameters_.obstacle_centers[3 * index],
        parameters_.obstacle_centers[3 * index + 1],
        parameters_.obstacle_centers[3 * index + 2]};
    if (!center.allFinite() || !std::isfinite(radius) || radius <= 0.0) {
      if (error != nullptr) {
        *error = "obstacle centers must be finite and radii positive";
      }
      return false;
    }
    obstacles_.push_back({center, radius});
  }

  actuator_row_begin_ = 0;
  joint_position_row_begin_ = actuator_row_begin_ + kDualArmDof;
  joint_velocity_row_begin_ =
      joint_position_row_begin_ + 2 * kDualArmDof;
  joint_acceleration_row_begin_ =
      joint_velocity_row_begin_ + 2 * kDualArmDof;
  table_row_begin_ = joint_acceleration_row_begin_ + kDualArmDof;
  self_row_begin_ = table_row_begin_ +
                    kArmCount * static_cast<int>(capsules_.size());
  inter_arm_row_begin_ = self_row_begin_ +
                         kArmCount * static_cast<int>(self_pairs_.size());
  obstacle_row_begin_ = inter_arm_row_begin_ +
                        static_cast<int>(inter_arm_pairs_.size());
  constraint_count_ = obstacle_row_begin_ +
      kArmCount * static_cast<int>(capsules_.size() * obstacles_.size());

  previous_constraint_jacobians_.assign(
      constraint_count_, PointJacobian::Zero());
  previous_constraint_jacobian_valid_.assign(constraint_count_, false);
  previous_collision_normals_.assign(
      constraint_count_, Eigen::Vector3d::UnitX());
  previous_collision_normal_valid_.assign(constraint_count_, false);
  return setupSolver(error);
}

bool CbfSafetyFilter::setupSolver(std::string* error) {
  p_values_.resize(kDualArmDof);
  p_rows_.resize(kDualArmDof);
  p_columns_.resize(kDualArmDof + 1);
  for (int column = 0; column < kDualArmDof; ++column) {
    p_values_[column] = static_cast<c_float>(
        parameters_.objective_weight(column % kJointsPerArm));
    p_rows_[column] = column;
    p_columns_[column] = column;
  }
  p_columns_[kDualArmDof] = kDualArmDof;

  a_values_.assign(constraint_count_ * kDualArmDof, 0.0);
  a_rows_.clear();
  solver_a_indices_.clear();
  // Joint dynamics and same-arm geometry never couple two arms. Rows disabled
  // by configuration cannot become active until the next configure(). Keep
  // their logical slots for diagnostics/history, but no structural zeros in A.
  const auto row_supports = [&](int row, int column) {
    if (row < joint_position_row_begin_) return row == column;
    int arm = -1;
    bool enabled = false;
    if (row < joint_velocity_row_begin_) {
      arm = (row - joint_position_row_begin_) / (2 * kJointsPerArm);
      enabled = parameters_.joint_position_enabled;
    } else if (row < joint_acceleration_row_begin_) {
      arm = (row - joint_velocity_row_begin_) / (2 * kJointsPerArm);
      enabled = parameters_.joint_velocity_enabled;
    } else if (row < table_row_begin_) {
      arm = (row - joint_acceleration_row_begin_) / kJointsPerArm;
      enabled = parameters_.joint_acceleration_enabled;
    } else if (row < self_row_begin_) {
      arm = (row - table_row_begin_) / static_cast<int>(capsules_.size());
      const int capsule = (row - table_row_begin_) % static_cast<int>(capsules_.size());
      enabled = parameters_.table_enabled &&
          capsules_[capsule].link_index >= parameters_.table_minimum_link_index;
    } else if (row < inter_arm_row_begin_) {
      arm = (row - self_row_begin_) / static_cast<int>(self_pairs_.size());
      enabled = parameters_.self_collision_enabled;
    } else if (row < obstacle_row_begin_) {
      return parameters_.inter_arm_collision_enabled &&
          parameters_.active_arms[0] && parameters_.active_arms[1];
    } else {
      arm = (row - obstacle_row_begin_) /
          static_cast<int>(capsules_.size() * obstacles_.size());
      enabled = parameters_.obstacle_collision_enabled;
    }
    return enabled && parameters_.active_arms[arm] && column / kJointsPerArm == arm;
  };
  // Drop only rows that can never be enabled by this configuration. Keep all
  // distance-activated collision rows, even when currently clear of obstacles.
  std::vector<c_int> solver_row(constraint_count_, -1);
  solver_logical_rows_.clear();
  for (int row = 0; row < constraint_count_; ++row) {
    for (int column = 0; column < kDualArmDof; ++column) {
      if (row_supports(row, column)) {
        solver_row[row] = static_cast<c_int>(solver_logical_rows_.size());
        solver_logical_rows_.push_back(row);
        break;
      }
    }
  }
  const auto solver_count = static_cast<c_int>(solver_logical_rows_.size());
  solver_lower_bound_.assign(solver_count, -OSQP_INFTY);
  solver_upper_bound_.assign(solver_count, OSQP_INFTY);
  a_columns_.resize(kDualArmDof + 1);
  for (int column = 0; column < kDualArmDof; ++column) {
    a_columns_[column] = static_cast<c_int>(a_rows_.size());
    for (int row = 0; row < constraint_count_; ++row) {
      if (row_supports(row, column)) {
        a_rows_.push_back(solver_row[row]);
        solver_a_indices_.push_back(column * constraint_count_ + row);
      }
    }
  }
  a_columns_[kDualArmDof] = static_cast<c_int>(a_rows_.size());
  solver_a_values_.assign(a_rows_.size(), 0.0);
  linear_cost_.assign(kDualArmDof, 0.0);
  lower_bound_.assign(constraint_count_, -OSQP_INFTY);
  upper_bound_.assign(constraint_count_, OSQP_INFTY);

  p_matrix_ = {kDualArmDof, kDualArmDof, kDualArmDof, p_columns_.data(),
               p_rows_.data(), p_values_.data(), -1};
  a_matrix_ = {static_cast<c_int>(solver_a_values_.size()), solver_count,
               kDualArmDof, a_columns_.data(), a_rows_.data(),
               solver_a_values_.data(), -1};
  data_.n = kDualArmDof;
  data_.m = solver_count;
  data_.P = &p_matrix_;
  data_.A = &a_matrix_;
  data_.q = linear_cost_.data();
  data_.l = solver_lower_bound_.data();
  data_.u = solver_upper_bound_.data();

  osqp_set_default_settings(&settings_);
  settings_.verbose = false;
  settings_.warm_start = true;
  settings_.polish = false;
  settings_.adaptive_rho = false;
  settings_.rho = parameters_.solver_rho;
  settings_.max_iter = parameters_.solver_max_iterations;
  settings_.eps_abs = parameters_.solver_absolute_tolerance;
  settings_.eps_rel = parameters_.solver_relative_tolerance;
  settings_.check_termination = 10;
  const c_int result = osqp_setup(&workspace_, &data_, &settings_);
  if (result != 0 || workspace_ == nullptr) {
    if (error != nullptr) {
      std::ostringstream stream;
      stream << "OSQP setup failed with code " << result;
      *error = stream.str();
    }
    cleanupSolver();
    return false;
  }
  reset();
  return true;
}

void CbfSafetyFilter::reset() {
  last_safe_torque_.setZero();
  if (last_hold_priority_arm_ >= 0 && workspace_ != nullptr) {
    for (int joint = 0; joint < kDualArmDof; ++joint) {
      p_values_[joint] = static_cast<c_float>(
          parameters_.objective_weight(joint % kJointsPerArm));
    }
    if (osqp_update_P(workspace_, p_values_.data(), OSQP_NULL,
                      kDualArmDof) == 0) {
      last_hold_priority_arm_ = -1;
    } else {
      for (int joint = 0; joint < kDualArmDof; ++joint) {
        p_values_[joint] = static_cast<c_float>(
            parameters_.objective_weight(joint % kJointsPerArm) *
            (joint / kJointsPerArm == last_hold_priority_arm_ ?
                 parameters_.parked_arm_weight_multiplier : 1.0));
      }
    }
  }
  std::fill(previous_constraint_jacobian_valid_.begin(),
            previous_constraint_jacobian_valid_.end(), false);
  std::fill(previous_collision_normal_valid_.begin(),
            previous_collision_normal_valid_.end(), false);
  if (workspace_ != nullptr) {
    std::array<c_float, kDualArmDof> zero{};
    static_cast<void>(osqp_warm_start_x(workspace_, zero.data()));
  }
}

void CbfSafetyFilter::clearRows() {
  invalid_constraint_ = false;
  std::fill(a_values_.begin(), a_values_.end(), 0.0);
  std::fill(lower_bound_.begin(), lower_bound_.end(), -OSQP_INFTY);
  std::fill(upper_bound_.begin(), upper_bound_.end(), OSQP_INFTY);
}

void CbfSafetyFilter::setRow(int row, const Vector14d& coefficients,
                             double lower, double upper) {
  if (!coefficients.allFinite() || !std::isfinite(lower) || !std::isfinite(upper) || lower > upper) {
    invalid_constraint_ = true;
    for (int column = 0; column < kDualArmDof; ++column)
      a_values_[column * constraint_count_ + row] = 0;
    lower_bound_[row] = 1; upper_bound_[row] = OSQP_INFTY;
    return;
  }
  for (int column = 0; column < kDualArmDof; ++column) {
    a_values_[column * constraint_count_ + row] =
        static_cast<c_float>(coefficients(column));
  }
  lower_bound_[row] = static_cast<c_float>(lower);
  upper_bound_[row] = static_cast<c_float>(upper);
}

void CbfSafetyFilter::disableRow(int row) {
  setRow(row, Vector14d::Zero(), -OSQP_INFTY, OSQP_INFTY);
  previous_constraint_jacobian_valid_[row] = false;
}

void CbfSafetyFilter::addLowerConstraint(int row,
                                         const Vector14d& coefficients,
                                         double lower, bool active,
                                         int* active_counter) {
  if (!active) {
    disableRow(row);
    return;
  }
  if (!coefficients.allFinite() || !std::isfinite(lower)) {
    invalid_constraint_ = true;
    setRow(row, Vector14d::Zero(), 1.0, OSQP_INFTY);
    return;
  }
  setRow(row, coefficients, lower, OSQP_INFTY);
  if (active_counter != nullptr) {
    ++(*active_counter);
  }
}

CbfSafetyFilter::Endpoint CbfSafetyFilter::endpoint(
    int arm, int frame_index, const Eigen::Vector3d& local_point,
    const ArmState& state) const {
  Endpoint result;
  const Eigen::Affine3d& world_from_base = identities_[arm].world_from_base;
  if (frame_index < 0) {
    result.position = world_from_base * local_point;
    return result;
  }

  const Eigen::Affine3d& base_from_frame = state.frame_pose[frame_index];
  const Eigen::Vector3d offset_in_base =
      base_from_frame.linear() * local_point;
  const Eigen::Vector3d point_in_base =
      base_from_frame.translation() + offset_in_base;
  result.position = world_from_base * point_in_base;
  const Eigen::Matrix<double, 3, kJointsPerArm> point_jacobian_base =
      state.frame_jacobian[frame_index].topRows<3>() -
      skew(offset_in_base) *
          state.frame_jacobian[frame_index].bottomRows<3>();
  result.jacobian.block<3, kJointsPerArm>(0, arm * kJointsPerArm) =
      world_from_base.linear() * point_jacobian_base;
  return result;
}

CbfSafetyFilter::WorldCapsule CbfSafetyFilter::worldCapsule(
    int arm, const Capsule& capsule, const ArmState& state) const {
  const Eigen::Vector3d half_axis =
      0.5 * capsule.length * capsule.direction.normalized();
  return {endpoint(arm, capsule.frame_index, capsule.center - half_axis, state),
          endpoint(arm, capsule.frame_index, capsule.center + half_axis, state),
          capsule.radius, capsule.link_index};
}

std::pair<double, double> CbfSafetyFilter::closestSegmentParameters(
    const Eigen::Vector3d& p1, const Eigen::Vector3d& q1,
    const Eigen::Vector3d& p2, const Eigen::Vector3d& q2) {
  constexpr double epsilon = 1.0e-12;
  const Eigen::Vector3d d1 = q1 - p1;
  const Eigen::Vector3d d2 = q2 - p2;
  const Eigen::Vector3d r = p1 - p2;
  const double a = d1.dot(d1);
  const double e = d2.dot(d2);
  const double f = d2.dot(r);
  double first = 0.0;
  double second = 0.0;
  if (a <= epsilon && e <= epsilon) {
    return {0.0, 0.0};
  }
  if (a <= epsilon) {
    second = std::clamp(f / e, 0.0, 1.0);
  } else {
    const double c = d1.dot(r);
    if (e <= epsilon) {
      first = std::clamp(-c / a, 0.0, 1.0);
    } else {
      const double b = d1.dot(d2);
      const double denominator = a * e - b * b;
      if (denominator > epsilon) {
        first = std::clamp((b * f - c * e) / denominator, 0.0, 1.0);
      }
      second = (b * first + f) / e;
      if (second < 0.0) {
        second = 0.0;
        first = std::clamp(-c / a, 0.0, 1.0);
      } else if (second > 1.0) {
        second = 1.0;
        first = std::clamp((b - c) / a, 0.0, 1.0);
      }
    }
  }
  return {first, second};
}

double CbfSafetyFilter::pointSegmentParameter(
    const Eigen::Vector3d& point, const Eigen::Vector3d& first,
    const Eigen::Vector3d& second) {
  const Eigen::Vector3d direction = second - first;
  const double squared_length = direction.squaredNorm();
  if (squared_length <= 1.0e-12) {
    return 0.0;
  }
  return std::clamp((point - first).dot(direction) / squared_length,
                    0.0, 1.0);
}

bool CbfSafetyFilter::selfPairExcluded(int first_link, int second_link) {
  if (first_link > second_link) {
    std::swap(first_link, second_link);
  }
  // Franka's single-arm SRDF exclusions. Inter-arm pairs never use this list.
  constexpr std::array<std::pair<int, int>, 27> excluded{{
      {0, 1}, {0, 2}, {0, 3}, {0, 4}, {1, 2}, {1, 3}, {1, 4},
      {2, 3}, {2, 4}, {2, 6}, {3, 4}, {3, 5}, {3, 6}, {3, 7},
      {4, 5}, {4, 6}, {4, 7}, {4, 8}, {5, 6}, {6, 7},
      {6, 8}, {7, 8}, {3, 9}, {4, 9}, {6, 9}, {7, 9}, {8, 9}}};
  return std::find(excluded.begin(), excluded.end(),
                   std::make_pair(first_link, second_link)) != excluded.end();
}

Eigen::Vector3d CbfSafetyFilter::jacobianDerivativeVelocity(
    int row, const PointJacobian& jacobian, const Vector14d& velocity,
    double period_seconds) {
  Eigen::Vector3d result = Eigen::Vector3d::Zero();
  if (period_seconds > 1.0e-6 &&
      previous_constraint_jacobian_valid_[row]) {
    result = ((jacobian - previous_constraint_jacobians_[row]) /
              period_seconds) * velocity;
  }
  previous_constraint_jacobians_[row] = jacobian;
  previous_constraint_jacobian_valid_[row] = true;
  return result;
}

void CbfSafetyFilter::buildConstraints(const DualArmState& state,
                                       double period_seconds,
                                       SafetyResult* result) {
  clearRows();
  result->minimum_table_clearance = kLargeClearance;
  result->minimum_self_clearance = kLargeClearance;
  result->minimum_inter_arm_clearance = kLargeClearance;
  result->minimum_obstacle_clearance = kLargeClearance;

  Matrix14d inverse_mass = Matrix14d::Zero();
  Vector14d coriolis = Vector14d::Zero();
  Vector14d velocity = Vector14d::Zero();
  for (int arm = 0; arm < kArmCount; ++arm) {
    inverse_mass.block<kJointsPerArm, kJointsPerArm>(
        arm * kJointsPerArm, arm * kJointsPerArm) =
        state.arm[arm].mass.ldlt().solve(Matrix7d::Identity());
    coriolis.segment<kJointsPerArm>(arm * kJointsPerArm) =
        state.arm[arm].coriolis;
    velocity.segment<kJointsPerArm>(arm * kJointsPerArm) =
        state.arm[arm].dq;
  }

  // Torque magnitude and the mandatory FER/Panda torque-rate condition share one
  // identity row per joint, yielding their exact interval intersection.
  const double maximum_delta =
      parameters_.maximum_torque_rate * std::clamp(period_seconds, 0.0, 0.001);
  for (int joint = 0; joint < kDualArmDof; ++joint) {
    const double magnitude =
        parameters_.maximum_joint_torque(joint % kJointsPerArm);
    const double lower =
        std::max(-magnitude, actuator_anchor_(joint) - maximum_delta);
    const double upper =
        std::min(magnitude, actuator_anchor_(joint) + maximum_delta);
    Vector14d row = Vector14d::Zero();
    row(joint) = 1.0;
    const bool active = parameters_.active_arms[joint / kJointsPerArm];
    setRow(actuator_row_begin_ + joint, row, active ? lower : 0, active ? upper : 0);
    ++result->active_constraints;
  }

  int row = joint_position_row_begin_;
  const double joint_omega = parameters_.joint_cbf_frequency;
  const double joint_damping = parameters_.joint_cbf_damping_ratio;
  for (int arm = 0; arm < kArmCount; ++arm) {
    for (int joint = 0; joint < kJointsPerArm; ++joint) {
      const int index = arm * kJointsPerArm + joint;
      const Vector14d acceleration_row = inverse_mass.row(index).transpose();
      const double acceleration_drift =
          -inverse_mass.row(index).dot(coriolis);

      const double lower_h = state.arm[arm].q(joint) -
          (parameters_.minimum_joint_position(joint) +
           parameters_.joint_position_margin);
      const double lower_rhs =
          -acceleration_drift - 2.0 * joint_damping * joint_omega *
                                    state.arm[arm].dq(joint) -
          joint_omega * joint_omega * lower_h;
      addLowerConstraint(row++, acceleration_row, lower_rhs,
                         parameters_.joint_position_enabled && parameters_.active_arms[arm],
                         &result->active_constraints);

      const double upper_h =
          (parameters_.maximum_joint_position(joint) -
           parameters_.joint_position_margin) - state.arm[arm].q(joint);
      const double upper_rhs = acceleration_drift +
          2.0 * joint_damping * joint_omega * state.arm[arm].dq(joint) -
          joint_omega * joint_omega * upper_h;
      addLowerConstraint(row++, -acceleration_row, upper_rhs,
                         parameters_.joint_position_enabled && parameters_.active_arms[arm],
                         &result->active_constraints);
      if (parameters_.joint_position_enabled && parameters_.active_arms[arm]) {
        result->joint_position_constraints += 2;
      }
    }
  }

  row = joint_velocity_row_begin_;
  for (int arm = 0; arm < kArmCount; ++arm) {
    for (int joint = 0; joint < kJointsPerArm; ++joint) {
      const int index = arm * kJointsPerArm + joint;
      const Vector14d acceleration_row = inverse_mass.row(index).transpose();
      const double inverse_mass_coriolis =
          inverse_mass.row(index).dot(coriolis);
      const double maximum_velocity =
          parameters_.joint_velocity_scale *
          parameters_.maximum_joint_velocity(joint);
      const double lower_h = maximum_velocity + state.arm[arm].dq(joint);
      addLowerConstraint(
          row++, acceleration_row,
          inverse_mass_coriolis - parameters_.velocity_cbf_gain * lower_h,
          parameters_.joint_velocity_enabled && parameters_.active_arms[arm], &result->active_constraints);
      const double upper_h = maximum_velocity - state.arm[arm].dq(joint);
      addLowerConstraint(
          row++, -acceleration_row,
          -inverse_mass_coriolis - parameters_.velocity_cbf_gain * upper_h,
          parameters_.joint_velocity_enabled && parameters_.active_arms[arm], &result->active_constraints);
      if (parameters_.joint_velocity_enabled && parameters_.active_arms[arm]) {
        result->joint_velocity_constraints += 2;
      }
    }
  }

  // Direct acceleration limits are two-sided linear inequalities under the
  // Franka torque convention qdd = M^-1 (tau - c).
  row = joint_acceleration_row_begin_;
  for (int arm = 0; arm < kArmCount; ++arm) {
    for (int joint = 0; joint < kJointsPerArm; ++joint, ++row) {
      if (!parameters_.joint_acceleration_enabled || !parameters_.active_arms[arm]) {
        disableRow(row);
        continue;
      }
      const int index = arm * kJointsPerArm + joint;
      const Vector14d coefficients = inverse_mass.row(index).transpose();
      const double drift = inverse_mass.row(index).dot(coriolis);
      const double limit = parameters_.maximum_joint_acceleration(joint);
      setRow(row, coefficients, -limit + drift, limit + drift);
      ++result->active_constraints;
      ++result->joint_acceleration_constraints;
    }
  }

  std::array<std::array<WorldCapsule, kCapsuleCount>, kArmCount> world_capsules{};
  for (int arm = 0; arm < kArmCount; ++arm) {
    for (size_t capsule = 0; capsule < capsules_.size(); ++capsule) {
      world_capsules[arm][capsule] =
          worldCapsule(arm, capsules_[capsule], state.arm[arm]);
    }
  }

  const double collision_omega = parameters_.collision_cbf_frequency;
  const double collision_damping = parameters_.collision_cbf_damping_ratio;
  const auto collision_row = [&](int constraint_row,
                                 const Eigen::Vector3d& separation,
                                 const PointJacobian& relative_jacobian,
                                 double clearance, double distance,
                                 bool enabled, int* type_counter) {
    const bool active = enabled &&
        clearance <= parameters_.collision_activation_distance;
    if (!active) {
      disableRow(constraint_row);
      return;
    }
    Eigen::Vector3d normal;
    if (distance > 1.0e-8) {
      normal = separation / distance;
    } else if (previous_collision_normal_valid_[constraint_row]) {
      // Preserve the last separation direction through a coincident closest
      // point. Disabling the row here would remove protection at overlap.
      normal = previous_collision_normals_[constraint_row];
    } else {
      // If activation starts at exact coincidence, choose the world axis with
      // the largest available torque authority for this pair.
      double best_authority = -1.0;
      normal = Eigen::Vector3d::UnitX();
      for (int axis = 0; axis < 3; ++axis) {
        const double authority =
            (inverse_mass * relative_jacobian.row(axis).transpose()).squaredNorm();
        if (authority > best_authority) {
          best_authority = authority;
          normal = Eigen::Vector3d::Unit(axis);
        }
      }
    }
    previous_collision_normals_[constraint_row] = normal;
    previous_collision_normal_valid_[constraint_row] = true;
    const Eigen::Vector3d relative_velocity = relative_jacobian * velocity;
    const double h_dot = normal.dot(relative_velocity);
    const Eigen::Vector3d jdot_velocity = jacobianDerivativeVelocity(
        constraint_row, relative_jacobian, velocity, period_seconds);
    const double curvature = distance > 1.0e-8
        ? relative_velocity.dot(
            (Eigen::Matrix3d::Identity() - normal * normal.transpose()) *
            relative_velocity) / distance
        : 0.0;
    const Vector14d coefficients =
        inverse_mass * relative_jacobian.transpose() * normal;
    const double drift = curvature + normal.dot(
        jdot_velocity - relative_jacobian * inverse_mass * coriolis);
    const double lower = -drift -
        2.0 * collision_damping * collision_omega * h_dot -
        collision_omega * collision_omega * clearance;
    addLowerConstraint(constraint_row, coefficients, lower, true,
                       &result->active_constraints);
    ++(*type_counter);
  };

  row = table_row_begin_;
  for (int arm = 0; arm < kArmCount; ++arm) {
    for (size_t capsule = 0; capsule < capsules_.size(); ++capsule, ++row) {
      const WorldCapsule& current = world_capsules[arm][capsule];
      const Endpoint& lowest = current.first.position.z() <
                                       current.second.position.z()
                                   ? current.first
                                   : current.second;
      const double clearance = lowest.position.z() - current.radius -
          parameters_.table_height - parameters_.table_minimum_clearance;
      if (parameters_.active_arms[arm] && current.link_index >= parameters_.table_minimum_link_index) {
        result->minimum_table_clearance =
            std::min(result->minimum_table_clearance, clearance);
      }
      const bool active = parameters_.active_arms[arm] && parameters_.table_enabled &&
          current.link_index >= parameters_.table_minimum_link_index &&
          clearance <= parameters_.collision_activation_distance;
      if (!active) {
        disableRow(row);
        continue;
      }
      const Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
      const double h_dot = normal.dot(lowest.jacobian * velocity);
      const Eigen::Vector3d jdot_velocity = jacobianDerivativeVelocity(
          row, lowest.jacobian, velocity, period_seconds);
      const Vector14d coefficients =
          inverse_mass * lowest.jacobian.transpose() * normal;
      const double drift = normal.dot(
          jdot_velocity - lowest.jacobian * inverse_mass * coriolis);
      const double lower = -drift -
          2.0 * collision_damping * collision_omega * h_dot -
          collision_omega * collision_omega * clearance;
      addLowerConstraint(row, coefficients, lower, true,
                         &result->active_constraints);
      ++result->table_constraints;
    }
  }

  row = self_row_begin_;
  for (int arm = 0; arm < kArmCount; ++arm) {
    for (const auto& pair : self_pairs_) {
      const WorldCapsule& first = world_capsules[arm][pair.first];
      const WorldCapsule& second = world_capsules[arm][pair.second];
      const auto parameters = closestSegmentParameters(
          first.first.position, first.second.position,
          second.first.position, second.second.position);
      const Eigen::Vector3d first_point = first.first.position +
          parameters.first * (first.second.position - first.first.position);
      const Eigen::Vector3d second_point = second.first.position +
          parameters.second * (second.second.position - second.first.position);
      const Eigen::Vector3d separation = first_point - second_point;
      const double distance = separation.norm();
      const double clearance = distance - first.radius - second.radius -
          parameters_.self_minimum_clearance;
      if (parameters_.active_arms[arm]) result->minimum_self_clearance =
          std::min(result->minimum_self_clearance, clearance);
      if (!parameters_.active_arms[arm] || !parameters_.self_collision_enabled ||
          clearance > parameters_.collision_activation_distance) {
        disableRow(row++);
        continue;
      }
      const PointJacobian first_jacobian = first.first.jacobian +
          parameters.first * (first.second.jacobian - first.first.jacobian);
      const PointJacobian second_jacobian = second.first.jacobian +
          parameters.second * (second.second.jacobian - second.first.jacobian);
      collision_row(row++, separation, first_jacobian - second_jacobian,
                    clearance, distance, true,
                    &result->self_collision_constraints);
    }
  }

  row = inter_arm_row_begin_;
  for (const auto& pair : inter_arm_pairs_) {
    const WorldCapsule& first = world_capsules[0][pair.first];
    const WorldCapsule& second = world_capsules[1][pair.second];
    const auto parameters = closestSegmentParameters(
        first.first.position, first.second.position,
        second.first.position, second.second.position);
    const Eigen::Vector3d first_point = first.first.position +
        parameters.first * (first.second.position - first.first.position);
    const Eigen::Vector3d second_point = second.first.position +
        parameters.second * (second.second.position - second.first.position);
    const Eigen::Vector3d separation = first_point - second_point;
    const double distance = separation.norm();
    const double clearance = distance - first.radius - second.radius -
        parameters_.inter_arm_minimum_clearance;
    if (parameters_.active_arms[0] && parameters_.active_arms[1]) result->minimum_inter_arm_clearance =
        std::min(result->minimum_inter_arm_clearance, clearance);
    if (!parameters_.active_arms[0] || !parameters_.active_arms[1] || !parameters_.inter_arm_collision_enabled ||
        clearance > parameters_.collision_activation_distance) {
      disableRow(row++);
      continue;
    }
    const PointJacobian first_jacobian = first.first.jacobian +
        parameters.first * (first.second.jacobian - first.first.jacobian);
    const PointJacobian second_jacobian = second.first.jacobian +
        parameters.second * (second.second.jacobian - second.first.jacobian);
    collision_row(row++, separation, first_jacobian - second_jacobian,
                  clearance, distance, true,
                  &result->inter_arm_constraints);
  }

  row = obstacle_row_begin_;
  for (int arm = 0; arm < kArmCount; ++arm) {
    for (const WorldCapsule& capsule : world_capsules[arm]) {
      for (const Obstacle& obstacle : obstacles_) {
        const double segment = pointSegmentParameter(
            obstacle.center, capsule.first.position, capsule.second.position);
        const Eigen::Vector3d point = capsule.first.position +
            segment * (capsule.second.position - capsule.first.position);
        const Eigen::Vector3d separation = point - obstacle.center;
        const double distance = separation.norm();
        const double clearance = distance - capsule.radius - obstacle.radius -
            parameters_.obstacle_minimum_clearance;
        if (parameters_.active_arms[arm]) result->minimum_obstacle_clearance =
            std::min(result->minimum_obstacle_clearance, clearance);
        if (!parameters_.active_arms[arm] || !parameters_.obstacle_collision_enabled ||
            clearance > parameters_.collision_activation_distance) {
          disableRow(row++);
          continue;
        }
        const PointJacobian point_jacobian = capsule.first.jacobian +
            segment * (capsule.second.jacobian - capsule.first.jacobian);
        collision_row(row++, separation, point_jacobian, clearance, distance,
                      true,
                      &result->obstacle_constraints);
      }
    }
  }
}

Vector14d CbfSafetyFilter::clampToActuatorBounds(
    const Vector14d& requested, double period_seconds) const {
  Vector14d result;
  const double maximum_delta =
      parameters_.maximum_torque_rate * std::clamp(period_seconds, 0.0, 0.001);
  for (int joint = 0; joint < kDualArmDof; ++joint) {
    if (!parameters_.active_arms[joint / kJointsPerArm]) { result[joint] = 0; continue; }
    const double magnitude =
        parameters_.maximum_joint_torque(joint % kJointsPerArm);
    result(joint) = std::clamp(
        requested(joint),
        std::max(-magnitude, actuator_anchor_(joint) - maximum_delta),
        std::min(magnitude, actuator_anchor_(joint) + maximum_delta));
  }
  return result;
}

Vector14d CbfSafetyFilter::boundedFallback(const DualArmState& state,
                                           double period_seconds) const {
  Vector14d damping = Vector14d::Zero();
  for (int arm = 0; arm < kArmCount; ++arm) {
    damping.segment<kJointsPerArm>(arm * kJointsPerArm) =
        state.arm[arm].coriolis -
        parameters_.fallback_damping * state.arm[arm].dq;
  }
  return clampToActuatorBounds(damping, period_seconds);
}

SafetyResult CbfSafetyFilter::filter(
    const Vector14d& nominal_torque, const DualArmState& supplied_state,
    double period_seconds, const std::array<bool, kArmCount>& motion_active,
    bool activity_valid) {
  SafetyResult result;
  const auto begin = std::chrono::steady_clock::now();
  const auto finish = [&] {
    result.filter_time_seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count();
    return result;
  };
  DualArmState state = supplied_state;
  bool valid_state = std::isfinite(period_seconds) && period_seconds >= 0 && period_seconds <= .02;
  for (int arm = 0; arm < kArmCount; ++arm) {
    if (!parameters_.active_arms[arm]) { state.arm[arm] = ArmState{}; continue; }
    const auto& s = state.arm[arm];
    valid_state &= s.q.allFinite() && s.dq.allFinite() && s.coriolis.allFinite() &&
                   s.previous_torque.allFinite() && s.mass.allFinite() &&
                   s.mass.isApprox(s.mass.transpose(), 1e-9);
    if (valid_state) {
      const Eigen::LLT<Matrix7d> factor(s.mass);
      valid_state &= factor.info() == Eigen::Success;
    }
    for (int frame = 0; frame < kModelFrames; ++frame) {
      const auto& rotation = s.frame_pose[frame].linear();
      valid_state &= s.frame_pose[frame].matrix().allFinite() && s.frame_jacobian[frame].allFinite() &&
          s.frame_pose[frame].matrix().row(3).isApprox(Eigen::RowVector4d(0,0,0,1), 1e-8) &&
          (rotation.transpose() * rotation).isApprox(Eigen::Matrix3d::Identity(), 1e-8) &&
          std::abs(rotation.determinant() - 1) < 1e-8;
    }
    actuator_anchor_.segment<kJointsPerArm>(arm * kJointsPerArm) = s.previous_torque;
    const double slew = parameters_.maximum_torque_rate * std::clamp(period_seconds, 0.0, .001);
    for (int j = 0; j < kJointsPerArm; ++j) {
      const double magnitude = parameters_.maximum_joint_torque[j];
      valid_state &= std::max(-magnitude, s.previous_torque[j] - slew) <=
                     std::min(magnitude, s.previous_torque[j] + slew);
    }
  }
  if (!valid_state) {
    result.fatal_input_fault = result.fallback_active = true;
    result.solver_status = "invalid_model_state_or_actuator_bounds";
    // Placeholder only: the arm adapter must terminate ownership, never send
    // torques computed from a nonfinite/invalid model or empty bound interval.
    return finish();
  }
  result.nominal_motion_active = motion_active;
  if (workspace_ == nullptr || !nominal_torque.allFinite()) {
    result.fallback_active = true;
    result.solver_status = "invalid_input_or_unconfigured";
    result.safe_torque = boundedFallback(state, period_seconds);
    last_safe_torque_ = result.safe_torque;
    return finish();
  }
  if (!parameters_.enabled) {
    result.safe_torque = clampToActuatorBounds(nominal_torque, period_seconds);
    result.solver_status = "disabled_actuator_bounds_only";
    result.solver_succeeded = true;
    result.intervention_norm = (result.safe_torque - nominal_torque).norm();
    for (int arm = 0; arm < kArmCount; ++arm) {
      result.arm_intervention_norm[arm] =
          (result.safe_torque.segment<kJointsPerArm>(arm * kJointsPerArm) -
           nominal_torque.segment<kJointsPerArm>(arm * kJointsPerArm)).norm();
    }
    last_safe_torque_ = result.safe_torque;
    return finish();
  }

  buildConstraints(state, period_seconds, &result);
  if (invalid_constraint_) {
    result.fallback_active = result.fatal_input_fault = true;
    result.solver_status = "invalid_active_constraint";
    result.safe_torque = boundedFallback(state, period_seconds);
    return finish();
  }
  int hold_priority_arm = -1;
  if (activity_valid && parameters_.asymmetric_inter_arm_priority_enabled &&
      parameters_.inter_arm_collision_enabled && parameters_.active_arms[0] && parameters_.active_arms[1]) {
    if (motion_active[0] != motion_active[1]) {
      hold_priority_arm = motion_active[0] ? 1 : 0;
    } else if (!motion_active[0] && !motion_active[1] &&
               result.minimum_inter_arm_clearance <=
                   parameters_.priority_release_clearance) {
      // Keep the last approacher's responsibility until the arms separate.
      hold_priority_arm = last_hold_priority_arm_;
    }
  }
  const auto constraint_violation = [this](const Vector14d& torque) {
    if (!torque.allFinite()) {
      return std::numeric_limits<double>::infinity();
    }
    double maximum = 0.0;
    for (int constraint = 0; constraint < constraint_count_; ++constraint) {
      double value = 0.0;
      for (int joint = 0; joint < kDualArmDof; ++joint) {
        value += a_values_[joint * constraint_count_ + constraint] *
                 torque(joint);
      }
      if (lower_bound_[constraint] > -0.5 * OSQP_INFTY) {
        maximum = std::max(
            maximum, static_cast<double>(lower_bound_[constraint]) - value);
      }
      if (upper_bound_[constraint] < 0.5 * OSQP_INFTY) {
        maximum = std::max(
            maximum, value - static_cast<double>(upper_bound_[constraint]));
      }
    }
    return maximum;
  };


  const auto start = std::chrono::steady_clock::now();
  c_int weight_result = 0;
  if (hold_priority_arm != last_hold_priority_arm_) {
    for (int joint = 0; joint < kDualArmDof; ++joint) {
      const int arm = joint / kJointsPerArm;
      p_values_[joint] = static_cast<c_float>(
          parameters_.objective_weight(joint % kJointsPerArm) *
          (arm == hold_priority_arm ?
               parameters_.parked_arm_weight_multiplier : 1.0));
    }
    weight_result = osqp_update_P(
        workspace_, p_values_.data(), OSQP_NULL, kDualArmDof);
    if (weight_result == 0) {
      last_hold_priority_arm_ = hold_priority_arm;
    } else {
      // Keep the next linear cost consistent with the Hessian that OSQP had
      // before the rejected update; the current cycle will use the fallback.
      for (int joint = 0; joint < kDualArmDof; ++joint) {
        p_values_[joint] = static_cast<c_float>(
            parameters_.objective_weight(joint % kJointsPerArm) *
            (joint / kJointsPerArm == last_hold_priority_arm_ ?
                 parameters_.parked_arm_weight_multiplier : 1.0));
      }
    }
  }
  result.hold_priority_arm = last_hold_priority_arm_;
  for (int joint = 0; joint < kDualArmDof; ++joint) {
    linear_cost_[joint] = static_cast<c_float>(
        -p_values_[joint] *
        nominal_torque(joint));
  }

  // A feasible nominal torque is the exact minimum of the positive weighted
  // squared-distance objective. Check every current logical row with zero
  // violation before accepting it; no iterative optimization is needed.
  Vector14d feasible_nominal = nominal_torque;
  for (int arm = 0; arm < kArmCount; ++arm)
    if (!parameters_.active_arms[arm]) feasible_nominal.segment<kJointsPerArm>(arm * kJointsPerArm).setZero();
  if (weight_result == 0 && constraint_violation(feasible_nominal) == 0. &&
      osqp_warm_start_x(workspace_, feasible_nominal.data()) == 0) {
    result.safe_torque = feasible_nominal;
    result.solver_status = "feasible_nominal";
    result.solver_succeeded = true;
    result.solver_time_seconds = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start).count();
    result.intervention_norm = (feasible_nominal - nominal_torque).norm();
    for (int arm = 0; arm < kArmCount; ++arm)
      result.arm_intervention_norm[arm] =
          (feasible_nominal.segment<kJointsPerArm>(arm * kJointsPerArm) -
           nominal_torque.segment<kJointsPerArm>(arm * kJointsPerArm)).norm();
    last_safe_torque_ = feasible_nominal;
    return finish();
  }

  for (size_t index = 0; index < solver_a_values_.size(); ++index)
    solver_a_values_[index] = a_values_[solver_a_indices_[index]];
  const c_int matrix_result = osqp_update_A(
      workspace_, solver_a_values_.data(), OSQP_NULL,
      static_cast<c_int>(solver_a_values_.size()));
  const c_int cost_result =
      osqp_update_lin_cost(workspace_, linear_cost_.data());
  for (size_t index = 0; index < solver_logical_rows_.size(); ++index) {
    const auto row = solver_logical_rows_[index];
    solver_lower_bound_[index] = lower_bound_[row];
    solver_upper_bound_[index] = upper_bound_[row];
  }
  const c_int bound_result = osqp_update_bounds(
      workspace_, solver_lower_bound_.data(), solver_upper_bound_.data());
  const c_int solve_result =
      (weight_result == 0 && matrix_result == 0 && cost_result == 0 &&
       bound_result == 0)
          ? osqp_solve(workspace_)
          : -1;
  result.solver_time_seconds = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - start).count();

  const bool has_iterate = solve_result == 0 && workspace_->info != nullptr &&
      workspace_->solution != nullptr && workspace_->solution->x != nullptr;
  const bool solved = has_iterate &&
      (workspace_->info->status_val == OSQP_SOLVED ||
       workspace_->info->status_val == OSQP_SOLVED_INACCURATE);
  const bool iteration_limit = has_iterate &&
      workspace_->info->status_val == OSQP_MAX_ITER_REACHED;
  if (weight_result != 0) {
    result.solver_status = "osqp_hessian_update_error";
  } else if (matrix_result != 0 || cost_result != 0 || bound_result != 0) {
    result.solver_status = "osqp_update_error";
  } else if (workspace_->info != nullptr) {
    result.solver_status = solver_status_name(workspace_->info->status_val);
    result.solver_iterations = workspace_->info->iter;
  } else {
    result.solver_status = "osqp_api_error";
  }


  bool candidate_available = false;
  Vector14d bounded_candidate = Vector14d::Zero();
  if (solved || iteration_limit) {
    for (int joint = 0; joint < kDualArmDof; ++joint) {
      result.safe_torque(joint) = workspace_->solution->x[joint];
    }
    if (result.safe_torque.allFinite()) {
      // OSQP may hit the fixed RT iteration budget after finding a feasible
      // primal point but before meeting its dual optimality tolerance. The
      // objective is soft; every actuator/CBF bound is hard. Retain such an
      // iterate only after independently checking *all* current bounds.
      result.safe_torque =
          clampToActuatorBounds(result.safe_torque, period_seconds);
      bounded_candidate = result.safe_torque;
      candidate_available = true;
      const double violation = constraint_violation(result.safe_torque);
      result.solver_candidate_constraint_violation = violation;
      result.solver_succeeded =
          violation <= parameters_.solver_feasibility_tolerance;
      if (iteration_limit && result.solver_succeeded) {
        result.solver_status = "maximum iterations reached (feasible iterate)";
      }
    }
  }
  if (!result.solver_succeeded) {
    result.fallback_active = true;
    result.safe_torque = boundedFallback(state, period_seconds);
    double best_violation = constraint_violation(result.safe_torque);

    // Damping is not automatically a safe fallback: on light wrist joints it
    // can violate the model-based acceleration/CBF rows by orders of
    // magnitude. Compare only actuator-bounded commands and use the one with
    // the smallest *measured* constraint violation. The fallback flag remains
    // set whenever no fully feasible QP solution was obtained.
    const Vector14d held_torque =
        clampToActuatorBounds(last_safe_torque_, period_seconds);
    const double held_violation = constraint_violation(held_torque);
    if (held_violation < best_violation) {
      result.safe_torque = held_torque;
      best_violation = held_violation;
    }
    if (candidate_available &&
        result.solver_candidate_constraint_violation < best_violation) {
      result.safe_torque = bounded_candidate;
    }
  }
  result.maximum_constraint_violation = constraint_violation(result.safe_torque);
  result.intervention_norm = (result.safe_torque - nominal_torque).norm();
  for (int arm = 0; arm < kArmCount; ++arm) {
    result.arm_intervention_norm[arm] =
        (result.safe_torque.segment<kJointsPerArm>(arm * kJointsPerArm) -
         nominal_torque.segment<kJointsPerArm>(arm * kJointsPerArm)).norm();
  }
  last_safe_torque_ = result.safe_torque;
  return finish();
}

}  // namespace safety_filter::cbf
