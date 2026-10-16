// Copyright (c) 2026 Jiachun
// Licensed under the Apache License, Version 2.0.

#pragma once

#include <osqp.h>

#include <array>
#include <cstddef>
#include <string>
#include <utility>
#include <vector>

#include "safety_filter/actuator_limits.hpp"
#include "safety_filter/cbf_types.hpp"

namespace safety_filter::cbf {

struct SafetyParameters {
  std::array<bool, kArmCount> active_arms{true, true};
  bool enabled{true};
  bool joint_position_enabled{true};
  bool joint_velocity_enabled{true};
  bool joint_acceleration_enabled{true};
  bool table_enabled{true};
  bool self_collision_enabled{true};
  bool inter_arm_collision_enabled{true};
  bool obstacle_collision_enabled{false};
  Vector7d minimum_joint_position{
      (Vector7d() << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973).finished()};
  Vector7d maximum_joint_position{
      (Vector7d() << 2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973).finished()};
  Vector7d maximum_joint_velocity{
      (Vector7d() << 2.175, 2.175, 2.175, 2.175, 2.61, 2.61, 2.61).finished()};
  Vector7d maximum_joint_acceleration{Vector7d::Constant(7.0)};
  Vector7d maximum_joint_torque{safety_filter::panda_torque_limits()};
  Vector7d objective_weight{Vector7d::Ones()};
  bool asymmetric_inter_arm_priority_enabled{true};
  double parked_arm_weight_multiplier{12.0};
  double priority_release_clearance{0.20};
  double maximum_torque_rate{safety_filter::panda_torque_rate_limit()};
  double joint_position_margin{0.10};
  double joint_velocity_scale{0.90};
  double joint_cbf_frequency{8.0};
  double joint_cbf_damping_ratio{1.0};
  double velocity_cbf_gain{10.0};
  double table_height{0.0};
  int table_minimum_link_index{2};
  double table_minimum_clearance{0.02};
  double self_minimum_clearance{0.03};
  double inter_arm_minimum_clearance{0.05};
  double obstacle_minimum_clearance{0.04};
  double collision_activation_distance{0.15};
  double collision_cbf_frequency{8.0};
  double collision_cbf_damping_ratio{1.2};
  double fallback_damping{5.0};
  int solver_max_iterations{100};
  double solver_rho{1.0};
  double solver_absolute_tolerance{1.0e-4};
  double solver_relative_tolerance{1.0e-4};
  double solver_feasibility_tolerance{1.0e-3};
  std::vector<double> obstacle_centers;
  std::vector<double> obstacle_radii;
};

struct SafetyResult {
  Vector14d safe_torque{Vector14d::Zero()};
  const char* solver_status{"not_configured"};
  double solver_time_seconds{0.0};
  double intervention_norm{0.0};
  double maximum_constraint_violation{0.0};
  double solver_candidate_constraint_violation{0.0};
  double minimum_table_clearance{1.0e9};
  double minimum_self_clearance{1.0e9};
  double minimum_inter_arm_clearance{1.0e9};
  double minimum_obstacle_clearance{1.0e9};
  int solver_iterations{0};
  int active_constraints{0};
  int joint_position_constraints{0};
  int joint_velocity_constraints{0};
  int joint_acceleration_constraints{0};
  int table_constraints{0};
  int self_collision_constraints{0};
  int inter_arm_constraints{0};
  int obstacle_constraints{0};
  int hold_priority_arm{-1};
  std::array<bool, kArmCount> nominal_motion_active{false, false};
  std::array<double, kArmCount> arm_intervention_norm{0.0, 0.0};
  bool solver_succeeded{false};
  bool fallback_active{false};
  bool fatal_input_fault{false};
  double filter_time_seconds{0};
};

// Fixed-pattern OSQP safety filter. The problem shape and sparse
// storage are allocated during configure(); update() only changes coefficients.
class CbfSafetyFilter {
 public:
  CbfSafetyFilter() = default;
  ~CbfSafetyFilter();
  CbfSafetyFilter(const CbfSafetyFilter&) = delete;
  CbfSafetyFilter& operator=(const CbfSafetyFilter&) = delete;

  bool configure(const SafetyParameters& parameters,
                 const std::array<ArmIdentity, kArmCount>& identities, std::string* error);
  void reset();
  SafetyResult filter(const Vector14d& nominal_torque, const DualArmState& state,
                      double period_seconds, const std::array<bool, kArmCount>& motion_active,
                      bool activity_valid = true);
  [[nodiscard]] bool configured() const { return workspace_ != nullptr; }
  [[nodiscard]] size_t solverConstraintCount() const { return solver_logical_rows_.size(); }

 private:
  struct Capsule {
    int link_index{0};
    int frame_index{-1};
    Eigen::Vector3d center{Eigen::Vector3d::Zero()};
    Eigen::Vector3d direction{Eigen::Vector3d::UnitZ()};
    double length{0.0};
    double radius{0.0};
  };

  struct Endpoint {
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    PointJacobian jacobian{PointJacobian::Zero()};
  };

  struct WorldCapsule {
    Endpoint first;
    Endpoint second;
    double radius{0.0};
    int link_index{0};
  };

  struct Obstacle {
    Eigen::Vector3d center{Eigen::Vector3d::Zero()};
    double radius{0.0};
  };

  void cleanupSolver();
  bool setupSolver(std::string* error);
  void clearRows();
  void setRow(int row, const Vector14d& coefficients, double lower, double upper);
  void disableRow(int row);
  void addLowerConstraint(int row, const Vector14d& coefficients, double lower, bool active,
                          int* active_counter);
  WorldCapsule worldCapsule(int arm, const Capsule& capsule, const ArmState& state) const;
  Endpoint endpoint(int arm, int frame_index, const Eigen::Vector3d& local_point,
                    const ArmState& state) const;
  static std::pair<double, double> closestSegmentParameters(const Eigen::Vector3d& p1,
                                                            const Eigen::Vector3d& q1,
                                                            const Eigen::Vector3d& p2,
                                                            const Eigen::Vector3d& q2);
  static double pointSegmentParameter(const Eigen::Vector3d& point, const Eigen::Vector3d& first,
                                      const Eigen::Vector3d& second);
  static bool selfPairExcluded(int first_link, int second_link);
  Eigen::Vector3d jacobianDerivativeVelocity(int row, const PointJacobian& jacobian,
                                             const Vector14d& velocity, double period_seconds);
  void buildConstraints(const DualArmState& state, double period_seconds, SafetyResult* result);
  Vector14d boundedFallback(const DualArmState& state, double period_seconds) const;
  Vector14d clampToActuatorBounds(const Vector14d& requested, double period_seconds) const;

  SafetyParameters parameters_{};
  std::array<ArmIdentity, kArmCount> identities_{};
  std::vector<Capsule> capsules_;
  std::vector<std::pair<int, int>> self_pairs_;
  std::vector<std::pair<int, int>> inter_arm_pairs_;
  std::vector<Obstacle> obstacles_;
  std::vector<PointJacobian> previous_constraint_jacobians_;
  std::vector<bool> previous_constraint_jacobian_valid_;
  std::vector<Eigen::Vector3d> previous_collision_normals_;
  std::vector<bool> previous_collision_normal_valid_;

  int constraint_count_{0};
  int actuator_row_begin_{0};
  int joint_position_row_begin_{0};
  int joint_velocity_row_begin_{0};
  int joint_acceleration_row_begin_{0};
  int table_row_begin_{0};
  int self_row_begin_{0};
  int inter_arm_row_begin_{0};
  int obstacle_row_begin_{0};

  std::vector<c_float> p_values_;
  std::vector<c_int> p_rows_;
  std::vector<c_int> p_columns_;
  std::vector<c_float> a_values_;
  // Dense logical rows retain independent feasibility checking. The solver
  // stores only coefficients that can be nonzero for the configured arm mask.
  std::vector<c_float> solver_a_values_;
  std::vector<size_t> solver_a_indices_;
  std::vector<int> solver_logical_rows_;
  std::vector<c_float> solver_lower_bound_, solver_upper_bound_;
  std::vector<c_int> a_rows_;
  std::vector<c_int> a_columns_;
  std::vector<c_float> linear_cost_;
  std::vector<c_float> lower_bound_;
  std::vector<c_float> upper_bound_;
  csc p_matrix_{};
  csc a_matrix_{};
  OSQPData data_{};
  OSQPSettings settings_{};
  OSQPWorkspace* workspace_{nullptr};
  Vector14d last_safe_torque_{Vector14d::Zero()};
  Vector14d actuator_anchor_{Vector14d::Zero()};
  bool invalid_constraint_{false};
  int last_hold_priority_arm_{-1};
};

}  // namespace safety_filter::cbf
