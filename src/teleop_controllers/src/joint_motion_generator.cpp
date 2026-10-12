// Copyright (c) 2023 Franka Robotics GmbH
// Copyright (c) 2026 Jiachun
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "teleop_controllers/joint_motion_generator.hpp"

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>

namespace teleop_controllers {

JointMotionGenerator::JointMotionGenerator(double speed_factor,
                                           const Vector7d &q_start,
                                           const Vector7d &q_goal)
    : q_start_(q_start), delta_q_(q_goal - q_start) {
  assert(speed_factor > 0.0 && speed_factor <= 1.0);
  dq_max_ *= speed_factor;
  ddq_max_start_ *= speed_factor;
  ddq_max_goal_ *= speed_factor;
  calculateSynchronizedValues();
}

bool JointMotionGenerator::calculateDesiredValues(
    double time, Vector7d *delta_q_desired) const {
  const Vector7i sign_delta_q = delta_q_.cwiseSign().cast<int>();
  const Vector7d constant_velocity_duration = t_2_sync_ - t_1_sync_;
  const Vector7d deceleration_duration = t_f_sync_ - t_2_sync_;
  std::array<bool, kJoints> joint_finished{};

  for (int index = 0; index < kJoints; ++index) {
    if (std::abs(delta_q_[index]) < kDeltaQMotionTolerance) {
      (*delta_q_desired)[index] = 0.0;
      joint_finished.at(index) = true;
    } else if (time < t_1_sync_[index]) {
      (*delta_q_desired)[index] = -dq_max_sync_[index] * sign_delta_q[index] /
                                  std::pow(t_1_sync_[index], 3.0) *
                                  (0.5 * time - t_1_sync_[index]) *
                                  std::pow(time, 3.0);
    } else if (time < t_2_sync_[index]) {
      (*delta_q_desired)[index] = q_1_[index] + (time - t_1_sync_[index]) *
                                                    dq_max_sync_[index] *
                                                    sign_delta_q[index];
    } else if (time < t_f_sync_[index]) {
      const double deceleration_time = deceleration_duration[index];
      const double constant_time = constant_velocity_duration[index];
      (*delta_q_desired)[index] =
          delta_q_[index] +
          0.5 *
              ((time - t_1_sync_[index] - 2.0 * deceleration_time -
                constant_time) *
                   std::pow(time - t_1_sync_[index] - constant_time, 3.0) /
                   std::pow(deceleration_time, 3.0) +
               (2.0 * time - 2.0 * t_1_sync_[index] - deceleration_time -
                2.0 * constant_time)) *
              dq_max_sync_[index] * sign_delta_q[index];
    } else {
      (*delta_q_desired)[index] = delta_q_[index];
      joint_finished.at(index) = true;
    }
  }

  return std::all_of(joint_finished.cbegin(), joint_finished.cend(),
                     [](bool finished) { return finished; });
}

void JointMotionGenerator::calculateSynchronizedValues() {
  Vector7d reachable_velocity(dq_max_);
  Vector7d finish_time = Vector7d::Zero();
  Vector7d deceleration_duration = Vector7d::Zero();
  Vector7d acceleration_duration = Vector7d::Zero();
  const Vector7i sign_delta_q = delta_q_.cwiseSign().cast<int>();

  for (int index = 0; index < kJoints; ++index) {
    if (std::abs(delta_q_[index]) <= kDeltaQMotionTolerance) {
      continue;
    }
    const double full_speed_distance =
        0.75 * std::pow(dq_max_[index], 2.0) / ddq_max_start_[index] +
        0.75 * std::pow(dq_max_[index], 2.0) / ddq_max_goal_[index];
    if (std::abs(delta_q_[index]) < full_speed_distance) {
      reachable_velocity[index] =
          std::sqrt((4.0 / 3.0) * delta_q_[index] * sign_delta_q[index] *
                    ddq_max_start_[index] * ddq_max_goal_[index] /
                    (ddq_max_start_[index] + ddq_max_goal_[index]));
    }
    acceleration_duration[index] =
        1.5 * reachable_velocity[index] / ddq_max_start_[index];
    deceleration_duration[index] =
        1.5 * reachable_velocity[index] / ddq_max_goal_[index];
    finish_time[index] = 0.5 * acceleration_duration[index] +
                         0.5 * deceleration_duration[index] +
                         std::abs(delta_q_[index]) / reachable_velocity[index];
  }

  const double synchronized_finish_time = finish_time.maxCoeff();
  for (int index = 0; index < kJoints; ++index) {
    if (std::abs(delta_q_[index]) <= kDeltaQMotionTolerance) {
      continue;
    }
    const double coefficient_a =
        0.75 * (ddq_max_goal_[index] + ddq_max_start_[index]);
    const double coefficient_b = -synchronized_finish_time *
                                 ddq_max_goal_[index] * ddq_max_start_[index];
    const double coefficient_c = std::abs(delta_q_[index]) *
                                 ddq_max_goal_[index] * ddq_max_start_[index];
    const double discriminant =
        std::max(0.0, coefficient_b * coefficient_b -
                          4.0 * coefficient_a * coefficient_c);

    dq_max_sync_[index] =
        (-coefficient_b - std::sqrt(discriminant)) / (2.0 * coefficient_a);
    t_1_sync_[index] = 1.5 * dq_max_sync_[index] / ddq_max_start_[index];
    const double synchronized_deceleration =
        1.5 * dq_max_sync_[index] / ddq_max_goal_[index];
    t_f_sync_[index] = 0.5 * t_1_sync_[index] +
                       0.5 * synchronized_deceleration +
                       std::abs(delta_q_[index]) / dq_max_sync_[index];
    t_2_sync_[index] = t_f_sync_[index] - synchronized_deceleration;
    q_1_[index] =
        0.5 * dq_max_sync_[index] * sign_delta_q[index] * t_1_sync_[index];
  }
}

std::pair<JointMotionGenerator::Vector7d, bool>
JointMotionGenerator::getDesiredJointPositions(double trajectory_time_seconds) {
  Vector7d delta_q_desired = Vector7d::Zero();
  const bool finished = calculateDesiredValues(
      std::max(0.0, trajectory_time_seconds), &delta_q_desired);
  return {q_start_ + delta_q_desired, finished};
}

JointMotionGenerator::Vector7d JointMotionGenerator::getDesiredJointVelocities(double time) const {
  Vector7d velocity = Vector7d::Zero();
  time = std::max(0.0, time);
  for (int j = 0; j < kJoints; ++j) {
    if (std::abs(delta_q_[j]) < kDeltaQMotionTolerance || time >= t_f_sync_[j]) continue;
    const double signed_speed = std::copysign(dq_max_sync_[j], delta_q_[j]);
    if (time < t_1_sync_[j]) {
      const double u = time / t_1_sync_[j];
      velocity[j] = signed_speed * (3 * u * u - 2 * u * u * u);
    } else if (time < t_2_sync_[j]) {
      velocity[j] = signed_speed;
    } else {
      const double u = (time - t_2_sync_[j]) / (t_f_sync_[j] - t_2_sync_[j]);
      velocity[j] = signed_speed * (1 - 3 * u * u + 2 * u * u * u);
    }
  }
  return velocity;
}
} // namespace teleop_controllers
