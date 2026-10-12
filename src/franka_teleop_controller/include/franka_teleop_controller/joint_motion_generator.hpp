// Copyright (c) 2023 Franka Robotics GmbH
// Copyright (c) 2026 Jiachun
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

#include <utility>

#include <Eigen/Core>

namespace franka_teleop_controller {

/// Synchronized, acceleration-limited seven-joint trajectory generator.
///
/// This is adapted from franka_example_controllers/MotionGenerator, which is
/// used by Franka's FR3 move_to_start_example_controller.
class JointMotionGenerator {
public:
  using Vector7d = Eigen::Matrix<double, 7, 1>;

  JointMotionGenerator(double speed_factor, const Vector7d &q_start,
                       const Vector7d &q_goal);

  std::pair<Vector7d, bool>
  getDesiredJointPositions(double trajectory_time_seconds);
  Vector7d getDesiredJointVelocities(double trajectory_time_seconds) const;

private:
  using Vector7i = Eigen::Matrix<int, 7, 1>;

  bool calculateDesiredValues(double time, Vector7d *delta_q_desired) const;
  void calculateSynchronizedValues();

  static constexpr double kDeltaQMotionTolerance = 1.0e-6;
  static constexpr int kJoints = 7;

  Vector7d q_start_{Vector7d::Zero()};
  Vector7d delta_q_{Vector7d::Zero()};
  Vector7d dq_max_sync_{Vector7d::Zero()};
  Vector7d t_1_sync_{Vector7d::Zero()};
  Vector7d t_2_sync_{Vector7d::Zero()};
  Vector7d t_f_sync_{Vector7d::Zero()};
  Vector7d q_1_{Vector7d::Zero()};

  Vector7d dq_max_{
      (Vector7d() << 2.0, 2.0, 2.0, 2.0, 2.5, 2.5, 2.5).finished()};
  Vector7d ddq_max_start_{Vector7d::Constant(5.0)};
  Vector7d ddq_max_goal_{Vector7d::Constant(5.0)};
};

} // namespace franka_teleop_controller
