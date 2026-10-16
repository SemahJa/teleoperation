#include <gtest/gtest.h>

#include "../src/tracking_message.hpp"

namespace ft = teleop_controllers;

TEST(Tracking, PreservesFourTorqueStagesAndPreviousCommandWithoutAligningThem) {
  ft::TrackingSnapshot s;
  s.hardware = true;
  s.cycle = 20;
  s.previous_cycle = 19;
  s.state_ns = 1000000000;
  s.control_ns = 1000000020;
  s.issued_ns = 1000000050;
  s.previous_issued_ns = 999000050;
  s.requested.source_ns = 990000000;
  s.control.nominal.setConstant(4);
  s.control.torque.setConstant(3);
  s.robot_desired.setConstant(2);
  s.measured_torque.setConstant(12);
  s.gravity.setConstant(10);
  s.previous_commanded.setConstant(2.5);
  const auto m = ft::tracking_message(s, "base", builtin_interfaces::msg::Time{},
                                      ft::Vector7::Constant(10), 900, true, 5, {});
  EXPECT_DOUBLE_EQ(m.nominal_torque[0], 4);
  EXPECT_DOUBLE_EQ(m.commanded_torque[0], 3);
  EXPECT_DOUBLE_EQ(m.robot_desired_torque[0], 2);
  EXPECT_DOUBLE_EQ(m.measured_torque[0], 12);
  EXPECT_DOUBLE_EQ(m.gravity_torque[0], 10);
  EXPECT_DOUBLE_EQ(m.previous_commanded_torque[0], 2.5);
  EXPECT_EQ(m.control_cycle, 20u);
  EXPECT_EQ(m.previous_control_cycle, 19u);
  EXPECT_TRUE(m.robot_torque_valid);
  EXPECT_TRUE(m.previous_command_valid);
  EXPECT_EQ(m.monitor_queue_drops, 5u);
  EXPECT_NEAR(m.command_age_ms, 10.00002, 1e-9);
  EXPECT_NEAR(m.normalized_torque_intervention, std::sqrt(7.) / 10., 1e-12);
}

TEST(Tracking, ComputesMeasuredTwistAndSignInvariantRotationError) {
  ft::TrackingSnapshot s;
  s.control.motion_enabled = true;
  s.state.jacobian.leftCols<6>().setIdentity();
  s.state.dq << .1, .2, .3, .4, .5, .6, 7.;
  s.state.position.x() = .01;
  s.state.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(.2, Eigen::Vector3d::UnitZ()));
  s.requested.requested_twist_valid = true;
  s.requested.requested_twist.setConstant(.7);
  const auto first = ft::tracking_message(s, "base", builtin_interfaces::msg::Time{},
                                          ft::Vector7::Ones(), 900, false, 0, {});
  EXPECT_DOUBLE_EQ(first.measured_twist.linear.x, .1);
  EXPECT_DOUBLE_EQ(first.measured_twist.angular.z, .6);
  EXPECT_DOUBLE_EQ(first.requested_twist.linear.x, .7);
  EXPECT_NEAR(first.position_error_norm, .01, 1e-12);
  EXPECT_NEAR(first.rotation_error_angle, .2, 1e-12);
  s.state.orientation.coeffs() *= -1;
  const auto second = ft::tracking_message(s, "base", builtin_interfaces::msg::Time{},
                                           ft::Vector7::Ones(), 900, false, 0, {});
  EXPECT_NEAR(first.rotation_error_angle, second.rotation_error_angle, 1e-12);
}

TEST(Tracking, MockAndInactiveSignalsAreExplicitlyUnavailable) {
  ft::TrackingSnapshot s;
  const auto m = ft::tracking_message(s, "base", builtin_interfaces::msg::Time{},
                                      ft::Vector7::Ones(), 900, false, 0, {});
  EXPECT_FALSE(m.robot_torque_valid);
  EXPECT_FALSE(m.gravity_valid);
  EXPECT_FALSE(m.previous_command_valid);
  EXPECT_FALSE(m.requested_pose_valid);
  EXPECT_FALSE(m.cartesian_reference_valid);
  EXPECT_TRUE(std::isnan(m.robot_desired_torque[0]));
  EXPECT_TRUE(std::isnan(m.measured_torque[0]));
  EXPECT_TRUE(std::isnan(m.reference_pose.position.x));
  EXPECT_TRUE(std::isnan(m.requested_twist.linear.x));
  EXPECT_TRUE(std::isnan(m.position_error_norm));
  EXPECT_DOUBLE_EQ(m.measured_twist.linear.x, 0);
}
