#pragma once
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <franka/robot_state.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <nlohmann/json.hpp>
#include <cmath>
#include <sstream>
#include <limits>
#include <stdexcept>
namespace panda_state_bringup {
struct Snapshot {
  sensor_msgs::msg::JointState joints;
  geometry_msgs::msg::PoseStamped pose;
  std_msgs::msg::String provenance;
};
inline Snapshot project(const franka::RobotState& s, const std_msgs::msg::Header& header,
                        uint64_t before, uint64_t received, uint64_t sequence,
                        const std::string& host_clock_id, const std::string& session,
                        uint64_t segment, uint16_t server_version, const std::string& robot_address="") {
  if (received < before || host_clock_id.empty() || session.empty())
    throw std::invalid_argument("invalid timing provenance");
  for (const auto& values : {s.q,s.dq,s.tau_J})
    for (double x:values) if (!std::isfinite(x)) throw std::invalid_argument("nonfinite measured joints");
  Eigen::Map<const Eigen::Matrix<double,4,4,Eigen::ColMajor>> t(s.O_T_EE.data());
  // Same legacy rounding bound as safety_filter/legacy_pose.hpp; no controller
  // behavior is changed. Raw O_T_EE is retained verbatim below.
  constexpr double rounding_limit=1e-4;
  const Eigen::Matrix3d raw=t.block<3,3>(0,0);
  const double residual=(raw.transpose()*raw-Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff();
  if (!t.allFinite() || !t.row(3).isApprox(Eigen::RowVector4d(0,0,0,1),1e-8) ||
      raw.determinant()<=0 || std::abs(raw.determinant()-1)>rounding_limit || residual>rounding_limit)
    throw std::invalid_argument("invalid measured O_T_EE");
  const Eigen::Matrix3d rotation=Eigen::Affine3d(t).rotation();
  const double correction=(rotation-raw).cwiseAbs().maxCoeff();
  if(!rotation.allFinite() || correction>rounding_limit ||
     !(rotation.transpose()*rotation).isApprox(Eigen::Matrix3d::Identity(),1e-8) || std::abs(rotation.determinant()-1)>=1e-8)
    throw std::invalid_argument("measured rotation cannot be represented within legacy rounding bound");
  Snapshot out;out.joints.header=header;out.pose.header=header;
  for(int j=0;j<7;++j)out.joints.name.push_back("panda_joint"+std::to_string(j+1));
  out.joints.position.assign(s.q.begin(),s.q.end());out.joints.velocity.assign(s.dq.begin(),s.dq.end());
  out.joints.effort.assign(s.tau_J.begin(),s.tau_J.end());
  out.pose.pose.position.x=t(0,3);out.pose.pose.position.y=t(1,3);out.pose.pose.position.z=t(2,3);
  Eigen::Quaterniond q(rotation);q.normalize();
  out.pose.pose.orientation.x=q.x();out.pose.pose.orientation.y=q.y();out.pose.pose.orientation.z=q.z();out.pose.pose.orientation.w=q.w();
  std::ostringstream native,mode;native.precision(std::numeric_limits<double>::max_digits10);native<<s;mode<<s.robot_mode;
  const int64_t ros_ns=static_cast<int64_t>(header.stamp.sec)*1000000000LL+header.stamp.nanosec;
  nlohmann::json doc={{"schema","panda_read_only_state_v1"},{"hardware_backend",true},{"read_only",true},
    {"session_id",session},{"sample_sequence",sequence},{"source_segment",segment},
    {"host_clock_id",host_clock_id},{"receipt_host_monotonic_ns",received},{"read_before_host_monotonic_ns",before},
    {"header_ros_ns",ros_ns},{"header_semantics","ROS observation after readOnce returns; not robot acquisition"},
    {"robot_address",robot_address},{"robot_clock_id","robot_ms:"+robot_address+":"+session},
    {"robot_time_ms",s.time.toMSec()},{"robot_clock_domain","robot_time_since_start_ms"},
    {"source_capture_uncertainty_ns",nullptr},{"execution_timestamp",nullptr},
    {"server_version",server_version},{"libfranka_version","0.9.2"},{"frame_id",header.frame_id},
    {"derived_pose_rotation_projection_max_abs_error",correction},{"raw_rotation_orthogonality_max_abs_error",residual},
    {"pose_representation","bounded orthonormal projection for quaternion only; original O_T_EE unchanged"},
    {"joint_position",s.q},{"joint_velocity",s.dq},{"measured_torque",s.tau_J},{"O_T_EE",s.O_T_EE},
    {"robot_mode",nlohmann::json::parse(mode.str())},{"robot_mode_code",static_cast<int>(s.robot_mode)},{"native_robot_state_text",native.str()}};
  out.provenance.data=doc.dump();return out;
}
}
