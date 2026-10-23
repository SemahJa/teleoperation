#pragma once
#include "tracking.h"
#include "usb_transport.h"

namespace quest_usb {
inline float unit_input(float value) {
  return std::isfinite(value) ? std::clamp(value, 0.0F, 1.0F) : 0.0F;
}
struct Vector3 { float x, y, z; };
struct Quaternion { float x, y, z, w; };

// Display-only deadman; ROS remains the authority for actual motion enable.
class LocalDeadman {
 public:
  void disarm() { armed = false; require_release = true; }
  void update(const TrackingFrame& frame, bool connected, bool clock_valid, bool transport_fault) {
    if (transport_fault || frame.recenter_count != generation ||
        (last_acquisition != 0 && (frame.acquisition_time_ns <= last_acquisition ||
         frame.acquisition_time_ns - last_acquisition > 100'000'000ULL))) disarm();
    generation = frame.recenter_count;
    last_acquisition = frame.acquisition_time_ns;
    const auto& p = frame.right_grip;
    const auto& in = frame.right_input;
    if (!frame.session_focused || !connected || !clock_valid || !p.action_active ||
        !p.valid || !p.active || !in.trigger_active || frame.acquisition_time_ns == 0) disarm();
    else if (in.trigger <= 0.25f) { armed = false; require_release = false; }
    else if (!require_release && in.trigger >= 0.75f) armed = true;
  }
  bool armed = false, require_release = true;
 private:
  uint64_t last_acquisition = 0;
  uint32_t generation = 0;
};

class QuestTelemetry {
 public:
  void submit(const TrackingFrame& frame) {
    reference_space_generation_ = frame.recenter_count;
    focused_ = frame.session_focused;
    for (int hand = 0; hand < 2; ++hand) {
      const auto& p = hand == 0 ? frame.left_grip : frame.right_grip;
      const auto& in = hand == 0 ? frame.left_input : frame.right_input;
      const bool right = hand == 1;
      submit_sample(right ? "right" : "left", frame.acquisition_time_ns, frame.xr_time_ns,
        {p.px,p.py,p.pz}, {p.qx,p.qy,p.qz,p.qw}, p.position_valid, p.orientation_valid,
        p.position_tracked, p.orientation_tracked, p.action_active,
        {p.vx,p.vy,p.vz}, {p.wx,p.wy,p.wz},
        p.linear_velocity_source == VelocitySource::Native,
        p.angular_velocity_source == VelocitySource::Native,
        in.trigger, in.trigger_active, in.squeeze, in.squeeze_active,
        right && in.primary, right && in.primary_active, right && in.secondary, right && in.secondary_active,
        !right && in.primary, !right && in.primary_active, !right && in.secondary, !right && in.secondary_active,
        !right && in.menu, !right && in.menu_active, in.thumb_click, in.thumb_click_active);
    }
  }
  UsbTransport transport_;
 private:
  std::string session_id_ = make_session_id();
  uint64_t sequence_ = 0;
  uint32_t reference_space_generation_ = 0;
  bool focused_ = false;
  void submit_sample(const char* controller, uint64_t acquisition, int64_t location_time,
                     Vector3 p, Quaternion q, bool p_valid, bool q_valid,
                     bool p_tracked, bool q_tracked, bool pose_active,
                     Vector3 linear, Vector3 angular,
                     bool linear_valid, bool angular_valid, float trigger, bool trigger_active,
                     float grip, bool grip_active, bool a, bool a_active, bool b, bool b_active,
                     bool x, bool x_active, bool y, bool y_active, bool menu, bool menu_active,
                     bool thumb_click, bool thumb_click_active) {
    std::ostringstream json;
    const uint64_t sequence = sequence_++;
    const ClockSnapshot clock = transport_.clock_snapshot(acquisition);
    json << std::setprecision(9)
         << "{\"version\":1,\"type\":\"controller_input\",\"session_id\":\"" << session_id_
         << "\",\"sequence\":\"" << sequence << "\",\"acquisition_time_ns\":\"" << acquisition
         << "\",\"openxr_location_time_ns\":\"" << static_cast<int64_t>(location_time)
         << "\",\"clock_sync_valid\":" << (clock.valid ? "true" : "false")
         << ",\"clock_offset_pc_minus_quest_ns\":\"" << clock.pc_minus_quest_ns
         << "\",\"clock_uncertainty_ns\":\"" << clock.uncertainty_ns
         << "\",\"clock_sync_age_ns\":\"" << clock.age_ns
         << "\",\"reference_space\":\"LOCAL\",\"reference_space_generation\":\""
         << reference_space_generation_ << "\",\"controller_id\":\"" << controller
         << "\",\"session_focused\":" << (focused_ ? "true" : "false")
         << ",\"pose\":{\"pose_action_active\":" << (pose_active ? "true" : "false")
         << ",\"position\":[" << p.x << ',' << p.y << ',' << p.z
         << "],\"orientation_xyzw\":[" << q.x << ',' << q.y << ',' << q.z << ',' << q.w
         << "],\"position_valid\":" << (p_valid ? "true" : "false")
         << ",\"orientation_valid\":" << (q_valid ? "true" : "false")
         << ",\"position_tracked\":" << (p_tracked ? "true" : "false")
         << ",\"orientation_tracked\":" << (q_tracked ? "true" : "false")
         << "},\"native_twist\":{\"linear\":[" << linear.x << ',' << linear.y << ',' << linear.z
         << "],\"angular\":[" << angular.x << ',' << angular.y << ',' << angular.z
         << "],\"linear_valid\":" << (linear_valid ? "true" : "false")
         << ",\"angular_valid\":" << (angular_valid ? "true" : "false")
         << "},\"inputs\":{\"trigger\":" << unit_input(trigger)
         << ",\"trigger_active\":" << (trigger_active ? "true" : "false")
         << ",\"grip\":" << unit_input(grip)
         << ",\"grip_active\":" << (grip_active ? "true" : "false")
         << ",\"thumbstick_click\":" << (thumb_click ? "true" : "false")
         << ",\"thumbstick_click_active\":" << (thumb_click_active ? "true" : "false");
    if (std::strcmp(controller, "right") == 0) {
      json << ",\"button_a\":" << (a ? "true" : "false")
           << ",\"button_a_active\":" << (a_active ? "true" : "false")
           << ",\"button_b\":" << (b ? "true" : "false")
           << ",\"button_b_active\":" << (b_active ? "true" : "false");
    } else {
      json << ",\"button_x\":" << (x ? "true" : "false")
           << ",\"button_x_active\":" << (x_active ? "true" : "false")
           << ",\"button_y\":" << (y ? "true" : "false")
           << ",\"button_y_active\":" << (y_active ? "true" : "false")
           << ",\"menu_button\":" << (menu ? "true" : "false")
           << ",\"menu_button_active\":" << (menu_active ? "true" : "false");
    }
    json << "}}";
    transport_.submit(controller, sequence, acquisition, json.str());
  }

};
}  // namespace quest_usb
