// Ported from thesis_ws; modified for USB telemetry and native-only validity.
#pragma once
#include <cstdint>

enum class VelocitySource : uint8_t {
    Invalid = 0,
    Native = 1,
};

// One device pose expressed in the LOCAL reference space (OpenXR right-handed,
// metres, quaternion in x,y,z,w order).
struct Pose {
    bool action_active = false;
    bool position_valid = false, orientation_valid = false;
    bool position_tracked = false, orientation_tracked = false;
    bool valid = false;   // position & orientation are valid this frame
    bool active = false;  // device is actively tracked (not extrapolated/stale)
    float px = 0, py = 0, pz = 0;
    float qx = 0, qy = 0, qz = 0, qw = 1;
    float vx = 0, vy = 0, vz = 0;
    float wx = 0, wy = 0, wz = 0;
    VelocitySource linear_velocity_source = VelocitySource::Invalid;
    VelocitySource angular_velocity_source = VelocitySource::Invalid;
};

// Controller input state.
struct ControllerInput {
    bool trigger_active = false, squeeze_active = false;
    bool primary_active = false, secondary_active = false, menu_active = false;
    bool thumb_active = false, thumb_click_active = false;
    float trigger = 0.0f;   // 0..1
    float squeeze = 0.0f;   // grip, 0..1
    float thumb_x = 0.0f;   // -1..1
    float thumb_y = 0.0f;   // -1..1
    bool thumb_click = false;
    bool primary = false;   // A (right) / X (left)
    bool secondary = false; // B (right) / Y (left)
    bool menu = false;      // left menu button only on Touch
};

// One frame of tracking data for all devices.
struct TrackingFrame {
    static constexpr uint32_t kProtocolVersion = 1;

    uint64_t acquisition_time_ns = 0;  // Quest CLOCK_MONOTONIC, not predicted time
    bool session_focused = false;
    double t = 0.0;      // seconds since app start
    int64_t xr_time_ns = 0;  // predicted display time used for xrLocateSpace
    uint64_t session_id = 0; // changes whenever the application process starts
    uint64_t seq = 0;    // monotonically increasing sequence number

    Pose hmd;
    Pose left_grip, right_grip;
    Pose left_aim, right_aim;
    ControllerInput left_input, right_input;

    // Number of system recenters observed so far (reference space changes).
    uint32_t recenter_count = 0;
};
