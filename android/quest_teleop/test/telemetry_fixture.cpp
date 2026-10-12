#include "telemetry.h"
#include <cassert>

using namespace quest_usb;

TrackingFrame valid_frame() {
  TrackingFrame f;
  f.session_focused = true;
  f.acquisition_time_ns = 1;
  auto& p = f.right_grip;
  p.action_active = p.valid = p.active = true;
  p.position_valid = p.orientation_valid = p.position_tracked = p.orientation_tracked = true;
  f.right_input.trigger_active = true;
  return f;
}

void test_deadman() {
  auto f = valid_frame();
  LocalDeadman d;
  auto update = [&] { ++f.acquisition_time_ns; d.update(f, true, true, false); };
  f.right_input.trigger = 1.0f;
  update(); assert(!d.armed && d.require_release);
  f.right_input.trigger = 0.0f;
  update(); assert(!d.armed && !d.require_release);
  f.right_input.trigger = 0.8f;
  update(); assert(d.armed);
  f.right_input.trigger = 0.5f;
  update(); assert(d.armed);
  f.right_input.trigger = 0.2f;
  update(); assert(!d.armed);
  f.right_input.trigger = 1.0f;
  update(); assert(d.armed);
  f.right_input.trigger_active = false;
  update(); assert(!d.armed && d.require_release);
  f.right_input.trigger_active = true;
  update(); assert(!d.armed);
  f.right_input.trigger = 0.0f; update();
  f.right_input.trigger = 1.0f; update(); assert(d.armed);
  d.update(f, false, true, false); assert(!d.armed && d.require_release);
  update(); assert(!d.armed);
  f.right_input.trigger = 0.0f; update();
  f.right_input.trigger = 1.0f; update(); assert(d.armed);
  d.update(f, true, false, false); assert(!d.armed && d.require_release);
  f.right_input.trigger = 0.0f; update();
  f.right_input.trigger = 1.0f; update(); assert(d.armed);
  ++f.recenter_count; update(); assert(!d.armed && d.require_release);
  f.right_input.trigger = 0.0f; update();
  f.right_input.trigger = 1.0f; update(); assert(d.armed);
  f.acquisition_time_ns += 100'000'001; update(); assert(!d.armed);
  f.right_input.trigger = 0.0f; update();
  f.right_input.trigger = 1.0f; update(); assert(d.armed);
  f.right_grip.active = false; update(); assert(!d.armed);
  f.right_grip.active = true;
  f.right_input.trigger = 0.0f; update();
  f.right_input.trigger = 1.0f; update(); assert(d.armed);
  f.session_focused = false; update(); assert(!d.armed);
  f.session_focused = true;
  f.right_input.trigger = 0.0f; update();
  f.right_input.trigger = 1.0f; update(); assert(d.armed);
  d.update(f, true, true, true); assert(!d.armed && d.require_release);
  assert(unit_input(std::numeric_limits<float>::quiet_NaN()) == 0.0f);
}

int main() {
  test_deadman();
  QuestTelemetry telemetry;
  auto frame = valid_frame();
  frame.left_grip = frame.right_grip;
  frame.left_grip.px = -0.25f;
  frame.left_grip.linear_velocity_source = VelocitySource::Invalid;
  frame.left_grip.angular_velocity_source = VelocitySource::Native;
  frame.left_grip.wy = 0.2f;
  frame.right_grip.px = 0.25f;
  frame.right_grip.linear_velocity_source = VelocitySource::Native;
  frame.right_grip.vx = 0.1f;
  frame.right_grip.angular_velocity_source = VelocitySource::Invalid;
  frame.left_input.primary = frame.left_input.primary_active = true;
  frame.left_input.secondary_active = true;
  frame.left_input.menu_active = true;
  frame.left_input.trigger_active = false;
  frame.left_input.squeeze_active = true;
  frame.left_input.squeeze = 0.4f;
  frame.right_input.primary = frame.right_input.primary_active = true;
  frame.right_input.secondary = false;
  frame.right_input.secondary_active = false;
  frame.right_input.trigger = 0.9f;
  for (int i = 0; i < 180; ++i) {
    frame.acquisition_time_ns = monotonic_time_ns();
    frame.xr_time_ns = static_cast<int64_t>(frame.acquisition_time_ns + 10'000'000);
    if (i == 90) ++frame.recenter_count;
    telemetry.submit(frame);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  telemetry.transport_.stop();
}
