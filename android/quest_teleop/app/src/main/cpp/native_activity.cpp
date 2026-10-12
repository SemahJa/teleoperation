#include <android_native_app_glue.h>
#include "openxr_program.h"
#include "telemetry.h"

extern "C" void android_main(android_app* app) {
  OpenXrProgram program;
  if (!program.Create(app)) { LOGE("OpenXR initialization failed"); return; }
  quest_usb::QuestTelemetry telemetry;
  quest_usb::LocalDeadman deadman;
  while (!app->destroyRequested) {
    int events = 0;
    android_poll_source* source = nullptr;
    while (true) {
      int id = ALooper_pollOnce(program.IsSessionRunning() ? 0 : 100, nullptr, &events,
                               reinterpret_cast<void**>(&source));
      if (id < 0) break;
      if (source) source->process(app, source);
      if (app->destroyRequested) break;
    }
    if (app->destroyRequested) break;
    bool exit = false;
    program.PollXrEvents(&exit);
    if (exit) break;
    if (!program.IsSessionRunning()) { deadman.disarm(); continue; }
    int64_t display_time = 0;
    if (!program.BeginFrame(&display_time)) { deadman.disarm(); continue; }
    TrackingFrame frame;
    program.Locate(&frame, display_time);
    const auto clock = telemetry.transport_.clock_snapshot(frame.acquisition_time_ns);
    deadman.update(frame, telemetry.transport_.connected(), clock.valid,
                   telemetry.transport_.take_disarm_request());
    telemetry.submit(frame);
    program.SetUsbStatus(telemetry.transport_.connected(), clock.valid, clock.uncertainty_ns,
                         deadman.armed, deadman.require_release, quest_usb::kBridgePort);
    program.EndFrame(display_time, frame);
  }
  telemetry.transport_.stop();
  program.Destroy();
}
