// Ported from thesis_ws; modified for USB status and safe raw tracking.
#pragma once

#include <jni.h>
#include <string>
#include "tracking.h"

struct android_app;

// Native OpenXR tracking, passthrough, world-fixed diagnostic HUD and aim rays.
// Telemetry and all robot decisions remain outside this renderer.
class OpenXrProgram {
public:
    OpenXrProgram();
    ~OpenXrProgram();
    OpenXrProgram(const OpenXrProgram&) = delete;
    OpenXrProgram& operator=(const OpenXrProgram&) = delete;

    // Full init. Returns false on unrecoverable failure.
    bool Create(android_app* app);

    // Pumps OpenXR events (session state, recenter, loss). Sets *exitLoop when
    // the app should terminate.
    void PollXrEvents(bool* exitLoop);

    bool IsSessionRunning() const { return session_running_; }

    // Waits for and begins a frame. On success fills *predictedDisplayTime and
    // returns true; caller must then call Locate() and EndFrame().
    bool BeginFrame(int64_t* predictedDisplayTime);

    // Locates all devices in the LOCAL space at the given time and reads inputs.
    void Locate(TrackingFrame* out, int64_t predictedDisplayTime);

    // Local diagnostics only; this is never a robot-arming acknowledgement.
    void SetUsbStatus(bool connected, bool clockValid, uint64_t uncertaintyNs,
                      bool rightArmed, bool requireRelease, int port);

    // Ends the frame. Renders the debug HUD as a world-fixed quad layer when the
    // compositor asks us to render; otherwise submits zero layers.
    void EndFrame(int64_t predictedDisplayTime, const TrackingFrame& frame);

    void Destroy();

private:
    struct Impl;
    Impl* impl_;

    bool session_running_ = false;
};

