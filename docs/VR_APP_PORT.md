# Quest app port review — 2026-10-10

This document records the headset port and its verification. The native OpenXR
app remains the current implementation; it does not require Unity. Use the
[main README](../README.md) for build/install and USB commands. Current ROS
controller behavior is documented separately in
[PACKAGE_STRUCTURE.md](PACKAGE_STRUCTURE.md).

Reviewed both repositories' VR apps, transport/message contracts, mapping
and gripper behavior, container definitions, and this repository's
`docs/AGENTS.md`. Existing uncommitted changes were preserved. The source
workspace contains several repositories; the relevant source app is under
`thesis_ws/src/vr_teleoperation/quest_openxr`.

## Compatibility decisions

| Area | thesis_ws implementation | Result in franka_teleoperation |
| --- | --- | --- |
| Headset app | Native OpenXR lifecycle, HUD, passthrough, aim rays, both hands | Ported into `android/quest_teleop`; readable per-hand diagnostics replace color blocks. |
| Transport | UDP broadcast/unicast, LAN subscription | Existing loopback TCP over physical USB/ADB reverse retained verbatim. No UDP code or Wi-Fi permissions copied. |
| Timing | Predicted OpenXR location time and process-relative time | Existing Quest CLOCK_MONOTONIC acquisition time, four-timestamp synchronization, 10 ms uncertainty gate, 100 ms age checks retained. OpenXR prediction time remains separate. |
| Velocity | Native values with pose-derived fallback | Independent native validity flags retained. No on-headset differentiation; invalid remains invalid. |
| Inputs | Per-hand primary/secondary actions, aim/thumbstick | Source per-hand bindings adopted; A/B and X/Y/menu still serialize separately with activity flags. Thumbstick/aim/HMD remain local diagnostics. |
| Recovery | FR3 mapper can re-anchor during brief tracking recovery | Current Panda mapper's release/re-engagement policy retained; no mapper changes. |
| Robot backend | ROS Jazzy, dual FR3, centralized CBF-QP, continuous gripper width, guarded dual homing | Kept Humble/Panda ownership and discrete grip toggles. Homing and CBF were implemented later in ROS; no FR3 backend packages were copied. |
| Build/deployment | Different APK identity, Gradle/NDK, Maven loader, host build instructions | Existing APK identity, pinned OpenXR SDK loader, container/toolchain/signing and scripts retained. Gradle now selects the existing container's NDK 26.3.11579264 explicitly. |

The port preserves the source app's main headset functionality while adapting
the incompatible transport and safety semantics. The passthrough extension
is discovered at runtime and is optional in the manifest. Unsupported or
failed passthrough setup uses the opaque compositor background. The
passthrough composition layer keeps `XR_NULL_HANDLE` as its space.

The HUD labels its trigger latch as local and reports robot arming as unknown.
Inactive trigger actions, invalid tracking/focus/clock, transport faults,
sample gaps, and reference-space changes disarm this display latch. Only a
valid trigger release followed by a new press enables it. No robot command
or hardware activation was added.

## Verification

- Android `gradle assembleDebug` passes inside the existing Android container,
  with NDK 26.3.11579264 and warnings treated as errors for app C++.
- Existing ROS regression suite passes: 16 mapper tests and one bridge-to-mapper
  integration test, using mock measured poses and no physical FCI consumer.
- New actual C++ adapter/USB sender regression passes against a mock loopback
  TCP peer. It checks role-specific button values/activity, native linear and
  angular validity independently, acquisition versus prediction time,
  sequence/session/reference generation, framing, and rejection/recovery of
  a clock estimate above/below the unchanged 10 ms gate. C++ assertions cover
  diagnostic hysteresis and re-engagement after transport/clock/tracking/focus,
  inactive-trigger, recenter, and timing faults.
- File hashes captured before the port confirm no changes to `docker/`,
  `scripts/`, `src/`, or `config/` during this task. The extracted `UsbTransport`
  class matches the original implementation byte-for-byte. The source
  `thesis_ws` workspace was read only.

## Remaining runtime checks

Update 2026-10-10: the user confirms all display checks and USB unplug/replug
behavior pass. Supplied telemetry shows valid tracking/twist/timing, stable
right A in two consecutive samples, and left Y pressed. Opposite-hand button
placeholders are now removed by the ROS `buttons` list; see
`quest-usb-protocol.md`. The checks below record the original acceptance plan;
they do not imply the confirmed display checks need to be repeated. Sustained
device button flicker is not reproduced in the supplied samples.

Use the existing containerized USB install/reverse workflow for a robot-free
Quest check. No APK was installed or headset/robot connection made by this task.

1. Confirm readable upright left/right diagnostics, controller aim rays, and
   panel placement after recentering.
2. Retest passthrough and inspect `xrEndFrame` logs for the previously reported
   `XR_ERROR_LAYER_INVALID`. Build success does not prove compositor behavior.
3. Hold each controller's buttons and inspect the matching per-hand ROS topic
   alongside HUD value/activity. The port adopts the source app's shared
   primary/secondary actions with per-hand subaction paths; it adds no debounce
   or filter and does not claim the reported device flicker is resolved.
4. Unplug/reconnect USB and interrupt tracking/focus; verify invalid telemetry,
   clock gating, and release/re-engagement with the robot-free mapper.

Later ROS controller/gripper work is now implemented; see the
[implementation record](IMPLEMENTATION_PROGRESS.md). The port checks above are
historical evidence and do not authorize hardware operation.
