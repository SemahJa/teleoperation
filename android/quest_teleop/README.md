# Franka Quest USB app

The native OpenXR lifecycle, world-fixed diagnostic panel, bitmap font,
passthrough underlay, and controller aim rays are ported from
`thesis_ws/src/vr_teleoperation/quest_openxr`. The app retains this repository's
USB-only transport and existing ROS input contract. See
[`docs/VR_APP_PORT.md`](../../docs/VR_APP_PORT.md) for the review, adaptations,
verification results, and remaining on-headset checks.

Build from the repository root using the existing container:

```bash
docker compose -f docker/android-compose.yaml run --rm android-build gradle assembleDebug
```

The APK is `app/build/outputs/apk/debug/app-debug.apk`. Its package remains
`org.franka.questteleop`; existing install/reverse scripts and the persisted
debug-signing volume apply. NDK 26.3.11579264 matches the existing Docker image.

The HUD shows USB connection, clock validity/uncertainty, right-controller
**local** armed/release state, session/recenter status, each controller's
position/quaternion/native twist and raw input activity, and HMD position.
`P/Q` indicate position/orientation validity; `T` shows their independent
tracking flags. Velocity labels `N/I` mean native/invalid. Inputs display
`value/active`, so inactive actions remain distinguishable from released ones.
ROS messages list buttons by name (`a/b` for right, `x/y/menu` for left) with
separate `pressed` and `active` values, without opposite-hand placeholders.
The panel is placed in front of the initial gaze and placed again on a LOCAL
reference-space change. Aim rays are diagnostic visuals with no command actions.

The app has no robot-arming feedback or robot command path. Local armed status
is a diagnostic trigger latch; actual motion permission belongs to the ROS
mapper and future controller safety implementation. The existing PC-side
implementation order is unchanged.

Implementation:

- `openxr_program.cpp`, `tracking.h`, `font8x8.h`, `logger.h`: adapted OpenXR app;
- `usb_transport.h`: existing bounded TCP transport, extracted unchanged;
- `telemetry.h`: existing wire serialization adapted to the new tracking frame,
  plus the diagnostic deadman latch;
- `native_activity.cpp`: Android event/frame loop and app-to-USB adapter.

UDP, Wi-Fi subscription/configuration, on-headset velocity differentiation,
FR3 control, homing, and continuous gripper-width mapping are not part of this
port. HMD/aim/thumbstick data are local diagnostics; wire packets retain the
existing controller fields. Attribution and upstream licenses are retained in
`THIRD_PARTY_LICENSES`.

Robot-free adapter regression test, from the repository root:

```bash
docker compose -f docker/compose.yaml run --rm panda-ros2 \
  python3 /ws/android/quest_teleop/test/test_telemetry.py
```

This compiles the actual C++ adapter and sender against an Android logging
stub, exercises the display latch, and sends synthetic tracking frames to an
isolated loopback TCP peer. It does not access ADB, the headset, or FCI.

User runtime acceptance on 2026-10-10 confirms the display, passthrough,
recenter/aim-ray visuals and USB unplug/replug behavior. The ROS-only button
schema cleanup keeps the validated APK compatible; reinstalling it is unnecessary.
