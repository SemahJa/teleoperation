# Reproducible development environments

## ROS 2 workspace

Use the existing ROS container on Ubuntu 22.04 with ROS 2 Humble. The robot-control process must run on the deployment host's PREEMPT_RT kernel; Docker does not provide the kernel. Do not launch the physical controller until the new controller's hardware-safety acceptance gate is complete.

The package boundaries and launch commands are described in [PACKAGE_STRUCTURE.md](PACKAGE_STRUCTURE.md): `quest_teleop_interfaces`, `quest_teleop`, `recording`, `teleop_controllers`, `teleop_gripper`, `safety_filter` and `teleop_bringup`. The legacy `panda_bringup` and `panda_demos` packages are intentionally unchanged. Build the new workspace with the existing Humble container workflow after importing pinned third-party dependencies.

The USB listener is available as `ros2 launch quest_teleop quest_usb_bridge.launch.py`. Start the ADB reverse mapping with `scripts/quest_adb_reverse.sh`. Wire framing and JSON schema are documented in `docs/quest-usb-protocol.md`. The headset application now uses the OpenXR app/HUD port from `thesis_ws`, with optional passthrough, controller aim rays, and readable per-hand pose/native twist/input diagnostics. It sends the existing versioned framed telemetry over USB/ADB. The HUD reports USB connection, clock validity/uncertainty, and right-controller local trigger/deadman state; robot arming is explicitly unknown. The bridge publishes raw telemetry and source-time validity after clock synchronization. Mapping is implemented and tested against mock measured poses; the atomic arm consumer, Hand worker, independent homing and Panda CBF runtime now pass robot-free acceptance. Integrated single/dual launch and left Menu recording commands are in `docs/TELEOPERATION_RUNTIME.md`. Physical commissioning remains separate. See `docs/VR_APP_PORT.md` for port details and runtime checks.

## Android / ADB toolchain

Build the Android toolchain image and enter its build container:

```bash
docker compose -f docker/android-compose.yaml build
docker compose -f docker/android-compose.yaml run --rm android-build gradle assembleDebug
```

The image pins Android API 34, build tools 34.0.0, NDK 26.3.11579264, JDK 17, and Gradle 8.7. The Quest APK executes on the headset; the container only provides build and deployment tools.

For USB ADB, connect the authorized Quest over USB and run the ADB service with access to `/dev/bus/usb`. The ADB and ROS services use Linux host networking so ADB reverse reaches the loopback-bound ROS listener; the ADB server listens only on host loopback and has no published port.

```bash
docker compose -f docker/android-compose.yaml up -d quest-adb-server
docker compose -f docker/android-compose.yaml run --rm quest-adb adb devices
```

Authorize the host key in the Quest headset. Never enable wireless ADB. USB permissions and hotplug behavior depend on the deployment host and must be verified there. Install and start the APK with `scripts/quest_install.sh`.

## Recording

`ros2 launch teleop_bringup robot_free.launch.py` loads validated per-arm mapper configuration and optional recording. Recording is disabled by default; enable it with `recording_enabled:=true`. Replace its topics via `recording_topics:='["/selected/topic"]'` or a custom `recording_config` YAML file. Bags default to `/ws/log/quest_bags` inside the existing mounted workspace. See `docs/ROBOT_FREE_MAPPING.md` for complete commands, two-arm mock configuration, and manual calibration conventions.

## Verification status

Verified 2026-10-10: the adapted Android APK builds in the existing Docker environment using the explicitly pinned NDK 26.3.11579264; mapper and robot-free bridge integration tests pass; the actual C++ headset adapter/USB sender passes a mock TCP regression test. The user confirms successful Quest display/passthrough/recenter/aim-ray and USB reconnection checks. Supplied samples show stable held right A and pressed left Y, with valid native twist and clock uncertainty below 1.4 ms. The ROS button schema now lists only each hand's physical buttons; restart bridge/subscriber processes after rebuilding interfaces. The validated APK remains compatible. Deployment hardware was not inspected and no FCI connection was made.


The integrated controller launch defaults to a single right-controlled mock,
with independent left Menu recording. Use single-left or dual configuration as
documented in `docs/TELEOPERATION_RUNTIME.md`. The original robot-free mapper
launch retains its explicit launch-time recording option. Controller runtime
and Hand settings are in `config/teleoperation_runtime.yaml`; CBF design and
physical limitations are in `docs/CBF_PANDA.md`. The combined robot-free suite
passes 81 individual cases; physical compatibility/RT/clearance/stop acceptance
has not been performed.
