# Dual RealSense camera bringup — Phase 1

Camera-only package. No robot controller, Hand, Quest, safety or recorder changes.
Do not launch teleoperation to test these cameras. Worktree:
`~/aljabnouni/franka_teleoperation_cameras`, branch `feat/dual-realsense-cameras`.

## 0. Verified baseline and reused components

On 2026-10-23, `git fetch origin main` verified
`e682b17716a3a246a981a64e9e85d28b103a5f97` as both FETCH_HEAD and origin/main.
Original main remains at 8995b2c, behind by three, with zero divergent commits.
The new branch starts at e682b17; no pull/merge/reset/switch/commit was performed.
Original Android generated changes were preserved (porcelain status compared).

Changes since 8995b2c include optional independent mapped-pose/velocity filtering
and causal interpolation; orientation-only commands; four-button raw Quest
messages with thumbstick opening; revised held-close/stop Hand behavior and
separate reporting; runtime isolation; and single/dual launch config changes.
Telemetry still preserves original source, receipt and control clocks in coherent
TrackingSample messages, with full-rate recording separate from live reporting.

Reinspected recording/configuration.py, runtime_configuration.py and integrated
teleoperation.launch.py before implementing. The current recording default is
left controller input plus each arm's raw controller input, mapped command,
measured pose and tracking_samples. It does NOT include gripper_events or robot
state by default. Menu recording takes the publisher intersection at episode
start. GripperStatus has a ROS header, state, operation and counters, without
structured source/request/start/completion timestamps; the incoming implementation
uses latest-only reporting. Do not interpret gripper_events as a lossless operation
log or console publication time as physical actuation time. Preserve original
messages and timestamps when designing future datasets.

The strict combined teleoperation schema accepts only its known sections. Camera
settings therefore live in a separate package and launch; no camera keys were
added to existing runtime YAML. The official standalone RealSense executable and
ament/ROS launch conventions are reused; there is no robot launch inclusion.

Dependency caveat: the repository has four gitlinks but no .gitmodules. Consequently
`git submodule status` fails on generated Android OpenXR entries. Franka/libfranka
gitlinks are empty in the new worktree. Camera-only colcon builds use only
src/camera_bringup and need none of those dependencies. Do not recursively
initialize these malformed gitlinks. Full robot builds require the existing
third_party.repos/scripts/fetch_teleop_dependencies.sh workflow, outside this phase.

## 1. Dependencies and design

Native host: Ubuntu 22.04.5, ROS 2 Humble, realtime kernel 5.15.0-1058-realtime.
Reuse the already installed official realsense2_camera 4.56.4 and native SDK 2.56.5.
No host packages were changed. Installed Debian versions:

- ros-humble-realsense2-camera: 4.56.4-2jammy.20251228.100105
- ros-humble-realsense2-camera-msgs: 4.56.4-2jammy.20251218.035951
- librealsense2: 2.56.5-0~realsense.17054

Wrapper startup reports built with 2.56.4 and running with 2.56.5. This pair passed
bounded native RGB-D tests; the build/runtime mismatch warning remains. Upstream
4.56.4 belongs to the SDK 2.56 release family; this is observed compatibility,
not a claim of exhaustive upstream certification of this exact binary pairing.
Sources: [official release](https://github.com/realsenseai/realsense-ros/releases/tag/4.56.4)
and [versioned wrapper documentation](https://github.com/realsenseai/realsense-ros/blob/4.56.4/README.md).

`ldd /opt/ros/humble/lib/librealsense2_camera.so` resolves SDK 2.56 in
/lib/x86_64-linux-gnu. The discrepancy in enumeration is explained by PATH:
`/opt/ros/humble/bin/rs-enumerate-devices` belongs to ros-humble-librealsense2
2.55.1 and resolves /opt/ros/humble/lib/x86_64-linux-gnu/librealsense2.so.2.55.
`/usr/bin/rs-enumerate-devices` belongs to native librealsense2-utils 2.56.5 and
resolves SDK 2.56. Use the latter for matching native diagnostics. No global
LD_LIBRARY_PATH changes, SDK removal or native library mounts into Docker.

Two separate processes select quoted serial numbers (wrapper underscore prefix
prevents numeric coercion). Node names camera_name=wrist/scene and namespace
/cameras yield private stream prefixes /cameras/wrist and /cameras/scene. Both
configs are validated before either process starts, including unique serials.
The wrapper waits/reconnects for missing devices; no process-exit handler stops
the other process. No composable shared camera container or controller thread.

Each role has its own YAML and profile selection. Target: RGB8 + Z16,
640x480 at 30 FPS, alignment to color, enable_sync=true. IMU, infrared,
pointcloud and combined RGBD message are disabled. Camera-local optical TF is
published, without invented camera-to-robot/world extrinsics. enable_sync is
per-device RGB/depth pairing, NOT synchronization between cameras or the robot.

## 2. Native deployment and commands

From this worktree, build only the camera package:

```bash
source /opt/ros/humble/setup.bash
colcon --log-base /tmp/cameras-colcon-log build \
  --base-paths src/camera_bringup \
  --build-base /tmp/cameras-build --install-base /tmp/cameras-install
source /tmp/cameras-install/setup.bash
ros2 launch camera_bringup cameras.launch.py cameras:=wrist
# Or cameras:=scene / cameras:=both. Stop an existing camera launch first.
```

Keep camera test terminals on an unused domain (137 was used here):

```bash
export ROS_DOMAIN_ID=137 ROS_LOCALHOST_ONLY=1
ros2 launch camera_bringup cameras.launch.py cameras:=both
# In a second equally sourced terminal with the same domain:
ros2 run camera_bringup smoke_test --roles wrist scene --seconds 12
```

Each role publishes, among other wrapper topics:

- /cameras/ROLE/color/image_raw
- /cameras/ROLE/color/camera_info
- /cameras/ROLE/aligned_depth_to_color/image_raw
- /cameras/ROLE/aligned_depth_to_color/camera_info

Optical frames: ROLE_color_optical_frame and ROLE_depth_optical_frame.
Aligned depth uses ROLE_color_optical_frame. Depth image encoding is 16UC1.

Independent manual fallback (never selected automatically):

```bash
ros2 launch camera_bringup cameras.launch.py cameras:=both \
  wrist_profile:=target scene_profile:=usb2_15fps
# Scene-only fallback verification:
ros2 run camera_bringup smoke_test --roles scene --expected-fps 15 --seconds 12
```

For other resolutions/FPS, copy a role YAML, edit its profiles, then pass
wrist_config:=/absolute/path.yaml or scene_config:=/absolute/path.yaml. Booleans
control color/depth/alignment; aligned depth requires both streams and sync
requires matching FPS. This phase's smoke checker expects 640x480; adjust the
checker for other resolutions. Unsupported profiles can cause the upstream
wrapper to fall back: always inspect its actual Open profile logs and verify
received dimensions/rates; this launch never deliberately substitutes a profile.

## 3. Docker deployment

The existing local panda-ros2-fci-4.2.2:dev image (c1e54a7814cc) has no RealSense
packages. docker/cameras.Dockerfile derives a NEW image, compiles SDK v2.56.5
(RSUSB backend, no kernel patch) and wrapper 4.56.4 together, then builds this
package. Source release tags are fixed and checked against SDK commit
38a41441971387197193ad3aeae3cefe6a11f2cb and wrapper commit
bafc21080c5c8e259dadbb309797949aee0dd950; no unversioned RealSense apt installs.
It refuses an already SDK-populated base. General Ubuntu/ROS build dependencies
are not a complete package-lock snapshot. Host udev rules already exist.

```bash
cd ~/aljabnouni/franka_teleoperation_cameras
# Existing robot image must already be present. No robot service is started.
docker compose -f docker/cameras-compose.yaml build
ROS_DOMAIN_ID=137 ROS_LOCALHOST_ONLY=1 \
  docker compose -f docker/cameras-compose.yaml up cameras
# Optional environment selection before up:
# CAMERAS=scene SCENE_PROFILE=usb2_15fps
# In another terminal, same compose environment:
docker compose -f docker/cameras-compose.yaml exec cameras \
  bash -lc 'source /opt/camera_ws/install/setup.bash; ros2 run camera_bringup smoke_test --roles wrist scene --seconds 12'
docker compose -f docker/cameras-compose.yaml down
```

The separate service uses host networking and USB device cgroup major 189, with
/dev/bus/usb mounted for enumeration/reconnect and /run/udev read-only. No
privileged mode, realtime capability, robot launch or SDK host mount. USB access
covers USB devices rather than one fixed bus path so serial-based reconnect can
work. Use the same ROS_DOMAIN_ID as teleoperation only when later ready for a
shared graph. Avoid native and container camera drivers opening the same devices.
The original compose file, existing image and Android ADB container are untouched.

## 4. Actual validation and limits

See TEST_RESULTS.md and evidence/*.json for measured results, warnings and scope.
No physical robot/Hand command, teleoperation launch, recorder change, automatic
commit or merge was performed. Phase 2 is deliberately deferred: long streaming,
camera/robot timestamp alignment, extrinsics/robot TF, MCAP, recorder topics/QoS,
episode metadata and offline dataset validation.
