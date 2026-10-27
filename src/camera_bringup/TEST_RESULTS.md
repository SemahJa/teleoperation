# Phase 1 execution record — 2026-10-23

## Step 0 — isolate and reassess

Goal/rationale: work from fetched current code without altering the operator checkout.
Reused: Git worktrees and existing repository configuration/launch/recording code
for read-only architectural review. Decision: standalone camera package, separate
configuration schema, no teleoperation launch inclusion.

Commands: `git fetch origin main`, `git rev-parse origin/main FETCH_HEAD`,
`git rev-list --left-right --count main...origin/main`, history/source diffs,
`git worktree add -b feat/dual-realsense-cameras /home/mirmi/aljabnouni/franka_teleoperation_cameras origin/main`,
`git ls-files --stage`, `git submodule status`, dpkg-query, ldd,
`/usr/bin/rs-enumerate-devices`, `lsusb -t`, Docker version/image inspection.

Result: verified base e682b17716a3a246a981a64e9e85d28b103a5f97; local main is
0 commits ahead / 3 behind. Original porcelain status unchanged. No original
source/configuration files changed. Submodule initialization is malformed because
.gitmodules is absent; camera-only build avoids those unrelated dependencies.
Host/native SDK linkage and recorder/Hand limitations are detailed in README.md.

## Step 1 — standalone bringup and dependency pins

Goal/rationale: independent serial-selected cameras outside the 1 kHz controller.
Reused: installed official wrapper 4.56.4, SDK 2.56.5, ROS Humble ament/launch.
Decision: two executable processes; per-role YAML, explicit target/fallback;
aligned RGB-D with camera-local TF and synchronization only.

Added files:
- CMakeLists.txt, package.xml, camera_bringup/__init__.py
- camera_bringup/configuration.py, launch/cameras.launch.py
- config/wrist.yaml, config/scene.yaml
- camera_bringup/smoke_test.py, test/test_configuration.py
- README.md, TEST_RESULTS.md, evidence/*
- ../../docker/cameras.Dockerfile, ../../docker/cameras-compose.yaml

Commands: sourced /opt/ros/humble/setup.bash; camera-only `colcon build` with
--base-paths src/camera_bringup, --build-base /tmp/cameras-build,
--install-base /tmp/cameras-install and --log-base /tmp/cameras-colcon-log.
`PYTHONPATH=src/camera_bringup python3 -m pytest -q src/camera_bringup/test`:
**5 passed**. Build, Python syntax, manifest parsing and scoped whitespace checks pass.

## Step 2 — bounded native camera acceptance

Goal/rationale: verify one camera at a time before simultaneous delivery.
Reused: native wrapper and source-installed camera package. Decision: isolated ROS
domain 137 with localhost-only discovery, no recorder/controller/Hand nodes.

Commands: `ros2 launch camera_bringup cameras.launch.py cameras:=wrist`, then
scene, then both; `ros2 run camera_bringup smoke_test --roles ... --seconds 12`.
Separate fallback run used cameras:=scene scene_profile:=usb2_15fps and
--expected-fps 15. Missing-peer run substituted scene_config with a temporary
nonexistent serial 000000000000 and checked wrist for eight seconds.
Launch processes were stopped with SIGINT and exited cleanly.

USB: wrist serial 934222072657 at 5000 Mbps, FW 5.17.0.10; scene serial
238222073675 at 480 Mbps, FW 5.17.3.10. Both advertise 640x480 RGB/depth
at 30 and 15 FPS. The scene USB2 connection sustained the target during these
short checks: no throughput failure was observed and no downgrade was applied.
The explicit 15 FPS fallback is also verified, not silently selected.

## Step 3 — independent Docker deployment

Goal/rationale: reuse the existing ROS image without changing robot deployment.
Reused: local panda-ros2-fci-4.2.2:dev image and its installed ROS tools.
Decision: new franka-dual-realsense:4.56.4-sdk2.56.5 image, SDK RSUSB backend,
wrapper compiled against the same SDK, dedicated franka-cameras Compose project.
SDK/wrapper source commits are checked against the pinned releases.

Commands: `docker compose -f docker/cameras-compose.yaml config --quiet`,
`docker build -f docker/cameras.Dockerfile -t franka-dual-realsense:4.56.4-sdk2.56.5 .`,
Compose up -d --no-build, exec smoke_test for both cameras, node/topic lists and
ldd inspection, then Compose down. All tests used domain 137. Image build passed.
Runtime reports both built-with and running-with SDK 2.56.5; ldd resolves
/usr/local/lib/librealsense2.so.2.56. Graph contains just wrist/scene camera nodes;
no IMU/pointcloud topics. Camera test container removed afterward.

The initial Docker run saw USB-interface-busy/power-state errors and a wrist
streamer watchdog during startup. The wrapper retried and the scene started
late: 228 RGB / 227 depth frames during a 12-second checker interval.
The initial checker passed on active-stream rate alone; that does NOT demonstrate
full-window delivery. Initial raw JSON/log are preserved as docker-initial.*.
The checker was tightened to require >=70% of expected full-window image count,
in addition to >=70% expected arrival rate, dimensions, valid stamps, CameraInfo,
matching per-camera RGB/depth stamps and optical TF. Final test used an explicit
18-second startup wait. The final image includes this stronger checker.
Startup retries and control-transfer warnings remain a deployment limitation,
not a claim of immediate readiness or long-term stability.

## Measurements

All images are 640x480; RGB is rgb8, aligned depth 16UC1 in each role's color
optical frame. Corresponding CameraInfo and both optical TF frames were observed.
Counts and active arrival rates are distinct; rates alone do not prove coverage.

| Test / camera | Observation seconds | RGB / depth frames | RGB / depth Hz | Shared stamps |
|---|---:|---:|---:|---:|
| Native wrist: wrist | 12 | 360 / 360 | 29.98 / 29.98 | 360 |
| Native scene: scene | 12 | 360 / 360 | 29.98 / 29.98 | 360 |
| Native simultaneous: wrist | 12 | 358 / 359 | 29.9 / 29.9 | 358 |
| Native simultaneous: scene | 12 | 360 / 360 | 29.98 / 29.98 | 360 |
| Native scene fallback: scene | 12 | 180 / 181 | 14.99 / 14.99 | 180 |
| Missing scene serial (wrist continues): wrist | 8 | 240 / 241 | 29.98 / 29.98 | 240 |
| Docker simultaneous after 18 s startup wait: wrist | 12 | 358 / 359 | 29.82 / 29.9 | 358 |
| Docker simultaneous after 18 s startup wait: scene | 12 | 339 / 337 | 30.15 / 29.99 | 337 |

Original native JSON evidence is retained unchanged; the native image counts also
satisfy the final checker's stronger full-window count threshold. Shared stamps
are per camera only, not camera-to-camera or robot alignment.

## Remaining limits and review boundary

Native startup warns about compile SDK 2.56.4 versus runtime 2.56.5 and an invalid
firmware power_line_frequency default (3 outside supported 0..2); streams still
opened the explicitly requested profiles. The source-built Docker wrapper avoids
the SDK mismatch but exhibits USB initialization/control-transfer warnings.
No physical unplug/replug, reconnect timing, lengthy streaming, visual calibration
accuracy, clock alignment or image-quality assessment was performed. A missing
serial test establishes peer startup isolation, not every physical failure mode.
The smoke checker is specific to 640x480 and accepts a 70% delivery floor; it is
not a lossless-recording acceptance test. Camera QoS design remains Phase 2.

No existing robot/Hand/safety/recording implementation or Quest integration was
modified. No host packages installed/removed. No physical robot/gripper commands,
teleoperation launch, commit or merge. Phase 2 awaits review.
