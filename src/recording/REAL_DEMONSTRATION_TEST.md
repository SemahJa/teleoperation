# Real demonstration collection test — 2026-10-26

Result: real arm recording and integrated camera/Quest/telemetry capture passed.
Recorded gripper transitions remain unvalidated: gripper operations occurred before
Menu started the saved episode. This is a validation episode, not a complete arm-and-gripper BC example.

## Setup and changes

Operator confirmed the former joint-limit issue resolved, this Panda cleared for
controlled motion, arm plus gripper operation authorized, and empty gripper/full
workspace clear. Initial SDK snapshots showed User stopped. After the operator
released the emergency stop, SDK reported Idle. No automatic recovery/unlocking.

Built ten current-worktree packages in Docker, isolated under `build/hardware_demo`
and `install/hardware_demo`. Reused original SDK0.9.2 through original checkout
mounted read-only at `/ws`; no SDK upgrade. Copied missing OSQP0.6.3 source and its
QDLDL dependency from the original checkout, revision
`0dd00a578cf1c2691c5c379965d504c75bf6cfad`, matching the repository pin.

Created `log/real_demonstration_20261026/{hardware,mock}.yaml`, `environment.sh`,
`pipeline.launch.py`, `start_hardware.sh`, readiness/smoke/analysis JSON and startup
logs. Only recording settings differ between hardware.yaml and the source
single_panda.yaml: demonstration profile, MCAP, late discovery, cache268435456,
same-host provenance, output directory. All arm/Hand/gains/filters/safety values
are identical to the existing configuration. No controller or gripper source edits.
Existing CBF setting remains disabled; existing actuator magnitude/rate limits
remain enforced. Mock deployment tested first. Real startup used the existing
operator_confirm gate with the operator's explicit authorization.

## Tests and saved data

Seven package suites (camera_bringup, demonstration_timing, recording, safety_filter,
teleop_gripper, teleop_controllers, teleop_bringup): colcon test-result reports
213 tests, zero errors/failures/skips. This aggregate includes CTest/pytest/gtest
reporting. Build: ten packages passed. Mock live camera check12s and real hardware
camera check8s passed. Tests use mock owners; actual hardware is validated separately.

Episode: `log/real_demonstrations/quest_20261026T121156Z_14899d0d`.
Menu start/stop finalized readable MCAP, zero readback validation errors.
Duration12.982254s;1,137,433,601bytes;87.614MB/s over the bag duration;28 topics.
Raw RGB rgb8 and aligned depth16UC1 decode correctly,640×480. Depth units are
preserved; independent metric depth-scale verification is still needed.

| Stream | Saved count | Observed header FPS |
|---|---:|---:|
| Wrist RGB / aligned depth |361 /361|29.978 /29.978|
| Scene RGB / aligned depth |361 /361|29.972 /29.972|
| Each of four metadata and four CameraInfo streams |361 each|~30|
| TrackingSample full stream |12,892|~1kHz|
| Joint states / teleop anchor pose |1,301 each|~100Hz|
| FrankaState |1,300|~100Hz|
| Cartesian commands |1,043|variable|
| Quest right / left |924 /923|~71Hz|
| GripperStatus |1|latched pre-episode status|

TrackingSamples all hardware_backend=true;10,481 active,2,411 position_hold.
Zero control-cycle gaps/reorders; max monitor_queue_drops=0. All12,892 recorded
source/receipt/output order checks passed. Source/receipt/output are host timing
observations, not actuator acknowledgements. Measured end-effector ranges across
this episode:27.74mm X,13.59mm Y,42.17mm Z. Genuine joint and pose changes recorded.
Raw Quest trigger reached1.0; grip remained0 throughout this episode. The single
latched GripperStatus preserves width and counters, but is not a fresh continuous
measurement. Successful empty-hand calibration and gripper close/stop operations
appear in hardware_startup.log before the episode. Recorded opening/closing
transitions were not verified; do not invent or carry forward measured gripper
state for training.

No detected internal image/header/native frame-counter gaps, duplicates, resets,
or non-monotonic timestamps. RGB/aligned-depth header matches361/361 per camera.
Independent image observer and bag have zero observer-only/bag-only identities
inside common windows. These do not exclude a shared upstream loss. Images cover
about12.01s of the12.98s bag: recorder discovery/boundary coverage differs from
steady delivery; do not treat the entire episode as camera-covered. Recorder-wide
portable drop counter remains unavailable/null. USB2 scene remained explicit30FPS.
Docker SDK startup emitted transient USB_BUSY/power/watchdog notifications before
the successful stream checks; preserved in logs, not erased by a clean smoke result.

## Timing and alignment

All camera metadata reports global_time. Wrist/scene nearest ROS-header phase:
median3.516ms, range2.663–6.521ms. Equal RGB/aligned-depth headers are wrapper
relationships, not exposure-time equivalence. Same-camera firmware metadata
exposure-midpoint differences median depth-minus-color:-0.830ms wrist,-0.727ms scene;
these are not a calibrated cross-device capture bound.

ROS/system→host-monotonic empirical fit envelope48,261ns; fitted rate
1.0000000337296984 over this short recording. This measures the host-clock bridge,
not camera or encoder capture accuracy or a long-run drift guarantee. Quest
reported clock uncertainty0.399–0.583ms. Cross-camera exposure synchronization and
capture-time uncertainty remain unknown. Extrinsics remain uncalibrated.

Provisional50ms alignment:361 anchors; all four camera streams361/361 matched;
robot355/361; causal action317/361; gripper0/361. Six robot anchors are conservatively
masked near segment transitions;44 actions lack an admissible causal match. The
old latched gripper event falls outside a measured clock relationship and is
rejected; no fabricated state. Original timestamps/raw messages retained.
`timing_records.jsonl`, `alignment_provisional.json`, and both manifests are saved
alongside MCAP. Matching50ms and a small host fit error do not establish exposure
synchronization or physical execution time.

Missing requested optional topics: /tf, /panda_1/measured_ee_pose,
/panda_1/read_only_state, /panda_1/watchdog_events. Actual controller pose is recorded
in teleop_anchor_pose and TrackingSample. The separate read-only publisher was
intentionally absent while the real owner held FCI. No watchdog event occurred.

## Reproduce

Run only in a cleared lab session with Panda192.168.3.101 ready for FCI, empty
fingers/full travel clear, Quest authorized, and controls released. The saved
runtime YAML is a snapshot: review it before reuse if the installation changes.
Both existing Docker images/dependency prefixes must remain available.

```bash
cd ~/aljabnouni/franka_teleoperation_cameras
bash log/real_demonstration_20261026/start_hardware.sh
```

This uses the current-worktree build, loopback-limited large-SHM XML, host IPC,
ROS_DOMAIN_ID99, existing ADB container/forwarding/app, two cameras, existing
hardware teleoperation launch and existing Menu recorder. It starts real torque
control and moves the empty gripper for calibration. Wait for readiness, release
Menu then press once to start. Confirm recorder phase before performing actions:

```bash
docker exec real-demonstration bash -lc 'source /camera_ws/log/real_demonstration_20261026/environment.sh; ros2 topic echo /quest/recording_status --once --qos-durability transient_local --full-length'
```

Wait for phase recording and initial camera discovery (~1s in this test), then
perform the demonstration. Release trigger/grip and press Menu once to stop;
wait for phase idle after validation. Right A requests homing: avoid it in a small
demonstration test. Ctrl-C stops the launch; external graceful stop:

```bash
docker kill --signal=SIGINT real-demonstration
```

Offline inspect/validate without connecting hardware:

```bash
docker run --rm --network none --user "$(id -u):$(id -g)"  -v ~/aljabnouni/franka_teleoperation:/ws:ro  -v "$PWD:/camera_ws:rw" -w /camera_ws --entrypoint bash  franka-demonstration-recording:phase5 -lc  'source /camera_ws/log/real_demonstration_20261026/environment.sh; ros2 run recording validate_episode /camera_ws/log/real_demonstrations/quest_20261026T121156Z_14899d0d; python3 /camera_ws/log/real_demonstration_20261026/inspect_real_episode.py /camera_ws/log/real_demonstrations/quest_20261026T121156Z_14899d0d'
```

Rerun package tests in the same offline container/mounts (mock tests require its
loopback DDS but no robot network): source environment.sh, then:

```bash
colcon --log-base log/hardware_tests test --build-base build/hardware_demo --install-base install/hardware_demo --packages-select safety_filter teleop_gripper teleop_controllers teleop_bringup recording camera_bringup demonstration_timing --executor sequential
colcon test-result --test-result-base build/hardware_demo --verbose
```

Build command used inside that container, sourcing ROS/camera overlay and exporting
CMAKE_PREFIX_PATH=/ws/install/libfranka:$CMAKE_PREFIX_PATH and
LD_LIBRARY_PATH=/ws/install/libfranka/lib:$LD_LIBRARY_PATH:

```bash
colcon --log-base log/hardware_build build --base-paths /ws/src/third_party/franka_arm_ros2/franka_msgs src/quest_teleop_interfaces src/quest_teleop src/safety_filter src/teleop_controllers src/teleop_gripper src/recording src/demonstration_timing src/teleop_bringup src/camera_bringup --build-base build/hardware_demo --install-base install/hardware_demo --executor sequential --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release
```

## Review boundary

All seven real pipeline processes finished cleanly; post-shutdown snapshot Idle.
Test-added ADB reverse removed; preexisting ADB container preserved. Original
checkout/configuration/Docker images preserved; no commit/merge. Next validation
is a clearly confirmed recording containing gripper close/release/open transitions,
with their sparse/uncertain measurement contract retained. BC export must honor
missing masks, startup boundaries and action timing semantics. No full dataset
export, extrinsic calibration, or exposure synchronization claim is included.
