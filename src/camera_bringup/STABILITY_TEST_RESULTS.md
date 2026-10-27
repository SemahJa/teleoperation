# Loss investigation and five-minute Phase 2 validation — 2026-10-26

**The requested five-minute native dual RGB-D recording passed with the explicitly selected large shared-memory profile.** Zero missing frame identities were detected inside the common observer/bag window; no frame-header or metadata-counter gaps, duplicates or timestamp reversals were detected in the recorded steady interval. This is bounded evidence for this installation, not a guarantee of lossless recording for arbitrary durations or workloads.

No Panda controller, physical gripper, calibration, robot configuration or safety logic was started/changed. No commits or merges. Physical extrinsics remain deferred. Existing stream profiles and the original checkout remain preserved.

## Method and measured transport effect

Native Ubuntu22.04/ROS2 Humble, FastDDS2.6.8/rmw_fastrtps_cpp6.2.7, wrapper4.56.4 compiled againstSDK2.56.4/runtimeSDK2.56.5 (warning retained). Wrist D435934222072657, USB3.2/5000Mbps; scene D435i238222073675, USB2.1/480Mbps. Both explicitly selected640x480,RGB8/aligned16UC1,30FPS. No fallback or kernel/network tuning.

Separate wrist/scene/both trials used10seconds without recording, then15seconds requested MCAP recording after4seconds warmup. All used the existing MenuRecorder programmatically with Menu disabled, camera topics plus tf_static and timing/health, best-effort image QoS and256MiB recorder cache. Additional raw/header subscribers used history30; images were not retained by diagnostics. Recordings and observer identities were compared only over their common interval. The scene/wrist-only tests were not physical robot tests.

| Default transport trial | Recorded RGB / depth | Estimated RGB / depth header gaps |
| --- | --- | --- |
| Wrist only | 431 / 440 | 10 / 1 |
| Scene only | 433 / 441 | 6 / 0 |
| Both: wrist | 347 / 359 | 82 / 72 |
| Both: scene | 386 / 427 | 44 / 4 |

Default dual-camera no-recording baseline received300frames in each image stream over10seconds with no estimated gaps. Metadata counters stayed continuous in the short recording trials. Some frames observed independently were absent from the dual bag (wristRGB16/depth44,sceneRGB7/depth1), demonstrating recorder-path misses. Independent RGB observers also missed many samples: shared gaps do not prove camera publication failures. Host UDP error counters did not increase in the first wrist comparison; low kernel UDP buffer defaults alone do not explain all observed losses.

The opt-in `config/fastdds_local_large_frames.xml` configures64MiB shared-memory segments, queue capacity2048 and UDP restricted to127.0.0.1, with built-in transports disabled. Observed default segments were approximately537KiB; the new segments approximately65MiB including overhead. Stream settings, recorder/cache and host sysctls were unchanged. `ROS_LOCALHOST_ONLY=0` is required with this explicit profile so Humble does not append its own default transports; UDP remains restricted by the XML. This is scoped to launched processes, never applied to real-time control globally.

A20second-requested dual comparison with the new profile recorded590frames in each RGB/depth/metadata stream, zero estimated gaps and exact independent-observer/bag identity agreement. The controlled A/B change identifies DDS transport buffering as a major contributor here; it does not uniquely identify every internal buffer/drop mechanism or certify every future workload.

Relevant primary references: [FastDDS large-data guidance](https://fast-dds.docs.eprosima.com/en/v2.6.11/fastdds/use_cases/large_data/large_data.html), [shared-memory sizing guidance](https://fast-dds.docs.eprosima.com/en/2.x/fastdds/transport/shared_memory/shared_memory.html), [Humble localhost transport construction](https://github.com/ros2/rmw_fastrtps/blob/humble/rmw_fastrtps_shared_cpp/src/participant.cpp).

## Five-minute result

Requested300seconds; MCAP receipt interval299.650686717seconds, excluding recorder discovery/startup boundaries. Episode:

`log/camera_stability_20261026/large_shm_both_5min/bags/quest_20261026T105141Z_31c545f9`

| Recorded stream | Frames | Header FPS | Estimated header gaps | Duplicates / backwards |
| --- | --- | --- | --- | --- |
| Wrist RGB | 8,984 | 29.97818 | 0 | 0 / 0 |
| Wrist aligned depth | 8,984 | 29.97818 | 0 | 0 / 0 |
| Scene RGB | 8,983 | 29.97801 | 0 | 0 / 0 |
| Scene aligned depth | 8,983 | 29.97801 | 0 | 0 / 0 |

Each camera's color/depth metadata counts match its image counts with no frame-counter gaps/duplicates/resets. Common-window image multiset differences: observer-only0,bag-only0 for all four streams. Representative RGB/depth decode passed and all32 sparse pixel payload checksums matched. CameraInfo, metadata, original timestamps, optical TF, host clock observations and health remain recorded. Camera clocks remain independent; matching image headers and `global_time` labels do not establish exposure synchronization.

MCAP27,635,520,142bytes (27.64GB decimal); mean92,225,786bytes/s (92.23MB/s). Storage warning scan empty; no portable recorder-drop counter is available, so loss acceptance uses independent identities and timestamp/counter checks rather than a claimed storage counter of zero. Host UDP receive/send-buffer errors increased by0 during recording plus validation; counters are host-global, not process attribution. No sustained stream disconnect/stall was detected in the bag. A scene depth-start hardware warning was logged before the measurement interval and is retained, not treated as clean startup.

Offline validation took176.49seconds on this host; it reads raw data and computes derived diagnostics/alignment. A reusable driver now exposes a separate `--validation-timeout` (default600seconds). The recording's five-minute duration does not include validation. Resource samples are in `large_shm_both_5min/resource_samples.jsonl`; no RAM-growth/leak guarantee is inferred from this short run.

## Separate fault/recovery test

Both cameras launched as independent processes. Stopped wrist ROS launch, waited for drain, observed seven seconds with wristRGB0 and sceneRGB210. Relaunched wrist by serial; subsequent eight-second window received wristRGB226 and sceneRGB240, including wrist startup latency. Same MCAP recorder continued and finalized readable output. Scene recorded736RGB/depth/metadata samples with no header gaps throughout the trial. Wrist's planned outage/startup gaps remain in raw data and diagnostics.

This verifies process isolation, late reappearance and recorder continuation. **Physical USB unplug/replug, USB bus/power faults and automatic SDK hotplug recovery were not tested.** Those remain distinct hardware-validation limitations.

## Reproduce and operate

Complete camera+Quest+mock+Menu pipeline with the tested profile:

```bash
cd ~/aljabnouni/franka_teleoperation_cameras
CAMERA_TRANSPORT=large_shm ROS_DOMAIN_ID=99 scripts/start_demonstration_recording.sh
```

Default behavior stays unchanged when `CAMERA_TRANSPORT` is omitted. No stream or SDK fallback occurs. The script explicitly selects this XML only for FastDDS. Do not source it globally into a robot-control environment. Native validation did not start Quest or arm/gripper processes; the integrated startup procedure launches only the existing bridge/mapper/mock pose/recorder and cameras.

Camera-only trial (choose a fresh label; existing output is never overwritten):

```bash
source /opt/ros/humble/setup.bash
source install/demonstration/local_setup.bash
export ROS_DOMAIN_ID=139 ROS_LOCALHOST_ONLY=0
export FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/src/camera_bringup/config/fastdds_local_large_frames.xml"
python3 scripts/validate_camera_recording.py --roles both \
 --baseline-seconds 10 --seconds 300 --label five_min_repeat \
 --output-root "$PWD/log/camera_stability_repeats"
```

For default A/B runs, unset `FASTRTPS_DEFAULT_PROFILES_FILE`, use `ROS_LOCALHOST_ONLY=1`, choose roles wrist/scene/both and15seconds. Do not run multiple camera launches against the same serial simultaneously. Driver keeps15GiB free-space reserve and graceful shutdown/finalization, writes small result JSON alongside the raw bag, and starts no robot/gripper.

Exact executed driver and process-recovery reproducer, stdout, per-camera launch logs, manifests and native test outputs are preserved in `log/camera_stability_20261026`.45 relevant tests passed again:camera5,recording17,timing23. New transport XML was parsed by real ROS participants and exercised by live cameras/recorder; shell syntax and diff whitespace checks passed. No controller code or Phase3 clock/alignment algorithm was changed.

Optional Docker image must be rebuilt to include this new config. For the same local transport within one host-network/host-IPC recording container, use `ROS_LOCALHOST_ONLY=0` and `FASTRTPS_DEFAULT_PROFILES_FILE=/opt/demonstration_ws/install/camera_bringup/share/camera_bringup/config/fastdds_local_large_frames.xml`. This follow-up validated native streaming; container live streaming is still not qualified.

## Acceptance boundary

The loss investigation and requested five-minute sustained native camera/MCAP test are complete. No detected losses within this run is stronger than merely readable MCAP, but not an absolute no-loss guarantee. Startup/discovery boundaries are excluded explicitly. Longer sessions, other USB/cable topologies, power faults, physical hotplug and live Docker streaming remain unverified. Real Panda/gripper telemetry and physical execution/safety remain separate tasks. Exposure synchronization and extrinsics remain uncalibrated; calibration is deferred as requested.
